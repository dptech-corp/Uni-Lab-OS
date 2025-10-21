import json
import logging
import traceback
import uuid
import xml.etree.ElementTree as ET
from typing import Any, Dict, List

import networkx as nx
import matplotlib.pyplot as plt
import requests

logger = logging.getLogger(__name__)


class SimpleGraph:
    """简单的有向图实现，用于构建工作流图"""

    def __init__(self):
        self.nodes = {}
        self.edges = []

    def add_node(self, node_id, **attrs):
        """添加节点"""
        self.nodes[node_id] = attrs

    def add_edge(self, source, target, **attrs):
        """添加边"""
        edge = {"source": source, "target": target, **attrs}
        self.edges.append(edge)

    def to_dict(self):
        """转换为工作流图格式"""
        nodes_list = []
        for node_id, attrs in self.nodes.items():
            node_attrs = attrs.copy()
            params = node_attrs.pop("parameters", {}) or {}
            node_attrs.update(params)
            nodes_list.append({"id": node_id, **node_attrs})

        return {
            "directed": True,
            "multigraph": False,
            "graph": {},
            "nodes": nodes_list,
            "links": self.edges,
        }


def extract_json_from_markdown(text: str) -> str:
    """从markdown代码块中提取JSON"""
    text = text.strip()
    if text.startswith("```json\n"):
        text = text[8:]
    if text.startswith("```\n"):
        text = text[4:]
    if text.endswith("\n```"):
        text = text[:-4]
    return text


def convert_to_type(val: str) -> Any:
    """将字符串值转换为适当的数据类型"""
    if val == "True":
        return True
    if val == "False":
        return False
    if val == "?":
        return None
    if val.endswith(" g"):
        return float(val.split(" ")[0])
    if val.endswith("mg"):
        return float(val.split("mg")[0])
    elif val.endswith("mmol"):
        return float(val.split("mmol")[0]) / 1000
    elif val.endswith("mol"):
        return float(val.split("mol")[0])
    elif val.endswith("ml"):
        return float(val.split("ml")[0])
    elif val.endswith("RPM"):
        return float(val.split("RPM")[0])
    elif val.endswith(" °C"):
        return float(val.split(" ")[0])
    elif val.endswith(" %"):
        return float(val.split(" ")[0])
    return val


def refactor_data(data: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """统一的数据重构函数，根据操作类型自动选择模板"""
    refactored_data = []

    # 定义操作映射，包含生物实验和有机化学的所有操作
    OPERATION_MAPPING = {
        # 生物实验操作
        "transfer_liquid": "SynBioFactory-liquid_handler.transfer_liquid",
        "transfer": "SynBioFactory-liquid_handler.biomek-transfer",
        "incubation": "SynBioFactory-liquid_handler.biomek-incubation",
        "move_labware": "SynBioFactory-liquid_handler.biomek-move_labware",
        "oscillation": "SynBioFactory-liquid_handler.biomek-oscillation",
        # 有机化学操作
        "HeatChillToTemp": "SynBioFactory-workstation-HeatChillProtocol",
        "StopHeatChill": "SynBioFactory-workstation-HeatChillStopProtocol",
        "StartHeatChill": "SynBioFactory-workstation-HeatChillStartProtocol",
        "HeatChill": "SynBioFactory-workstation-HeatChillProtocol",
        "Dissolve": "SynBioFactory-workstation-DissolveProtocol",
        "Transfer": "SynBioFactory-workstation-TransferProtocol",
        "Evaporate": "SynBioFactory-workstation-EvaporateProtocol",
        "Recrystallize": "SynBioFactory-workstation-RecrystallizeProtocol",
        "Filter": "SynBioFactory-workstation-FilterProtocol",
        "Dry": "SynBioFactory-workstation-DryProtocol",
        "Add": "SynBioFactory-workstation-AddProtocol",
    }

    UNSUPPORTED_OPERATIONS = ["Purge", "Wait", "Stir", "ResetHandling"]

    for step in data:
        operation = step.get("action")
        if not operation or operation in UNSUPPORTED_OPERATIONS:
            continue

        # 处理重复操作
        if operation == "Repeat":
            times = step.get("times", step.get("parameters", {}).get("times", 1))
            sub_steps = step.get("steps", step.get("parameters", {}).get("steps", []))
            for i in range(int(times)):
                sub_data = refactor_data(sub_steps)
                refactored_data.extend(sub_data)
            continue

        # 获取模板名称
        template = OPERATION_MAPPING.get(operation)
        if not template:
            # 自动推断模板类型
            if operation.lower() in ["transfer", "incubation", "move_labware", "oscillation"]:
                template = f"SynBioFactory-liquid_handler.biomek-{operation}"
            else:
                template = f"SynBioFactory-workstation-{operation}Protocol"

        # 创建步骤数据
        step_data = {
            "template": template,
            "description": step.get("description", step.get("purpose", f"{operation} operation")),
            "lab_node_type": "Device",
            "parameters": step.get("parameters", step.get("action_args", {})),
        }
        refactored_data.append(step_data)

    return refactored_data


def build_protocol_graph(
    labware_info: List[Dict[str, Any]], protocol_steps: List[Dict[str, Any]], workstation_name: str
) -> SimpleGraph:
    """统一的协议图构建函数，根据设备类型自动选择构建逻辑"""
    G = SimpleGraph()
    resource_last_writer = {}
    LAB_NAME = "SynBioFactory"

    protocol_steps = refactor_data(protocol_steps)

    # 检查协议步骤中的模板来判断协议类型
    has_biomek_template = any("biomek" in step.get("template", "") for step in protocol_steps)

    if has_biomek_template:
        # 生物实验协议图构建
        for labware_id, labware in labware_info.items():
            node_id = str(uuid.uuid4())

            labware_attrs = labware.copy()
            labware_id = labware_attrs.pop("id", labware_attrs.get("name", f"labware_{uuid.uuid4()}"))
            labware_attrs["description"] = labware_id
            labware_attrs["lab_node_type"] = (
                "Reagent" if "Plate" in str(labware_id) else "Labware" if "Rack" in str(labware_id) else "Sample"
            )
            labware_attrs["device_id"] = workstation_name

            G.add_node(node_id, template=f"{LAB_NAME}-host_node-create_resource", **labware_attrs)
            resource_last_writer[labware_id] = f"{node_id}:labware"

        # 处理协议步骤
        for i, step in enumerate(protocol_steps):
            node_id = str(uuid.uuid4())
            G.add_node(node_id, **step)

            # 添加控制流边
            G.add_edge(prev_node, node_id, source_port="ready", target_port="ready")
            prev_node = node_id

            # 处理物料流
            params = step.get("parameters", {})
            if "sources" in params and params["sources"] in resource_last_writer:
                source_node, source_port = resource_last_writer[params["sources"]].split(":")
                G.add_edge(source_node, node_id, source_port=source_port, target_port="labware")

            if "targets" in params:
                resource_last_writer[params["targets"]] = f"{node_id}:labware"

        # 添加协议结束节点
        end_id = str(uuid.uuid4())
        G.add_node(end_id, template=f"{LAB_NAME}-liquid_handler.biomek-run_protocol")
        G.add_edge(prev_node, end_id, source_port="ready", target_port="ready")

    else:
        # 有机化学协议图构建
        WORKSTATION_ID = workstation_name

        # 为所有labware创建资源节点
        for item_id, item in labware_info.items():
            # item_id = item.get("id") or item.get("name", f"item_{uuid.uuid4()}")
            node_id = str(uuid.uuid4())

            # 判断节点类型
            if item.get("type") == "hardware" or "reactor" in str(item_id).lower():
                if "reactor" not in str(item_id).lower():
                    continue
                lab_node_type = "Sample"
                description = f"Prepare Reactor: {item_id}"
                liquid_type = []
                liquid_volume = []
            else:
                lab_node_type = "Reagent"
                description = f"Add Reagent to Flask: {item_id}"
                liquid_type = [item_id]
                liquid_volume = [1e5]

            G.add_node(
                node_id,
                template=f"{LAB_NAME}-host_node-create_resource",
                description=description,
                lab_node_type=lab_node_type,
                res_id=item_id,
                device_id=WORKSTATION_ID,
                class_name="container",
                parent=WORKSTATION_ID,
                bind_locations={"x": 0.0, "y": 0.0, "z": 0.0},
                liquid_input_slot=[-1],
                liquid_type=liquid_type,
                liquid_volume=liquid_volume,
                slot_on_deck="",
                role=item.get("role", ""),
            )
            resource_last_writer[item_id] = f"{node_id}:labware"

        last_control_node_id = None

        # 处理协议步骤
        for step in protocol_steps:
            node_id = str(uuid.uuid4())
            G.add_node(node_id, **step)

            # 控制流
            if last_control_node_id is not None:
                G.add_edge(last_control_node_id, node_id, source_port="ready", target_port="ready")
            last_control_node_id = node_id

            # 物料流
            params = step.get("parameters", {})
            input_resources = {
                "Vessel": params.get("vessel"),
                "ToVessel": params.get("to_vessel"),
                "FromVessel": params.get("from_vessel"),
                "reagent": params.get("reagent"),
                "solvent": params.get("solvent"),
                "compound": params.get("compound"),
                "sources": params.get("sources"),
                "targets": params.get("targets"),
            }

            for target_port, resource_name in input_resources.items():
                if resource_name and resource_name in resource_last_writer:
                    source_node, source_port = resource_last_writer[resource_name].split(":")
                    G.add_edge(source_node, node_id, source_port=source_port, target_port=target_port)

            output_resources = {
                "VesselOut": params.get("vessel"),
                "FromVesselOut": params.get("from_vessel"),
                "ToVesselOut": params.get("to_vessel"),
                "FiltrateOut": params.get("filtrate_vessel"),
                "reagent": params.get("reagent"),
                "solvent": params.get("solvent"),
                "compound": params.get("compound"),
                "sources_out": params.get("sources"),
                "targets_out": params.get("targets"),
            }

            for source_port, resource_name in output_resources.items():
                if resource_name:
                    resource_last_writer[resource_name] = f"{node_id}:{source_port}"

    return G


def draw_protocol_graph(protocol_graph: SimpleGraph, output_path: str):
    """
    (辅助功能) 使用 networkx 和 matplotlib 绘制协议工作流图，用于可视化。
    """
    if not protocol_graph:
        print("Cannot draw graph: Graph object is empty.")
        return

    G = nx.DiGraph()

    for node_id, attrs in protocol_graph.nodes.items():
        label = attrs.get("description", attrs.get("template", node_id[:8]))
        G.add_node(node_id, label=label, **attrs)

    for edge in protocol_graph.edges:
        G.add_edge(edge["source"], edge["target"])

    plt.figure(figsize=(20, 15))
    try:
        pos = nx.nx_agraph.graphviz_layout(G, prog="dot")
    except Exception:
        pos = nx.shell_layout(G)  # Fallback layout

    node_labels = {node: data["label"] for node, data in G.nodes(data=True)}
    nx.draw(
        G,
        pos,
        with_labels=False,
        node_size=2500,
        node_color="skyblue",
        node_shape="o",
        edge_color="gray",
        width=1.5,
        arrowsize=15,
    )
    nx.draw_networkx_labels(G, pos, labels=node_labels, font_size=8, font_weight="bold")

    plt.title("Chemical Protocol Workflow Graph", size=15)
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close()
    print(f"  - Visualization saved to '{output_path}'")


from networkx.drawing.nx_agraph import to_agraph
import re

COMPASS = {"n","e","s","w","ne","nw","se","sw","c"}

def _is_compass(port: str) -> bool:
    return isinstance(port, str) and port.lower() in COMPASS

def draw_protocol_graph_with_ports(protocol_graph, output_path: str, rankdir: str = "LR"):
    """
    使用 Graphviz 端口语法绘制协议工作流图。
    - 若边上的 source_port/target_port 是 compass（n/e/s/w/...），直接用 compass。
    - 否则自动为节点创建 record 形状并定义命名端口 <portname>。
    最终由 PyGraphviz 渲染并输出到 output_path（后缀决定格式，如 .png/.svg/.pdf）。
    """
    if not protocol_graph:
        print("Cannot draw graph: Graph object is empty.")
        return

    # 1) 先用 networkx 搭建有向图，保留端口属性
    G = nx.DiGraph()
    for node_id, attrs in protocol_graph.nodes.items():
        label = attrs.get("description", attrs.get("template", node_id[:8]))
        # 保留一个干净的“中心标签”，用于放在 record 的中间槽
        G.add_node(node_id, _core_label=str(label), **{k:v for k,v in attrs.items() if k not in ("label",)})

    edges_data = []
    in_ports_by_node  = {}  # 收集命名输入端口
    out_ports_by_node = {}  # 收集命名输出端口

    for edge in protocol_graph.edges:
        u = edge["source"]
        v = edge["target"]
        sp = edge.get("source_port")
        tp = edge.get("target_port")

        # 记录到图里（保留原始端口信息）
        G.add_edge(u, v, source_port=sp, target_port=tp)
        edges_data.append((u, v, sp, tp))

        # 如果不是 compass，就按“命名端口”先归类，等会儿给节点造 record
        if sp and not _is_compass(sp):
            out_ports_by_node.setdefault(u, set()).add(str(sp))
        if tp and not _is_compass(tp):
            in_ports_by_node.setdefault(v, set()).add(str(tp))

    # 2) 转为 AGraph，使用 Graphviz 渲染
    A = to_agraph(G)
    A.graph_attr.update(rankdir=rankdir, splines="true", concentrate="false", fontsize="10")
    A.node_attr.update(shape="box", style="rounded,filled", fillcolor="lightyellow", color="#999999", fontname="Helvetica")
    A.edge_attr.update(arrowsize="0.8", color="#666666")

    # 3) 为需要命名端口的节点设置 record 形状与 label
    #    左列 = 输入端口；中间 = 核心标签；右列 = 输出端口
    for n in A.nodes():
        node = A.get_node(n)
        core = G.nodes[n].get("_core_label", n)

        in_ports  = sorted(in_ports_by_node.get(n, []))
        out_ports = sorted(out_ports_by_node.get(n, []))

        # 如果该节点涉及命名端口，则用 record；否则保留原 box
        if in_ports or out_ports:
            def port_fields(ports):
                if not ports:
                    return " "  # 必须留一个空槽占位
                # 每个端口一个小格子，<p> name
                return "|".join(f"<{re.sub(r'[^A-Za-z0-9_:.|-]', '_', p)}> {p}" for p in ports)

            left  = port_fields(in_ports)
            right = port_fields(out_ports)

            # 三栏：左(入) | 中(节点名) | 右(出)
            record_label = f"{{ {left} | {core} | {right} }}"
            node.attr.update(shape="record", label=record_label)
        else:
            # 没有命名端口：普通盒子，显示核心标签
            node.attr.update(label=str(core))

    # 4) 给边设置 headport / tailport
    #    - 若端口为 compass：直接用 compass（e.g., headport="e"）
    #    - 若端口为命名端口：使用在 record 中定义的 <port> 名（同名即可）
    for (u, v, sp, tp) in edges_data:
        e = A.get_edge(u, v)

        # Graphviz 属性：tail 是源，head 是目标
        if sp:
            if _is_compass(sp):
                e.attr["tailport"] = sp.lower()
            else:
                # 与 record label 中 <port> 名一致；特殊字符已在 label 中做了清洗
                e.attr["tailport"] = re.sub(r'[^A-Za-z0-9_:.|-]', '_', str(sp))

        if tp:
            if _is_compass(tp):
                e.attr["headport"] = tp.lower()
            else:
                e.attr["headport"] = re.sub(r'[^A-Za-z0-9_:.|-]', '_', str(tp))

        # 可选：若想让边更贴边缘，可设置 constraint/spline 等
        # e.attr["arrowhead"] = "vee"

    # 5) 输出
    A.draw(output_path, prog="dot")
    print(f"  - Port-aware workflow rendered to '{output_path}'")


def flatten_xdl_procedure(procedure_elem: ET.Element) -> List[ET.Element]:
    """展平嵌套的XDL程序结构"""
    flattened_operations = []
    TEMP_UNSUPPORTED_PROTOCOL = ["Purge", "Wait", "Stir", "ResetHandling"]

    def extract_operations(element: ET.Element):
        if element.tag not in ["Prep", "Reaction", "Workup", "Purification", "Procedure"]:
            if element.tag not in TEMP_UNSUPPORTED_PROTOCOL:
                flattened_operations.append(element)

        for child in element:
            extract_operations(child)

    for child in procedure_elem:
        extract_operations(child)

    return flattened_operations


def parse_xdl_content(xdl_content: str) -> tuple:
    """解析XDL内容"""
    try:
        xdl_content_cleaned = "".join(c for c in xdl_content if c.isprintable())
        root = ET.fromstring(xdl_content_cleaned)

        synthesis_elem = root.find("Synthesis")
        if synthesis_elem is None:
            return None, None, None

        # 解析硬件组件
        hardware_elem = synthesis_elem.find("Hardware")
        hardware = []
        if hardware_elem is not None:
            hardware = [{"id": c.get("id"), "type": c.get("type")} for c in hardware_elem.findall("Component")]

        # 解析试剂
        reagents_elem = synthesis_elem.find("Reagents")
        reagents = []
        if reagents_elem is not None:
            reagents = [{"name": r.get("name"), "role": r.get("role", "")} for r in reagents_elem.findall("Reagent")]

        # 解析程序
        procedure_elem = synthesis_elem.find("Procedure")
        if procedure_elem is None:
            return None, None, None

        flattened_operations = flatten_xdl_procedure(procedure_elem)
        return hardware, reagents, flattened_operations

    except ET.ParseError as e:
        raise ValueError(f"Invalid XDL format: {e}")


def convert_xdl_to_dict(xdl_content: str) -> Dict[str, Any]:
    """
    将XDL XML格式转换为标准的字典格式

    Args:
        xdl_content: XDL XML内容

    Returns:
        转换结果，包含步骤和器材信息
    """
    try:
        hardware, reagents, flattened_operations = parse_xdl_content(xdl_content)
        if hardware is None:
            return {"error": "Failed to parse XDL content", "success": False}

        # 将XDL元素转换为字典格式
        steps_data = []
        for elem in flattened_operations:
            # 转换参数类型
            parameters = {}
            for key, val in elem.attrib.items():
                converted_val = convert_to_type(val)
                if converted_val is not None:
                    parameters[key] = converted_val

            step_dict = {
                "operation": elem.tag,
                "parameters": parameters,
                "description": elem.get("purpose", f"Operation: {elem.tag}"),
            }
            steps_data.append(step_dict)

        # 合并硬件和试剂为统一的labware_info格式
        labware_data = []
        labware_data.extend({"id": hw["id"], "type": "hardware", **hw} for hw in hardware)
        labware_data.extend({"name": reagent["name"], "type": "reagent", **reagent} for reagent in reagents)

        return {
            "success": True,
            "steps": steps_data,
            "labware": labware_data,
            "message": f"Successfully converted XDL to dict format. Found {len(steps_data)} steps and {len(labware_data)} labware items.",
        }

    except Exception as e:
        error_msg = f"XDL conversion failed: {str(e)}"
        logger.error(error_msg)
        return {"error": error_msg, "success": False}


def create_workflow(
    steps_info: str,
    labware_info: str,
    workflow_name: str = "Generated Workflow",
    workstation_name: str = "workstation",
    workflow_description: str = "Auto-generated workflow from protocol",
) -> Dict[str, Any]:
    """
    创建工作流，输入数据已经是统一的字典格式

    Args:
        steps_info: 步骤信息 (JSON字符串，已经是list of dict格式)
        labware_info: 实验器材和试剂信息 (JSON字符串，已经是list of dict格式)
        workflow_name: 工作流名称
        workflow_description: 工作流描述

    Returns:
        创建结果，包含工作流UUID和详细信息
    """
    try:
        # 直接解析JSON数据
        steps_info_clean = extract_json_from_markdown(steps_info)
        labware_info_clean = extract_json_from_markdown(labware_info)

        steps_data = json.loads(steps_info_clean)
        labware_data = json.loads(labware_info_clean)

        # 统一处理所有数据
        protocol_graph = build_protocol_graph(labware_data, steps_data, workstation_name=workstation_name)

        # 检测协议类型（用于标签）
        protocol_type = "bio" if any("biomek" in step.get("template", "") for step in refactored_steps) else "organic"

        # 转换为工作流格式
        data = protocol_graph.to_dict()

        # 转换节点格式
        for i, node in enumerate(data["nodes"]):
            description = node.get("description", "")
            onode = {
                "template": node.pop("template"),
                "id": node["id"],
                "lab_node_type": node.get("lab_node_type", "Device"),
                "name": description or f"Node {i + 1}",
                "params": {"default": node},
                "handles": {},
            }

            # 处理边连接
            for edge in data["links"]:
                if edge["source"] == node["id"]:
                    source_port = edge.get("source_port", "output")
                    if source_port not in onode["handles"]:
                        onode["handles"][source_port] = {"type": "source"}

                if edge["target"] == node["id"]:
                    target_port = edge.get("target_port", "input")
                    if target_port not in onode["handles"]:
                        onode["handles"][target_port] = {"type": "target"}

            data["nodes"][i] = onode

        # 发送到API创建工作流
        api_secret = configs.Lab.Key
        if not api_secret:
            return {"error": "API SecretKey is not configured", "success": False}

        # Step 1: 创建工作流
        workflow_url = f"{configs.Lab.Api}/api/v1/workflow/"
        headers = {
            "Content-Type": "application/json",
        }
        params = {"secret_key": api_secret}

        graph_data = {"name": workflow_name, **data}

        logger.info(f"Creating workflow: {workflow_name}")
        response = requests.post(
            workflow_url, params=params, json=graph_data, headers=headers, timeout=configs.Lab.Timeout
        )
        response.raise_for_status()

        workflow_info = response.json()

        if workflow_info.get("code") != 0:
            error_msg = f"API returned an error: {workflow_info.get('msg', 'Unknown Error')}"
            logger.error(error_msg)
            return {"error": error_msg, "success": False}

        workflow_uuid = workflow_info.get("data", {}).get("uuid")
        if not workflow_uuid:
            return {"error": "Failed to get workflow UUID from response", "success": False}

        # Step 2: 添加到模板库（可选）
        try:
            library_url = f"{configs.Lab.Api}/api/flociety/vs/workflows/library/"
            lib_payload = {
                "workflow_uuid": workflow_uuid,
                "title": workflow_name,
                "description": workflow_description,
                "labels": [protocol_type.title(), "Auto-generated"],
            }

            library_response = requests.post(
                library_url, params=params, json=lib_payload, headers=headers, timeout=configs.Lab.Timeout
            )
            library_response.raise_for_status()

            library_info = library_response.json()
            logger.info(f"Workflow added to library: {library_info}")

            return {
                "success": True,
                "workflow_uuid": workflow_uuid,
                "workflow_info": workflow_info.get("data"),
                "library_info": library_info.get("data"),
                "protocol_type": protocol_type,
                "message": f"Workflow '{workflow_name}' created successfully",
            }

        except Exception as e:
            # 即使添加到库失败，工作流创建仍然成功
            logger.warning(f"Failed to add workflow to library: {str(e)}")
            return {
                "success": True,
                "workflow_uuid": workflow_uuid,
                "workflow_info": workflow_info.get("data"),
                "protocol_type": protocol_type,
                "message": f"Workflow '{workflow_name}' created successfully (library addition failed)",
            }

    except requests.exceptions.RequestException as e:
        error_msg = f"Network error when calling API: {str(e)}"
        logger.error(error_msg)
        return {"error": error_msg, "success": False}
    except json.JSONDecodeError as e:
        error_msg = f"JSON parsing error: {str(e)}"
        logger.error(error_msg)
        return {"error": error_msg, "success": False}
    except Exception as e:
        error_msg = f"An unexpected error occurred: {str(e)}"
        logger.error(error_msg)
        logger.error(traceback.format_exc())
        return {"error": error_msg, "success": False}
