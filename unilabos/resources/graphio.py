import importlib
import inspect
import json
import os.path
import traceback
from typing import Union, Any, Dict, List, Tuple
import uuid
import networkx as nx
from pylabrobot.resources import ResourceHolder
from unilabos_msgs.msg import Resource

from unilabos.config.config import BasicConfig
from unilabos.resources.container import RegularContainer
from unilabos.resources.itemized_carrier import ItemizedCarrier
from unilabos.ros.msgs.message_converter import convert_to_ros_msg
from unilabos.ros.nodes.resource_tracker import (
    ResourceDictInstance,
    ResourceTreeSet,
)
from unilabos.utils import logger
from unilabos.utils.banner_print import print_status

try:
    from pylabrobot.resources.resource import Resource as ResourcePLR
except ImportError:
    pass
from typing import get_origin

physical_setup_graph: nx.Graph = None


def canonicalize_nodes_data(
    nodes: List[Dict[str, Any]], parent_relation: Dict[str, List[str]] = {}
) -> ResourceTreeSet:
    """
    标准化节点数据，使用 ResourceInstanceDictFlatten 进行规范化并创建 ResourceTreeSet

    Args:
        nodes: 原始节点列表
        parent_relation: 父子关系映射 {parent_id: [child_id1, child_id2, ...]}

    Returns:
        ResourceTreeSet: 标准化后的资源树集合
    """
    print_status(f"{len(nodes)} Resources loaded:", "info")

    # 第一步：基本预处理（处理graphml的label字段）
    for node in nodes:
        if node.get("label") is not None:
            node_id = node.pop("label")
            node["id"] = node["name"] = node_id
        if not isinstance(node.get("config"), dict):
            node["config"] = {}
        if not node.get("type"):
            node["type"] = "device"
            print_status(f"Warning: Node {node.get('id', 'unknown')} missing 'type', defaulting to 'device'", "warning")
        if node.get("name", None) is None:
            node["name"] = node.get("id")
            print_status(f"Warning: Node {node.get('id', 'unknown')} missing 'name', defaulting to {node['name']}", "warning")
        if not isinstance(node.get("position"), dict):
            node["position"] = {"position": {}}
            x = node.pop("x", None)
            if x is not None:
                node["position"]["position"]["x"] = x
            y = node.pop("y", None)
            if y is not None:
                node["position"]["position"]["y"] = y
            z = node.pop("z", None)
            if z is not None:
                node["position"]["position"]["z"] = z
        if "sample_id" in node:
            sample_id = node.pop("sample_id")
            if sample_id:
                logger.error(f"{node}的sample_id参数已弃用，sample_id: {sample_id}")
        for k in list(node.keys()):
            if k not in ["id", "uuid", "name", "description", "schema", "model", "icon", "parent_uuid", "parent", "type", "class", "position", "config", "data", "children"]:
                v = node.pop(k)
                node["config"][k] = v

    # 第二步：处理parent_relation
    id2idx = {node["id"]: idx for idx, node in enumerate(nodes)}
    for parent, children in parent_relation.items():
        if parent in id2idx:
            nodes[id2idx[parent]]["children"] = children
            for child in children:
                if child in id2idx:
                    nodes[id2idx[child]]["parent"] = parent

    # 第三步：使用 ResourceInstanceDictFlatten 标准化每个节点
    standardized_instances = []
    known_nodes: Dict[str, ResourceDictInstance] = {}  # {node_id: ResourceDictInstance}
    uuid_to_instance: Dict[str, ResourceDictInstance] = {}  # {uuid: ResourceDictInstance}

    for node in nodes:
        try:
            print_status(f"DeviceId: {node['id']}, Class: {node['class']}", "info")
            # 使用标准化方法
            resource_instance = ResourceDictInstance.get_resource_instance_from_dict(node)
            known_nodes[node["id"]] = resource_instance
            uuid_to_instance[resource_instance.res_content.uuid] = resource_instance
            standardized_instances.append(resource_instance)
        except Exception as e:
            print_status(f"Failed to standardize node {node.get('id', 'unknown')}:\n{traceback.format_exc()}", "error")
            continue

    # 第四步：建立 parent 和 children 关系
    for node in nodes:
        node_id = node["id"]
        if node_id not in known_nodes:
            continue

        current_instance = known_nodes[node_id]

        # 优先使用 parent_uuid 进行匹配，如果不存在则使用 parent
        parent_uuid = node.get("parent_uuid")
        parent_id = node.get("parent")
        parent_instance = None

        # 优先用 parent_uuid 匹配
        if parent_uuid and parent_uuid in uuid_to_instance:
            parent_instance = uuid_to_instance[parent_uuid]
        # 否则用 parent_id 匹配
        elif parent_id and parent_id in known_nodes:
            parent_instance = known_nodes[parent_id]

        # 设置 parent 引用
        if parent_instance:
            current_instance.res_content.parent = parent_instance.res_content
            # 将当前节点添加到父节点的 children 列表
            parent_instance.children.append(current_instance)

    # 第五步：创建 ResourceTreeSet
    resource_tree_set = ResourceTreeSet.from_nested_list(standardized_instances)
    return resource_tree_set


def canonicalize_links_ports(links: List[Dict[str, Any]], resource_tree_set: ResourceTreeSet) -> List[Dict[str, Any]]:
    """
    标准化边/连接的端口信息

    Args:
        links: 原始连接列表
        resource_tree_set: 资源树集合，用于获取节点的UUID信息

    Returns:
        标准化后的连接列表
    """
    # 构建 id 到 uuid 的映射
    id_to_uuid: Dict[str, str] = {}
    for node in resource_tree_set.all_nodes:
        id_to_uuid[node.res_content.id] = node.res_content.uuid

    # 第一遍处理：将字符串类型的port转换为字典格式
    for link in links:
        port = link.get("port")
        if link.get("type", "physical") == "physical":
            link["type"] = "fluid"
        if isinstance(port, int):
            port = str(port)
        if isinstance(port, str):
            port_str = port.strip()
            if port_str.startswith("(") and port_str.endswith(")"):
                # 处理格式为 "(A,B)" 的情况
                content = port_str[1:-1].strip()
                parts = [p.strip() for p in content.split(",", 1)]
                source_port = parts[0]
                dest_port = parts[1] if len(parts) > 1 else None
            else:
                # 处理格式为 "A" 的情况
                source_port = port_str
                dest_port = None
            link["port"] = {link["source"]: source_port, link["target"]: dest_port}
        elif not isinstance(port, dict):
            # 若port既非字符串也非字典，初始化为空结构
            link["port"] = {link["source"]: None, link["target"]: None}

    # 构建边字典，键为(source节点, target节点)，值为对应的port信息
    edges = {(link["source"], link["target"]): link["port"] for link in links}

    # 第二遍处理：填充反向边的dest信息
    delete_reverses = []
    for i, link in enumerate(links):
        s, t = link["source"], link["target"]
        current_port = link["port"]
        if current_port.get(t) is None:
            reverse_key = (t, s)
            reverse_port = edges.get(reverse_key)
            if reverse_port:
                reverse_source = reverse_port.get(s)
                if reverse_source is not None:
                    # 设置当前边的dest为反向边的source
                    current_port[t] = reverse_source
                    delete_reverses.append(i)
            else:
                # 若不存在反向边，初始化为空结构
                current_port[t] = current_port[s]
    # 删除已被使用反向端口信息的反向边
    standardized_links = [link for i, link in enumerate(links) if i not in delete_reverses]

    # 第三遍处理：为每个 link 添加 source_uuid 和 target_uuid
    for link in standardized_links:
        source_id = link.get("source")
        target_id = link.get("target")

        # 添加 source_uuid
        if source_id and source_id in id_to_uuid:
            link["source_uuid"] = id_to_uuid[source_id]

        # 添加 target_uuid
        if target_id and target_id in id_to_uuid:
            link["target_uuid"] = id_to_uuid[target_id]

    return standardized_links


def handle_communications(G: nx.Graph):
    available_communication_types = ["serial", "io_device", "plc", "io"]
    for e, edata in G.edges.items():
        if edata.get("type", "physical") != "communication":
            continue
        if G.nodes[e[0]].get("class") in available_communication_types:
            device_comm, device = e[0], e[1]
        elif G.nodes[e[1]].get("class") in available_communication_types:
            device_comm, device = e[1], e[0]
        else:
            continue

        if G.nodes[device_comm].get("class") == "serial":
            G.nodes[device]["config"]["port"] = device_comm
        elif G.nodes[device_comm].get("class") == "io_device":
            print(f'!!! Modify {device}\'s io_device_port to {edata["port"][device_comm]}')
            G.nodes[device]["config"]["io_device_port"] = int(edata["port"][device_comm])


def read_node_link_json(
    json_info: Union[str, Dict[str, Any]],
) -> tuple[nx.Graph, ResourceTreeSet, List[Dict[str, Any]]]:
    """
    读取节点-边的JSON数据并构建图

    Args:
        json_info: JSON文件路径或字典数据

    Returns:
        tuple[nx.Graph, ResourceTreeSet, List[Dict[str, Any]]]:
            返回NetworkX图对象、资源树集合和标准化后的连接列表
    """
    global physical_setup_graph
    if isinstance(json_info, str):
        data = json.load(open(json_info, encoding="utf-8"))
    else:
        data = json_info

    # 标准化节点数据并创建 ResourceTreeSet
    nodes = data.get("nodes", [])
    resource_tree_set = canonicalize_nodes_data(nodes)

    # 标准化边数据
    links = data.get("links", [])
    standardized_links = canonicalize_links_ports(links, resource_tree_set)

    # 构建 NetworkX 图（需要转换回 dict 格式）
    # 从 ResourceTreeSet 获取所有节点
    graph_data = {
        "nodes": [node.res_content.model_dump(by_alias=True) for node in resource_tree_set.all_nodes],
        "links": standardized_links,
    }
    physical_setup_graph = nx.node_link_graph(graph_data, edges="links", multigraph=False)
    handle_communications(physical_setup_graph)

    return physical_setup_graph, resource_tree_set, standardized_links


def modify_to_backend_format(data: list[dict[str, Any]]) -> list[dict[str, Any]]:
    for edge in data:
        port = edge.pop("port", {})
        source = edge["source"]
        target = edge["target"]
        if source in port:
            edge["sourceHandle"] = port[source]
        elif "source_port" in edge:
            edge["sourceHandle"] = edge.pop("source_port")
        if target in port:
            edge["targetHandle"] = port[target]
        elif "target_port" in edge:
            edge["targetHandle"] = edge.pop("target_port")
        edge["id"] = f"reactflow__edge-{source}-{edge['sourceHandle']}-{target}-{edge['targetHandle']}"
        for key in ["source_port", "target_port"]:
            if key in edge:
                edge.pop(key)
    return data


def read_graphml(graphml_file: str) -> tuple[nx.Graph, ResourceTreeSet, List[Dict[str, Any]]]:
    """
    读取GraphML文件并构建图

    Args:
        graphml_file: GraphML文件路径

    Returns:
        tuple[nx.Graph, ResourceTreeSet, List[Dict[str, Any]]]:
            返回NetworkX图对象、资源树集合和标准化后的连接列表
    """
    global physical_setup_graph

    G = nx.read_graphml(graphml_file)
    mapping = {}
    parent_relation = {}
    for node in G.nodes():
        label = G.nodes[node].pop("label", G.nodes[node].get("id", G.nodes[node].get("name", "NaN")))
        mapping[node] = label
        if "::" in node:
            parent = mapping[node.split("::")[0]]
            if parent not in parent_relation:
                parent_relation[parent] = []
            parent_relation[parent].append(label)

    G2 = nx.relabel_nodes(G, mapping)
    data = nx.node_link_data(G2)

    # 标准化节点数据并创建 ResourceTreeSet
    nodes = data.get("nodes", [])
    resource_tree_set = canonicalize_nodes_data(nodes, parent_relation=parent_relation)

    # 标准化边数据
    links = data.get("links", [])
    standardized_links = canonicalize_links_ports(links, resource_tree_set)

    # 构建 NetworkX 图（需要转换回 dict 格式）
    # 从 ResourceTreeSet 获取所有节点
    graph_data = {
        "nodes": [node.res_content.model_dump(by_alias=True) for node in resource_tree_set.all_nodes],
        "links": standardized_links,
    }
    dump_json_path = os.path.join(BasicConfig.working_dir, os.path.basename(graphml_file).rsplit(".")[0] + ".json")
    with open(dump_json_path, "w", encoding="utf-8") as f:
        f.write(json.dumps(graph_data, indent=4, ensure_ascii=False))
        print_status(f"GraphML converted to JSON and saved to {dump_json_path}", "info")
    physical_setup_graph = nx.node_link_graph(graph_data, link="links", multigraph=False)
    handle_communications(physical_setup_graph)

    return physical_setup_graph, resource_tree_set, standardized_links


def dict_from_graph(graph: nx.Graph) -> dict:
    nodes_copy = {node_id: {"id": node_id, **node} for node_id, node in graph.nodes(data=True)}
    return nodes_copy


def dict_to_tree(nodes: dict, devices_only: bool = False) -> list[dict]:
    # 将节点转换为字典，以便通过 ID 快速查找
    nodes_list = [node for node in nodes.values() if node.get("type") == "device" or not devices_only]
    id_list = [node["id"] for node in nodes_list]
    is_root = {node["id"]: True for node in nodes_list}

    # 初始化每个节点的 children 为包含节点字典的列表
    for node in nodes_list:
        node["children"] = [nodes[child_id] for child_id in node.get("children", [])]
        for child_id in node.get("children", []):
            if child_id in is_root:
                is_root[child_id] = False

    # 找到根节点并返回
    root_nodes = [node for node in nodes_list if is_root.get(node["id"], False) or len(nodes_list) == 1]

    # 如果存在多个根节点，返回所有根节点
    return root_nodes


def dict_to_nested_dict(nodes: dict, devices_only: bool = False) -> dict:
    # 将节点转换为字典，以便通过 ID 快速查找
    nodes_list = [node for node in nodes.values() if node.get("type") == "device" or not devices_only]
    is_root = {node["id"]: True for node in nodes_list}

    # 初始化每个节点的 children 为包含节点字典的列表
    for node in nodes_list:
        node["children"] = {
            child_id: nodes[child_id]
            for child_id in node.get("children", [])
            if nodes[child_id].get("type") == "device" or not devices_only
        }
        for child_id in node.get("children", []):
            if child_id in is_root:
                is_root[child_id] = False
        if len(node["children"]) > 0 and node["type"].lower() == "device":
            node["config"]["children"] = node["children"]

    # 找到根节点并返回
    root_nodes = {node["id"]: node for node in nodes_list if is_root.get(node["id"], False) or len(nodes_list) == 1}

    # 如果存在多个根节点，返回所有根节点
    return root_nodes


def list_to_nested_dict(nodes: list[dict]) -> dict:
    nodes_dict = {node["id"]: node for node in nodes}
    return dict_to_nested_dict(nodes_dict)


def tree_to_list(tree: list[dict]) -> list[dict]:
    def _tree_to_list(tree: list[dict], result: list[dict]):
        for node_ in tree:
            node = node_.copy()
            result.append(node)
            if node.get("children"):
                _tree_to_list(node["children"], result)
            node["children"] = [n["id"] for n in node["children"]]

    result = []
    _tree_to_list(tree, result)
    return result


def nested_dict_to_list(nested_dict: dict) -> list[dict]:  # FIXME 是tree？
    """
    将嵌套字典转换为扁平列表

    嵌套字典的层次结构将通过children属性表示

    Args:
        nested_dict: 嵌套的字典结构

    Returns:
        扁平化的字典列表
    """
    result = []

    # 如果输入本身是一个节点，先添加它
    if "id" in nested_dict:
        node = nested_dict.copy()
        # 暂存子节点
        children_dict = node.get("children", {})
        # 如果children是字典，将其转换为键列表
        if isinstance(children_dict, dict):
            node["children"] = list(children_dict.keys())
        elif not isinstance(children_dict, list):
            node["children"] = []
        result.append(node)

        # 处理子节点字典
        if isinstance(children_dict, dict):
            for child_id, child_data in children_dict.items():
                if isinstance(child_data, dict):
                    # 为子节点添加ID（如果不存在）
                    if "id" not in child_data:
                        child_data["id"] = child_id
                    # 递归处理子节点
                    result.extend(nested_dict_to_list(child_data))

    # 处理children字段
    elif "children" in nested_dict:
        children_dict = nested_dict.get("children", {})
        if isinstance(children_dict, dict):
            for child_id, child_data in children_dict.items():
                if isinstance(child_data, dict):
                    # 为子节点添加ID（如果不存在）
                    if "id" not in child_data:
                        child_data["id"] = child_id
                    # 递归处理子节点
                    result.extend(nested_dict_to_list(child_data))

    return result


def convert_resources_to_type(
    resources_list: list[dict], resource_type: Union[type, list[type]], *, plr_model: bool = False
) -> Union[list[dict], dict, None, "ResourcePLR"]:
    """
    Convert resources to a given type (PyLabRobot or NestedDict) from flattened list of dictionaries.

    Args:
        resources: List of resources in the flattened dictionary format.
        resource_type: Type of the resources to convert to.
        plr_model: 是否有plr_model类型

    Returns:
        List of resources in the given type.
    """
    if resource_type == dict or resource_type == str:
        return list_to_nested_dict(resources_list)
    elif isinstance(resource_type, type) and issubclass(resource_type, ResourcePLR):
        if isinstance(resources_list, dict):
            return resource_ulab_to_plr(resources_list, plr_model)
        resources_tree = dict_to_tree({r["id"]: r for r in resources_list})
        return resource_ulab_to_plr(resources_tree[0], plr_model)
    elif isinstance(resource_type, list):
        if all((get_origin(t) is Union) for t in resource_type):
            resources_tree = dict_to_tree({r["id"]: r for r in resources_list})
            return [resource_ulab_to_plr(r, plr_model) for r in resources_tree]
        elif all(issubclass(t, ResourcePLR) for t in resource_type):
            resources_tree = dict_to_tree({r["id"]: r for r in resources_list})
            return [resource_ulab_to_plr(r, plr_model) for r in resources_tree]
    else:
        return None


def convert_resources_from_type(
    resources_list, resource_type: Union[type, list[type]], *, is_plr: bool = False
) -> Union[list[dict], dict, None, "ResourcePLR"]:
    """
    Convert resources from a given type (PyLabRobot or NestedDict) to flattened list of dictionaries.

    Args:
        resources_list: List of resources in the given type.
        resource_type: Type of the resources to convert from.

    Returns:
        List of resources in the flattened dictionary format.
    """
    if resource_type == dict:
        return nested_dict_to_list(resources_list)
    elif isinstance(resource_type, type) and issubclass(resource_type, ResourcePLR):
        resources_tree = [resource_plr_to_ulab(resources_list)]
        return tree_to_list(resources_tree)
    elif isinstance(resource_type, list):
        if all((get_origin(t) is Union) for t in resource_type):
            resources_tree = [resource_plr_to_ulab(r) for r in resources_list]
            return tree_to_list(resources_tree)
        elif is_plr or all(issubclass(t, ResourcePLR) for t in resource_type):
            resources_tree = [resource_plr_to_ulab(r) for r in resources_list]
            return tree_to_list(resources_tree)
    else:
        return None


def resource_ulab_to_plr(resource: dict, plr_model=False) -> "ResourcePLR":
    """
    Resource有model字段，但是Deck下没有，这个plr由外面判断传入
    """
    if ResourcePLR is None:
        raise ImportError("pylabrobot not found")

    all_states = {resource["id"]: resource["data"]}

    def resource_ulab_to_plr_inner(resource: dict):
        all_states[resource["name"]] = resource["data"]
        d = {
            "name": resource["name"],
            "type": resource["type"],
            "size_x": resource["config"].get("size_x", 0),
            "size_y": resource["config"].get("size_y", 0),
            "size_z": resource["config"].get("size_z", 0),
            "location": {**resource["position"], "type": "Coordinate"},
            "rotation": {"x": 0, "y": 0, "z": 0, "type": "Rotation"},  # Resource如果没有rotation，是plr版本太低
            "category": resource["type"],
            "model": resource["config"].get("model", None),  # resource中deck没有model
            "children": (
                [resource_ulab_to_plr_inner(child) for child in resource["children"]]
                if isinstance(resource["children"], list)
                else [resource_ulab_to_plr_inner(child) for child_id, child in resource["children"].items()]
            ),
            "parent_name": resource["parent"] if resource["parent"] is not None else None,
            **resource["config"],
        }
        if not plr_model:
            d.pop("model")
        return d

    d = resource_ulab_to_plr_inner(resource)
    """无法通过Resource进行反序列化，例如TipSpot必须内部序列化好，直接用TipSpot序列化会多参数，导致出错"""
    from pylabrobot.utils.object_parsing import find_subclass

    sub_cls = find_subclass(d["type"], ResourcePLR)
    spect = inspect.signature(sub_cls)
    if "category" not in spect.parameters:
        d.pop("category")
    resource_plr = sub_cls.deserialize(d, allow_marshal=True)
    resource_plr.load_all_state(all_states)
    return resource_plr


def resource_plr_to_ulab(resource_plr: "ResourcePLR", parent_name: str = None, with_children=True):
    def replace_plr_type_to_ulab(source: str):
        replace_info = {
            "plate": "plate",
            "well": "well",
            "tip_spot": "container",
            "trash": "container",
            "deck": "deck",
            "tip_rack": "container",
        }
        if source in replace_info:
            return replace_info[source]
        else:
            print("转换pylabrobot的时候，出现未知类型", source)
            return "container"

    def resource_plr_to_ulab_inner(d: dict, all_states: dict, child=True) -> dict:
        r = {
            "id": d["name"],
            "name": d["name"],
            "sample_id": None,
            "children": [resource_plr_to_ulab_inner(child, all_states) for child in d["children"]] if child else [],
            "parent": d["parent_name"] if d["parent_name"] else parent_name if parent_name else None,
            "type": replace_plr_type_to_ulab(d.get("category")),  # FIXME plr自带的type是python class name
            "class": d.get("class", ""),
            "position": (
                {"x": d["location"]["x"], "y": d["location"]["y"], "z": d["location"]["z"]}
                if d["location"]
                else {"x": 0, "y": 0, "z": 0}
            ),
            "config": {k: v for k, v in d.items() if k not in ["name", "children", "parent_name", "location"]},
            "data": all_states[d["name"]],
        }
        return r

    d = resource_plr.serialize()
    all_states = resource_plr.serialize_all_state()
    r = resource_plr_to_ulab_inner(d, all_states, with_children)

    return r


def resource_bioyond_to_plr(bioyond_materials: list[dict], type_mapping: Dict[str, Tuple[str, str]] = {}, deck: Any = None) -> list[dict]:
    """
    将 bioyond 物料格式转换为 ulab 物料格式

    Args:
        bioyond_materials: bioyond 系统的物料查询结果列表
        type_mapping: 物料类型映射字典，格式 {bioyond_type: [plr_class_name, class_uuid]}
        location_id_mapping: 库位 ID 到名称的映射字典，格式 {location_id: location_name}

    Returns:
        pylabrobot 格式的物料列表
    """
    plr_materials = []

    for material in bioyond_materials:
        className = (
            type_mapping.get(material.get("typeName"), ("RegularContainer", ""))[0] if type_mapping else "RegularContainer"
        )

        plr_material: ResourcePLR = initialize_resource(
            {"name": material["name"], "class": className}, resource_type=ResourcePLR
        )
        plr_material.code = material.get("code", "") and material.get("barCode", "") or ""
        plr_material.unilabos_uuid = str(uuid.uuid4())

        # 处理子物料（detail）
        if material.get("detail") and len(material["detail"]) > 0:
            for bottle in reversed(plr_material.children):
                plr_material.unassign_child_resource(bottle)
            child_ids = []
            for detail in material["detail"]:
                number = (
                    (detail.get("z", 0) - 1) * plr_material.num_items_x * plr_material.num_items_y
                    + (detail.get("y", 0) - 1) * plr_material.num_items_y
                    + (detail.get("x", 0) - 1)
                )
                typeName = detail.get("typeName", detail.get("name", ""))
                if typeName in type_mapping:
                    bottle = plr_material[number] = initialize_resource(
                        {"name": f'{detail["name"]}_{number}', "class": type_mapping[typeName][0]}, resource_type=ResourcePLR
                    )
                    bottle.tracker.liquids = [
                        (detail["name"], float(detail.get("quantity", 0)) if detail.get("quantity") else 0)
                    ]
                    bottle.code = detail.get("code", "")
        else:
            bottle = plr_material[0] if plr_material.capacity > 0 else plr_material
            bottle.tracker.liquids = [
                (material["name"], float(material.get("quantity", 0)) if material.get("quantity") else 0)
            ]

        plr_materials.append(plr_material)

        if deck and hasattr(deck, "warehouses"):
            for loc in material.get("locations", []):
                if hasattr(deck, "warehouses") and loc.get("whName") in deck.warehouses:
                    warehouse = deck.warehouses[loc["whName"]]
                    idx = (
                        (loc.get("y", 0) - 1) * warehouse.num_items_x * warehouse.num_items_y
                        + (loc.get("x", 0) - 1) * warehouse.num_items_x
                        + (loc.get("z", 0) - 1)
                    )
                    if 0 <= idx < warehouse.capacity:
                        if warehouse[idx] is None or isinstance(warehouse[idx], ResourceHolder):
                            warehouse[idx] = plr_material

    return plr_materials


def resource_plr_to_bioyond(plr_resources: list[ResourcePLR], type_mapping: dict = {}, warehouse_mapping: dict = {}) -> list[dict]:
    bioyond_materials = []
    for resource in plr_resources:
        if hasattr(resource, "capacity") and resource.capacity > 1:
            material = {
                "typeId": type_mapping.get(resource.model)[1],
                "name": resource.name,
                "unit": "个",
                "quantity": 1,
                "details": [],
                "Parameters": "{}"
            }
            for bottle in resource.children:
                if isinstance(resource, ItemizedCarrier):
                    site = resource.get_child_identifier(bottle)
                else:
                    site = {"x": bottle.location.x - 1, "y": bottle.location.y - 1}
                detail_item = {
                    "typeId": type_mapping.get(bottle.model)[1],
                    "name": bottle.name,
                    "code": bottle.code if hasattr(bottle, "code") else "",
                    "quantity": sum(qty for _, qty in bottle.tracker.liquids) if hasattr(bottle, "tracker") else 0,
                    "x": site["x"] + 1,
                    "y": site["y"] + 1,
                    "molecular": 1,
                    "Parameters": json.dumps({"molecular": 1})
                }
                material["details"].append(detail_item)
        else:
            bottle = resource[0] if resource.capacity > 0 else resource
            material = {
                "typeId": "3a14196b-24f2-ca49-9081-0cab8021bf1a",
                "name": resource.get("name", ""),
                "unit": "",
                "quantity": sum(qty for _, qty in bottle.tracker.liquids) if hasattr(bottle, "tracker") else 0,
                "Parameters": "{}"
            }

        if resource.parent is not None and isinstance(resource.parent, ItemizedCarrier):
            site_in_parent = resource.parent.get_child_identifier(resource)
            material["locations"] = [
                {
                    "id": warehouse_mapping[resource.parent.name]["site_uuids"][site_in_parent["identifier"]],
                    "whid": warehouse_mapping[resource.parent.name]["uuid"],
                    "whName": resource.parent.name,
                    "x": site_in_parent["z"] + 1,
                    "y": site_in_parent["y"] + 1,
                    "z": 1,
                    "quantity": 0
                }
            ],

        print(f"material_data: {material}")
        bioyond_materials.append(material)
    return bioyond_materials


def initialize_resource(resource_config: dict, resource_type: Any = None) -> Union[list[dict], ResourcePLR]:
    """Initializes a resource based on its configuration.

    If the config is detailed, then do nothing;
    If it is a string, then import the appropriate class and create an instance of it.

    Args:
        resource_config (dict): The configuration dictionary for the resource, which includes the class type and other parameters.

    Returns:
        None
    """
    from unilabos.registry.registry import lab_registry

    resource_class_config = resource_config.get("class", None)
    if resource_class_config is None:
        return [resource_config]
    elif type(resource_class_config) == str:
        # Allow special resource class names to be used
        if resource_class_config not in lab_registry.resource_type_registry:
            return [resource_config]
        # If the resource class is a string, look up the class in the
        # resource_type_registry and import it
        resource_class_config = resource_config["class"] = lab_registry.resource_type_registry[resource_class_config][
            "class"
        ]
    if type(resource_class_config) == dict:
        module = importlib.import_module(resource_class_config["module"].split(":")[0])
        mclass = resource_class_config["module"].split(":")[1]
        RESOURCE = getattr(module, mclass)

        if resource_class_config["type"] == "pylabrobot":
            resource_plr = RESOURCE(name=resource_config["name"])
            if resource_type != ResourcePLR:
                r = resource_plr_to_ulab(resource_plr=resource_plr, parent_name=resource_config.get("parent", None))
                # r = resource_plr_to_ulab(resource_plr=resource_plr)
                if resource_config.get("position") is not None:
                    r["position"] = resource_config["position"]
                r = tree_to_list([r])
            else:
                r = resource_plr
        elif resource_class_config["type"] == "unilabos":
            raise ValueError(f"No more support for unilabos Resource class {resource_class_config}")
            res_instance: RegularContainer = RESOURCE(id=resource_config["name"])
            res_instance.ulr_resource = convert_to_ros_msg(
                Resource, {k: v for k, v in resource_config.items() if k != "class"}
            )
            r = [res_instance.get_ulr_resource_as_dict()]
        elif isinstance(RESOURCE, dict):
            r = [RESOURCE.copy()]

    return r


def initialize_resources(resources_config) -> list[dict]:
    """Initializes a list of resources based on their configuration.

    If the config is detailed, then do nothing;
    If it is a string, then import the appropriate class and create an instance of it.

    Args:
        resources_config (list[dict]): The configuration dictionary for the resources, which includes the class type and other parameters.

    Returns:
        None
    """

    resources = []
    for resource_config in resources_config:
        resources.extend(initialize_resource(resource_config))

    return resources
