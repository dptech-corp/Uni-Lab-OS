import importlib
import inspect
import json
from typing import Union
import numpy as np
import networkx as nx

try:
    from pylabrobot.resources.resource import Resource as ResourcePLR
except ImportError:
    pass


physical_setup_graph: nx.Graph = None


def canonicalize_nodes_data(data: dict, parent_relation: dict = {}) -> dict:
    for node in data.get("nodes", []):
        if node.get("label") is not None:
            id = node.pop("label")
            node["id"] = node["name"] = id
        if "id" not in node:
            node["id"] = node.get("name", "NaN")
        if "name" not in node:
            node["name"] = node["id"]
        if node.get("position") is None:
            node["position"] = {
                "x": node.pop("x", 0.0),
                "y": node.pop("y", 0.0),
                "z": node.pop("z", 0.0),
            }
        if node.get("config") is None:
            node["config"] = {}
            node["data"] = {}
            for k in list(node.keys()):
                if k not in [
                    "id",
                    "name",
                    "class",
                    "type",
                    "position",
                    "children",
                    "parent",
                    "config",
                    "data",
                ]:
                    if k in ["chemical", "current_volume"]:
                        if node["data"].get("liquids") is None:
                            node["data"]["liquids"] = [{}]
                    if k == "chemical":
                        node["data"]["liquids"][0]["liquid_name"] = node.pop(k)
                    elif k == "current_volume":
                        node["data"]["liquids"][0]["liquid_volume"] = node.pop(k)
                    elif k == "max_volume":
                        node["data"]["max_volume"] = node.pop(k)
                    elif k == "url":
                        node.pop(k)
                    else:
                        node["config"][k] = node.pop(k)
        if "class" not in node:
            node["class"] = None
        if "type" not in node:
            node["type"] = (
                "container"
                if node["class"] is None
                else "device" if node["class"] not in ["container", "plate"] else node["class"]
            )
        if "children" not in node:
            node["children"] = []

    id2idx = {node_data["id"]: idx for idx, node_data in enumerate(data["nodes"])}
    for parent, children in parent_relation.items():
        data["nodes"][id2idx[parent]]["children"] = children
        for child in children:
            data["nodes"][id2idx[child]]["parent"] = parent
    return data


def canonicalize_links_ports(data: dict) -> dict:
    # 第一遍处理：将字符串类型的port转换为字典格式
    for link in data.get("links", []):
        port = link.get("port")
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
    edges = {(link["source"], link["target"]): link["port"] for link in data.get("links", [])}

    # 第二遍处理：填充反向边的dest信息
    delete_reverses = []
    for i, link in enumerate(data.get("links", [])):
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
    data["links"] = [link for i, link in enumerate(data.get("links", [])) if i not in delete_reverses]

    return data


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


def read_node_link_json(json_file):
    global physical_setup_graph

    data = json.load(open(json_file, encoding="utf-8"))
    data = canonicalize_nodes_data(data)
    data = canonicalize_links_ports(data)

    physical_setup_graph = nx.node_link_graph(data, multigraph=False)  # edges="links" 3.6 warning
    handle_communications(physical_setup_graph)
    return physical_setup_graph


def read_graphml(graphml_file):
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
    data = canonicalize_nodes_data(data, parent_relation=parent_relation)
    data = canonicalize_links_ports(data)

    physical_setup_graph = nx.node_link_graph(data, edges="links", multigraph=False)  # edges="links" 3.6 warning
    handle_communications(physical_setup_graph)
    return physical_setup_graph


def dict_from_graph(graph: nx.Graph) -> dict:
    nodes_copy = {node_id: {"id": node_id, **node} for node_id, node in graph.nodes(data=True)}
    return nodes_copy


def dict_to_tree(nodes: dict, devices_only: bool = False) -> list[dict]:
    # 将节点转换为字典，以便通过 ID 快速查找
    nodes_list = [node for node in nodes.values() if node.get("type") == "device" or not devices_only]

    # 初始化每个节点的 children 为包含节点字典的列表
    for node in nodes_list:
        node["children"] = [nodes[child_id] for child_id in node.get("children", [])]

    # 找到根节点并返回
    root_nodes = [
        node for node in nodes_list if len(nodes_list) == 1 or node.get("parent", node.get("parent_name")) in [None, "", "None", np.nan]
    ]

    # 如果存在多个根节点，返回所有根节点
    return root_nodes


def dict_to_nested_dict(nodes: dict, devices_only: bool = False) -> dict:
    # 将节点转换为字典，以便通过 ID 快速查找
    nodes_list = [node for node in nodes.values() if node.get("type") == "device" or not devices_only]

    # 初始化每个节点的 children 为包含节点字典的列表
    for node in nodes_list:
        node["children"] = {
            child_id: nodes[child_id]
            for child_id in node.get("children", [])
            if nodes[child_id].get("type") == "device" or not devices_only
        }
        if len(node["children"]) > 0 and node["type"].lower() == "device" and devices_only:
            node["config"]["children"] = node["children"]

    # 找到根节点并返回
    root_nodes = {
        node["id"]: node
        for node in nodes_list
        if node.get("parent", node.get("parent_name")) in [None, "", "None", np.nan]
    }

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
    resources_list: list[dict], resource_type: type, *, plr_model: bool = False
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
    if resource_type == dict:
        return list_to_nested_dict(resources_list)
    elif isinstance(resource_type, type) and issubclass(resource_type, ResourcePLR):
        if isinstance(resources_list, dict):
            return resource_ulab_to_plr(resources_list, plr_model)
        resources_tree = dict_to_tree({r["id"]: r for r in resources_list})
        return resource_ulab_to_plr(resources_tree[0], plr_model)
    elif isinstance(resource_type, list) and all(issubclass(t, ResourcePLR) for t in resource_type):
        resources_tree = dict_to_tree({r["id"]: r for r in resources_list})
        return [resource_ulab_to_plr(r, plr_model) for r in resources_tree]
    else:
        return None


def convert_resources_from_type(resources_list, resource_type: type) -> Union[list[dict], dict, None, "ResourcePLR"]:
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
    elif isinstance(resource_type, list) and all(issubclass(t, ResourcePLR) for t in resource_type):
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


def resource_plr_to_ulab(resource_plr: "ResourcePLR", parent_name: str = None):
    def resource_plr_to_ulab_inner(d: dict, all_states: dict) -> dict:
        r = {
            "id": d["name"],
            "name": d["name"],
            "sample_id": None,
            "children": [resource_plr_to_ulab_inner(child, all_states) for child in d["children"]],
            "parent": d["parent_name"] if d["parent_name"] else parent_name if parent_name else None,
            "type": "device",  # FIXME plr自带的type是python class name
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
    r = resource_plr_to_ulab_inner(d, all_states)

    return r


def initialize_resource(resource_config: dict, lab_registry: dict) -> list[dict]:
    """Initializes a resource based on its configuration.

    If the config is detailed, then do nothing;
    If it is a string, then import the appropriate class and create an instance of it.

    Args:
        resource_config (dict): The configuration dictionary for the resource, which includes the class type and other parameters.

    Returns:
        None
    """
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
            r = resource_plr_to_ulab(resource_plr=resource_plr, parent_name=resource_config.get("parent", None))
            # r = resource_plr_to_ulab(resource_plr=resource_plr)
            if resource_config.get("position") is not None:
                r["position"] = resource_config["position"]
            r = tree_to_list([r])
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

    from unilabos.registry.registry import lab_registry
    resources = []
    for resource_config in resources_config:
        if resource_config["parent"] == "tip_rack" or resource_config["parent"] == "plate_well":
            continue
        resources.extend(initialize_resource(resource_config, lab_registry))

    return resources
