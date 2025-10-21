import pytest
import json
from scripts.workflow import build_protocol_graph, draw_protocol_graph, draw_protocol_graph_with_ports


@pytest.mark.parametrize("protocol_name", [
    "example_bio",
    # "bioyond_materials_liquidhandling_1",
])
def test_build_protocol_graph(protocol_name):
    d = json.load(open(f"{protocol_name}.json"))
    graph = build_protocol_graph(labware_info=d["reagent"], protocol_steps=d["workflow"], workstation_name="PRCXi")
    draw_protocol_graph_with_ports(graph, "graph.png")
    print(graph)