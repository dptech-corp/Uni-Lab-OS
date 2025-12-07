import json
from os import PathLike

from unilabos.workflow.common import build_protocol_graph


def from_labwares_and_steps(data_path: PathLike):
    with data_path.open("r", encoding="utf-8") as fp:
        d = json.load(fp)

    if "workflow" in d and "reagent" in d:
        protocol_steps = d["workflow"]
        labware_info = d["reagent"]
    elif "steps_info" in d and "labware_info" in d:
        protocol_steps = _normalize_steps(d["steps_info"])
        labware_info = _normalize_labware(d["labware_info"])
    else:
        raise ValueError("Unsupported protocol format")

    graph = build_protocol_graph(
        labware_info=labware_info,
        protocol_steps=protocol_steps,
        workstation_name="PRCXi",
    )