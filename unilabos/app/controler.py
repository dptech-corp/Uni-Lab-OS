
import json
import traceback
import uuid
from unilabos.app.model import JobAddReq, JobData
from unilabos.ros.nodes.presets.host_node import HostNode
from unilabos.utils.type_check import serialize_result_info


def get_resources() -> tuple:
    if HostNode.get_instance() is None:
        return False, "Host node not initialized"

    return True, HostNode.get_instance().resources_config

def devices() -> tuple:
    if HostNode.get_instance() is None:
        return False, "Host node not initialized"
    
    return True, HostNode.get_instance().devices_config

def job_info(id: str):
    get_goal_status = HostNode.get_instance().get_goal_status(id)
    return JobData(jobId=id, status=get_goal_status)

def job_add(req: JobAddReq) -> JobData:
    if req.job_id is None:
        req.job_id = str(uuid.uuid4())
    action_name = req.data["action"]
    action_type = req.data.get("action_type", "LocalUnknown")
    action_args = req.data.get("action_kwargs", None)  # 兼容老版本，后续删除
    if action_args is None:
        action_args = req.data.get("action_args")
    else:
        if "command" in action_args:
            action_args = action_args["command"]
    # print(f"job_add:{req.device_id} {action_name} {action_kwargs}")
    try:
        HostNode.get_instance().send_goal(req.device_id, action_type=action_type, action_name=action_name, action_kwargs=action_args, goal_uuid=req.job_id, server_info=req.server_info)
    except Exception as e:
        for bridge in HostNode.get_instance().bridges:
            if hasattr(bridge, "publish_job_status"):
                bridge.publish_job_status({}, req.job_id, "failed", serialize_result_info(traceback.format_exc(), False, {}))
    return JobData(jobId=req.job_id)
