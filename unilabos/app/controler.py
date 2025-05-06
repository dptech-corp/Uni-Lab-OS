
import json
import uuid
from unilabos.app.model import JobAddReq, JobData
from unilabos.ros.nodes.presets.host_node import HostNode


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
    action_kwargs = req.data["action_kwargs"]
    req.data['action'] = action_name
    if action_name == "execute_command_from_outer":
        action_kwargs = {"command": json.dumps(action_kwargs)}
    elif "command" in action_kwargs:
        action_kwargs = action_kwargs["command"]
    print(f"job_add:{req.device_id} {action_name} {action_kwargs}")
    HostNode.get_instance().send_goal(req.device_id, action_name=action_name, action_kwargs=action_kwargs, goal_uuid=req.job_id)
    return JobData(jobId=req.job_id)
