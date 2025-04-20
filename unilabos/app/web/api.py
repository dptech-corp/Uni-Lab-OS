"""
API模块

提供API路由和处理函数
"""

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import asyncio

from unilabos.app.controler import devices, job_add, job_info
from unilabos.app.model import (
    Resp,
    RespCode,
    JobStatusResp,
    JobAddResp,
    JobAddReq,
    JobStepFinishReq,
    JobPreintakeFinishReq,
    JobFinishReq,
)
from unilabos.app.web.utils.host_utils import get_host_node_info

# 创建API路由器
api = APIRouter()
admin = APIRouter()

# 存储所有活动的WebSocket连接
active_connections: set[WebSocket] = set()


async def broadcast_device_status():
    """广播设备状态到所有连接的客户端"""
    while True:
        try:
            # 获取最新的设备状态
            host_info = get_host_node_info()
            if host_info["available"]:
                # 准备要发送的数据
                status_data = {
                    "type": "device_status",
                    "data": {
                        "device_status": host_info["device_status"],
                        "device_status_timestamps": host_info["device_status_timestamps"],
                    },
                }
                # 发送到所有连接的客户端
                for connection in active_connections:
                    try:
                        await connection.send_json(status_data)
                    except Exception as e:
                        print(f"Error sending to client: {e}")
                        active_connections.remove(connection)
            await asyncio.sleep(1)  # 每秒更新一次
        except Exception as e:
            print(f"Error in broadcast: {e}")
            await asyncio.sleep(1)


@api.websocket("/ws/device_status")
async def websocket_device_status(websocket: WebSocket):
    """WebSocket端点，用于实时获取设备状态"""
    await websocket.accept()
    active_connections.add(websocket)
    try:
        while True:
            # 保持连接活跃
            await websocket.receive_text()
    except WebSocketDisconnect:
        active_connections.remove(websocket)
    except Exception as e:
        print(f"WebSocket error: {e}")
        active_connections.remove(websocket)


@api.get("/resources", summary="Resource list", response_model=Resp)
def get_resources():
    """获取资源列表"""
    isok, data = devices()
    if not isok:
        return Resp(code=RespCode.ErrorHostNotInit, message=str(data))

    return Resp(data=dict(data))


@api.get("/repository", summary="Raw Material list", response_model=Resp)
def get_raw_material():
    """获取原材料列表"""
    return Resp(data={})


@api.post("/repository", summary="Raw Material set", response_model=Resp)
def post_raw_material():
    """设置原材料"""
    return Resp(data={})


@api.get("/devices", summary="Device list", response_model=Resp)
def get_devices():
    """获取设备列表"""
    isok, data = devices()
    if not isok:
        return Resp(code=RespCode.ErrorHostNotInit, message=str(data))

    return Resp(data=dict(data))


@api.get("/devices/{id}/info", summary="Device info", response_model=Resp)
def device_info(id: str):
    """获取设备信息"""
    return Resp(data={})


@api.get("/job/{id}/status", summary="Job status", response_model=JobStatusResp)
def job_status(id: str):
    """获取任务状态"""
    data = job_info(id)
    return JobStatusResp(data=data)


@api.post("/job/add", summary="Create job", response_model=JobAddResp)
def post_job_add(req: JobAddReq):
    """创建任务"""
    device_id = req.device_id
    if not req.data:
        return Resp(code=RespCode.ErrorInvalidReq, message="Invalid request data")

    req.device_id = device_id
    data = job_add(req)
    return JobAddResp(data=data)


@api.post("/job/step_finish", summary="步骤完成推送", response_model=Resp)
def callback_step_finish(req: JobStepFinishReq):
    """任务步骤完成回调"""
    print(req)
    return Resp(data={})


@api.post("/job/preintake_finish", summary="通量完成推送", response_model=Resp)
def callback_preintake_finish(req: JobPreintakeFinishReq):
    """通量完成回调"""
    print(req)
    return Resp(data={})


@api.post("/job/finish", summary="完成推送", response_model=Resp)
def callback_order_finish(req: JobFinishReq):
    """任务完成回调"""
    print(req)
    return Resp(data={})


@admin.get("/device_models", summary="Device model list", response_model=Resp)
def admin_device_models():
    """获取设备模型列表"""
    return Resp(data={})


@admin.post("/device_model/add", summary="Add Device model", response_model=Resp)
def admin_device_model_add():
    """添加设备模型"""
    return Resp(data={})


@admin.delete("/device_model/{id}", summary="Delete device model", response_model=Resp)
def admin_device_model_del(id: str):
    """删除设备模型"""
    return Resp(data={})


@admin.get("/devices", summary="Device list", response_model=Resp)
def admin_devices():
    """获取设备列表(管理员)"""
    return Resp(data={})


@admin.post("/devices/add", summary="Add Device", response_model=Resp)
def admin_device_add():
    """添加设备"""
    return Resp(data={})


@admin.delete("/devices/{id}", summary="Delete device", response_model=Resp)
def admin_device_del(id: str):
    """删除设备"""
    return Resp(data={})


def setup_api_routes(app):
    """设置API路由"""
    app.include_router(admin, prefix="/admin/v1", tags=["admin"])
    app.include_router(api, prefix="/api/v1", tags=["api"])

    # 启动广播任务
    @app.on_event("startup")
    async def startup_event():
        asyncio.create_task(broadcast_device_status())
