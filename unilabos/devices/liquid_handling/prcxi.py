#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
prcxi9300.py – 完整版 PRCXI 9300 Python SDK（支持新增方案）
"""

import socket, json, contextlib
from typing import Any, List, Dict, Optional


class PRCXIError(RuntimeError):
    """Lilith 返回 Success=false 时抛出的业务异常"""


class PRCXI9300:
    # ---------------------------------------------------- 基础初始化
    def __init__(self, host: str = "127.0.0.1", port: int = 9999,
                 timeout: float = 10.0) -> None:
        self.host, self.port, self.timeout = host, port, timeout

    # ---------------------------------------------------- 公共底座
    @staticmethod
    def _len_prefix(n: int) -> bytes:
        return bytes.fromhex(format(n, "016x"))

    def _raw_request(self, payload: str) -> str:
        with contextlib.closing(socket.socket()) as sock:
            sock.settimeout(self.timeout)
            sock.connect((self.host, self.port))
            data = payload.encode()
            sock.sendall(self._len_prefix(len(data)) + data)

            chunks, first = [], True
            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                if first:
                    chunk, first = chunk[8:], False  # 去掉首8字节长度
                chunks.append(chunk)
            return b"".join(chunks).decode()

    def _call(self, service: str, method: str,
              params: Optional[list] = None) -> Any:
        payload = json.dumps(
            {"ServiceName": service,
             "MethodName": method,
             "Paramters": params or []},
            separators=(",", ":")
        )
        resp = json.loads(self._raw_request(payload))
        if not resp.get("Success", False):
            raise PRCXIError(resp.get("Msg", "Unknown error"))
        data = resp.get("Data")
        try:
            return json.loads(data)
        except (TypeError, json.JSONDecodeError):
            return data

    # ---------------------------------------------------- 方案相关（ISolution）
    def list_solutions(self) -> List[Dict[str, Any]]:
        """GetSolutionList"""
        return self._call("ISolution", "GetSolutionList")

    def load_solution(self, solution_id: str) -> bool:
        """LoadSolution"""
        return self._call("ISolution", "LoadSolution", [solution_id])

    def add_solution(self, name: str, matrix_id: str,
                     steps: List[Dict[str, Any]]) -> str:
        """AddSolution → 返回新方案 GUID"""
        return self._call("ISolution", "AddSolution",
                          [name, matrix_id, steps])

    # ---------------------------------------------------- 自动化控制（IAutomation）
    def start(self) -> bool:
        return self._call("IAutomation", "Start")

    def stop(self) -> bool:
        """Stop"""
        return self._call("IAutomation", "Stop")

    def reset(self) -> bool:
        """Reset"""
        return self._call("IAutomation", "Reset")

    def pause(self) -> bool:
        """Pause"""
        return self._call("IAutomation", "Pause")

    def resume(self) -> bool:
        """Resume"""
        return self._call("IAutomation", "Resume")

    def get_error_code(self) -> Optional[str]:
        """GetErrorCode"""
        return self._call("IAutomation", "GetErrorCode")

    def clear_error_code(self) -> bool:
        """RemoveErrorCodet"""
        return self._call("IAutomation", "RemoveErrorCodet")

    # ---------------------------------------------------- 运行状态（IMachineState）
    def step_state_list(self) -> List[Dict[str, Any]]:
        """GetStepStateList"""
        return self._call("IMachineState", "GetStepStateList")

    def step_status(self, seq_num: int) -> Dict[str, Any]:
        """GetStepStatus"""
        return self._call("IMachineState", "GetStepStatus", [seq_num])

    def step_state(self, seq_num: int) -> Dict[str, Any]:
        """GetStepState"""
        return self._call("IMachineState", "GetStepState", [seq_num])

    def axis_location(self, axis_num: int = 1) -> Dict[str, Any]:
        """GetLocation"""
        return self._call("IMachineState", "GetLocation", [axis_num])

    # ---------------------------------------------------- 版位矩阵（IMatrix）
    def list_matrices(self) -> List[Dict[str, Any]]:
        """GetWorkTabletMatrices"""
        return self._call("IMatrix", "GetWorkTabletMatrices")

    def matrix_by_id(self, matrix_id: str) -> Dict[str, Any]:
        """GetWorkTabletMatrixById"""
        return self._call("IMatrix", "GetWorkTabletMatrixById", [matrix_id])

    # ---------------------------------------------------- 辅助：一键运行
    def run_solution(self, solution_id: str, channel_idx: int = 1) -> None:
        self.load_solution(solution_id)
        self.start(channel_idx)


# ---------------------------------------------------- 辅助类 StepData 工具
def build_step(
    axis: str,
    function: str,
    dosage: int,
    plate_no: int,
    is_whole_plate: bool,
    hole_row: int,
    hole_col: int,
    blending_times: int,
    balance_height: int,
    plate_or_hole: str,
    hole_numbers: str,
    assist_fun1: str = "",
    assist_fun2: str = "",
    assist_fun3: str = "",
    assist_fun4: str = "",
    assist_fun5: str = "",
    liquid_method: str = "NormalDispense"
) -> Dict[str, Any]:
    return {
        "StepAxis": axis,
        "Function": function,
        "DosageNum": dosage,
        "PlateNo": plate_no,
        "IsWholePlate": is_whole_plate,
        "HoleRow": hole_row,
        "HoleCol": hole_col,
        "BlendingTimes": blending_times,
        "BalanceHeight": balance_height,
        "PlateOrHoleNum": plate_or_hole,
        "AssistFun1": assist_fun1,
        "AssistFun2": assist_fun2,
        "AssistFun3": assist_fun3,
        "AssistFun4": assist_fun4,
        "AssistFun5": assist_fun5,
        "HoleNumbers": hole_numbers,
        "LiquidDispensingMethod": liquid_method
    }

if __name__ == "__main__":
    # 连接
    # client = PRCXI9300("192.168.43.236", 9999)
    client = PRCXI9300("10.10.35.80", 9999)


    # 获取方案
    solutions = client.list_solutions()
    print(solutions)

    # client.reset()

    solutions = client.list_solutions()
    if not solutions:
        raise RuntimeError("未获取到任何方案")

    solution_id = solutions[0]["Id"]  # ✅ 这是方案 Id
    client.load_solution(solution_id)
    client.start()  # 默认通道 1
#
#     #
#     # # 查询状态
#     # steps = client.step_state_list()
#     # for step in steps:
#     #     print(f"Step {step['SequenceNumber']} status: {step['ProcessState']}")
#     #
#     # # 暂停/继续/停止
#     # client.pause()
#     # client.resume()
#     # client.stop()
#     # client.reset()
#     #
#     # # 错误码处理
#     # error = client.get_error_code()
#     # print(f"Error code: {error}")
#     # client.clear_error_code()
#     #
#     # # 新增方案
#     # steps = [
#     #     build_step("Left", "Load", 0, 1, False, 1, 1, 0, 0, "H1-8,T1", "1,2,3,4,5,6,7,8"),
#     #     build_step("Left", "Imbibing", 10, 2, True, 1, 1, 0, 0, "T2", "1"),
#     #     build_step("Left", "Tapping", 10, 4, False, 1, 1, 0, 0, "H1-8,T4", "1,2,3,4,5,6,7,8"),
#     #     build_step("Left", "Blending", 10, 3, True, 1, 1, 5, 0, "T3", "1"),
#     #     build_step("Left", "UnLoad", 0, 1, False, 1, 1, 0, 0, "H1-8,T1", "1,2,3,4,5,6,7,8")
#     # ]
#     # matrix_list = client.list_matrices()
#     # matrix_id = matrix_list[0]['MatrixId']
#     # #
#     # new_solution_id = client.add_solution(f"test_solution", matrix_id, steps)
#     # print(f"New solution created: {new_solution_id}")
#
#     import time
#
#     # 建立连接
#     client = PRCXI9300("192.168.43.236", 9999)
#
#     # 定义步骤列表
#     steps = [
#         build_step("Left", "Load", 0, 1, False, 1, 1, 0, 0, "H1-8,T1", "1,2,3,4,5,6,7,8"),
#         build_step("Left", "Imbibing", 10, 2, True, 1, 1, 0, 0, "T2", "1"),
#         build_step("Left", "Tapping", 10, 4, False, 1, 1, 0, 0, "H1-8,T4", "1,2,3,4,5,6,7,8"),
#         build_step("Left", "Blending", 10, 3, True, 1, 1, 5, 0, "T3", "1"),
#         build_step("Left", "UnLoad", 0, 1, False, 1, 1, 0, 0, "H1-8,T1", "1,2,3,4,5,6,7,8")
#     ]
#
#     # 获取可用矩阵 ID
#     matrix_list = client.list_matrices()
#     if not matrix_list:
#         raise RuntimeError("未获取到任何矩阵 MatrixId")
#     matrix_id = matrix_list[0]['MatrixId']
#
#     # 创建方案
#     plan_name = f"test_solution_{int(time.time())}"  # 确保唯一名称
#     new_solution_id = client.add_solution(plan_name, matrix_id, steps)
#     print(f"✅ 新增方案: {new_solution_id}（名称: {plan_name}）")
#
#     # 加载并运行方案
#     print("🚀 加载并启动方案 ...")
#     client.load_solution(new_solution_id)
#     client.start()
#
#     # 可选：延时几秒后查看状态
#     time.sleep(3)
#     print("📋 步骤运行状态：")
#     for step in client.step_state_list():
#         print(f"Step {step['SequenceNumber']} 状态: {step['ProcessState']}")

# if __name__ == "__main__":
#     import time
#
#     # 建立连接
#     client = PRCXI9300("10.10.35.80", 9999)
#
#     # 定义步骤列表
#     steps = [
#         build_step("Left", "Load", 0, 1, False, 1, 1, 0, 0, "H1-8,T1", "1,2,3,4,5,6,7,8"),
#         build_step("Left", "Imbibing", 10, 2, True, 1, 1, 0, 0, "T2", "1"),
#         build_step("Left", "Tapping", 10, 4, False, 1, 1, 0, 0, "H1-8,T4", "1,2,3,4,5,6,7,8"),
#         build_step("Left", "Blending", 10, 3, True, 1, 1, 5, 0, "T3", "1"),
#         build_step("Left", "UnLoad", 0, 1, False, 1, 1, 0, 0, "H1-8,T1", "1,2,3,4,5,6,7,8")
#     ]
#
#     # 获取可用矩阵 ID
#     matrix_list = client.list_matrices()
#     if not matrix_list:
#         raise RuntimeError("❌ 未获取到任何矩阵 MatrixId")
#     matrix_id = matrix_list[0].get("MatrixId")
#     print(f"✅ 获取矩阵 ID: {matrix_id}")
#
#     # 创建方案
#     plan_name = f"test_solution_{int(time.time())}"
#     try:
#         new_solution_id = client.add_solution(plan_name, matrix_id, steps)
#         assert new_solution_id, "方案创建失败，返回空 ID"
#         print(f"✅ 新增方案成功: {plan_name}, ID: {new_solution_id}")
#     except Exception as e:
#         print(f"❌ 方案创建失败: {e}")
#         raise
#
#     # 加载方案
#     try:
#         load_ok = client.load_solution(new_solution_id)
#         assert load_ok, "加载方案失败"
#         print("✅ 加载方案成功")
#     except Exception as e:
#         print(f"❌ 加载方案失败: {e}")
#         raise
#
#     # 启动方案
#     try:
#         start_ok = client.start()
#         assert start_ok, "启动方案失败"
#         print("✅ 方案已启动")
#     except Exception as e:
#         print(f"❌ 启动失败: {e}")
#         raise
#
#     # 等待设备反馈后获取状态
#     time.sleep(3)
#     try:
#         steps_status = client.step_state_list()
#         print(f"📋 共加载步骤: {len(steps_status)} 个")
#         for step in steps_status:
#             print(f"→ Step {step['SequenceNumber']} 状态: {step['ProcessState']}")
#     except Exception as e:
#         print(f"⚠️ 获取步骤状态失败: {e}")
