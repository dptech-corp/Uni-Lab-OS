# bioyond_rpc.py
"""
BioyondV1RPC类定义 - 包含所有RPC接口和业务逻辑
"""

from enum import Enum
from datetime import datetime, timezone
from unilabos.device_comms.rpc import BaseRequest
from typing import Optional, List, Dict, Any
import json
from unilabos.devices.workstation.bioyond_studio.config import WORKFLOW_TO_SECTION_MAP, WORKFLOW_STEP_IDS, LOCATION_MAPPING


class SimpleLogger:
    """简单的日志记录器"""
    def info(self, msg): print(f"[INFO] {msg}")
    def error(self, msg): print(f"[ERROR] {msg}")
    def debug(self, msg): print(f"[DEBUG] {msg}")
    def warning(self, msg): print(f"[WARNING] {msg}")
    def critical(self, msg): print(f"[CRITICAL] {msg}")


class MachineState(Enum):
    INITIAL = 0
    STOPPED = 1
    RUNNING = 2
    PAUSED = 3
    ERROR_PAUSED = 4
    ERROR_STOPPED = 5


class MaterialType(Enum):
    Consumables = 0
    Sample = 1
    Reagent = 2
    Product = 3


class BioyondV1RPC(BaseRequest):
    def __init__(self, config):
        super().__init__()
        print("开始初始化")
        self.config = config
        self.api_key = config["api_key"]
        self.host = config["api_host"]
        self._logger = SimpleLogger()
        self.is_running = False
        self.workflow_mappings = {}
        self.workflow_sequence = []
        self.pending_task_params = []
        self.material_cache = {}
        self._load_material_cache()

        if "workflow_mappings" in config:
            self._set_workflow_mappings(config["workflow_mappings"])

    def _set_workflow_mappings(self, mappings: Dict[str, str]):
        self.workflow_mappings = mappings
        print(f"设置工作流映射配置: {mappings}")

    def _get_workflow(self, web_workflow_name: str) -> str:
        if web_workflow_name not in self.workflow_mappings:
            print(f"未找到工作流映射配置: {web_workflow_name}")
            return ""
        workflow_id = self.workflow_mappings[web_workflow_name]
        print(f"获取工作流: {web_workflow_name} -> {workflow_id}")
        return workflow_id

    def process_web_workflows(self, json_str: str) -> Dict[str, str]:
        try:
            data = json.loads(json_str)
            web_workflow_list = data.get("web_workflow_list", [])
        except json.JSONDecodeError:
            print(f"无效的JSON字符串: {json_str}")
            return {}

        result = {}
        self.workflow_sequence = []

        for web_name in web_workflow_list:
            workflow_id = self._get_workflow(web_name)
            if workflow_id:
                result[web_name] = workflow_id
                self.workflow_sequence.append(workflow_id)
            else:
                print(f"无法获取工作流ID: {web_name}")

        print(f"工作流执行顺序: {self.workflow_sequence}")
        return result

    def get_workflow_sequence(self) -> List[str]:
        id_to_name = {workflow_id: name for name, workflow_id in self.workflow_mappings.items()}
        workflow_names = []
        for workflow_id in self.workflow_sequence:
            workflow_names.append(id_to_name.get(workflow_id, workflow_id))
        return workflow_names

    def append_to_workflow_sequence(self, json_str: str) -> bool:
        try:
            data = json.loads(json_str)
            web_workflow_name = data.get("web_workflow_name", "")
        except:
            return False

        workflow_id = self._get_workflow(web_workflow_name)
        if workflow_id:
            self.workflow_sequence.append(workflow_id)
            print(f"添加工作流到执行顺序: {web_workflow_name} -> {workflow_id}")

    def set_workflow_sequence(self, json_str: str) -> List[str]:
        try:
            data = json.loads(json_str)
            web_workflow_names = data.get("web_workflow_names", [])
        except:
            return []

        sequence = []
        for web_name in web_workflow_names:
            workflow_id = self._get_workflow(web_name)
            if workflow_id:
                sequence.append(workflow_id)

        self.workflow_sequence = sequence
        print(f"设置工作流执行顺序: {self.workflow_sequence}")
        return self.workflow_sequence.copy()

    def get_all_workflows(self) -> Dict[str, str]:
        return self.workflow_mappings.copy()

    def clear_workflows(self):
        self.workflow_sequence = []
        print("清空工作流执行顺序")

    def get_current_time_iso8601(self) -> str:
        current_time = datetime.now(timezone.utc).isoformat(timespec='milliseconds')
        return current_time.replace("+00:00", "Z")

    # 物料查询接口
    def stock_material(self, json_str: str) -> list:
        try:
            params = json.loads(json_str)
        except:
            return []

        response = self.post(
            url=f'{self.host}/api/lims/storage/stock-material',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response or response['code'] != 1:
            return []
        return response.get("data", [])

    # 工作流列表查询
    def query_workflow(self, json_str: str) -> dict:
        try:
            params = json.loads(json_str)
        except:
            return {}

        response = self.post(
            url=f'{self.host}/api/lims/workflow/work-flow-list',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response or response['code'] != 1:
            return {}
        return response.get("data", {})

    def validate_workflow_parameters(self, workflows: List[Dict[str, Any]]) -> Dict[str, Any]:
        """验证工作流参数格式

        Args:
            workflows (List[Dict[str, Any]]): 工作流列表

        Returns:
            Dict[str, Any]: 验证结果
        """
        try:
            validation_errors = []

            for i, workflow in enumerate(workflows):
                workflow_errors = []

                # 检查基本结构
                if not isinstance(workflow, dict):
                    workflow_errors.append("工作流必须是字典类型")
                    continue

                if "id" not in workflow:
                    workflow_errors.append("缺少必要的 'id' 字段")

                # 检查 stepParameters（如果存在）
                if "stepParameters" in workflow:
                    step_params = workflow["stepParameters"]

                    if not isinstance(step_params, dict):
                        workflow_errors.append("stepParameters 必须是字典类型")
                    else:
                        # 验证参数结构
                        for step_id, modules in step_params.items():
                            if not isinstance(modules, dict):
                                workflow_errors.append(f"步骤 {step_id} 的模块配置必须是字典类型")
                                continue

                            for module_name, params in modules.items():
                                if not isinstance(params, list):
                                    workflow_errors.append(f"步骤 {step_id} 模块 {module_name} 的参数必须是列表类型")
                                    continue

                                for j, param in enumerate(params):
                                    if not isinstance(param, dict):
                                        workflow_errors.append(f"步骤 {step_id} 模块 {module_name} 参数 {j} 必须是字典类型")
                                    elif "key" not in param or "value" not in param:
                                        workflow_errors.append(f"步骤 {step_id} 模块 {module_name} 参数 {j} 必须包含 key 和 value")

                if workflow_errors:
                    validation_errors.append({
                        "workflow_index": i,
                        "workflow_id": workflow.get("id", "unknown"),
                        "errors": workflow_errors
                    })

            if validation_errors:
                return {
                    "valid": False,
                    "errors": validation_errors,
                    "message": f"发现 {len(validation_errors)} 个工作流存在验证错误"
                }
            else:
                return {
                    "valid": True,
                    "message": f"所有 {len(workflows)} 个工作流验证通过"
                }

        except Exception as e:
            return {
                "valid": False,
                "errors": [{"general_error": str(e)}],
                "message": f"验证过程中发生异常: {str(e)}"
            }

    def get_workflow_parameter_template(self) -> Dict[str, Any]:
        """获取工作流参数模板

        Returns:
            Dict[str, Any]: 参数模板和说明
        """
        return {
            "template": {
                "name": "拼接后的长工作流的名称",
                "workflows": [
                    {
                        "id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
                        "stepParameters": {
                            "步骤ID (UUID)": {
                                "模块名称": [
                                    {
                                        "key": "参数键名",
                                        "value": "参数值或变量引用 {{index-m-n}}"
                                    }
                                ]
                            }
                        }
                    }
                ]
            },
            "parameter_descriptions": {
                "name": "拼接后的长工作流名称",
                "workflows": "待合并的子工作流列表",
                "id": "子工作流 ID，对应工作流列表中 workflows 数组中每个对象的 id 字段",
                "stepParameters": "步骤参数配置，如果子工作流没有参数则不需要填写"
            },
            "common_modules": {
                "反应模块-开始搅拌": {
                    "description": "反应模块搅拌控制",
                    "common_parameters": ["temperature"]
                },
                "通量-配置": {
                    "description": "通量配置模块",
                    "common_parameters": ["cutoff", "assignMaterialName"]
                },
                "烧杯溶液放置位-烧杯吸液分液": {
                    "description": "烧杯液体处理模块",
                    "common_parameters": ["titrationType", "assignMaterialName", "volume"]
                }
            },
            "variable_reference_format": {
                "format": "{{index-m-n}}",
                "description": {
                    "index": "该步骤所在子工作流的拼接顺序(从 1 开始)",
                    "m": "拼接前该步骤在子工作流内部的 m 值",
                    "n": "拼接前该步骤在子工作流内部的 n 值"
                }
            }
        }

    # 工作流步骤查询接口
    def workflow_step_query(self, json_str: str) -> dict:
        try:
            data = json.loads(json_str)
            workflow_id = data.get("workflow_id", "")
        except:
            return {}

        response = self.post(
            url=f'{self.host}/api/lims/workflow/sub-workflow-step-parameters',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": workflow_id,
            })

        if not response or response['code'] != 1:
            return {}
        return response.get("data", {})

    # 任务推送接口
    def create_order(self, json_str: str) -> dict:
        try:
            params = json.loads(json_str)
        except Exception as e:
            result = str({"success": False, "error": f"create_order:处理JSON时出错: {str(e)}", "method": "create_order"})
            return result

        print('===============', json.dumps(params))

        request_params = {
            "apiKey": self.api_key,
            "requestTime": self.get_current_time_iso8601(),
            "data": params
        }

        response = self.post(
            url=f'{self.host}/api/lims/order/order',
            params=request_params)

        if response['code'] != 1:
            print(f"create order error: {response.get('message')}")

        print(f"create order data: {response['data']}")
        result = str(response.get("data", {}))
        return result

    # 查询任务列表
    def order_query(self, json_str: str) -> dict:
        try:
            params = json.loads(json_str)
        except:
            return {}

        response = self.post(
            url=f'{self.host}/api/lims/order/order-list',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response or response['code'] != 1:
            return {}
        return response.get("data", {})

    # 任务明细查询
    def order_report(self, json_str: str) -> dict:
        try:
            data = json.loads(json_str)
            order_id = data.get("order_id", "")
        except:
            return {}

        response = self.post(
            url=f'{self.host}/api/lims/order/order-report',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": order_id,
            })

        if not response or response['code'] != 1:
            return {}
        return response.get("data", {})

    def material_id_query(self, json_str: str) -> dict:
        """
        查询物料id
        json_str 格式为JSON字符串:
        '{"material123"}'
        """
        params = json_str

        response = self.post(
            url=f'{self.host}/api/lims/storage/workflow-sample-locations',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response:
            return {}

        if response['code'] != 1:
            print(f"material_id_query error: {response.get('message')}")
            return {}

        print(f"material_id_query data: {response['data']}")
        return response.get("data", {})

    # 任务取出接口
    def order_takeout(self, json_str: str) -> int:
        try:
            data = json.loads(json_str)
            params = {
                "orderId": data.get("order_id", ""),
                "preintakeId": data.get("preintake_id", "")
            }
        except:
            return 0

        response = self.post(
            url=f'{self.host}/api/lims/order/order-takeout',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params,
            })

        if not response or response['code'] != 1:
            return 0
        return response.get("code", 0)

    # 设备列表查询
    def device_list(self, json_str: str = "") -> list:
        device_no = None
        if json_str:
            try:
                data = json.loads(json_str)
                device_no = data.get("device_no", None)
            except:
                pass

        url = f'{self.host}/api/lims/device/device-list'
        if device_no:
            url += f'/{device_no}'

        response = self.post(
            url=url,
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
            })

        if not response or response['code'] != 1:
            return []
        return response.get("data", [])

    # 设备操作
    def device_operation(self, json_str: str) -> int:
        try:
            data = json.loads(json_str)
            params = {
                "deviceNo": data.get("device_no", ""),
                "operationType": data.get("operation_type", 0),
                "operationParams": data.get("operation_params", {})
            }
        except:
            return 0

        response = self.post(
            url=f'{self.host}/api/lims/device/device-operation',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params,
            })

        if not response or response['code'] != 1:
            return 0
        return response.get("code", 0)

    # 调度器状态查询
    def scheduler_status(self) -> dict:
        response = self.post(
            url=f'{self.host}/api/lims/scheduler/scheduler-status',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
            })

        if not response or response['code'] != 1:
            return {}
        return response.get("data", {})

    # 调度器启动
    def scheduler_start(self) -> int:
        response = self.post(
            url=f'{self.host}/api/lims/scheduler/start',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
            })

        if not response or response['code'] != 1:
            return 0
        return response.get("code", 0)

    # 调度器暂停
    def scheduler_pause(self) -> int:
        response = self.post(
            url=f'{self.host}/api/lims/scheduler/scheduler-pause',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
            })

        if not response or response['code'] != 1:
            return 0
        return response.get("code", 0)

    # 调度器继续
    def scheduler_continue(self) -> int:
        response = self.post(
            url=f'{self.host}/api/lims/scheduler/scheduler-continue',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
            })

        if not response or response['code'] != 1:
            return 0
        return response.get("code", 0)

    # 调度器停止
    def scheduler_stop(self) -> int:
        response = self.post(
            url=f'{self.host}/api/lims/scheduler/scheduler-stop',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
            })

        if not response or response['code'] != 1:
            return 0
        return response.get("code", 0)

    # 调度器重置
    def scheduler_reset(self) -> int:
        response = self.post(
            url=f'{self.host}/api/lims/scheduler/scheduler-reset',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
            })

        if not response or response['code'] != 1:
            return 0
        return response.get("code", 0)

    # 取消任务
    def cancel_order(self, json_str: str) -> bool:
        try:
            data = json.loads(json_str)
            order_id = data.get("order_id", "")
        except:
            return False

        response = self.post(
            url=f'{self.host}/api/lims/order/cancel-order',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": order_id,
            })

        if not response or response['code'] != 1:
            return False
        return True

    # 获取可拼接工作流
    def query_split_workflow(self) -> list:
        response = self.post(
            url=f'{self.host}/api/lims/workflow/split-workflow-list',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
            })

        if not response or response['code'] != 1:
            return []
        return str(response.get("data", {}))

    # 合并工作流
    def merge_workflow(self, json_str: str) -> dict:
        try:
            data = json.loads(json_str)
            params = {
                "name": data.get("name", ""),
                "workflowIds": data.get("workflow_ids", [])
            }
        except:
            return {}

        response = self.post(
            url=f'{self.host}/api/lims/workflow/merge-workflow',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params,
            })

        if not response or response['code'] != 1:
            return {}
        return response.get("data", {})

    # 合并工作流并设置参数 API
    def merge_workflow_with_parameters(self, json_str: str) -> dict:
        """合并工作流并设置参数

        调用 Bioyond API: /api/lims/workflow/merge-workflow-with-parameters

        Args:
            json_str (str): JSON 字符串，包含工作流合并配置数据

        Returns:
            dict: API 响应结果，包含 code、message 和 timestamp
        """
        try:
            # 解析输入的 JSON 数据
            data = json.loads(json_str)

            # 构造 API 请求参数
            params = {
                "name": data.get("name", ""),
                "workflows": data.get("workflows", [])
            }

            # 验证必要参数
            if not params["name"]:
                return {
                    "code": 0,
                    "message": "工作流名称不能为空",
                    "timestamp": int(datetime.now().timestamp() * 1000)
                }

            if not params["workflows"]:
                return {
                    "code": 0,
                    "message": "工作流列表不能为空",
                    "timestamp": int(datetime.now().timestamp() * 1000)
                }

        except json.JSONDecodeError as e:
            return {
                "code": 0,
                "message": f"JSON 解析错误: {str(e)}",
                "timestamp": int(datetime.now().timestamp() * 1000)
            }
        except Exception as e:
            return {
                "code": 0,
                "message": f"参数处理错误: {str(e)}",
                "timestamp": int(datetime.now().timestamp() * 1000)
            }

        # 发送 POST 请求到 Bioyond API
        try:
            response = self.post(
                url=f'{self.host}/api/lims/workflow/merge-workflow-with-parameters',
                params={
                    "apiKey": self.api_key,
                    "requestTime": self.get_current_time_iso8601(),
                    "data": params,
                })

            # 处理响应
            if not response:
                return {
                    "code": 0,
                    "message": "API 请求失败，未收到响应",
                    "timestamp": int(datetime.now().timestamp() * 1000)
                }

            # 返回完整的响应结果
            return {
                "code": response.get("code", 0),
                "message": response.get("message", ""),
                "timestamp": response.get("timestamp", int(datetime.now().timestamp() * 1000))
            }

        except Exception as e:
            return {
                "code": 0,
                "message": f"API 请求异常: {str(e)}",
                "timestamp": int(datetime.now().timestamp() * 1000)
            }

    # 合并当前工作流序列
    def merge_sequence_workflow(self, json_str: str) -> dict:
        try:
            data = json.loads(json_str)
            name = data.get("name", "合并工作流")
        except:
            return {}

        if not self.workflow_sequence:
            print("工作流序列为空，无法合并")
            return {}

        params = {
            "name": name,
            "workflowIds": self.workflow_sequence
        }

        response = self.post(
            url=f'{self.host}/api/lims/workflow/merge-workflow',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params,
            })

        if not response or response['code'] != 1:
            return {}
        return response.get("data", {})

    # 发布任务
    def process_and_execute_workflow(self, workflow_name: str, task_name: str) -> dict:
        web_workflow_list = self.get_workflow_sequence()
        workflow_name = workflow_name

        pending_params_backup = self.pending_task_params.copy()
        print(f"保存pending_task_params副本，共{len(pending_params_backup)}个参数")

        # 1. 处理网页工作流列表
        print(f"处理网页工作流列表: {web_workflow_list}")
        web_workflow_json = json.dumps({"web_workflow_list": web_workflow_list})
        workflows_result = self.process_web_workflows(web_workflow_json)

        if not workflows_result:
            error_msg = "处理网页工作流列表失败"
            print(error_msg)
            result = str({"success": False, "error": f"process_and_execute_workflow:{error_msg}", "method": "process_and_execute_workflow", "step": "process_web_workflows"})
            return result

        # 2. 合并工作流序列
        print(f"合并工作流序列，名称: {workflow_name}")
        merge_json = json.dumps({"name": workflow_name})
        merged_workflow = self.merge_sequence_workflow(merge_json)
        print(f"合并工作流序列结果: {merged_workflow}")

        if not merged_workflow:
            error_msg = "合并工作流序列失败"
            print(error_msg)
            result = str({"success": False, "error": f"process_and_execute_workflow:{error_msg}", "method": "process_and_execute_workflow", "step": "merge_sequence_workflow"})
            return result

        # 3. 合并所有参数并创建任务
        workflow_name = merged_workflow.get("name", "")
        workflow_id = merged_workflow.get("subWorkflows", [{}])[0].get("id", "")
        print(f"使用工作流创建任务: {workflow_name} (ID: {workflow_id})")

        workflow_query_json = json.dumps({"workflow_id": workflow_id})
        workflow_params_structure = self.workflow_step_query(workflow_query_json)

        self.pending_task_params = pending_params_backup
        print(f"恢复pending_task_params，共{len(self.pending_task_params)}个参数")

        param_values = self.generate_task_param_values(workflow_params_structure)

        task_params = [{
            "orderCode": f"BSO{self.get_current_time_iso8601().replace('-', '').replace('T', '').replace(':', '').replace('.', '')[:14]}",
            "orderName": f"实验-{self.get_current_time_iso8601()[:10].replace('-', '')}",
            "workFlowId": workflow_id,
            "borderNumber": 1,
            "paramValues": param_values,
            "extendProperties": ""
        }]

        task_json = json.dumps(task_params)
        print(f"创建任务参数: {type(task_json)}")
        result = self.create_order(task_json)

        if not result:
            error_msg = "创建任务失败"
            print(error_msg)
            result = str({"success": False, "error": f"process_and_execute_workflow:{error_msg}", "method": "process_and_execute_workflow", "step": "create_order"})
            return result

        print(f"任务创建成功: {result}")
        self.pending_task_params.clear()
        print("已清空pending_task_params")

        return {
            "success": True,
            "workflow": {"name": workflow_name, "id": workflow_id},
            "task": result,
            "method": "process_and_execute_workflow"
        }

    # 生成任务参数
    def generate_task_param_values(self, workflow_params_structure):
        if not workflow_params_structure:
            print("workflow_params_structure为空")
            return {}

        data = workflow_params_structure

        # 从pending_task_params中提取实际参数值，按DisplaySectionName和Key组织
        pending_params_by_section = {}
        print(f"开始处理pending_task_params，共{len(self.pending_task_params)}个任务参数组")

        # 获取工作流执行顺序，用于按顺序匹配参数
        workflow_sequence = self.get_workflow_sequence()
        print(f"工作流执行顺序: {workflow_sequence}")

        workflow_index = 0

        for i, task_param in enumerate(self.pending_task_params):
            if 'param_values' in task_param:
                print(f"处理第{i+1}个任务参数组，包含{len(task_param['param_values'])}个步骤")

                if workflow_index < len(workflow_sequence):
                    current_workflow = workflow_sequence[workflow_index]
                    section_name = WORKFLOW_TO_SECTION_MAP.get(current_workflow)
                    print(f"  匹配到工作流: {current_workflow} -> {section_name}")
                    workflow_index += 1
                else:
                    print(f"  警告: 参数组{i+1}超出了工作流序列范围")
                    continue

                if not section_name:
                    print(f"  警告: 工作流{current_workflow}没有对应的DisplaySectionName")
                    continue

                if section_name not in pending_params_by_section:
                    pending_params_by_section[section_name] = {}

                for step_id, param_list in task_param['param_values'].items():
                    print(f"    步骤ID: {step_id}，参数数量: {len(param_list)}")

                    for param_item in param_list:
                        key = param_item.get('Key', '')
                        value = param_item.get('Value', '')
                        m = param_item.get('m', 0)
                        n = param_item.get('n', 0)
                        print(f"    参数: {key} = {value} (m={m}, n={n}) -> 分组到{section_name}")

                        param_key = f"{section_name}.{key}"
                        if param_key not in pending_params_by_section[section_name]:
                            pending_params_by_section[section_name][param_key] = []

                        pending_params_by_section[section_name][param_key].append({
                            'value': value,
                            'm': m,
                            'n': n
                        })

        print(f"pending_params_by_section构建完成，包含{len(pending_params_by_section)}个分组")

        # 收集所有参数，过滤TaskDisplayable为0的项
        filtered_params = []

        for step_id, step_info in data.items():
            if isinstance(step_info, list):
                for step_item in step_info:
                    param_list = step_item.get("parameterList", [])
                    for param in param_list:
                        if param.get("TaskDisplayable") == 0:
                            continue

                        param_with_step = param.copy()
                        param_with_step['step_id'] = step_id
                        param_with_step['step_name'] = step_item.get("name", "")
                        param_with_step['step_m'] = step_item.get("m", 0)
                        param_with_step['step_n'] = step_item.get("n", 0)
                        filtered_params.append(param_with_step)

        # 按DisplaySectionIndex排序
        filtered_params.sort(key=lambda x: x.get('DisplaySectionIndex', 0))

        # 生成参数映射
        param_mapping = {}
        step_params = {}
        for param in filtered_params:
            step_id = param['step_id']
            if step_id not in step_params:
                step_params[step_id] = []
            step_params[step_id].append(param)

        # 为每个步骤生成参数
        for step_id, params in step_params.items():
            param_list = []
            for param in params:
                key = param.get('Key', '')
                display_section_index = param.get('DisplaySectionIndex', 0)
                step_m = param.get('step_m', 0)
                step_n = param.get('step_n', 0)

                section_name = param.get('DisplaySectionName', '')
                param_key = f"{section_name}.{key}"

                if section_name in pending_params_by_section and param_key in pending_params_by_section[section_name]:
                    pending_param_list = pending_params_by_section[section_name][param_key]
                    if pending_param_list:
                        pending_param = pending_param_list[0]
                        value = pending_param['value']
                        m = step_m
                        n = step_n
                        print(f"      匹配成功: {section_name}.{key} = {value} (m={m}, n={n})")
                        pending_param_list.pop(0)
                    else:
                        value = "1"
                        m = step_m
                        n = step_n
                        print(f"      匹配失败: {section_name}.{key}，参数列表为空，使用默认值 = {value}")
                else:
                    value = "1"
                    m = display_section_index
                    n = step_n
                    print(f"      匹配失败: {section_name}.{key}，使用默认值 = {value} (m={m}, n={n})")

                param_item = {
                    "m": m,
                    "n": n,
                    "key": key,
                    "value": str(value).strip()
                }
                param_list.append(param_item)

            if param_list:
                param_mapping[step_id] = param_list

        print(f"生成任务参数值，包含 {len(param_mapping)} 个步骤")
        return param_mapping

    # 工作流方法
    def reactor_taken_out(self):
        """反应器取出"""
        self.append_to_workflow_sequence('{"web_workflow_name": "reactor_taken_out"}')
        reactor_taken_out_params = {"param_values": {}}
        self.pending_task_params.append(reactor_taken_out_params)
        print(f"成功添加反应器取出工作流")
        print(f"当前队列长度: {len(self.pending_task_params)}")
        return json.dumps({"suc": True})

    def reactor_taken_in(self, assign_material_name: str, cutoff: str = "900000", temperature: float = -10.00):
        """反应器放入"""
        self.append_to_workflow_sequence('{"web_workflow_name": "reactor_taken_in"}')
        material_id = self._get_material_id_by_name(assign_material_name)

        if isinstance(temperature, str):
            temperature = float(temperature)

        step_id = WORKFLOW_STEP_IDS["reactor_taken_in"]["config"]
        reactor_taken_in_params = {
            "param_values": {
                step_id: [
                    {"m": 0, "n": 3, "Key": "cutoff", "Value": cutoff},
                    {"m": 0, "n": 3, "Key": "temperature", "Value": f"{temperature:.2f}"},
                    {"m": 0, "n": 3, "Key": "assignMaterialName", "Value": material_id}
                ]
            }
        }

        self.pending_task_params.append(reactor_taken_in_params)
        print(f"成功添加反应器放入参数: material={assign_material_name}->ID:{material_id}, cutoff={cutoff}, temp={temperature:.2f}")
        print(f"当前队列长度: {len(self.pending_task_params)}")
        return json.dumps({"suc": True})

    def solid_feeding_vials(self, material_id: str, time: str = "0", torque_variation: str = "1",
                            assign_material_name: str = None, temperature: float = 25.00):
        """固体进料小瓶"""
        self.append_to_workflow_sequence('{"web_workflow_name": "Solid_feeding_vials"}')
        material_id_m = self._get_material_id_by_name(assign_material_name)

        if isinstance(temperature, str):
            temperature = float(temperature)

        feeding_id = WORKFLOW_STEP_IDS["solid_feeding_vials"]["feeding"]
        observe_id = WORKFLOW_STEP_IDS["solid_feeding_vials"]["observe"]

        solid_feeding_vials_params = {
            "param_values": {
                feeding_id: [
                    {"m": 0, "n": 3, "Key": "materialId", "Value": material_id},
                    {"m": 0, "n": 3, "Key": "assignMaterialName", "Value": material_id_m}
                ],
                observe_id: [
                    {"m": 1, "n": 0, "Key": "time", "Value": time},
                    {"m": 1, "n": 0, "Key": "torqueVariation", "Value": torque_variation},
                    {"m": 1, "n": 0, "Key": "temperature", "Value": f"{temperature:.2f}"}
                ]
            }
        }

        self.pending_task_params.append(solid_feeding_vials_params)
        print(f"成功添加固体进料小瓶参数: material_id={material_id}, time={time}min, temp={temperature:.2f}°C")
        print(f"当前队列长度: {len(self.pending_task_params)}")
        return json.dumps({"suc": True})

    def liquid_feeding_vials_non_titration(self, volumeFormula: str, assign_material_name: str,
                                          titration_type: str = "1", time: str = "0",
                                          torque_variation: str = "1", temperature: float = 25.00):
        """液体进料小瓶(非滴定)"""
        self.append_to_workflow_sequence('{"web_workflow_name": "Liquid_feeding_vials(non-titration)"}')
        material_id = self._get_material_id_by_name(assign_material_name)

        if isinstance(temperature, str):
            temperature = float(temperature)

        liquid_id = WORKFLOW_STEP_IDS["liquid_feeding_vials_non_titration"]["liquid"]
        observe_id = WORKFLOW_STEP_IDS["liquid_feeding_vials_non_titration"]["observe"]

        params = {
            "param_values": {
                liquid_id: [
                    {"m": 0, "n": 3, "Key": "volumeFormula", "Value": volumeFormula},
                    {"m": 0, "n": 3, "Key": "assignMaterialName", "Value": material_id},
                    {"m": 0, "n": 3, "Key": "titrationType", "Value": titration_type}
                ],
                observe_id: [
                    {"m": 1, "n": 0, "Key": "time", "Value": time},
                    {"m": 1, "n": 0, "Key": "torqueVariation", "Value": torque_variation},
                    {"m": 1, "n": 0, "Key": "temperature", "Value": f"{temperature:.2f}"}
                ]
            }
        }

        self.pending_task_params.append(params)
        print(f"成功添加液体进料小瓶(非滴定)参数: volume={volumeFormula}μL, material={assign_material_name}->ID:{material_id}")
        print(f"当前队列长度: {len(self.pending_task_params)}")
        return json.dumps({"suc": True})

    def liquid_feeding_solvents(self, assign_material_name: str, volume: str, titration_type: str = "1",
                               time: str = "360", torque_variation: str = "2", temperature: float = 25.00):
        """液体进料溶剂"""
        self.append_to_workflow_sequence('{"web_workflow_name": "Liquid_feeding_solvents"}')
        material_id = self._get_material_id_by_name(assign_material_name)

        if isinstance(temperature, str):
            temperature = float(temperature)

        liquid_id = WORKFLOW_STEP_IDS["liquid_feeding_solvents"]["liquid"]
        observe_id = WORKFLOW_STEP_IDS["liquid_feeding_solvents"]["observe"]

        params = {
            "param_values": {
                liquid_id: [
                    {"m": 0, "n": 1, "Key": "titrationType", "Value": titration_type},
                    {"m": 0, "n": 1, "Key": "volume", "Value": volume},
                    {"m": 0, "n": 1, "Key": "assignMaterialName", "Value": material_id}
                ],
                observe_id: [
                    {"m": 1, "n": 0, "Key": "time", "Value": time},
                    {"m": 1, "n": 0, "Key": "torqueVariation", "Value": torque_variation},
                    {"m": 1, "n": 0, "Key": "temperature", "Value": f"{temperature:.2f}"}
                ]
            }
        }

        self.pending_task_params.append(params)
        print(f"成功添加液体进料溶剂参数: material={assign_material_name}->ID:{material_id}, volume={volume}μL")
        print(f"当前队列长度: {len(self.pending_task_params)}")
        return json.dumps({"suc": True})

    def liquid_feeding_titration(self, volume_formula: str, assign_material_name: str, titration_type: str = "1",
                                time: str = "90", torque_variation: int = 2, temperature: float = 25.00):
        """液体进料(滴定)"""
        self.append_to_workflow_sequence('{"web_workflow_name": "Liquid_feeding(titration)"}')
        material_id = self._get_material_id_by_name(assign_material_name)

        if isinstance(temperature, str):
            temperature = float(temperature)

        liquid_id = WORKFLOW_STEP_IDS["liquid_feeding_titration"]["liquid"]
        observe_id = WORKFLOW_STEP_IDS["liquid_feeding_titration"]["observe"]

        params = {
            "param_values": {
                liquid_id: [
                    {"m": 0, "n": 3, "Key": "volumeFormula", "Value": volume_formula},
                    {"m": 0, "n": 3, "Key": "titrationType", "Value": titration_type},
                    {"m": 0, "n": 3, "Key": "assignMaterialName", "Value": material_id}
                ],
                observe_id: [
                    {"m": 1, "n": 0, "Key": "time", "Value": time},
                    {"m": 1, "n": 0, "Key": "torqueVariation", "Value": str(torque_variation)},
                    {"m": 1, "n": 0, "Key": "temperature", "Value": f"{temperature:.2f}"}
                ]
            }
        }

        self.pending_task_params.append(params)
        print(f"成功添加液体进料滴定参数: volume={volume_formula}μL, material={assign_material_name}->ID:{material_id}")
        print(f"当前队列长度: {len(self.pending_task_params)}")
        return json.dumps({"suc": True})

    def liquid_feeding_beaker(self, volume: str = "35000", assign_material_name: str = "BAPP",
                             time: str = "0", torque_variation: str = "1", titrationType: str = "1",
                             temperature: float = 25.00):
        """液体进料烧杯"""
        self.append_to_workflow_sequence('{"web_workflow_name": "liquid_feeding_beaker"}')
        material_id = self._get_material_id_by_name(assign_material_name)

        if isinstance(temperature, str):
            temperature = float(temperature)

        liquid_id = WORKFLOW_STEP_IDS["liquid_feeding_beaker"]["liquid"]
        observe_id = WORKFLOW_STEP_IDS["liquid_feeding_beaker"]["observe"]

        params = {
            "param_values": {
                liquid_id: [
                    {"m": 0, "n": 2, "Key": "volume", "Value": volume},
                    {"m": 0, "n": 2, "Key": "assignMaterialName", "Value": material_id},
                    {"m": 0, "n": 2, "Key": "titrationType", "Value": titrationType}
                ],
                observe_id: [
                    {"m": 1, "n": 0, "Key": "time", "Value": time},
                    {"m": 1, "n": 0, "Key": "torqueVariation", "Value": torque_variation},
                    {"m": 1, "n": 0, "Key": "temperature", "Value": f"{temperature:.2f}"}
                ]
            }
        }

        self.pending_task_params.append(params)
        print(f"成功添加液体进料烧杯参数: volume={volume}μL, material={assign_material_name}->ID:{material_id}")
        print(f"当前队列长度: {len(self.pending_task_params)}")
        return json.dumps({"suc": True})

    # 辅助方法
    def _load_material_cache(self):
        """预加载材料列表到缓存中"""
        try:
            print("正在加载材料列表缓存...")
            stock_query = '{"typeMode": 2, "includeDetail": true}'
            stock_result = self.stock_material(stock_query)

            if isinstance(stock_result, str):
                stock_data = json.loads(stock_result)
            else:
                stock_data = stock_result

            materials = stock_data
            for material in materials:
                material_name = material.get("name")
                material_id = material.get("id")
                if material_name and material_id:
                    self.material_cache[material_name] = material_id

            print(f"材料列表缓存加载完成，共加载 {len(self.material_cache)} 个材料")

        except Exception as e:
            print(f"加载材料列表缓存时出错: {e}")
            self.material_cache = {}

    def _get_material_id_by_name(self, material_name_or_id: str) -> str:
        """根据材料名称获取材料ID"""
        if len(material_name_or_id) > 20 and '-' in material_name_or_id:
            return material_name_or_id

        if material_name_or_id in self.material_cache:
            material_id = self.material_cache[material_name_or_id]
            print(f"从缓存找到材料: {material_name_or_id} -> ID: {material_id}")
            return material_id

        print(f"警告: 未在缓存中找到材料名称 '{material_name_or_id}'，将使用原值")
        return material_name_or_id

    def refresh_material_cache(self):
        """刷新材料列表缓存"""
        print("正在刷新材料列表缓存...")
        self._load_material_cache()

    def get_available_materials(self):
        """获取所有可用的材料名称列表"""
        return list(self.material_cache.keys())

    # 物料管理接口
    def add_material(self, json_str: str) -> dict:
        """添加新的物料"""
        try:
            params = json.loads(json_str)
        except:
            return {}

        response = self.post(
            url=f'{self.host}/api/lims/storage/material',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response or response['code'] != 1:
            return {}
        return response.get("data", {})

    def query_matial_type_id(self, data) -> list:
        """查找物料typeid"""
        response = self.post(
            url=f'{self.host}/api/lims/storage/material-types',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": data
            })

        if not response or response['code'] != 1:
            return []
        return str(response.get("data", {}))

    def query_warehouse_by_material_type(self, type_id: str) -> dict:
        """查询物料类型可以入库的库位"""
        params = {"typeId": type_id}

        response = self.post(
            url=f'{self.host}/api/lims/storage/warehouse-info-by-mat-type-id',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response or response['code'] != 1:
            return {}
        return response.get("data", {})

    def material_inbound(self, material_id: str, location_name: str) -> dict:
        """指定库位入库一个物料"""
        location_id = LOCATION_MAPPING.get(location_name, location_name)

        params = {
            "materialId": material_id,
            "locationId": location_id
        }

        response = self.post(
            url=f'{self.host}/api/lims/storage/inbound',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response or response['code'] != 1:
            return {}
        return response.get("data", {})

    def delete_material(self, material_id: str) -> dict:
        """删除尚未入库的物料"""
        response = self.post(
            url=f'{self.host}/api/lims/storage/delete-material',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": material_id
            })

        if not response or response['code'] != 1:
            return {}
        return response.get("data", {})

    def material_outbound(self, material_id: str, location_name: str, quantity: int) -> dict:
        """指定库位出库物料"""
        location_id = LOCATION_MAPPING.get(location_name, location_name)

        params = {
            "materialId": material_id,
            "locationId": location_id,
            "quantity": quantity
        }

        response = self.post(
            url=f'{self.host}/api/lims/storage/outbound',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response or response['code'] != 1:
            return {}
        return response

    def get_logger(self):
        return self._logger

    # ==================== 配液站特有方法 ====================

    def sample_waste_removal(self, order_id: str) -> dict:
        """
        样品/废料取出接口

        参数:
        - order_id: 订单ID

        返回: 取出结果
        """
        params = {"orderId": order_id}

        response = self.post(
            url=f'{self.host}/api/lims/order/take-out',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response:
            return {}

        if response['code'] != 1:
            self._logger.error(f"样品废料取出错误: {response.get('message', '')}")
            return {}

        return response.get("data", {})

    def dispensing_material_inbound(self, material_id: str, location_id: str) -> dict:
        """
        配液站物料入库接口

        参数:
        - material_id: 物料ID
        - location_id: 库位ID

        返回: 入库结果
        """
        params = {
            "materialId": material_id,
            "locationId": location_id
        }

        response = self.post(
            url=f'{self.host}/api/lims/storage/inbound',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response:
            return {}

        if response['code'] != 1:
            self._logger.error(f"配液站物料入库错误: {response.get('message', '')}")
            return {}

        return response.get("data", {})

    def dispensing_material_outbound(self, material_id: str, location_id: str, quantity: int) -> dict:
        """
        配液站物料出库接口

        参数:
        - material_id: 物料ID
        - location_id: 库位ID
        - quantity: 出库数量

        返回: 出库结果
        """
        params = {
            "materialId": material_id,
            "locationId": location_id,
            "quantity": quantity
        }

        response = self.post(
            url=f'{self.host}/api/lims/storage/outbound',
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": params
            })

        if not response:
            return {}

        if response['code'] != 1:
            self._logger.error(f"配液站物料出库错误: {response.get('message', '')}")
            return {}

        return response.get("data", {})

    def create_90_10_vial_feeding_task(self,
                                       order_name: str = None,
                                       speed: str = None,
                                       temperature: str = None,
                                       delay_time: str = None,
                                       percent_90_1_assign_material_name: str = None,
                                       percent_90_1_target_weigh: str = None,
                                       percent_90_2_assign_material_name: str = None,
                                       percent_90_2_target_weigh: str = None,
                                       percent_90_3_assign_material_name: str = None,
                                       percent_90_3_target_weigh: str = None,
                                       percent_10_1_assign_material_name: str = None,
                                       percent_10_1_target_weigh: str = None,
                                       percent_10_1_volume: str = None,
                                       percent_10_1_liquid_material_name: str = None,
                                       percent_10_2_assign_material_name: str = None,
                                       percent_10_2_target_weigh: str = None,
                                       percent_10_2_volume: str = None,
                                       percent_10_2_liquid_material_name: str = None,
                                       percent_10_3_assign_material_name: str = None,
                                       percent_10_3_target_weigh: str = None,
                                       percent_10_3_volume: str = None,
                                       percent_10_3_liquid_material_name: str = None,
                                       hold_m_name: str = None) -> dict:
        """
        创建90%10%小瓶投料任务

        参数说明:
        - order_name: 任务名称，如果为None则使用默认名称
        - speed: 搅拌速度，如果为None则使用默认值400
        - temperature: 温度，如果为None则使用默认值40
        - delay_time: 延迟时间，如果为None则使用默认值600
        - percent_90_1_assign_material_name: 90%_1物料名称
        - percent_90_1_target_weigh: 90%_1目标重量
        - percent_90_2_assign_material_name: 90%_2物料名称
        - percent_90_2_target_weigh: 90%_2目标重量
        - percent_90_3_assign_material_name: 90%_3物料名称
        - percent_90_3_target_weigh: 90%_3目标重量
        - percent_10_1_assign_material_name: 10%_1固体物料名称
        - percent_10_1_target_weigh: 10%_1固体目标重量
        - percent_10_1_volume: 10%_1液体体积
        - percent_10_1_liquid_material_name: 10%_1液体物料名称
        - percent_10_2_assign_material_name: 10%_2固体物料名称
        - percent_10_2_target_weigh: 10%_2固体目标重量
        - percent_10_2_volume: 10%_2液体体积
        - percent_10_2_liquid_material_name: 10%_2液体物料名称
        - percent_10_3_assign_material_name: 10%_3固体物料名称
        - percent_10_3_target_weigh: 10%_3固体目标重量
        - percent_10_3_volume: 10%_3液体体积
        - percent_10_3_liquid_material_name: 10%_3液体物料名称
        - hold_m_name: 库位名称，如"C01"，用于查找对应的holdMId

        返回: 任务创建结果
        """
        # 设置默认值
        if order_name is None:
            order_name = f"90%10%小瓶投料任务_{self.get_current_time_iso8601()}"
        if speed is None:
            speed = "400"
        if temperature is None:
            temperature = "20"
        if delay_time is None:
            delay_time = "600"

        # 获取工作流ID
        workflow_id = "3a19310d-16b9-9d81-b109-0748e953694b"  # 90%10%小瓶投料工作流ID

        # 查询holdMId
        holdMId = None
        if hold_m_name:
            holdMId_response = self.material_id_query(hold_m_name)
            if holdMId_response:
                holdMId = holdMId_response

        # 构建订单数据
        order_data = [{
            "code": order_name,
            "Name": "90%10%小瓶投料任务",
            "workflowName": "90%10%小瓶投料",
            "borderNumber": 1,
            "paramValues": {
                workflow_id: [
                    # 搅拌速度
                    {"m": 3, "n": 2, "key": "speed", "value": speed},
                    # 温度
                    {"m": 3, "n": 2, "key": "temperature", "value": temperature},
                    # 延迟时间
                    {"m": 3, "n": 2, "key": "delayTime", "value": delay_time},
                    # 90%_1固体物料
                    {"m": 3, "n": 2, "key": "90%_1_assignMaterialName", "value": percent_90_1_assign_material_name},
                    {"m": 3, "n": 2, "key": "90%_1_targetWeigh", "value": percent_90_1_target_weigh},
                    # 90%_2固体物料
                    {"m": 3, "n": 2, "key": "90%_2_assignMaterialName", "value": percent_90_2_assign_material_name},
                    {"m": 3, "n": 2, "key": "90%_2_targetWeigh", "value": percent_90_2_target_weigh},
                    # 90%_3固体物料
                    {"m": 3, "n": 2, "key": "90%_3_assignMaterialName", "value": percent_90_3_assign_material_name},
                    {"m": 3, "n": 2, "key": "90%_3_targetWeigh", "value": percent_90_3_target_weigh},
                    # 10%_1液体物料
                    {"m": 3, "n": 2, "key": "10%_1_assignMaterialName", "value": percent_10_1_assign_material_name},
                    {"m": 3, "n": 2, "key": "10%_1_targetWeigh", "value": percent_10_1_target_weigh},
                    {"m": 3, "n": 2, "key": "10%_1_volume", "value": percent_10_1_volume},
                    {"m": 3, "n": 2, "key": "10%_1_liquidMaterialName", "value": percent_10_1_liquid_material_name},
                    # 10%_2液体物料
                    {"m": 3, "n": 2, "key": "10%_2_assignMaterialName", "value": percent_10_2_assign_material_name},
                    {"m": 3, "n": 2, "key": "10%_2_targetWeigh", "value": percent_10_2_target_weigh},
                    {"m": 3, "n": 2, "key": "10%_2_volume", "value": percent_10_2_volume},
                    {"m": 3, "n": 2, "key": "10%_2_liquidMaterialName", "value": percent_10_2_liquid_material_name},
                    # 10%_3液体物料
                    {"m": 3, "n": 2, "key": "10%_3_assignMaterialName", "value": percent_10_3_assign_material_name},
                    {"m": 3, "n": 2, "key": "10%_3_targetWeigh", "value": percent_10_3_target_weigh},
                    {"m": 3, "n": 2, "key": "10%_3_volume", "value": percent_10_3_volume},
                    {"m": 3, "n": 2, "key": "10%_3_liquidMaterialName", "value": percent_10_3_liquid_material_name}
                ]
            },
            "ExtendProperties": f"{{{holdMId}:null}}" if holdMId else "{}"
        }]

        try:
            # 调用create_order方法创建任务
            result = self.create_order(json.dumps(order_data, ensure_ascii=False))
            self._logger.info(f"90%10%小瓶投料任务创建成功: {result}")
            return result

        except Exception as e:
            error_msg = f"90%10%小瓶投料任务创建异常: {str(e)}"
            self._logger.error(error_msg)
            return {"error": error_msg}

    def create_diamine_solution_task(self,
                                    order_name: str = None,
                                    material_name: str = None,
                                    target_weigh: str = None,
                                    volume: str = None,
                                    liquid_material_name: str = "NMP",
                                    speed: str = None,
                                    temperature: str = None,
                                    delay_time: str = None,
                                    hold_m_name: str = None) -> dict:
        """
        创建二胺溶液配置任务

        参数说明:
        - order_name: 任务名称，如果为None则使用默认名称
        - material_name: 固体物料名称，必填
        - target_weigh: 固体目标重量，必填
        - volume: 液体体积，必填
        - liquid_material_name: 液体物料名称，默认为NMP
        - speed: 搅拌速度，如果为None则使用默认值400
        - temperature: 温度，如果为None则使用默认值25
        - delay_time: 延迟时间，如果为None则使用默认值600
        - hold_m_name: 库位名称，如"ODA-1"，用于查找对应的holdMId

        返回: 任务创建结果
        """
        # 验证必填参数
        if not material_name or not target_weigh or not volume:
            return {
                "status": "error",
                "message": "material_name、target_weigh和volume为必填参数"
            }

        # 设置默认值
        if order_name is None:
            order_name = f"二胺溶液配置任务_{self.get_current_time_iso8601()}"
        if speed is None:
            speed = "400"
        if temperature is None:
            temperature = "25"
        if delay_time is None:
            delay_time = "600"

        # 获取工作流ID
        workflow_id = "1"

        # 查询holdMId
        holdMId = None
        if hold_m_name:
            try:
                material_query_params = json.dumps({"materialName": hold_m_name})
                material_response = self.material_id_query(material_query_params)
                if material_response and material_response.get("code") == 1:
                    data = material_response.get("data", [])
                    if data:
                        holdMId = data[0].get("id")
                        self._logger.info(f"查询到holdMId: {holdMId} for {hold_m_name}")
                    else:
                        self._logger.warning(f"未找到物料: {hold_m_name}")
                else:
                    self._logger.error(f"查询物料ID失败: {material_response}")
            except Exception as e:
                self._logger.error(f"查询holdMId时发生错误: {e}")

        # 构建order_data
        order_data = {
            "workflowId": workflow_id,
            "orderName": order_name,
            "params": {
                "1": speed,  # 搅拌速度
                "2": temperature,  # 温度
                "3": delay_time,  # 延迟时间
                "4": material_name,  # 固体物料名称
                "5": target_weigh,  # 固体目标重量
                "6": volume,  # 液体体积
                "7": liquid_material_name  # 液体物料名称
            }
        }

        if holdMId:
            order_data["holdMId"] = holdMId

        try:
            # 使用create_order方法创建任务
            order_params = json.dumps(order_data)
            response = self.create_order(order_params)
            return response
        except Exception as e:
            self._logger.error(f"创建二胺溶液配置任务时发生错误: {e}")
            return {"status": "error", "message": f"创建任务失败: {str(e)}"}

    def create_batch_90_10_vial_feeding_task(self, json_str: str) -> dict:
        """
        创建批量90%10%小瓶投料任务

        接受JSON输入，支持多个90%10%小瓶投料任务的批量创建

        JSON格式示例:
        {
            "batch_name": "批量90%10%小瓶投料任务_20240101",
            "tasks": [
                {
                    "order_name": "小瓶投料任务1",
                    "hold_m_name": "C01",
                    "percent_90_1_assign_material_name": "物料A",
                    "percent_90_1_target_weigh": "10.5",
                    "percent_10_1_assign_material_name": "物料B",
                    "percent_10_1_target_weigh": "5.2",
                    "percent_10_1_volume": "50.0",
                    "percent_10_1_liquid_material_name": "NMP",
                    "speed": "400",
                    "temperature": "40",
                    "delay_time": "600"
                }
            ],
            "global_settings": {
                "speed": "400",
                "temperature": "40",
                "delay_time": "600"
            }
        }

        参数说明:
        - batch_name: 批量任务名称，可选
        - tasks: 任务列表，每个任务包含90%10%小瓶投料参数
        - global_settings: 全局默认设置，当单个任务未指定参数时使用

        返回: 批量任务创建结果
        """
        try:
            # 解析JSON输入
            data = json.loads(json_str)

            # 获取批量任务参数
            batch_name = data.get("batch_name", f"批量90%10%小瓶投料任务_{self.get_current_time_iso8601()}")
            tasks = data.get("tasks", [])
            global_settings = data.get("global_settings", {})

            if not tasks:
                return {
                    "status": "error",
                    "message": "任务列表不能为空"
                }

            # 批量创建结果
            batch_results = {
                "batch_name": batch_name,
                "total_tasks": len(tasks),
                "successful_tasks": 0,
                "failed_tasks": 0,
                "task_results": []
            }

            self._logger.info(f"开始创建批量90%10%小瓶投料任务: {batch_name}, 包含 {len(tasks)} 个子任务")

            # 逐个创建任务
            for i, task in enumerate(tasks):
                try:
                    # 合并全局设置和任务特定设置
                    task_params = {**global_settings, **task}

                    # 验证必填参数 - hold_m_name是必须的
                    if not task_params.get("hold_m_name"):
                        error_msg = f"任务 {i+1} 缺少必填参数: hold_m_name"
                        self._logger.error(error_msg)
                        batch_results["task_results"].append({
                            "task_index": i + 1,
                            "status": "error",
                            "message": error_msg
                        })
                        batch_results["failed_tasks"] += 1
                        continue

                    # 设置任务名称
                    if not task_params.get("order_name"):
                        task_params["order_name"] = f"{batch_name}_任务{i+1}"

                    # 调用单个90%10%小瓶投料任务创建方法
                    task_result = self.create_90_10_vial_feeding_task(
                        order_name=task_params.get("order_name"),
                        speed=task_params.get("speed"),
                        temperature=task_params.get("temperature"),
                        delay_time=task_params.get("delay_time"),
                        percent_90_1_assign_material_name=task_params.get("percent_90_1_assign_material_name"),
                        percent_90_1_target_weigh=task_params.get("percent_90_1_target_weigh"),
                        percent_90_2_assign_material_name=task_params.get("percent_90_2_assign_material_name"),
                        percent_90_2_target_weigh=task_params.get("percent_90_2_target_weigh"),
                        percent_90_3_assign_material_name=task_params.get("percent_90_3_assign_material_name"),
                        percent_90_3_target_weigh=task_params.get("percent_90_3_target_weigh"),
                        percent_10_1_assign_material_name=task_params.get("percent_10_1_assign_material_name"),
                        percent_10_1_target_weigh=task_params.get("percent_10_1_target_weigh"),
                        percent_10_1_volume=task_params.get("percent_10_1_volume"),
                        percent_10_1_liquid_material_name=task_params.get("percent_10_1_liquid_material_name"),
                        percent_10_2_assign_material_name=task_params.get("percent_10_2_assign_material_name"),
                        percent_10_2_target_weigh=task_params.get("percent_10_2_target_weigh"),
                        percent_10_2_volume=task_params.get("percent_10_2_volume"),
                        percent_10_2_liquid_material_name=task_params.get("percent_10_2_liquid_material_name"),
                        percent_10_3_assign_material_name=task_params.get("percent_10_3_assign_material_name"),
                        percent_10_3_target_weigh=task_params.get("percent_10_3_target_weigh"),
                        percent_10_3_volume=task_params.get("percent_10_3_volume"),
                        percent_10_3_liquid_material_name=task_params.get("percent_10_3_liquid_material_name"),
                        hold_m_name=task_params.get("hold_m_name")
                    )

                    # 记录任务结果
                    if isinstance(task_result, dict) and task_result.get("status") != "error":
                        batch_results["successful_tasks"] += 1
                        batch_results["task_results"].append({
                            "task_index": i + 1,
                            "task_name": task_params.get("order_name"),
                            "status": "success",
                            "result": task_result
                        })
                        self._logger.info(f"任务 {i+1} 创建成功: {task_params.get('order_name')}")
                    else:
                        batch_results["failed_tasks"] += 1
                        batch_results["task_results"].append({
                            "task_index": i + 1,
                            "task_name": task_params.get("order_name"),
                            "status": "error",
                            "message": str(task_result)
                        })
                        self._logger.error(f"任务 {i+1} 创建失败: {task_result}")

                except Exception as e:
                    error_msg = f"任务 {i+1} 处理时发生异常: {str(e)}"
                    self._logger.error(error_msg)
                    batch_results["failed_tasks"] += 1
                    batch_results["task_results"].append({
                        "task_index": i + 1,
                        "status": "error",
                        "message": error_msg
                    })

            # 设置批量任务整体状态
            if batch_results["failed_tasks"] == 0:
                batch_results["status"] = "success"
                batch_results["message"] = f"批量90%10%小瓶投料任务全部创建成功，共 {batch_results['successful_tasks']} 个任务"
            elif batch_results["successful_tasks"] == 0:
                batch_results["status"] = "error"
                batch_results["message"] = f"批量90%10%小瓶投料任务全部创建失败，共 {batch_results['failed_tasks']} 个任务"
            else:
                batch_results["status"] = "partial_success"
                batch_results["message"] = f"批量90%10%小瓶投料任务部分成功，成功 {batch_results['successful_tasks']} 个，失败 {batch_results['failed_tasks']} 个"

            self._logger.info(f"批量90%10%小瓶投料任务完成: {batch_results['message']}")
            return batch_results

        except json.JSONDecodeError as e:
            error_msg = f"JSON解析失败: {str(e)}"
            self._logger.error(error_msg)
            return {"status": "error", "message": error_msg}
        except Exception as e:
            error_msg = f"创建批量90%10%小瓶投料任务时发生错误: {str(e)}"
            self._logger.error(error_msg)
            return {"status": "error", "message": error_msg}

    def create_batch_diamine_solution_task(self, json_str: str) -> dict:
        """
        创建批量二胺溶液配制任务

        接受JSON输入，支持多个二胺溶液配制任务的批量创建

        JSON格式示例:
        {
            "batch_name": "批量二胺溶液配制任务_20240101",
            "tasks": [
                {
                    "order_name": "二胺溶液配制任务1",
                    "material_name": "物料A",
                    "target_weigh": "10.5",
                    "volume": "50.0",
                    "liquid_material_name": "NMP",
                    "speed": "400",
                    "temperature": "25",
                    "delay_time": "600",
                    "hold_m_name": "A01"
                },
                {
                    "order_name": "二胺溶液配制任务2",
                    "material_name": "物料B",
                    "target_weigh": "15.2",
                    "volume": "75.0",
                    "liquid_material_name": "DMF",
                    "speed": "350",
                    "temperature": "30",
                    "delay_time": "800",
                    "hold_m_name": "B02"
                }
            ],
            "global_settings": {
                "speed": "400",
                "temperature": "25",
                "delay_time": "600",
                "liquid_material_name": "NMP"
            }
        }

        参数说明:
        - batch_name: 批量任务名称，可选
        - tasks: 任务列表，每个任务包含二胺溶液配制参数
        - global_settings: 全局默认设置，当单个任务未指定参数时使用

        每个任务参数:
        - order_name: 任务名称
        - material_name: 物料名称，必填
        - target_weigh: 目标重量，必填
        - volume: 体积，必填
        - liquid_material_name: 液体物料名称，可选
        - speed: 搅拌速度，可选
        - temperature: 温度，可选
        - delay_time: 延迟时间，可选
        - hold_m_name: 库位名称，可选

        返回: 批量任务创建结果
        """
        try:
            # 解析JSON输入
            data = json.loads(json_str)

            # 获取批量任务参数
            batch_name = data.get("batch_name", f"批量二胺溶液配制任务_{self.get_current_time_iso8601()}")
            tasks = data.get("tasks", [])
            global_settings = data.get("global_settings", {})

            if not tasks:
                return {
                    "status": "error",
                    "message": "任务列表不能为空"
                }

            # 批量创建结果
            batch_results = {
                "batch_name": batch_name,
                "total_tasks": len(tasks),
                "successful_tasks": 0,
                "failed_tasks": 0,
                "task_results": []
            }

            self._logger.info(f"开始创建批量二胺溶液配制任务: {batch_name}, 包含 {len(tasks)} 个子任务")

            # 逐个创建任务
            for i, task in enumerate(tasks):
                try:
                    # 合并全局设置和任务特定设置
                    task_params = {**global_settings, **task}

                    # 验证必填参数
                    required_params = ["material_name", "target_weigh", "volume"]
                    missing_params = [param for param in required_params if not task_params.get(param)]

                    if missing_params:
                        error_msg = f"任务 {i+1} 缺少必填参数: {', '.join(missing_params)}"
                        self._logger.error(error_msg)
                        batch_results["task_results"].append({
                            "task_index": i + 1,
                            "status": "error",
                            "message": error_msg
                        })
                        batch_results["failed_tasks"] += 1
                        continue

                    # 设置任务名称
                    if not task_params.get("order_name"):
                        task_params["order_name"] = f"{batch_name}_任务{i+1}"

                    # 调用单个二胺溶液配制任务创建方法
                    task_result = self.create_diamine_solution_task(
                        order_name=task_params.get("order_name"),
                        material_name=task_params.get("material_name"),
                        target_weigh=task_params.get("target_weigh"),
                        volume=task_params.get("volume"),
                        liquid_material_name=task_params.get("liquid_material_name", "NMP"),
                        speed=task_params.get("speed"),
                        temperature=task_params.get("temperature"),
                        delay_time=task_params.get("delay_time"),
                        hold_m_name=task_params.get("hold_m_name")
                    )

                    # 记录任务结果
                    if isinstance(task_result, dict) and task_result.get("status") != "error":
                        batch_results["successful_tasks"] += 1
                        batch_results["task_results"].append({
                            "task_index": i + 1,
                            "task_name": task_params.get("order_name"),
                            "status": "success",
                            "result": task_result
                        })
                        self._logger.info(f"任务 {i+1} 创建成功: {task_params.get('order_name')}")
                    else:
                        batch_results["failed_tasks"] += 1
                        batch_results["task_results"].append({
                            "task_index": i + 1,
                            "task_name": task_params.get("order_name"),
                            "status": "error",
                            "message": str(task_result)
                        })
                        self._logger.error(f"任务 {i+1} 创建失败: {task_result}")

                except Exception as e:
                    error_msg = f"滴定液任务 {i+1} 处理时发生异常: {str(e)}"
                    self._logger.error(error_msg)
                    batch_results["failed_tasks"] += 1
                    batch_results["task_results"].append({
                        "task_index": i + 1,
                        "status": "error",
                        "message": error_msg
                    })

            # 设置批量任务整体状态
            if batch_results["failed_tasks"] == 0:
                batch_results["status"] = "success"
                batch_results["message"] = f"批量滴定液任务全部创建成功，共 {batch_results['successful_tasks']} 个任务"
            elif batch_results["successful_tasks"] == 0:
                batch_results["status"] = "error"
                batch_results["message"] = f"批量滴定液任务全部创建失败，共 {batch_results['failed_tasks']} 个任务"
            else:
                batch_results["status"] = "partial_success"
                batch_results["message"] = f"批量滴定液任务部分成功，成功 {batch_results['successful_tasks']} 个，失败 {batch_results['failed_tasks']} 个"

            self._logger.info(f"批量滴定液任务完成: {batch_results['message']}")
            return batch_results

        except json.JSONDecodeError as e:
            error_msg = f"JSON解析失败: {str(e)}"
            self._logger.error(error_msg)
            return {"status": "error", "message": error_msg}
        except Exception as e:
            error_msg = f"创建批量滴定液任务时发生错误: {str(e)}"
            self._logger.error(error_msg)
            return {"status": "error", "message": error_msg}
