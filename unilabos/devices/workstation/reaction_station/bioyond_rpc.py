# bioyond_rpc.py
"""
BioyondV1RPC类定义 - 包含所有RPC接口和业务逻辑
"""

from enum import Enum
from datetime import datetime, timezone
from unilabos.device_comms.rpc import BaseRequest
from typing import Optional, List, Dict, Any
import json
from config import WORKFLOW_TO_SECTION_MAP, WORKFLOW_STEP_IDS, LOCATION_MAPPING


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