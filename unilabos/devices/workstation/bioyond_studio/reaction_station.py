import json
from typing import List, Dict, Any
from unilabos.devices.workstation.bioyond_studio.station import BioyondWorkstation
from unilabos.devices.workstation.bioyond_studio.config import (
    WORKFLOW_STEP_IDS,
    WORKFLOW_TO_SECTION_MAP
)


class BioyondReactionStation(BioyondWorkstation):
    """Bioyond反应站类
    
    继承自BioyondWorkstation,提供反应站特定的业务方法
    """
    
    def __init__(self, config: dict = None, deck=None):
        """初始化反应站
        
        Args:
            config: 配置字典,应包含workflow_mappings等配置
            deck: Deck对象
        """
        # 如果 deck 作为独立参数传入,使用它;否则尝试从 config 中提取
        if deck is None and config:
            deck = config.get('deck')
        
        # 调试信息:检查传入的config
        print(f"BioyondReactionStation初始化 - config包含workflow_mappings: {'workflow_mappings' in (config or {})}")
        if config and 'workflow_mappings' in config:
            print(f"workflow_mappings内容: {config['workflow_mappings']}")
        
        # 将 config 作为 bioyond_config 传递给父类
        super().__init__(bioyond_config=config, deck=deck)
        
        # 调试信息:检查初始化后的workflow_mappings
        print(f"BioyondReactionStation初始化完成 - workflow_mappings: {self.workflow_mappings}")
        print(f"workflow_mappings长度: {len(self.workflow_mappings)}")

    # ==================== 工作流方法 ====================

    def reactor_taken_out(self):
        """反应器取出"""
        self.append_to_workflow_sequence('{"web_workflow_name": "reactor_taken_out"}')
        reactor_taken_out_params = {"param_values": {}}
        self.pending_task_params.append(reactor_taken_out_params)
        print(f"成功添加反应器取出工作流")
        print(f"当前队列长度: {len(self.pending_task_params)}")
        return json.dumps({"suc": True})

    def reactor_taken_in(
        self, 
        assign_material_name: str, 
        cutoff: str = "900000", 
        temperature: float = -10.00
    ):
        """反应器放入
        
        Args:
            assign_material_name: 物料名称
            cutoff: 截止参数
            temperature: 温度
        """
        self.append_to_workflow_sequence('{"web_workflow_name": "reactor_taken_in"}')
        material_id = self.hardware_interface._get_material_id_by_name(assign_material_name)

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

    def solid_feeding_vials(
        self, 
        material_id: str, 
        time: str = "0", 
        torque_variation: str = "1",
        assign_material_name: str = None, 
        temperature: float = 25.00
    ):
        """固体进料小瓶
        
        Args:
            material_id: 物料ID
            time: 时间
            torque_variation: 扭矩变化
            assign_material_name: 物料名称
            temperature: 温度
        """
        self.append_to_workflow_sequence('{"web_workflow_name": "Solid_feeding_vials"}')
        material_id_m = self.hardware_interface._get_material_id_by_name(assign_material_name)

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

    def liquid_feeding_vials_non_titration(
        self, 
        volumeFormula: str, 
        assign_material_name: str,
        titration_type: str = "1", 
        time: str = "0",
        torque_variation: str = "1", 
        temperature: float = 25.00
    ):
        """液体进料小瓶(非滴定)
        
        Args:
            volumeFormula: 体积公式
            assign_material_name: 物料名称
            titration_type: 滴定类型
            time: 时间
            torque_variation: 扭矩变化
            temperature: 温度
        """
        self.append_to_workflow_sequence('{"web_workflow_name": "Liquid_feeding_vials(non-titration)"}')
        material_id = self.hardware_interface._get_material_id_by_name(assign_material_name)

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

    def liquid_feeding_solvents(
        self, 
        assign_material_name: str, 
        volume: str, 
        titration_type: str = "1",
        time: str = "360", 
        torque_variation: str = "2", 
        temperature: float = 25.00
    ):
        """液体进料溶剂
        
        Args:
            assign_material_name: 物料名称
            volume: 体积
            titration_type: 滴定类型
            time: 时间
            torque_variation: 扭矩变化
            temperature: 温度
        """
        self.append_to_workflow_sequence('{"web_workflow_name": "Liquid_feeding_solvents"}')
        material_id = self.hardware_interface._get_material_id_by_name(assign_material_name)

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

    def liquid_feeding_titration(
        self, 
        volume_formula: str, 
        assign_material_name: str, 
        titration_type: str = "1",
        time: str = "90", 
        torque_variation: int = 2, 
        temperature: float = 25.00
    ):
        """液体进料(滴定)
        
        Args:
            volume_formula: 体积公式
            assign_material_name: 物料名称
            titration_type: 滴定类型
            time: 时间
            torque_variation: 扭矩变化
            temperature: 温度
        """
        self.append_to_workflow_sequence('{"web_workflow_name": "Liquid_feeding(titration)"}')
        material_id = self.hardware_interface._get_material_id_by_name(assign_material_name)

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

    def liquid_feeding_beaker(
        self, 
        volume: str = "35000", 
        assign_material_name: str = "BAPP",
        time: str = "0", 
        torque_variation: str = "1", 
        titrationType: str = "1",
        temperature: float = 25.00
    ):
        """液体进料烧杯
        
        Args:
            volume: 体积
            assign_material_name: 物料名称
            time: 时间
            torque_variation: 扭矩变化
            titrationType: 滴定类型
            temperature: 温度
        """
        self.append_to_workflow_sequence('{"web_workflow_name": "liquid_feeding_beaker"}')
        material_id = self.hardware_interface._get_material_id_by_name(assign_material_name)

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

    # ==================== 工作流管理方法 ====================

    def get_workflow_sequence(self) -> List[str]:
        """获取当前工作流执行顺序
        
        Returns:
            工作流名称列表
        """
        id_to_name = {workflow_id: name for name, workflow_id in self.workflow_mappings.items()}
        workflow_names = []
        for workflow_id in self.workflow_sequence:
            workflow_names.append(id_to_name.get(workflow_id, workflow_id))
        return workflow_names

    def workflow_step_query(self, workflow_id: str) -> dict:
        """查询工作流步骤参数
        
        Args:
            workflow_id: 工作流ID
            
        Returns:
            工作流步骤参数字典
        """
        return self.hardware_interface.workflow_step_query(workflow_id)

    def create_order(self, json_str: str) -> dict:
        """创建订单
        
        Args:
            json_str: 订单参数的JSON字符串
            
        Returns:
            创建结果
        """
        return self.hardware_interface.create_order(json_str)

    # ==================== 工作流执行核心方法 ====================

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
        # 新API只返回状态信息，需要适配处理
        if isinstance(merged_workflow, dict) and merged_workflow.get("code") == 1:
            # 新API返回格式：{code: 1, message: "", timestamp: 0}
            # 使用传入的工作流名称和生成的临时ID
            final_workflow_name = workflow_name
            workflow_id = f"merged_{workflow_name}_{self.hardware_interface.get_current_time_iso8601().replace('-', '').replace('T', '').replace(':', '').replace('.', '')[:14]}"
            print(f"新API合并成功，使用工作流创建任务: {final_workflow_name} (临时ID: {workflow_id})")
        else:
            # 旧API返回格式：包含详细工作流信息
            final_workflow_name = merged_workflow.get("name", workflow_name)
            workflow_id = merged_workflow.get("subWorkflows", [{}])[0].get("id", "")
            print(f"旧API格式，使用工作流创建任务: {final_workflow_name} (ID: {workflow_id})")
            
        if not workflow_id:
            error_msg = "无法获取工作流ID"
            print(error_msg)
            result = str({"success": False, "error": f"process_and_execute_workflow:{error_msg}", "method": "process_and_execute_workflow", "step": "get_workflow_id"})
            return result

        workflow_query_json = json.dumps({"workflow_id": workflow_id})
        workflow_params_structure = self.workflow_step_query(workflow_query_json)

        self.pending_task_params = pending_params_backup
        print(f"恢复pending_task_params，共{len(self.pending_task_params)}个参数")

        param_values = self.generate_task_param_values(workflow_params_structure)

        task_params = [{
            "orderCode": f"BSO{self.hardware_interface.get_current_time_iso8601().replace('-', '').replace('T', '').replace(':', '').replace('.', '')[:14]}",
            "orderName": f"实验-{self.hardware_interface.get_current_time_iso8601()[:10].replace('-', '')}",
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
            "workflow": {"name": final_workflow_name, "id": workflow_id},
            "task": result,
            "method": "process_and_execute_workflow"
        }

    def merge_sequence_workflow(self, json_str: str) -> dict:
        """合并当前工作流序列
        
        Args:
            json_str: 包含name等参数的JSON字符串
            
        Returns:
            合并结果
        """
        try:
            data = json.loads(json_str)
            name = data.get("name", "合并工作流")
            step_parameters = data.get("stepParameters", {})
            variables = data.get("variables", {})
        except json.JSONDecodeError:
            return {}

        if not self.workflow_sequence:
            print("工作流序列为空,无法合并")
            return {}

        # 将工作流ID列表转换为新API要求的格式
        workflows = [{"id": workflow_id} for workflow_id in self.workflow_sequence]
        
        # 构建新的API参数格式
        params = {
            "name": name,
            "workflows": workflows,
            "stepParameters": step_parameters,
            "variables": variables
        }

        # 使用新的API接口
        response = self.hardware_interface.post(
            url=f'{self.hardware_interface.host}/api/lims/workflow/merge-workflow-with-parameters',
            params={
                "apiKey": self.hardware_interface.api_key,
                "requestTime": self.hardware_interface.get_current_time_iso8601(),
                "data": params,
            })

        if not response or response['code'] != 1:
            return {}
        return response.get("data", {})

    def generate_task_param_values(self, workflow_params_structure: dict) -> dict:
        """生成任务参数值
        
        根据工作流参数结构和待处理的任务参数,生成最终的任务参数值
        
        Args:
            workflow_params_structure: 工作流参数结构
            
        Returns:
            任务参数值字典
        """
        if not workflow_params_structure:
            print("workflow_params_structure为空")
            return {}

        data = workflow_params_structure

        # 从pending_task_params中提取实际参数值,按DisplaySectionName和Key组织
        pending_params_by_section = {}
        print(f"开始处理pending_task_params,共{len(self.pending_task_params)}个任务参数组")

        # 获取工作流执行顺序,用于按顺序匹配参数
        workflow_sequence = self.get_workflow_sequence()
        print(f"工作流执行顺序: {workflow_sequence}")

        workflow_index = 0

        # 遍历所有待处理的任务参数
        for i, task_param in enumerate(self.pending_task_params):
            if 'param_values' in task_param:
                print(f"处理第{i+1}个任务参数组,包含{len(task_param['param_values'])}个步骤")

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

                # 处理每个步骤的参数
                for step_id, param_list in task_param['param_values'].items():
                    print(f"    步骤ID: {step_id},参数数量: {len(param_list)}")

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

        print(f"pending_params_by_section构建完成,包含{len(pending_params_by_section)}个分组")

        # 收集所有参数,过滤TaskDisplayable为0的项
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
                        print(f"      匹配失败: {section_name}.{key},参数列表为空,使用默认值 = {value}")
                else:
                    value = "1"
                    m = display_section_index
                    n = step_n
                    print(f"      匹配失败: {section_name}.{key},使用默认值 = {value} (m={m}, n={n})")

                param_item = {
                    "m": m,
                    "n": n,
                    "key": key,
                    "value": str(value).strip()
                }
                param_list.append(param_item)

            if param_list:
                param_mapping[step_id] = param_list

        print(f"生成任务参数值,包含 {len(param_mapping)} 个步骤")
        return param_mapping