import json
import requests
from typing import List, Dict, Any
from unilabos.devices.workstation.bioyond_studio.station import BioyondWorkstation
from unilabos.devices.workstation.bioyond_studio.config import (
    WORKFLOW_STEP_IDS,
    WORKFLOW_TO_SECTION_MAP,
    ACTION_NAMES
)
from unilabos.devices.workstation.bioyond_studio.config import API_CONFIG


class BioyondReactionStation(BioyondWorkstation):
    """Bioyond反应站类
    
    继承自BioyondWorkstation，提供反应站特定的业务方法
    """
    
    def __init__(self, config: dict = None, deck=None, protocol_type=None, **kwargs):
        """初始化反应站
        
        Args:
            config: 配置字典，应包含workflow_mappings等配置
            deck: Deck对象
            protocol_type: 协议类型（由ROS系统传递，此处忽略）
            **kwargs: 其他可能的参数
        """
        if deck is None and config:
            deck = config.get('deck')
        
        print(f"BioyondReactionStation初始化 - config包含workflow_mappings: {'workflow_mappings' in (config or {})}")
        if config and 'workflow_mappings' in config:
            print(f"workflow_mappings内容: {config['workflow_mappings']}")
        
        super().__init__(bioyond_config=config, deck=deck)
        
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
            assign_material_name: 物料名称（不能为空）
            cutoff: 截止值/通量配置（需为有效数字字符串，默认 "900000"）
            temperature: 温度上限（°C，范围：-50.00 至 100.00）
        
        Returns:
            str: JSON 字符串，格式为 {"suc": True}
        
        Raises:
            ValueError: 若物料名称无效或 cutoff 格式错误
        """
        if not assign_material_name:
            raise ValueError("物料名称不能为空")
        try:
            float(cutoff)
        except ValueError:
            raise ValueError("cutoff 必须是有效的数字字符串")
        
        self.append_to_workflow_sequence('{"web_workflow_name": "reactor_taken_in"}')
        material_id = self.hardware_interface._get_material_id_by_name(assign_material_name)
        if material_id is None:
            raise ValueError(f"无法找到物料 {assign_material_name} 的 ID")

        if isinstance(temperature, str):
            temperature = float(temperature)

        step_id = WORKFLOW_STEP_IDS["reactor_taken_in"]["config"]
        reactor_taken_in_params = {
            "param_values": {
                step_id: {
                    ACTION_NAMES["reactor_taken_in"]["config"]: [
                        {"m": 0, "n": 3, "Key": "cutoff", "Value": cutoff},
                        {"m": 0, "n": 3, "Key": "assignMaterialName", "Value": material_id}
                    ],
                    ACTION_NAMES["reactor_taken_in"]["stirring"]: [
                        {"m": 0, "n": 3, "Key": "temperature", "Value": f"{temperature:.2f}"}
                    ]
                }
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
        torque_variation: int = 1,
        assign_material_name: str = None, 
        temperature: float = 25.00
    ):
        """固体进料小瓶
        
        Args:
            material_id: 粉末类型ID
            time: 观察时间(分钟)
            torque_variation: 是否观察扭矩变化(int类型, 1=否, 2=是)
            assign_material_name: 物料名称(用于获取试剂瓶位ID)
            temperature: 温度上限(°C)
        """
        self.append_to_workflow_sequence('{"web_workflow_name": "Solid_feeding_vials"}')
        material_id_m = self.hardware_interface._get_material_id_by_name(assign_material_name) if assign_material_name else None

        if isinstance(temperature, str):
            temperature = float(temperature)

        feeding_step_id = WORKFLOW_STEP_IDS["solid_feeding_vials"]["feeding"]
        observe_step_id = WORKFLOW_STEP_IDS["solid_feeding_vials"]["observe"]
        
        solid_feeding_vials_params = {
            "param_values": {
                feeding_step_id: {
                    ACTION_NAMES["solid_feeding_vials"]["feeding"]: [
                        {"m": 0, "n": 3, "Key": "materialId", "Value": material_id},
                        {"m": 0, "n": 3, "Key": "assignMaterialName", "Value": material_id_m} if material_id_m else {}
                    ]
                },
                observe_step_id: {
                    ACTION_NAMES["solid_feeding_vials"]["observe"]: [
                        {"m": 1, "n": 0, "Key": "time", "Value": time},
                        {"m": 1, "n": 0, "Key": "torqueVariation", "Value": str(torque_variation)},
                        {"m": 1, "n": 0, "Key": "temperature", "Value": f"{temperature:.2f}"}
                    ]
                }
            }
        }

        self.pending_task_params.append(solid_feeding_vials_params)
        print(f"成功添加固体进料小瓶参数: material_id={material_id}, time={time}min, torque={torque_variation}, temp={temperature:.2f}°C")
        print(f"当前队列长度: {len(self.pending_task_params)}")
        return json.dumps({"suc": True})

    def liquid_feeding_vials_non_titration(
        self, 
        volume_formula: str,
        assign_material_name: str,
        titration_type: str = "1",
        time: str = "0",
        torque_variation: int = 1, 
        temperature: float = 25.00
    ):
        """液体进料小瓶(非滴定)
        
        Args:
            volume_formula: 分液公式(μL)
            assign_material_name: 物料名称
            titration_type: 是否滴定(1=滴定, 其他=非滴定)
            time: 观察时间(分钟)
            torque_variation: 是否观察扭矩变化(int类型, 1=否, 2=是)
            temperature: 温度(°C)
        """
        self.append_to_workflow_sequence('{"web_workflow_name": "Liquid_feeding_vials(non-titration)"}')
        material_id = self.hardware_interface._get_material_id_by_name(assign_material_name)
        if material_id is None:
            raise ValueError(f"无法找到物料 {assign_material_name} 的 ID")

        if isinstance(temperature, str):
            temperature = float(temperature)

        liquid_step_id = WORKFLOW_STEP_IDS["liquid_feeding_vials_non_titration"]["liquid"]
        observe_step_id = WORKFLOW_STEP_IDS["liquid_feeding_vials_non_titration"]["observe"]
        
        params = {
            "param_values": {
                liquid_step_id: {
                    ACTION_NAMES["liquid_feeding_vials_non_titration"]["liquid"]: [
                        {"m": 0, "n": 3, "Key": "volumeFormula", "Value": volume_formula},
                        {"m": 0, "n": 3, "Key": "assignMaterialName", "Value": material_id},
                        {"m": 0, "n": 3, "Key": "titrationType", "Value": titration_type}
                    ]
                },
                observe_step_id: {
                    ACTION_NAMES["liquid_feeding_vials_non_titration"]["observe"]: [
                        {"m": 1, "n": 0, "Key": "time", "Value": time},
                        {"m": 1, "n": 0, "Key": "torqueVariation", "Value": str(torque_variation)},
                        {"m": 1, "n": 0, "Key": "temperature", "Value": f"{temperature:.2f}"}
                    ]
                }
            }
        }

        self.pending_task_params.append(params)
        print(f"成功添加液体进料小瓶(非滴定)参数: volume={volume_formula}μL, material={assign_material_name}->ID:{material_id}")
        print(f"当前队列长度: {len(self.pending_task_params)}")
        return json.dumps({"suc": True})

    def liquid_feeding_solvents(
        self, 
        assign_material_name: str, 
        volume: str, 
        titration_type: str = "1",
        time: str = "360", 
        torque_variation: int = 2, 
        temperature: float = 25.00
    ):
        """液体进料-溶剂
        
        Args:
            assign_material_name: 物料名称
            volume: 分液量(μL)
            titration_type: 是否滴定
            time: 观察时间(分钟)
            torque_variation: 是否观察扭矩变化(int类型, 1=否, 2=是)
            temperature: 温度上限(°C)
        """
        self.append_to_workflow_sequence('{"web_workflow_name": "Liquid_feeding_solvents"}')
        material_id = self.hardware_interface._get_material_id_by_name(assign_material_name)
        if material_id is None:
            raise ValueError(f"无法找到物料 {assign_material_name} 的 ID")

        if isinstance(temperature, str):
            temperature = float(temperature)
        
        liquid_step_id = WORKFLOW_STEP_IDS["liquid_feeding_solvents"]["liquid"]
        observe_step_id = WORKFLOW_STEP_IDS["liquid_feeding_solvents"]["observe"]
        
        params = {
            "param_values": {
                liquid_step_id: {
                    ACTION_NAMES["liquid_feeding_solvents"]["liquid"]: [
                        {"m": 0, "n": 1, "Key": "titrationType", "Value": titration_type},
                        {"m": 0, "n": 1, "Key": "volume", "Value": volume},
                        {"m": 0, "n": 1, "Key": "assignMaterialName", "Value": material_id}
                    ]
                },
                observe_step_id: {
                    ACTION_NAMES["liquid_feeding_solvents"]["observe"]: [
                        {"m": 1, "n": 0, "Key": "time", "Value": time},
                        {"m": 1, "n": 0, "Key": "torqueVariation", "Value": str(torque_variation)},
                        {"m": 1, "n": 0, "Key": "temperature", "Value": f"{temperature:.2f}"}
                    ]
                }
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
            volume_formula: 分液公式(μL)
            assign_material_name: 物料名称
            titration_type: 是否滴定
            time: 观察时间(分钟)
            torque_variation: 是否观察扭矩变化(int类型, 1=否, 2=是)
            temperature: 温度(°C)
        """
        self.append_to_workflow_sequence('{"web_workflow_name": "Liquid_feeding(titration)"}')
        material_id = self.hardware_interface._get_material_id_by_name(assign_material_name)
        if material_id is None:
            raise ValueError(f"无法找到物料 {assign_material_name} 的 ID")

        if isinstance(temperature, str):
            temperature = float(temperature)

        liquid_step_id = WORKFLOW_STEP_IDS["liquid_feeding_titration"]["liquid"]
        observe_step_id = WORKFLOW_STEP_IDS["liquid_feeding_titration"]["observe"]

        params = {
            "param_values": {
                liquid_step_id: {
                    ACTION_NAMES["liquid_feeding_titration"]["liquid"]: [
                        {"m": 0, "n": 3, "Key": "volumeFormula", "Value": volume_formula},
                        {"m": 0, "n": 3, "Key": "titrationType", "Value": titration_type},
                        {"m": 0, "n": 3, "Key": "assignMaterialName", "Value": material_id}
                    ]
                },
                observe_step_id: {
                    ACTION_NAMES["liquid_feeding_titration"]["observe"]: [
                        {"m": 1, "n": 0, "Key": "time", "Value": time},
                        {"m": 1, "n": 0, "Key": "torqueVariation", "Value": str(torque_variation)},
                        {"m": 1, "n": 0, "Key": "temperature", "Value": f"{temperature:.2f}"}
                    ]
                }
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
        torque_variation: int = 1, 
        titration_type: str = "1",
        temperature: float = 25.00
    ):
        """液体进料烧杯
        
        Args:
            volume: 分液量(μL)
            assign_material_name: 物料名称(试剂瓶位)
            time: 观察时间(分钟)
            torque_variation: 是否观察扭矩变化(int类型, 1=否, 2=是)
            titration_type: 是否滴定
            temperature: 温度上限(°C)
        """
        self.append_to_workflow_sequence('{"web_workflow_name": "liquid_feeding_beaker"}')
        material_id = self.hardware_interface._get_material_id_by_name(assign_material_name)
        if material_id is None:
            raise ValueError(f"无法找到物料 {assign_material_name} 的 ID")

        if isinstance(temperature, str):
            temperature = float(temperature)

        liquid_step_id = WORKFLOW_STEP_IDS["liquid_feeding_beaker"]["liquid"]
        observe_step_id = WORKFLOW_STEP_IDS["liquid_feeding_beaker"]["observe"]

        params = {
            "param_values": {
                liquid_step_id: {
                    ACTION_NAMES["liquid_feeding_beaker"]["liquid"]: [
                        {"m": 0, "n": 2, "Key": "volume", "Value": volume},
                        {"m": 0, "n": 2, "Key": "assignMaterialName", "Value": material_id},
                        {"m": 0, "n": 2, "Key": "titrationType", "Value": titration_type}
                    ]
                },
                observe_step_id: {
                    ACTION_NAMES["liquid_feeding_beaker"]["observe"]: [
                        {"m": 1, "n": 0, "Key": "time", "Value": time},
                        {"m": 1, "n": 0, "Key": "torqueVariation", "Value": str(torque_variation)},
                        {"m": 1, "n": 0, "Key": "temperature", "Value": f"{temperature:.2f}"}
                    ]
                }
            }
        }

        self.pending_task_params.append(params)
        print(f"成功添加液体进料烧杯参数: volume={volume}μL, material={assign_material_name}->ID:{material_id}")
        print(f"当前队列长度: {len(self.pending_task_params)}")
        return json.dumps({"suc": True})
    
    def drip_back(
        self,
        assign_material_name: str,
        volume: str,
        titration_type: str = "1",
        time: str = "90",
        torque_variation: int = 2,
        temperature: float = 25.00
    ):
        """滴回去
        
        Args:
            assign_material_name: 物料名称(液体种类)
            volume: 分液量(μL)
            titration_type: 是否滴定
            time: 观察时间(分钟)
            torque_variation: 是否观察扭矩变化(int类型, 1=否, 2=是)
            temperature: 温度(°C)
        """
        self.append_to_workflow_sequence('{"web_workflow_name": "drip_back"}')
        material_id = self.hardware_interface._get_material_id_by_name(assign_material_name)
        if material_id is None:
            raise ValueError(f"无法找到物料 {assign_material_name} 的 ID")

        if isinstance(temperature, str):
            temperature = float(temperature)

        liquid_step_id = WORKFLOW_STEP_IDS["drip_back"]["liquid"]
        observe_step_id = WORKFLOW_STEP_IDS["drip_back"]["observe"]

        params = {
            "param_values": {
                liquid_step_id: {
                    ACTION_NAMES["drip_back"]["liquid"]: [
                        {"m": 0, "n": 1, "Key": "titrationType", "Value": titration_type},
                        {"m": 0, "n": 1, "Key": "assignMaterialName", "Value": material_id},
                        {"m": 0, "n": 1, "Key": "volume", "Value": volume}
                    ]
                },
                observe_step_id: {
                    ACTION_NAMES["drip_back"]["observe"]: [
                        {"m": 1, "n": 0, "Key": "time", "Value": time},
                        {"m": 1, "n": 0, "Key": "torqueVariation", "Value": str(torque_variation)},
                        {"m": 1, "n": 0, "Key": "temperature", "Value": f"{temperature:.2f}"}
                    ]
                }
            }
        }

        self.pending_task_params.append(params)
        print(f"成功添加滴回去参数: material={assign_material_name}->ID:{material_id}, volume={volume}μL")
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
            workflow_name = id_to_name.get(workflow_id, workflow_id)
            workflow_names.append(workflow_name)
        print(f"工作流序列: {workflow_names}")
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

    def process_web_workflows(self, web_workflow_json: str) -> List[Dict[str, str]]:
        """处理网页工作流列表
        
        Args:
            web_workflow_json: JSON 格式的网页工作流列表
            
        Returns:
            List[Dict[str, str]]: 包含工作流 ID 和名称的字典列表
        """
        try:
            web_workflow_data = json.loads(web_workflow_json)
            web_workflow_list = web_workflow_data.get("web_workflow_list", [])
            workflows_result = []
            for name in web_workflow_list:
                workflow_id = self.workflow_mappings.get(name, "")
                if not workflow_id:
                    print(f"警告：未找到工作流名称 {name} 对应的 ID")
                    continue
                workflows_result.append({"id": workflow_id, "name": name})
            print(f"process_web_workflows 输出: {workflows_result}")
            return workflows_result
        except json.JSONDecodeError as e:
            print(f"错误：无法解析 web_workflow_json: {e}")
            return []
        except Exception as e:
            print(f"错误：处理工作流失败: {e}")
            return []

    def process_and_execute_workflow(self, workflow_name: str, task_name: str) -> dict:
        """
        一站式处理工作流程：解析网页工作流列表，合并工作流(带参数)，然后发布任务
        
        Args:
            workflow_name: 合并后的工作流名称
            task_name: 任务名称
            
        Returns:
            任务创建结果
        """
        web_workflow_list = self.get_workflow_sequence()
        print(f"\n{'='*60}")
        print(f"📋 处理网页工作流列表: {web_workflow_list}")
        print(f"{'='*60}")
        
        web_workflow_json = json.dumps({"web_workflow_list": web_workflow_list})
        workflows_result = self.process_web_workflows(web_workflow_json)
        
        if not workflows_result:
            return self._create_error_result("处理网页工作流列表失败", "process_web_workflows")
        
        print(f"workflows_result 类型: {type(workflows_result)}")
        print(f"workflows_result 内容: {workflows_result}")
        
        workflows_with_params = self._build_workflows_with_parameters(workflows_result)
        
        merge_data = {
            "name": workflow_name,
            "workflows": workflows_with_params
        }
        
        # print(f"\n🔄 合并工作流（带参数），名称: {workflow_name}")
        merged_workflow = self.merge_workflow_with_parameters(json.dumps(merge_data))
        
        if not merged_workflow:
            return self._create_error_result("合并工作流失败", "merge_workflow_with_parameters")
        
        workflow_id = merged_workflow.get("subWorkflows", [{}])[0].get("id", "")
        # print(f"\n📤 使用工作流创建任务: {workflow_name} (ID: {workflow_id})")
        
        order_params = [{
            "orderCode": f"task_{self.hardware_interface.get_current_time_iso8601()}",
            "orderName": task_name,
            "workFlowId": workflow_id,
            "borderNumber": 1,
            "paramValues": {}
        }]
        
        result = self.create_order(json.dumps(order_params))
        
        if not result:
            return self._create_error_result("创建任务失败", "create_order")
        
        self.pending_task_params = []
        
        # print(f"\n✅ 任务创建成功: {result}")
        # print(f"\n✅ 任务创建成功")
        print(f"{'='*60}\n")
        return json.dumps({"success": True, "result": result})

    def _build_workflows_with_parameters(self, workflows_result: list) -> list:
        """
        构建带参数的工作流列表
        
        Args:
            workflows_result: 处理后的工作流列表（应为包含 id 和 name 的字典列表）
            
        Returns:
            符合新接口格式的工作流参数结构
        """
        workflows_with_params = []
        total_params = 0
        successful_params = 0
        failed_params = []

        for idx, workflow_info in enumerate(workflows_result):
            if not isinstance(workflow_info, dict):
                print(f"错误：workflows_result[{idx}] 不是字典，而是 {type(workflow_info)}: {workflow_info}")
                continue
            workflow_id = workflow_info.get("id")
            if not workflow_id:
                print(f"警告：workflows_result[{idx}] 缺少 'id' 键")
                continue
            workflow_name = workflow_info.get("name", "")
            # print(f"\n🔧 处理工作流 [{idx}]: {workflow_name} (ID: {workflow_id})")
            
            if idx >= len(self.pending_task_params):
                # print(f"   ⚠️ 无对应参数，跳过")
                workflows_with_params.append({"id": workflow_id})
                continue
            
            param_data = self.pending_task_params[idx]
            param_values = param_data.get("param_values", {})
            if not param_values:
                # print(f"   ⚠️ 参数为空，跳过")
                workflows_with_params.append({"id": workflow_id})
                continue
            
            step_parameters = {}
            for step_id, actions_dict in param_values.items():
                # print(f"   📍 步骤ID: {step_id}")
                for action_name, param_list in actions_dict.items():
                    # print(f"      🔹 模块: {action_name}, 参数数量: {len(param_list)}")
                    if step_id not in step_parameters:
                        step_parameters[step_id] = {}
                    if action_name not in step_parameters[step_id]:
                        step_parameters[step_id][action_name] = []
                    for param_item in param_list:
                        param_key = param_item.get("Key", "")
                        param_value = param_item.get("Value", "")
                        total_params += 1
                        step_parameters[step_id][action_name].append({
                            "Key": param_key,
                            "DisplayValue": param_value
                        })
                        successful_params += 1
                        # print(f"         ✓ {param_key} = {param_value}")

            workflows_with_params.append({
                "id": workflow_id,
                "stepParameters": step_parameters
            })

        self._print_mapping_stats(total_params, successful_params, failed_params)
        return workflows_with_params

    def _print_mapping_stats(self, total: int, success: int, failed: list):
        """打印参数映射统计"""
        print(f"\n{'='*20} 参数映射统计 {'='*20}")
        print(f"📊 总参数数量: {total}")
        print(f"✅ 成功映射: {success}")
        print(f"❌ 映射失败: {len(failed)}")
        if not failed:
            print("🎉 成功映射所有参数！")
        else:
            print(f"⚠️ 失败的参数: {', '.join(failed)}")
        success_rate = (success/total*100) if total > 0 else 0
        print(f"📈 映射成功率: {success_rate:.1f}%")
        print("="*60)

    def _create_error_result(self, error_msg: str, step: str) -> str:
        """创建统一的错误返回格式"""
        print(f"❌ {error_msg}")
        return json.dumps({
            "success": False,
            "error": f"process_and_execute_workflow: {error_msg}",
            "method": "process_and_execute_workflow",
            "step": step
        })

    def merge_workflow_with_parameters(self, json_str: str) -> dict:
        """
        调用新接口：合并工作流并传递参数
        
        Args:
            json_str: JSON格式的字符串，包含:
                - name: 工作流名称
                - workflows: [{"id": "工作流ID", "stepParameters": {...}}]
                
        Returns:
            合并后的工作流信息
        """
        try:
            data = json.loads(json_str)
            
            # 在工作流名称后面添加时间戳，避免重复
            if "name" in data and data["name"]:
                timestamp = self.hardware_interface.get_current_time_iso8601().replace(":", "-").replace(".", "-")
                original_name = data["name"]
                data["name"] = f"{original_name}_{timestamp}"
                print(f"🕒 工作流名称已添加时间戳: {original_name} -> {data['name']}")
            
            request_data = {
                "apiKey": API_CONFIG["api_key"],
                "requestTime": self.hardware_interface.get_current_time_iso8601(),
                "data": data
            }
            print(f"\n📤 发送合并请求:")
            print(f"   工作流名称: {data.get('name')}")
            print(f"   子工作流数量: {len(data.get('workflows', []))}")
            
            # 打印完整的POST请求内容
            print(f"\n🔍 POST请求详细内容:")
            print(f"   URL: {self.hardware_interface.host}/api/lims/workflow/merge-workflow-with-parameters")
            print(f"   Headers: {{'Content-Type': 'application/json'}}")
            print(f"   Request Data:")
            print(f"   {json.dumps(request_data, indent=4, ensure_ascii=False)}")
            # 
            response = requests.post(
                f"{self.hardware_interface.host}/api/lims/workflow/merge-workflow-with-parameters",
                json=request_data,
                headers={"Content-Type": "application/json"},
                timeout=30
            )
            
            # # 打印响应详细内容
            # print(f"\n📥 POST响应详细内容:")
            # print(f"   状态码: {response.status_code}")
            # print(f"   响应头: {dict(response.headers)}")
            # print(f"   响应体: {response.text}")
            # # 
            try:
                result = response.json()
                # # 
                # print(f"\n📋 解析后的响应JSON:")
                # print(f"   {json.dumps(result, indent=4, ensure_ascii=False)}")
                # # 
            except json.JSONDecodeError:
                print(f"❌ 服务器返回非 JSON 格式响应: {response.text}")
                return None
            
            if result.get("code") == 1:
                print(f"✅ 工作流合并成功（带参数）")
                return result.get("data", {})
            else:
                error_msg = result.get('message', '未知错误')
                print(f"❌ 工作流合并失败: {error_msg}")
                return None
                
        except requests.exceptions.Timeout:
            print(f"❌ 合并工作流请求超时")
            return None
        except requests.exceptions.RequestException as e:
            print(f"❌ 合并工作流网络异常: {str(e)}")
            return None
        except json.JSONDecodeError as e:
            print(f"❌ 合并工作流响应解析失败: {str(e)}")
            return None
        except Exception as e:
            print(f"❌ 合并工作流异常: {str(e)}")
            return None

    def _validate_and_refresh_workflow_if_needed(self, workflow_name: str) -> bool:
        """验证工作流ID是否有效，如果无效则重新合并
        
        Args:
            workflow_name: 工作流名称
            
        Returns:
            bool: 验证或刷新是否成功
        """
        print(f"\n🔍 验证工作流ID有效性...")
        if not self.workflow_sequence:
            print(f"   ⚠️ 工作流序列为空，需要重新合并")
            return False
        first_workflow_id = self.workflow_sequence[0]
        try:
            structure = self.workflow_step_query(first_workflow_id)
            if structure:
                print(f"   ✅ 工作流ID有效")
                return True
            else:
                print(f"   ⚠️ 工作流ID已过期，需要重新合并")
                return False
        except Exception as e:
            print(f"   ❌ 工作流ID验证失败: {e}")
            print(f"   💡 将重新合并工作流")
            return False