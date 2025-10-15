from datetime import datetime
import json

from unilabos.devices.workstation.bioyond_studio.bioyond_rpc import BioyondException
from unilabos.devices.workstation.bioyond_studio.station import BioyondWorkstation


class BioyondDispensingStation(BioyondWorkstation):
    def __init__(
        self, 
        config,
            # 桌子
        deck,
        *args,
        **kwargs,
        ):
        super().__init__(config, deck, *args, **kwargs)
        # self.config = config
        # self.api_key = config["api_key"]
        # self.host = config["api_host"]
        #
        # # 使用简单的Logger替代原来的logger
        # self._logger = SimpleLogger()
        # self.is_running = False

    # 90%10%小瓶投料任务创建方法
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
        
        异常:
        - BioyondException: 各种错误情况下的统一异常
        """
        try:
            # 1. 参数验证
            if not hold_m_name:
                raise BioyondException("hold_m_name 是必填参数")
            
            # 检查90%物料参数的完整性
            # 90%_1物料：如果有物料名称或目标重量，就必须有全部参数
            if percent_90_1_assign_material_name or percent_90_1_target_weigh:
                if not percent_90_1_assign_material_name:
                    raise BioyondException("90%_1物料：如果提供了目标重量，必须同时提供物料名称")
                if not percent_90_1_target_weigh:
                    raise BioyondException("90%_1物料：如果提供了物料名称，必须同时提供目标重量")
            
            # 90%_2物料：如果有物料名称或目标重量，就必须有全部参数
            if percent_90_2_assign_material_name or percent_90_2_target_weigh:
                if not percent_90_2_assign_material_name:
                    raise BioyondException("90%_2物料：如果提供了目标重量，必须同时提供物料名称")
                if not percent_90_2_target_weigh:
                    raise BioyondException("90%_2物料：如果提供了物料名称，必须同时提供目标重量")
            
            # 90%_3物料：如果有物料名称或目标重量，就必须有全部参数
            if percent_90_3_assign_material_name or percent_90_3_target_weigh:
                if not percent_90_3_assign_material_name:
                    raise BioyondException("90%_3物料：如果提供了目标重量，必须同时提供物料名称")
                if not percent_90_3_target_weigh:
                    raise BioyondException("90%_3物料：如果提供了物料名称，必须同时提供目标重量")
            
            # 检查10%物料参数的完整性
            # 10%_1物料：如果有物料名称、目标重量、体积或液体物料名称中的任何一个，就必须有全部参数
            if any([percent_10_1_assign_material_name, percent_10_1_target_weigh, percent_10_1_volume, percent_10_1_liquid_material_name]):
                if not percent_10_1_assign_material_name:
                    raise BioyondException("10%_1物料：如果提供了其他参数，必须同时提供固体物料名称")
                if not percent_10_1_target_weigh:
                    raise BioyondException("10%_1物料：如果提供了其他参数，必须同时提供固体目标重量")
                if not percent_10_1_volume:
                    raise BioyondException("10%_1物料：如果提供了其他参数，必须同时提供液体体积")
                if not percent_10_1_liquid_material_name:
                    raise BioyondException("10%_1物料：如果提供了其他参数，必须同时提供液体物料名称")
            
            # 10%_2物料：如果有物料名称、目标重量、体积或液体物料名称中的任何一个，就必须有全部参数
            if any([percent_10_2_assign_material_name, percent_10_2_target_weigh, percent_10_2_volume, percent_10_2_liquid_material_name]):
                if not percent_10_2_assign_material_name:
                    raise BioyondException("10%_2物料：如果提供了其他参数，必须同时提供固体物料名称")
                if not percent_10_2_target_weigh:
                    raise BioyondException("10%_2物料：如果提供了其他参数，必须同时提供固体目标重量")
                if not percent_10_2_volume:
                    raise BioyondException("10%_2物料：如果提供了其他参数，必须同时提供液体体积")
                if not percent_10_2_liquid_material_name:
                    raise BioyondException("10%_2物料：如果提供了其他参数，必须同时提供液体物料名称")
            
            # 10%_3物料：如果有物料名称、目标重量、体积或液体物料名称中的任何一个，就必须有全部参数
            if any([percent_10_3_assign_material_name, percent_10_3_target_weigh, percent_10_3_volume, percent_10_3_liquid_material_name]):
                if not percent_10_3_assign_material_name:
                    raise BioyondException("10%_3物料：如果提供了其他参数，必须同时提供固体物料名称")
                if not percent_10_3_target_weigh:
                    raise BioyondException("10%_3物料：如果提供了其他参数，必须同时提供固体目标重量")
                if not percent_10_3_volume:
                    raise BioyondException("10%_3物料：如果提供了其他参数，必须同时提供液体体积")
                if not percent_10_3_liquid_material_name:
                    raise BioyondException("10%_3物料：如果提供了其他参数，必须同时提供液体物料名称")
            
            # 2. 生成任务编码和设置默认值
            order_code = "task_vial_" + str(int(datetime.now().timestamp()))
            if order_name is None:
                order_name = "90%10%小瓶投料任务"
            if speed is None:
                speed = "400"
            if temperature is None:
                temperature = "40"
            if delay_time is None:
                delay_time = "600"
                
            # 3. 工作流ID
            workflow_id = "3a19310d-16b9-9d81-b109-0748e953694b"

            # 4. 查询工作流对应的holdMID
            material_info = self.hardware_interface.material_id_query(workflow_id)
            if not material_info:
                raise BioyondException(f"无法查询工作流 {workflow_id} 的物料信息")
            
            # 获取locations列表
            locations = material_info.get("locations", []) if isinstance(material_info, dict) else []
            if not locations:
                raise BioyondException(f"工作流 {workflow_id} 没有找到库位信息")
            
            # 查找指定名称的库位
            hold_mid = None
            for location in locations:
                if location.get("holdMName") == hold_m_name:
                    hold_mid = location.get("holdMId")
                    break
            
            if not hold_mid:
                raise BioyondException(f"未找到库位名称为 {hold_m_name} 的库位，请检查名称是否正确")
            
            extend_properties = f"{{\"{ hold_mid }\": {{}}}}"
            self.hardware_interface._logger.info(f"找到库位 {hold_m_name} 对应的holdMId: {hold_mid}")

            # 5. 构建任务参数
            order_data = {
                "orderCode": order_code,
                "orderName": order_name,
                "workflowId": workflow_id,
                "borderNumber": 1,
                "paramValues": {},
                "ExtendProperties": extend_properties
            }

            # 添加搅拌参数
            order_data["paramValues"]["e8264e47-c319-d9d9-8676-4dd5cb382b11"] = [
                {"m": 0, "n": 3, "Key": "speed", "Value": speed},
                {"m": 0, "n": 3, "Key": "temperature", "Value": temperature}
            ]

            # 添加延迟时间参数
            order_data["paramValues"]["dc5dba79-5e4b-8eae-cbc5-e93482e43b1f"] = [
                {"m": 0, "n": 4, "Key": "DelayTime", "Value": delay_time}
            ]

            # 添加90%_1参数
            if percent_90_1_assign_material_name is not None and percent_90_1_target_weigh is not None:
                order_data["paramValues"]["e7d3c0a3-25c2-c42d-c84b-860c4a5ef844"] = [
                    {"m": 15, "n": 1, "Key": "targetWeigh", "Value": percent_90_1_target_weigh},
                    {"m": 15, "n": 1, "Key": "assignMaterialName", "Value": percent_90_1_assign_material_name}
                ]

            # 添加90%_2参数
            if percent_90_2_assign_material_name is not None and percent_90_2_target_weigh is not None:
                order_data["paramValues"]["50b912c4-6c81-0734-1c8b-532428b2a4a5"] = [
                    {"m": 18, "n": 1, "Key": "targetWeigh", "Value": percent_90_2_target_weigh},
                    {"m": 18, "n": 1, "Key": "assignMaterialName", "Value": percent_90_2_assign_material_name}
                ]

            # 添加90%_3参数
            if percent_90_3_assign_material_name is not None and percent_90_3_target_weigh is not None:
                order_data["paramValues"]["9c3674b3-c7cb-946e-fa03-fa2861d8aec4"] = [
                    {"m": 21, "n": 1, "Key": "targetWeigh", "Value": percent_90_3_target_weigh},
                    {"m": 21, "n": 1, "Key": "assignMaterialName", "Value": percent_90_3_assign_material_name}
                ]

            # 添加10%_1固体参数
            if percent_10_1_assign_material_name is not None and percent_10_1_target_weigh is not None:
                order_data["paramValues"]["73a0bfd8-1967-45e9-4bab-c07ccd1a2727"] = [
                    {"m": 3, "n": 1, "Key": "targetWeigh", "Value": percent_10_1_target_weigh},
                    {"m": 3, "n": 1, "Key": "assignMaterialName", "Value": percent_10_1_assign_material_name}
                ]

            # 添加10%_1液体参数
            if percent_10_1_liquid_material_name is not None and percent_10_1_volume is not None:
                order_data["paramValues"]["39634d40-c623-473a-8e5f-bc301aca2522"] = [
                    {"m": 3, "n": 3, "Key": "volume", "Value": percent_10_1_volume},
                    {"m": 3, "n": 3, "Key": "assignMaterialName", "Value": percent_10_1_liquid_material_name}
                ]

            # 添加10%_2固体参数
            if percent_10_2_assign_material_name is not None and percent_10_2_target_weigh is not None:
                order_data["paramValues"]["2d9c16fa-2a19-cd47-a67b-3cadff9e3e3d"] = [
                    {"m": 7, "n": 1, "Key": "targetWeigh", "Value": percent_10_2_target_weigh},
                    {"m": 7, "n": 1, "Key": "assignMaterialName", "Value": percent_10_2_assign_material_name}
                ]

            # 添加10%_2液体参数
            if percent_10_2_liquid_material_name is not None and percent_10_2_volume is not None:
                order_data["paramValues"]["e60541bb-ed68-e839-7305-2b4abe38a13d"] = [
                    {"m": 7, "n": 3, "Key": "volume", "Value": percent_10_2_volume},
                    {"m": 7, "n": 3, "Key": "assignMaterialName", "Value": percent_10_2_liquid_material_name}
                ]

            # 添加10%_3固体参数
            if percent_10_3_assign_material_name is not None and percent_10_3_target_weigh is not None:
                order_data["paramValues"]["27494733-0f71-a916-7cd2-1929a0125f17"] = [
                    {"m": 11, "n": 1, "Key": "targetWeigh", "Value": percent_10_3_target_weigh},
                    {"m": 11, "n": 1, "Key": "assignMaterialName", "Value": percent_10_3_assign_material_name}
                ]

            # 添加10%_3液体参数
            if percent_10_3_liquid_material_name is not None and percent_10_3_volume is not None:
                order_data["paramValues"]["c8798c29-786f-6858-7d7f-5330b890f2a6"] = [
                    {"m": 11, "n": 3, "Key": "volume", "Value": percent_10_3_volume},
                    {"m": 11, "n": 3, "Key": "assignMaterialName", "Value": percent_10_3_liquid_material_name}
                ]

            # 6. 转换为JSON字符串并创建任务
            json_str = json.dumps([order_data], ensure_ascii=False)
            self.hardware_interface._logger.info(f"创建90%10%小瓶投料任务参数: {json_str}")

            # 7. 调用create_order方法创建任务
            result = self.hardware_interface.create_order(json_str)
            self.hardware_interface._logger.info(f"创建90%10%小瓶投料任务结果: {result}")
            return json.dumps({"suc": True})
            
        except BioyondException:
            # 重新抛出BioyondException
            raise
        except Exception as e:
            # 捕获其他未预期的异常，转换为BioyondException
            error_msg = f"创建90%10%小瓶投料任务时发生未预期的错误: {str(e)}"
            self.hardware_interface._logger.error(error_msg)
            raise BioyondException(error_msg)

    # 二胺溶液配置任务创建方法
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
        - temperature: 温度，如果为None则使用默认值20
        - delay_time: 延迟时间，如果为None则使用默认值600
        - hold_m_name: 库位名称，如"ODA-1"，用于查找对应的holdMId

        返回: 任务创建结果
        
        异常:
        - BioyondException: 各种错误情况下的统一异常
        """
        try:
            # 1. 参数验证
            if not material_name:
                raise BioyondException("material_name 是必填参数")
            if not target_weigh:
                raise BioyondException("target_weigh 是必填参数")
            if not volume:
                raise BioyondException("volume 是必填参数")
            if not hold_m_name:
                raise BioyondException("hold_m_name 是必填参数")
            
            
            # 2. 生成任务编码和设置默认值
            order_code = "task_oda_" + str(int(datetime.now().timestamp()))
            if order_name is None:
                order_name = f"二胺溶液配置-{material_name}"
            if speed is None:
                speed = "400"
            if temperature is None:
                temperature = "20"
            if delay_time is None:
                delay_time = "600"
                
            # 3. 工作流ID - 二胺溶液配置工作流
            workflow_id = "3a15d4a1-3bbe-76f9-a458-292896a338f5"
            
            # 4. 查询工作流对应的holdMID
            material_info = self.material_id_query(workflow_id)
            if not material_info:
                raise BioyondException(f"无法查询工作流 {workflow_id} 的物料信息")
            
            # 获取locations列表
            locations = material_info.get("locations", []) if isinstance(material_info, dict) else []
            if not locations:
                raise BioyondException(f"工作流 {workflow_id} 没有找到库位信息")
            
            # 查找指定名称的库位
            hold_mid = None
            for location in locations:
                if location.get("holdMName") == hold_m_name:
                    hold_mid = location.get("holdMId")
                    break
            
            if not hold_mid:
                raise BioyondException(f"未找到库位名称为 {hold_m_name} 的库位，请检查名称是否正确")
            
            extend_properties = f"{{\"{ hold_mid }\": {{}}}}"
            self.hardware_interface._logger.info(f"找到库位 {hold_m_name} 对应的holdMId: {hold_mid}")

            # 5. 构建任务参数
            order_data = {
                "orderCode": order_code,
                "orderName": order_name,
                "workflowId": workflow_id,
                "borderNumber": 1,
                "paramValues": {
                    # 固体物料参数
                    "3a15d4a1-3bde-f5bc-053f-1ae0bf1f357e": [
                        {"m": 3, "n": 2, "Key": "targetWeigh", "Value": target_weigh},
                        {"m": 3, "n": 2, "Key": "assignMaterialName", "Value": material_name}
                    ],
                    # 液体物料参数
                    "3a15d4a1-3bde-d584-b309-e661ae8f1c01": [
                        {"m": 3, "n": 3, "Key": "volume", "Value": volume},
                        {"m": 3, "n": 3, "Key": "assignMaterialName", "Value": liquid_material_name}
                    ],
                    # 搅拌参数
                    "3a15d4a1-3bde-8ec4-1ced-92efc97ed73d": [
                        {"m": 3, "n": 6, "Key": "speed", "Value": speed},
                        {"m": 3, "n": 6, "Key": "temperature", "Value": temperature}
                    ],
                    # 延迟时间参数
                    "3a15d4a1-3bde-3b92-83ff-8923a0addbbc": [
                        {"m": 3, "n": 7, "Key": "DelayTime", "Value": delay_time}
                    ]
                },
                "ExtendProperties": extend_properties
            }

            # 6. 转换为JSON字符串并创建任务
            json_str = json.dumps([order_data], ensure_ascii=False)
            self.hardware_interface._logger.info(f"创建二胺溶液配置任务参数: {json_str}")

            # 7. 调用create_order方法创建任务
            result = self.hardware_interface.create_order(json_str)
            self.hardware_interface._logger.info(f"创建二胺溶液配置任务结果: {result}")
            
            return json.dumps({"suc": True})
            
        except BioyondException:
            # 重新抛出BioyondException
            raise
        except Exception as e:
            # 捕获其他未预期的异常，转换为BioyondException
            error_msg = f"创建二胺溶液配置任务时发生未预期的错误: {str(e)}"
            self.hardware_interface._logger.error(error_msg)
            raise BioyondException(error_msg)


if __name__ == "__main__":
    bioyond = BioyondDispensingStation(config={
        "api_key": "DE9BDDA0",
        "api_host": "http://192.168.1.200:44388"
    })
    
    # 示例1：使用material_id_query查询工作流对应的holdMID
    workflow_id_1 = "3a15d4a1-3bbe-76f9-a458-292896a338f5"  # 二胺溶液配置工作流ID
    workflow_id_2 = "3a19310d-16b9-9d81-b109-0748e953694b"  # 90%10%小瓶投料工作流ID
    
    #示例2：创建二胺溶液配置任务 - ODA，指定库位名称
    # bioyond.create_diamine_solution_task(
    #         order_code="task_oda_" + str(int(datetime.now().timestamp())),
    #         order_name="二胺溶液配置-ODA",
    #         material_name="ODA-1",
    #         target_weigh="12.000",
    #         volume="60",
    #         liquid_material_name= "NMP",
    #         speed="400",
    #         temperature="20",
    #         delay_time="600",
    #         hold_m_name="烧杯ODA"
    #     )
    
    # bioyond.create_diamine_solution_task(
    #         order_code="task_pda_" + str(int(datetime.now().timestamp())),
    #         order_name="二胺溶液配置-PDA",
    #         material_name="PDA-1",
    #         target_weigh="4.178",
    #         volume="60",
    #         liquid_material_name= "NMP",
    #         speed="400",
    #         temperature="20",
    #         delay_time="600",
    #         hold_m_name="烧杯PDA-2"
    #     )
    
    # bioyond.create_diamine_solution_task(
    #         order_code="task_mpda_" + str(int(datetime.now().timestamp())),
    #         order_name="二胺溶液配置-MPDA",
    #         material_name="MPDA-1",
    #         target_weigh="3.298",
    #         volume="50",
    #         liquid_material_name= "NMP",
    #         speed="400",
    #         temperature="20",
    #         delay_time="600",
    #         hold_m_name="烧杯MPDA"
    #     )

    bioyond.material_id_query("3a19310d-16b9-9d81-b109-0748e953694b")
    bioyond.material_id_query("3a15d4a1-3bbe-76f9-a458-292896a338f5")
    
    
    #示例4：创建90%10%小瓶投料任务
    # vial_result = bioyond.create_90_10_vial_feeding_task(
    #     order_code="task_vial_" + str(int(datetime.now().timestamp())),
    #     order_name="90%10%小瓶投料-1",
    #     percent_90_1_assign_material_name="BTDA-1",
    #     percent_90_1_target_weigh="7.392",
    #     percent_90_2_assign_material_name="BTDA-1",
    #     percent_90_2_target_weigh="7.392",
    #     percent_90_3_assign_material_name="BTDA-2",
    #     percent_90_3_target_weigh="7.392",
    #     percent_10_1_assign_material_name="BTDA-2",
    #     percent_10_1_target_weigh="1.500",
    #     percent_10_1_volume="20",
    #     percent_10_1_liquid_material_name="NMP",
    #     # percent_10_2_assign_material_name="BTDA-c",
    #     # percent_10_2_target_weigh="1.2",
    #     # percent_10_2_volume="20",
    #     # percent_10_2_liquid_material_name="NMP",
    #     speed="400",
    #     temperature="60",
    #     delay_time="1200",
    #     hold_m_name="8.4分装板-1"
    #     )
    
    # vial_result = bioyond.create_90_10_vial_feeding_task(
    #     order_code="task_vial_" + str(int(datetime.now().timestamp())),
    #     order_name="90%10%小瓶投料-2",
    #     percent_90_1_assign_material_name="BPDA-1",
    #     percent_90_1_target_weigh="5.006",
    #     percent_90_2_assign_material_name="PMDA-1",
    #     percent_90_2_target_weigh="3.810",
    #     percent_90_3_assign_material_name="BPDA-1",
    #     percent_90_3_target_weigh="8.399",
    #     percent_10_1_assign_material_name="BPDA-1",
    #     percent_10_1_target_weigh="1.200",
    #     percent_10_1_volume="20",
    #     percent_10_1_liquid_material_name="NMP",
    #     percent_10_2_assign_material_name="BPDA-1",
    #     percent_10_2_target_weigh="1.200",
    #     percent_10_2_volume="20",
    #     percent_10_2_liquid_material_name="NMP",
    #     speed="400",
    #     temperature="60",
    #     delay_time="1200",
    #     hold_m_name="8.4分装板-2"
    #     )
    
    #启动调度器
    #bioyond.scheduler_start()

    #继续调度器
    #bioyond.scheduler_continue()

    result0 = bioyond.stock_material('{"typeMode": 0, "includeDetail": true}')
    result1 = bioyond.stock_material('{"typeMode": 1, "includeDetail": true}')
    result2 = bioyond.stock_material('{"typeMode": 2, "includeDetail": true}')

    matpos1 = bioyond.query_warehouse_by_material_type("3a14196e-b7a0-a5da-1931-35f3000281e9")
    matpos2 = bioyond.query_warehouse_by_material_type("3a14196e-5dfe-6e21-0c79-fe2036d052c4")
    matpos3 = bioyond.query_warehouse_by_material_type("3a14196b-24f2-ca49-9081-0cab8021bf1a")

    #样品板（里面有样品瓶）
    material_data_yp = {
    "typeId": "3a14196e-b7a0-a5da-1931-35f3000281e9",
    #"code": "物料编码001",
    #"barCode": "物料条码001", 
    "name": "8.4样品板",
    "unit": "个",
    "quantity": 1,
    "details": [
        {
        "typeId": "3a14196a-cf7d-8aea-48d8-b9662c7dba94",
        #"code": "物料编码001",
        "name": "BTDA-1",
        "quantity": 20,
        "x": 1,
        "y": 1, 
        #"unit": "单位"
        "molecular": 1,
        "Parameters":"{\"molecular\": 1}"
        },
        {
        "typeId": "3a14196a-cf7d-8aea-48d8-b9662c7dba94",
        #"code": "物料编码001",
        "name": "BPDA-1",
        "quantity": 20,
        "x": 2,
        "y": 1, #x1y2是A02
        #"unit": "单位"
        "molecular": 1,
        "Parameters":"{\"molecular\": 1}"
        },
        {
        "typeId": "3a14196a-cf7d-8aea-48d8-b9662c7dba94",
        #"code": "物料编码001",
        "name": "BTDA-2",
        "quantity": 20,
        "x": 1,
        "y": 2, #x1y2是A02
        #"unit": "单位"
        "molecular": 1,
        "Parameters":"{\"molecular\": 1}"
        },
        {
        "typeId": "3a14196a-cf7d-8aea-48d8-b9662c7dba94",
        #"code": "物料编码001",
        "name": "PMDA-1",
        "quantity": 20,
        "x": 2,
        "y": 2, #x1y2是A02
        #"unit": "单位"
        "molecular": 1,
        "Parameters":"{\"molecular\": 1}"
        }
    ],
    "Parameters":"{}"
    }

    material_data_yp = {
    "typeId": "3a14196e-b7a0-a5da-1931-35f3000281e9",
    #"code": "物料编码001",
    #"barCode": "物料条码001", 
    "name": "8.7样品板",
    "unit": "个",
    "quantity": 1,
    "details": [
        {
        "typeId": "3a14196a-cf7d-8aea-48d8-b9662c7dba94",
        #"code": "物料编码001",
        "name": "mianfen",
        "quantity": 13,
        "x": 1,
        "y": 1, 
        #"unit": "单位"
        "molecular": 1,
        "Parameters":"{\"molecular\": 1}"
        },
        {
        "typeId": "3a14196a-cf7d-8aea-48d8-b9662c7dba94",
        #"code": "物料编码001",
        "name": "mianfen2",
        "quantity": 13,
        "x": 1,
        "y": 2, #x1y2是A02
        #"unit": "单位"
        "molecular": 1,
        "Parameters":"{\"molecular\": 1}"
        }
    ],
    "Parameters":"{}"
    }

    #分装板
    material_data_fzb_1 = {
    "typeId": "3a14196e-5dfe-6e21-0c79-fe2036d052c4",
    #"code": "物料编码001",
    #"barCode": "物料条码001", 
    "name": "8.7分装板",
    "unit": "个",
    "quantity": 1,
    "details": [
        {
        "typeId": "3a14196c-76be-2279-4e22-7310d69aed68",
        #"code": "物料编码001",
        "name": "10%小瓶1",
        "quantity": 1,
        "x": 1,
        "y": 1, 
        #"unit": "单位"
        "molecular": 1,
        "Parameters":"{\"molecular\": 1}"
        },
        {
        "typeId": "3a14196c-76be-2279-4e22-7310d69aed68",
        #"code": "物料编码001",
        "name": "10%小瓶2",
        "quantity": 1,
        "x": 1,
        "y": 2, 
        #"unit": "单位"
        "molecular": 1,
        "Parameters":"{\"molecular\": 1}"
        },
        {
        "typeId": "3a14196c-76be-2279-4e22-7310d69aed68",
        #"code": "物料编码001",
        "name": "10%小瓶3",
        "quantity": 1,
        "x": 1,
        "y": 3, 
        #"unit": "单位"
        "molecular": 1,
        "Parameters":"{\"molecular\": 1}"
        },
        {
        "typeId": "3a14196c-cdcf-088d-dc7d-5cf38f0ad9ea",
        #"code": "物料编码001",
        "name": "90%小瓶1",
        "quantity": 1,
        "x": 2,
        "y": 1, #x1y2是A02
        #"unit": "单位"
        "molecular": 1,
        "Parameters":"{\"molecular\": 1}"
        },
        {
        "typeId": "3a14196c-cdcf-088d-dc7d-5cf38f0ad9ea",
        #"code": "物料编码001",
        "name": "90%小瓶2",
        "quantity": 1,
        "x": 2,
        "y": 2,
        #"unit": "单位"
        "molecular": 1,
        "Parameters":"{\"molecular\": 1}"
        },
        {
        "typeId": "3a14196c-cdcf-088d-dc7d-5cf38f0ad9ea",
        #"code": "物料编码001",
        "name": "90%小瓶3",
        "quantity": 1,
        "x": 2,
        "y": 3,
        "molecular": 1,
        "Parameters":"{\"molecular\": 1}"
        }
    ],
    "Parameters":"{}"
    }

    material_data_fzb_2 = {
    "typeId": "3a14196e-5dfe-6e21-0c79-fe2036d052c4",
    #"code": "物料编码001",
    #"barCode": "物料条码001", 
    "name": "8.4分装板-2",
    "unit": "个",
    "quantity": 1,
    "details": [
        {
        "typeId": "3a14196c-76be-2279-4e22-7310d69aed68",
        #"code": "物料编码001",
        "name": "10%小瓶1",
        "quantity": 1,
        "x": 1,
        "y": 1, 
        #"unit": "单位"
        "molecular": 1,
        "Parameters":"{\"molecular\": 1}"
        },
        {
        "typeId": "3a14196c-76be-2279-4e22-7310d69aed68",
        #"code": "物料编码001",
        "name": "10%小瓶2",
        "quantity": 1,
        "x": 1,
        "y": 2, 
        #"unit": "单位"
        "molecular": 1,
        "Parameters":"{\"molecular\": 1}"
        },
        {
        "typeId": "3a14196c-76be-2279-4e22-7310d69aed68",
        #"code": "物料编码001",
        "name": "10%小瓶3",
        "quantity": 1,
        "x": 1,
        "y": 3, 
        #"unit": "单位"
        "molecular": 1,
        "Parameters":"{\"molecular\": 1}"
        },
        {
        "typeId": "3a14196c-cdcf-088d-dc7d-5cf38f0ad9ea",
        #"code": "物料编码001",
        "name": "90%小瓶1",
        "quantity": 1,
        "x": 2,
        "y": 1, #x1y2是A02
        #"unit": "单位"
        "molecular": 1,
        "Parameters":"{\"molecular\": 1}"
        },
        {
        "typeId": "3a14196c-cdcf-088d-dc7d-5cf38f0ad9ea",
        #"code": "物料编码001",
        "name": "90%小瓶2",
        "quantity": 1,
        "x": 2,
        "y": 2,
        #"unit": "单位"
        "molecular": 1,
        "Parameters":"{\"molecular\": 1}"
        },
        {
        "typeId": "3a14196c-cdcf-088d-dc7d-5cf38f0ad9ea",
        #"code": "物料编码001",
        "name": "90%小瓶3",
        "quantity": 1,
        "x": 2,
        "y": 3,
        "molecular": 1,
        "Parameters":"{\"molecular\": 1}"
        }
    ],
    "Parameters":"{}"
    }

    #烧杯
    material_data_sb_oda = {
    "typeId": "3a14196b-24f2-ca49-9081-0cab8021bf1a",
    #"code": "物料编码001",
    #"barCode": "物料条码001", 
    "name": "mianfen1",
    "unit": "个",
    "quantity": 1,
    "Parameters":"{}"
    }

    material_data_sb_pda_2 = {
    "typeId": "3a14196b-24f2-ca49-9081-0cab8021bf1a",
    #"code": "物料编码001",
    #"barCode": "物料条码001", 
    "name": "mianfen2",
    "unit": "个",
    "quantity": 1,
    "Parameters":"{}"
    }

    # material_data_sb_mpda = {
    # "typeId": "3a14196b-24f2-ca49-9081-0cab8021bf1a",
    # #"code": "物料编码001",
    # #"barCode": "物料条码001", 
    # "name": "烧杯MPDA",
    # "unit": "个",
    # "quantity": 1,
    # "Parameters":"{}"
    # }


    #result_1 = bioyond.add_material(json.dumps(material_data_yp, ensure_ascii=False))
    #result_2 = bioyond.add_material(json.dumps(material_data_fzb_1, ensure_ascii=False))
    # result_3 = bioyond.add_material(json.dumps(material_data_fzb_2, ensure_ascii=False))
    # result_4 = bioyond.add_material(json.dumps(material_data_sb_oda, ensure_ascii=False))
    # result_5 = bioyond.add_material(json.dumps(material_data_sb_pda_2, ensure_ascii=False))
    # #result会返回id
    # #样品板1id：3a1b3e7d-339d-0291-dfd3-13e2a78fe521


    # #将指定物料入库到指定库位
    #bioyond.material_inbound(result_1, "3a14198e-6929-31f0-8a22-0f98f72260df")
    #bioyond.material_inbound(result_2, "3a14198e-6929-46fe-841e-03dd753f1e4a")
    # bioyond.material_inbound(result_3, "3a14198e-6929-72ac-32ce-9b50245682b8")
    # bioyond.material_inbound(result_4, "3a14198e-d724-e036-afdc-2ae39a7f3383")
    # bioyond.material_inbound(result_5, "3a14198e-d724-d818-6d4f-5725191a24b5")

    #bioyond.material_outbound(result_1, "3a14198e-6929-31f0-8a22-0f98f72260df")

    # bioyond.stock_material('{"typeMode": 2, "includeDetail": true}')

    query_order = {"status":"100", "pageCount": "10"}
    bioyond.order_query(json.dumps(query_order, ensure_ascii=False))

    # id = "3a1bce3c-4f31-c8f3-5525-f3b273bc34dc"
    # bioyond.sample_waste_removal(id)

