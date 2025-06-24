#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OPC-UA客户端程序
用于连接和控制制药/化工设备
"""

import asyncio
import logging
from asyncua import Client, ua
from asyncua.common.node import Node
import pandas as pd
from typing import Dict, Any, Optional
import json
# 设置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class OPCUADeviceClient:
    """OPC-UA设备客户端类"""
    
    def __init__(self, server_ip: str = "192.168.1.88", server_port: int = 4840):
        """
        初始化客户端
        :param server_ip: 服务器IP地址
        :param server_port: 服务器端口号
        """
        self.server_url = f"opc.tcp://{server_ip}:{server_port}"
        self.client = Client(self.server_url)
        self.connected = False
        
        # 设备变量映射表
        # self.variables = {
        #     "原料罐号码": {"type": "INT", "initial": 0, "node_id": None},
        #     "反应罐号码": {"type": "INT", "initial": 0, "node_id": None},
        #     "原料罐抓取触发": {"type": "BOOL", "initial": False, "node_id": None},
        #     "反应罐抓取触发": {"type": "BOOL", "initial": False, "node_id": None},
        #     "后处理动作触发": {"type": "BOOL", "initial": False, "node_id": None},
        #     "搅拌桨雾化快速": {"type": "REAL", "initial": 0.0, "node_id": None},
        #     "搅拌桨洗涤慢速": {"type": "REAL", "initial": 0.0, "node_id": None},
        #     "注射泵抽液速度": {"type": "STRING", "initial": "", "node_id": None},
        #     "注射泵推液速度": {"type": "STRING", "initial": "", "node_id": None},
        #     "抽原液次数": {"type": "INT", "initial": 0, "node_id": None},
        #     "洗涤加水量": {"type": "REAL", "initial": 0.0, "node_id": None},
        #     "最开始加水量": {"type": "REAL", "initial": 0.0, "node_id": None},
        #     "雾化压力百分比": {"type": "INT", "initial": 0, "node_id": None},
        #     "吸液针清洗触发": {"type": "BOOL", "initial": False, "node_id": None},
        #     "搅拌桨清洗触发": {"type": "BOOL", "initial": False, "node_id": None},
        #     "管路吹气触发": {"type": "BOOL", "initial": False, "node_id": None},
        #     "废液桶满报警": {"type": "BOOL", "initial": False, "node_id": None},
        #     "清水桶空报警": {"type": "BOOL", "initial": False, "node_id": None},
        #     "NMP桶空报警": {"type": "BOOL", "initial": False, "node_id": None},
        #     "丙酮桶空报警": {"type": "BOOL", "initial": False, "node_id": None},
        #     "门开报警": {"type": "BOOL", "initial": False, "node_id": None},
        # }

        self.variables = {
            "原料罐号码": {"type": "INT",  "initial": 0,"node_id": None},
            "反应罐号码": {"type": "INT",  "initial": 0,"node_id": None},
            "反应罐及原料罐抓取触发": {"type": "BOOL", "initial": False, "node_id": None},
            "后处理动作触发": {"type": "BOOL", "initial": False, "node_id": None},
            "搅拌浆雾化快速": {"type": "REAL", "initial": 0.0, "node_id": None},
            "搅拌浆洗涤慢速": {"type": "REAL", "initial": 0.0, "node_id": None},
            "注射泵抽液速度": {"type": "INT",  "initial": 0,  "node_id": None},
            "注射泵推液速度": {"type": "INT",  "initial": 0,    "node_id": None},
            "抽原液次数": {"type": "INT",  "initial": 0,    "node_id": None},
            "第1次洗涤加水量": {"type": "REAL", "initial": 0.0,  "node_id": None},
            "第2次洗涤加水量": {"type": "REAL", "initial": 0.0,  "node_id": None},
            "第1次粉末搅拌时间": {"type": "DINT", "initial": 0,    "node_id": None},
            "第2次粉末搅拌时间": {"type": "DINT", "initial": 0,    "node_id": None},
            "第1次粉末洗涤次数": {"type": "INT",  "initial": 0,    "node_id": None},
            "第2次粉末洗涤次数": {"type": "INT",  "initial": 0,    "node_id": None},
            "最开始加水量": {"type": "REAL", "initial": 0.0,  "node_id": None},
            "抽滤前搅拌时间": {"type": "DINT", "initial": 0,    "node_id": None},
            "雾化压力Kpa": {"type": "INT",  "initial": 0,    "node_id": None},
            "清洗及管路吹气触发": {"type": "BOOL", "initial": False, "node_id": None},
            "废液桶满报警": {"type": "BOOL", "initial": False, "node_id": None},
            "清水桶空报警": {"type": "BOOL", "initial": False, "node_id": None},
            "NMP桶空报警": {"type": "BOOL", "initial": False, "node_id": None},
            "丙酮桶空报警": {"type": "BOOL", "initial": False, "node_id": None},
            "门开报警": {"type": "BOOL", "initial": False, "node_id": None},
            "反应罐及原料罐抓取完成PLCtoPC": {"type": "BOOL", "initial": False, "node_id": None},
            "后处理动作完成PLCtoPC": {"type": "BOOL", "initial": False, "node_id": None},
            "清洗及管路吹气完成PLCtoPC": {"type": "BOOL", "initial": False, "node_id": None},
            "远程模式PLCtoPC": {"type": "BOOL", "initial": False, "node_id": None},
            "设备准备就绪PLCtoPC": {"type": "BOOL", "initial": False, "node_id": None},
            "NMP外壁清洗加注": {"type": "REAL", "initial": 0.0,   "node_id": None},
            "NMP外壁清洗次数": {"type": "INT",  "initial": 0,     "node_id": None},
            "NMP外壁清洗等待时间": {"type": "DINT", "initial": 0,    "node_id": None},
            "NMP外壁清洗抽废时间": {"type": "DINT", "initial": 0,    "node_id": None},
            "NMP内壁清洗加注": {"type": "REAL", "initial": 0.0,   "node_id": None},
            "NMP内壁清洗次数": {"type": "INT",  "initial": 0,     "node_id": None},
            "NMP泵清洗抽次数": {"type": "INT",  "initial": 0,     "node_id": None},
            "NMP内壁清洗抽废时间": {"type": "DINT", "initial": 0,    "node_id": None},
            "NMP搅拌桨清洗加注": {"type": "REAL", "initial": 0.0,   "node_id": None},
            "NMP搅拌桨清洗次数": {"type": "INT",  "initial": 0,     "node_id": None},
            "NMP搅拌桨清洗等待时间": {"type": "DINT", "initial": 0,    "node_id": None},
            "NMP搅拌桨清洗抽废时间": {"type": "DINT", "initial": 0,    "node_id": None},
            "清水外壁清洗加注":   {"type": "REAL", "initial": 0.0,  "node_id": None},
            "清水外壁清洗次数":   {"type": "INT",  "initial": 0,    "node_id": None},
            "清水外壁清洗等待时间": {"type": "DINT", "initial": 0,    "node_id": None},
            "清水外壁清洗抽废时间": {"type": "DINT", "initial": 0,    "node_id": None},
            "清水内壁清洗加注":   {"type": "REAL", "initial": 0.0,  "node_id": None},
            "清水内壁清洗次数":   {"type": "INT",  "initial": 0,    "node_id": None},
            "清水泵清洗抽次数":   {"type": "INT",  "initial": 0,    "node_id": None},
            "清水内壁清洗抽废时间": {"type": "DINT", "initial": 0,    "node_id": None},
            "清水搅拌桨清洗加注": {"type": "REAL", "initial": 0.0,  "node_id": None},
            "清水搅拌桨清洗次数": {"type": "INT",  "initial": 0,    "node_id": None},
            "清水搅拌桨清洗等待时间": {"type": "DINT", "initial": 0,    "node_id": None},
            "清水搅拌桨清洗抽废时间": {"type": "DINT", "initial": 0,    "node_id": None},
            "丙酮外壁清洗加注":   {"type": "REAL", "initial": 0.0,  "node_id": None},
            "丙酮外壁清洗次数":   {"type": "INT",  "initial": 0,    "node_id": None},
            "丙酮外壁清洗等待时间": {"type": "DINT", "initial": 0,    "node_id": None},
            "丙酮外壁清洗抽废时间": {"type": "DINT", "initial": 0,    "node_id": None},
            "丙酮内壁清洗加注":   {"type": "REAL", "initial": 0.0,  "node_id": None},
            "丙酮内壁清洗次数":   {"type": "INT",  "initial": 0,    "node_id": None},
            "丙酮泵清洗抽次数":   {"type": "INT",  "initial": 0,    "node_id": None},
            "丙酮内壁清洗抽废时间": {"type": "DINT", "initial": 0,    "node_id": None},
            "丙酮搅拌桨清洗加注": {"type": "REAL", "initial": 0.0,  "node_id": None},
            "丙酮搅拌桨清洗次数": {"type": "INT",  "initial": 0,    "node_id": None},
            "丙酮搅拌桨清洗等待时间": {"type": "DINT", "initial": 0,    "node_id": None},
            "丙酮搅拌桨清洗抽废时间": {"type": "DINT", "initial": 0,    "node_id": None},
            "管道吹气时间":       {"type": "DINT", "initial": 0,    "node_id": None},
            "注射泵正向空抽次数": {"type": "INT",  "initial": 0,    "node_id": None},
            "注射泵反向空抽次数": {"type": "INT",  "initial": 0,    "node_id": None},
        }


    @property
    def get_variables(self):
        return self.variables

    async def connect(self):
        """连接到OPC-UA服务器"""
        try:
            await self.client.connect()
            self.connected = True
            logger.info(f"已成功连接到OPC-UA服务器: {self.server_url}")
            
            # 获取根节点
            root = self.client.get_root_node()
            objects = await root.get_child("0:Objects")
            
            # 尝试查找设备节点 (通常在Objects下)
            await self._find_device_nodes(objects)
            
        except Exception as e:
            logger.error(f"连接失败: {e}")
            self.connected = False
            raise

    async def _find_device_nodes(self, objects_node):
        """查找设备节点"""
        try:
            # 获取Objects下的所有子节点
            children = await objects_node.get_children()
            
            for child in children:
                browse_name = await child.read_browse_name()
                logger.info(f"发现节点: {browse_name}")
                
                # 尝试在子节点中查找变量
                await self._search_variables_in_node(child)
                
        except Exception as e:
            logger.error(f"查找设备节点时出错: {e}")

    async def _search_variables_in_node(self, node):
        """在指定节点中搜索变量"""
        try:
            children = await node.get_children()
            for child in children:
                browse_name = await child.read_browse_name()
                variable_name = browse_name.Name
                
                # 检查是否是我们需要的变量
                if variable_name in self.variables:
                    self.variables[variable_name]["node_id"] = child
                    logger.info(f"找到变量: {variable_name}")
                
                # 递归搜索子节点
                try:
                    await self._search_variables_in_node(child)
                except:
                    pass  # 如果节点没有子节点，忽略错误
                    
        except Exception as e:
            pass  # 忽略搜索过程中的错误

    async def disconnect(self):
        """断开连接"""
        if self.connected:
            await self.client.disconnect()
            self.connected = False
            logger.info("已断开与OPC-UA服务器的连接")

    async def read_variable(self, variable_name: str) -> Any:
        """读取指定变量的值"""
        if not self.connected:
            raise ConnectionError("未连接到OPC-UA服务器")
        
        if variable_name not in self.variables:
            raise ValueError(f"未知变量: {variable_name}")
        
        node = self.variables[variable_name]["node_id"]
        if node is None:
            raise ValueError(f"未找到变量节点: {variable_name}")
        
        try:
            value = await node.read_value()
            logger.info(f"读取变量 {variable_name}: {value}")
            return value
        except Exception as e:
            logger.error(f"读取变量 {variable_name} 失败: {e}")
            raise

    async def write_variable(self, command: str):
    # """
    # 写入指定变量的值
    # Args:
    #     command: 一个JSON格式的字符串，包含变量名和值
    #             variable_name (str): 要写入的变量名
    #             value (Any): 要写入的值
    # """
    # 将JSON字符串转换为字典
        cmd_str = command.replace("'", '"')
        cmd_dict = json.loads(cmd_str)
        
        # 提取变量名和值
        variable_name = cmd_dict["variable_name"]
        value = cmd_dict["value"]
        
        if not self.connected:
            raise ConnectionError("未连接到OPC-UA服务器")
        
        if variable_name not in self.variables:
            raise ValueError(f"未知变量: {variable_name}")
        
        node = self.variables[variable_name]["node_id"]
        if node is None:
            raise ValueError(f"未找到变量节点: {variable_name}")
        
        try:
            var_type = self.variables[variable_name]["type"]
            
            # 对于INT类型，直接尝试INT16
            if var_type == "INT":
                int_value = int(value)
                if -32768 <= int_value <= 32767:
                    variant = ua.Variant(int_value, ua.VariantType.Int16)
                    await node.write_value(variant)
                    logger.info(f"写入变量 {variable_name}: {int_value} (Int16)")
                    return
                else:
                    raise ValueError(f"值 {int_value} 超出 Int16 范围 (-32768 到 32767)")
            
            # 对于BOOL类型，直接使用Boolean
            elif var_type == "BOOL":
                bool_value = bool(value)
                variant = ua.Variant(bool_value, ua.VariantType.Boolean)
                await node.write_value(variant)
                logger.info(f"写入变量 {variable_name}: {bool_value} (Boolean)")
                return
            
            # 对于其他类型，使用原有逻辑
            else:
                # 首先读取节点的当前值来确定正确的数据类型
                current_value = await node.read_value()
                current_variant = await node.read_data_value()
                
                # 根据服务器的实际数据类型转换值
                converted_value = self._convert_value_with_variant_type(value, var_type, current_variant.Value.VariantType)
                
                # 创建具有正确VariantType的DataValue
                data_value = ua.DataValue(ua.Variant(converted_value, current_variant.Value.VariantType))
                await node.write_value(data_value.Value)
                
                logger.info(f"写入变量 {variable_name}: {converted_value} (类型: {current_variant.Value.VariantType})")
                
        except Exception as e:
            logger.error(f"写入变量 {variable_name} 失败: {e}")
            raise

    def _convert_value_with_variant_type(self, value: Any, var_type: str, variant_type) -> Any:
        """根据变量类型和OPC-UA VariantType转换值"""
        try:
            if var_type == "INT":
                # 确保值在INT16范围内 (-32768 到 32767)
                int_value = int(value)
                if variant_type == ua.VariantType.Int16:
                    if -32768 <= int_value <= 32767:
                        return int_value
                    else:
                        raise ValueError(f"值 {int_value} 超出 Int16 范围 (-32768 到 32767)")
                elif variant_type == ua.VariantType.UInt16:
                    if 0 <= int_value <= 65535:
                        return int_value
                    else:
                        raise ValueError(f"值 {int_value} 超出 UInt16 范围 (0 到 65535)")
                elif variant_type in [ua.VariantType.Int32, ua.VariantType.UInt32]:
                    return int_value
                elif variant_type in [ua.VariantType.Int64, ua.VariantType.UInt64]:
                    return int_value
                else:
                    return int_value
            elif var_type == "BOOL":
                return bool(value)
            elif var_type == "REAL":
                # 根据实际的VariantType选择浮点类型
                if variant_type == ua.VariantType.Float:
                    return float(value)
                elif variant_type == ua.VariantType.Double:
                    return float(value)
                else:
                    return float(value)
            elif var_type == "STRING":
                return str(value)
            else:
                return value
        except Exception as e:
            logger.warning(f"类型转换失败，使用原始值: {e}")
            return value

    def _convert_value(self, value: Any, var_type: str) -> Any:
        """根据变量类型转换值（简化版本）"""
        if var_type == "INT":
            return int(value)
        elif var_type == "BOOL":
            return bool(value)
        elif var_type == "REAL":
            return float(value)
        elif var_type == "STRING":
            return str(value)
        else:
            return value

    async def read_all_variables(self) -> Dict[str, Any]:
        """读取所有变量的值"""
        results = {}
        for variable_name in self.variables.keys():
            try:
                value = await self.read_variable(variable_name)
                results[variable_name] = value
            except Exception as e:
                logger.warning(f"读取变量 {variable_name} 失败: {e}")
                results[variable_name] = None
        results = str(results)
        return results

    async def write_multiple_variables(self, command: str):
        """批量写入多个变量
        Args:
            command: 一个JSON格式的字符串，包含变量名和值的字典
                    variables (dict): 变量名和值的字典，格式为 {"变量名1": 值1, "变量名2": 值2, ...}
        """
        # 将JSON字符串转换为字典
        cmd_str = command.replace("'", '"')
        cmd_dict = json.loads(cmd_str)
        
        # 提取变量字典
        variables_dict = cmd_dict["variables"]
        
        for variable_name, value in variables_dict.items():
            try:
                # 构建单个变量的写入命令
                single_cmd = json.dumps({"variable_name": variable_name, "value": value})
                await self.write_variable(single_cmd)
            except Exception as e:
                logger.error(f"写入变量 {variable_name} 失败: {e}")

    # 设备控制的便捷方法
    async def set_tank_numbers(self, command: str):
        """设置原料罐和反应罐号码
        Args:
            command: 一个JSON格式的字符串，包含罐号码配置
                    raw_material_tank (int): 原料罐号码
                    reaction_tank (int): 反应罐号码
                    safe_mode (bool): 是否使用安全模式写入（默认为false）
        """
        # 将JSON字符串转换为字典
        cmd_str = command.replace("'", '"')
        cmd_dict = json.loads(cmd_str)
        
        # 提取参数
        raw_material_tank = cmd_dict["raw_material_tank"]
        reaction_tank = cmd_dict["reaction_tank"]
        safe_mode = cmd_dict.get("safe_mode", False)
        
        if safe_mode:
            # 构建安全写入变量的命令
            raw_cmd = json.dumps({"variable_name": "原料罐号码", "value": raw_material_tank})
            reaction_cmd = json.dumps({"variable_name": "反应罐号码", "value": reaction_tank})
            
            await self.write_variable_safe(raw_cmd)
            await self.write_variable_safe(reaction_cmd)
        else:
            # 构建批量写入的命令
            variables = {
                "原料罐号码": raw_material_tank,
                "反应罐号码": reaction_tank
            }
            variables_cmd = json.dumps({"variables": variables})
            
            await self.write_multiple_variables(variables_cmd)

    async def trigger_grab_actions(self, command: str):
        """触发抓取动作
        Args:
            command: 一个JSON格式的字符串，包含抓取动作的配置
                    raw_material (bool): 是否触发原料罐抓取动作
                    reaction (bool): 是否触发反应罐抓取动作
        """
        # 将JSON字符串转换为字典
        cmd_str = command.replace("'", '"')
        cmd_dict = json.loads(cmd_str)
        
        # 提取参数
        raw_material = cmd_dict.get("raw_material", False)
        reaction = cmd_dict.get("reaction", False)
        
        variables = {}
        if raw_material:
            variables["原料罐抓取触发"] = True
        if reaction:
            variables["反应罐抓取触发"] = True
        
        if variables:
            # 构建写入多个变量的命令
            variables_cmd = json.dumps({"variables": variables})
            await self.write_multiple_variables(variables_cmd)

    async def set_stirring_speeds(self, command: str):
        """设置搅拌桨速度
        Args:
            command: 一个JSON格式的字符串，包含搅拌桨速度配置
                    atomization_speed (float): 雾化快速搅拌速度
                    washing_speed (float): 洗涤慢速搅拌速度
        """
        # 将JSON字符串转换为字典
        cmd_str = command.replace("'", '"')
        cmd_dict = json.loads(cmd_str)
        
        # 提取参数
        atomization_speed = cmd_dict.get("atomization_speed", None)
        washing_speed = cmd_dict.get("washing_speed", None)
        
        variables = {}
        if atomization_speed is not None:
            variables["搅拌桨雾化快速"] = atomization_speed
        if washing_speed is not None:
            variables["搅拌桨洗涤慢速"] = washing_speed
        
        if variables:
            # 构建写入多个变量的命令
            variables_cmd = json.dumps({"variables": variables})
            await self.write_multiple_variables(variables_cmd)

    async def set_pump_speeds(self, command: str):
        """设置注射泵速度
        Args:
            command: 一个JSON格式的字符串，包含注射泵速度配置
                    suction_speed (str): 注射泵抽液速度
                    push_speed (str): 注射泵推液速度
        """
        # 将JSON字符串转换为字典
        cmd_str = command.replace("'", '"')
        cmd_dict = json.loads(cmd_str)
        
        # 提取参数
        suction_speed = cmd_dict.get("suction_speed", None)
        push_speed = cmd_dict.get("push_speed", None)
        
        variables = {}
        if suction_speed is not None:
            variables["注射泵抽液速度"] = suction_speed
        if push_speed is not None:
            variables["注射泵推液速度"] = push_speed
        
        if variables:
            # 构建写入多个变量的命令
            variables_cmd = json.dumps({"variables": variables})
            await self.write_multiple_variables(variables_cmd)

    async def set_liquid_parameters(self, command: str):
        """设置液体相关参数
        Args:
            command: 一个JSON格式的字符串，包含液体参数配置
                    suction_times (int): 抽原液次数
                    washing_water (float): 洗涤加水量
                    initial_water (float): 最开始加水量
                    atomization_pressure (int): 雾化压力百分比
        """
        # 将JSON字符串转换为字典
        cmd_str = command.replace("'", '"')
        cmd_dict = json.loads(cmd_str)
        
        # 提取参数
        suction_times = cmd_dict.get("suction_times", None)
        washing_water = cmd_dict.get("washing_water", None)
        initial_water = cmd_dict.get("initial_water", None)
        atomization_pressure = cmd_dict.get("atomization_pressure", None)
        
        variables = {}
        if suction_times is not None:
            variables["抽原液次数"] = suction_times
        if washing_water is not None:
            variables["洗涤加水量"] = washing_water
        if initial_water is not None:
            variables["最开始加水量"] = initial_water
        if atomization_pressure is not None:
            variables["雾化压力百分比"] = atomization_pressure
        
        if variables:
            # 构建写入多个变量的命令
            variables_cmd = json.dumps({"variables": variables})
            await self.write_multiple_variables(variables_cmd)

    async def trigger_cleaning_actions(self, command: str):
        """触发清洗动作
        Args:
            command: 一个JSON格式的字符串，包含清洗动作配置
                    needle (bool): 是否触发吸液针清洗
                    stirrer (bool): 是否触发搅拌桨清洗
                    pipeline (bool): 是否触发管路吹气
        """
        # 将JSON字符串转换为字典
        cmd_str = command.replace("'", '"')
        cmd_dict = json.loads(cmd_str)
        
        # 提取参数
        needle = cmd_dict.get("needle", False)
        stirrer = cmd_dict.get("stirrer", False)
        pipeline = cmd_dict.get("pipeline", False)
        
        variables = {}
        if needle:
            variables["吸液针清洗触发"] = True
        if stirrer:
            variables["搅拌桨清洗触发"] = True
        if pipeline:
            variables["管路吹气触发"] = True
        
        if variables:
            # 构建写入多个变量的命令
            variables_cmd = json.dumps({"variables": variables})
            await self.write_multiple_variables(variables_cmd)

    async def check_alarms(self) -> Dict[str, bool]:
        """检查所有报警状态"""
        alarm_variables = [
            "废液桶满报警", "清水桶空报警", "NMP桶空报警", "丙酮桶空报警", "门开报警"
        ]
        
        alarms = {}
        for alarm in alarm_variables:
            try:
                status = await self.read_variable(alarm)
                alarms[alarm] = bool(status)
            except Exception as e:
                logger.warning(f"读取报警状态 {alarm} 失败: {e}")
                alarms[alarm] = None
        
        return str(alarms)

    async def emergency_stop(self):
        """紧急停止 - 关闭所有触发器"""
        stop_variables = {
            "原料罐抓取触发": False,
            "反应罐抓取触发": False,
            "后处理动作触发": False,
            "吸液针清洗触发": False,
            "搅拌桨清洗触发": False,
            "管路吹气触发": False,
        }
        
        await self.write_multiple_variables(stop_variables)
        logger.info("已执行紧急停止操作")

    async def diagnose_variable_types(self):
        """诊断所有变量的实际数据类型"""
        print("\n=== 变量类型诊断 ===")
        for variable_name, variable_info in self.variables.items():
            node = variable_info["node_id"]
            if node is not None:
                try:
                    # 读取数据类型信息
                    data_value = await node.read_data_value()
                    variant_type = data_value.Value.VariantType
                    current_value = data_value.Value.Value
                    
                    # 读取节点的数据类型属性
                    try:
                        data_type = await node.read_data_type()
                        print(f"{variable_name}:")
                        print(f"  当前值: {current_value}")
                        print(f"  VariantType: {variant_type}")
                        print(f"  DataType: {data_type}")
                        print(f"  我们的类型: {variable_info['type']}")
                        print()
                    except:
                        print(f"{variable_name}:")
                        print(f"  当前值: {current_value}")
                        print(f"  VariantType: {variant_type}")
                        print(f"  我们的类型: {variable_info['type']}")
                        print()
                        
                except Exception as e:
                    print(f"{variable_name}: 读取失败 - {e}")
            else:
                print(f"{variable_name}: 节点未找到")

    async def write_variable_safe(self, command: str):
        """安全写入变量 - 包含详细的错误信息
        Args:
            command: 一个JSON格式的字符串，包含变量名和值
                    variable_name (str): 要写入的变量名
                    value (Any): 要写入的值
        """
        # 将JSON字符串转换为字典
        cmd_str = command.replace("'", '"')
        cmd_dict = json.loads(cmd_str)
        
        # 提取变量名和值
        variable_name = cmd_dict["variable_name"]
        value = cmd_dict["value"]
        
        if not self.connected:
            raise ConnectionError("未连接到OPC-UA服务器")
        
        if variable_name not in self.variables:
            raise ValueError(f"未知变量: {variable_name}")
        
        node = self.variables[variable_name]["node_id"]
        if node is None:
            raise ValueError(f"未找到变量节点: {variable_name}")
        
        try:
            # 读取当前值和类型信息
            current_data_value = await node.read_data_value()
            current_variant_type = current_data_value.Value.VariantType
            current_value = current_data_value.Value.Value
            
            print(f"尝试写入变量: {variable_name}")
            print(f"  当前值: {current_value} (类型: {current_variant_type})")
            print(f"  目标值: {value}")
            
            # 尝试多种类型转换方式
            success = False
            
            # 方法1: 直接使用当前的VariantType
            try:
                var_type = self.variables[variable_name]["type"]
                converted_value = self._convert_value_with_variant_type(value, var_type, current_variant_type)
                variant = ua.Variant(converted_value, current_variant_type)
                await node.write_value(variant)
                print(f"  ✅ 成功写入: {converted_value}")
                success = True
            except Exception as e1:
                print(f"  ❌ 方法1失败: {e1}")
                
                # 方法2: 让OPC-UA自动推断类型
                try:
                    converted_value = self._convert_value(value, var_type)
                    await node.write_value(converted_value)
                    print(f"  ✅ 成功写入 (自动类型): {converted_value}")
                    success = True
                except Exception as e2:
                    print(f"  ❌ 方法2失败: {e2}")
                    
                    # 方法3: 尝试不同的数值类型
                    if var_type == "INT":
                        for vt in [ua.VariantType.Int16, ua.VariantType.Int32, ua.VariantType.UInt16, ua.VariantType.UInt32]:
                            try:
                                variant = ua.Variant(int(value), vt)
                                await node.write_value(variant)
                                print(f"  ✅ 成功写入 (类型 {vt}): {int(value)}")
                                success = True
                                break
                            except:
                                continue
                    elif var_type == "REAL":
                        for vt in [ua.VariantType.Float, ua.VariantType.Double]:
                            try:
                                variant = ua.Variant(float(value), vt)
                                await node.write_value(variant)
                                print(f"  ✅ 成功写入 (类型 {vt}): {float(value)}")
                                success = True
                                break
                            except:
                                continue
            
            if not success:
                raise Exception("所有写入方法都失败了")
            
        except Exception as e:
            logger.error(f"写入变量 {variable_name} 失败: {e}")
            raise
    
    def is_connected(self):
        return self.connected


async def main():
    """主函数 - 演示客户端使用"""
    client = OPCUADeviceClient()
    
    try:
        # 连接到设备
        await client.connect()
        
        # 读取所有变量的当前状态
        print("\n=== 读取所有变量状态 ===")
        all_values = await client.read_all_variables()
        for name, value in all_values.items():
            print(f"{name}: {value}")
        
        # # 设置罐号码
        # print("\n=== 设置罐号码 ===")
        # await client.set_tank_numbers('{"raw_material_tank": 1, "reaction_tank": 2}')
        
        # # 设置搅拌速度
        # print("\n=== 设置搅拌速度 ===")
        # await client.set_stirring_speeds('{"atomization_speed": 1000.0, "washing_speed": 500.0}')
        
        # # 设置液体参数
        # print("\n=== 设置液体参数 ===")
        # await client.set_liquid_parameters('{"suction_times": 3, "washing_water": 100.0, "initial_water": 200.0, "atomization_pressure": 80}')
        
        # # 检查报警状态
        # print("\n=== 检查报警状态 ===")
        # alarms = await client.check_alarms()
        # for alarm_name, status in alarms.items():
        #     if status:
        #         print(f"⚠️  {alarm_name}: 报警中")
        #     else:
        #         print(f"✅ {alarm_name}: 正常")
        
        # # 等待一段时间
        # await asyncio.sleep(2)
        
        # # 读取更新后的状态
        # print("\n=== 读取更新后的状态 ===")
        # updated_values = await client.read_all_variables()
        # for name, value in updated_values.items():
        #     print(f"{name}: {value}")
        
    except Exception as e:
        logger.error(f"操作失败: {e}")
    
    finally:
        # 断开连接
        await client.disconnect()


if __name__ == "__main__":
    # 运行主程序
    asyncio.run(main()) 