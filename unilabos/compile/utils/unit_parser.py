"""
统一的单位解析工具模块
支持时间、体积、质量等各种单位的解析
"""

import re
import logging
from typing import Union

logger = logging.getLogger(__name__)

def debug_print(message, prefix="[UNIT_PARSER]"):
    """调试输出"""
    print(f"{prefix} {message}", flush=True)
    logger.info(f"{prefix} {message}")

def parse_time_with_units(time_input: Union[str, float, int], default_unit: str = "s") -> float:
    """
    解析带单位的时间输入
    
    Args:
        time_input: 时间输入（如 "30 min", "1 h", "300", "?", 60.0）
        default_unit: 默认单位（默认为秒）
    
    Returns:
        float: 时间（秒）
    """
    if not time_input:
        return 0.0
    
    # 处理数值输入
    if isinstance(time_input, (int, float)):
        result = float(time_input)
        debug_print(f"数值时间输入: {time_input} → {result}s（默认单位）")
        return result
    
    # 处理字符串输入
    time_str = str(time_input).lower().strip()
    debug_print(f"解析时间字符串: '{time_str}'")
    
    # 处理特殊值
    if time_str in ['?', 'unknown', 'tbd', 'to be determined']:
        default_time = 300.0  # 5分钟默认值
        debug_print(f"检测到未知时间，使用默认值: {default_time}s")
        return default_time
    
    # 如果是纯数字，使用默认单位
    try:
        value = float(time_str)
        if default_unit == "s":
            result = value
        elif default_unit in ["min", "minute"]:
            result = value * 60.0
        elif default_unit in ["h", "hour"]:
            result = value * 3600.0
        else:
            result = value  # 默认秒
        debug_print(f"纯数字输入: {time_str} → {result}s（单位: {default_unit}）")
        return result
    except ValueError:
        pass
    
    # 使用正则表达式匹配数字和单位
    pattern = r'(\d+\.?\d*)\s*([a-z]*)'
    match = re.match(pattern, time_str)
    
    if not match:
        debug_print(f"⚠️ 无法解析时间: '{time_str}'，使用默认值: 60s")
        return 60.0
    
    value = float(match.group(1))
    unit = match.group(2) or default_unit
    
    # 单位转换映射
    unit_multipliers = {
        # 秒
        's': 1.0,
        'sec': 1.0,
        'second': 1.0,
        'seconds': 1.0,
        
        # 分钟
        'm': 60.0,
        'min': 60.0,
        'mins': 60.0,
        'minute': 60.0,
        'minutes': 60.0,
        
        # 小时
        'h': 3600.0,
        'hr': 3600.0,
        'hrs': 3600.0,
        'hour': 3600.0,
        'hours': 3600.0,
        
        # 天
        'd': 86400.0,
        'day': 86400.0,
        'days': 86400.0,
    }
    
    multiplier = unit_multipliers.get(unit, 1.0)
    result = value * multiplier
    
    debug_print(f"时间解析: '{time_str}' → {value} {unit} → {result}s")
    return result

def parse_volume_with_units(volume_input: Union[str, float, int], default_unit: str = "mL") -> float:
    """
    解析带单位的体积输入
    
    Args:
        volume_input: 体积输入（如 "100 mL", "2.5 L", "500", "?", 100.0）
        default_unit: 默认单位（默认为毫升）
    
    Returns:
        float: 体积（毫升）
    """
    if not volume_input:
        return 0.0
    
    # 处理数值输入
    if isinstance(volume_input, (int, float)):
        result = float(volume_input)
        debug_print(f"数值体积输入: {volume_input} → {result}mL（默认单位）")
        return result
    
    # 处理字符串输入
    volume_str = str(volume_input).lower().strip()
    debug_print(f"解析体积字符串: '{volume_str}'")
    
    # 处理特殊值
    if volume_str in ['?', 'unknown', 'tbd', 'to be determined']:
        default_volume = 50.0  # 50mL默认值
        debug_print(f"检测到未知体积，使用默认值: {default_volume}mL")
        return default_volume
    
    # 如果是纯数字，使用默认单位
    try:
        value = float(volume_str)
        if default_unit.lower() in ["ml", "milliliter"]:
            result = value
        elif default_unit.lower() in ["l", "liter"]:
            result = value * 1000.0
        elif default_unit.lower() in ["μl", "ul", "microliter"]:
            result = value / 1000.0
        else:
            result = value  # 默认mL
        debug_print(f"纯数字输入: {volume_str} → {result}mL（单位: {default_unit}）")
        return result
    except ValueError:
        pass
    
    # 移除空格并提取数字和单位
    volume_clean = re.sub(r'\s+', '', volume_str)
    
    # 匹配数字和单位的正则表达式
    match = re.match(r'([0-9]*\.?[0-9]+)\s*(ml|l|μl|ul|microliter|milliliter|liter)?', volume_clean)
    
    if not match:
        debug_print(f"⚠️ 无法解析体积: '{volume_str}'，使用默认值: 50mL")
        return 50.0
    
    value = float(match.group(1))
    unit = match.group(2) or default_unit.lower()
    
    # 转换为毫升
    if unit in ['l', 'liter']:
        volume = value * 1000.0  # L -> mL
    elif unit in ['μl', 'ul', 'microliter']:
        volume = value / 1000.0  # μL -> mL
    else:  # ml, milliliter 或默认
        volume = value  # 已经是mL
    
    debug_print(f"体积解析: '{volume_str}' → {value} {unit} → {volume}mL")
    return volume

# 测试函数
def test_unit_parser():
    """测试单位解析功能"""
    print("=== 单位解析器测试 ===")
    
    # 测试时间解析
    time_tests = [
        "30 min", "1 h", "300", "5.5 h", "?", 60.0, "2 hours", "30 s"
    ]
    
    print("\n时间解析测试:")
    for time_input in time_tests:
        result = parse_time_with_units(time_input)
        print(f"  {time_input} → {result}s ({result/60:.1f}min)")
    
    # 测试体积解析
    volume_tests = [
        "100 mL", "2.5 L", "500", "?", 100.0, "500 μL", "1 liter"
    ]
    
    print("\n体积解析测试:")
    for volume_input in volume_tests:
        result = parse_volume_with_units(volume_input)
        print(f"  {volume_input} → {result}mL")
    
    print("\n✅ 测试完成")

if __name__ == "__main__":
    test_unit_parser()