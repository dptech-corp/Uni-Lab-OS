#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
串口设备检测脚本

快速检测系统中可用的串口设备，帮助用户找到正确的串口号

使用方法: python check_serial_ports.py
"""

import glob
import platform
import os
import subprocess
import sys


def check_macos_serial_ports():
    """检查macOS系统的串口设备"""
    print("🍎 macOS 串口设备检测:")
    print("-" * 40)
    
    # 检查所有cu设备
    cu_ports = glob.glob("/dev/cu.*")
    tty_ports = glob.glob("/dev/tty.*")
    
    print("📱 USB串口设备 (/dev/cu.*):")
    usb_ports = [p for p in cu_ports if 'usb' in p.lower()]
    if usb_ports:
        for port in sorted(usb_ports):
            print(f"  ✅ {port}")
    else:
        print("  ❌ 未找到USB串口设备")
    
    print("\n📡 蓝牙设备 (/dev/cu.Bluetooth*):")
    bt_ports = [p for p in cu_ports if 'bluetooth' in p.lower()]
    if bt_ports:
        for port in sorted(bt_ports):
            print(f"  📶 {port}")
    else:
        print("  ❌ 未找到蓝牙串口设备")
    
    print("\n🔌 其他串口设备:")
    other_ports = [p for p in cu_ports if 'usb' not in p.lower() and 'bluetooth' not in p.lower()]
    if other_ports:
        for port in sorted(other_ports):
            print(f"  🔗 {port}")
    else:
        print("  ❌ 未找到其他串口设备")
    
    return usb_ports + other_ports


def check_linux_serial_ports():
    """检查Linux系统的串口设备"""
    print("🐧 Linux 串口设备检测:")
    print("-" * 40)
    
    usb_ports = glob.glob("/dev/ttyUSB*")
    acm_ports = glob.glob("/dev/ttyACM*")
    
    print("📱 USB串口设备 (/dev/ttyUSB*):")
    if usb_ports:
        for port in sorted(usb_ports):
            print(f"  ✅ {port}")
    else:
        print("  ❌ 未找到USB串口设备")
    
    print("\n📡 ACM设备 (/dev/ttyACM*):")
    if acm_ports:
        for port in sorted(acm_ports):
            print(f"  ✅ {port}")
    else:
        print("  ❌ 未找到ACM设备")
    
    return usb_ports + acm_ports


def check_windows_serial_ports():
    """检查Windows系统的串口设备"""
    print("🪟 Windows 串口设备检测:")
    print("-" * 40)
    print("请在设备管理器中查看 '端口(COM和LPT)' 部分")
    print("常见的串口格式: COM1, COM2, COM3, ...")
    return []


def check_port_permissions(port):
    """检查串口权限"""
    try:
        # 检查文件是否存在
        if not os.path.exists(port):
            return False, "设备不存在"
        
        # 检查读写权限
        if os.access(port, os.R_OK | os.W_OK):
            return True, "权限正常"
        else:
            return False, "权限不足"
    except Exception as e:
        return False, f"检查失败: {e}"


def check_port_usage(port):
    """检查串口是否被占用"""
    try:
        if platform.system() == "Darwin":  # macOS
            result = subprocess.run(
                ["lsof", port], 
                capture_output=True, 
                text=True
            )
            if result.returncode == 0:
                return True, "被占用"
            else:
                return False, "未被占用"
        else:
            return None, "无法检测"
    except Exception as e:
        return None, f"检测失败: {e}"


def test_port_connection(port, baudrates=[9600, 115200, 57600, 38400, 19200]):
    """测试串口连接"""
    print(f"\n🔍 测试串口: {port}")
    print("-" * 30)
    
    # 检查权限
    perm_ok, perm_msg = check_port_permissions(port)
    print(f"  权限检查: {'✅' if perm_ok else '❌'} {perm_msg}")
    
    if not perm_ok:
        print(f"  💡 修复建议: sudo chmod 666 {port}")
        return False
    
    # 检查占用
    usage_status, usage_msg = check_port_usage(port)
    if usage_status is not None:
        print(f"  占用检查: {'⚠️' if usage_status else '✅'} {usage_msg}")
    
    # 尝试不同波特率
    print("  📡 波特率测试:")
    try:
        import serial
        
        for baudrate in baudrates:
            try:
                ser = serial.Serial(port, baudrate, timeout=1)
                ser.close()
                print(f"    {baudrate}: ✅ 连接成功")
            except serial.SerialException as e:
                print(f"    {baudrate}: ❌ {e}")
            except Exception as e:
                print(f"    {baudrate}: ❌ {e}")
        
        return True
        
    except ImportError:
        print("    ⚠️ 需要安装pyserial: pip install pyserial")
        return False


def main():
    """主函数"""
    print("=" * 60)
    print("🔍 串口设备检测工具")
    print("=" * 60)
    
    system = platform.system()
    
    if system == "Darwin":
        ports = check_macos_serial_ports()
    elif system == "Linux":
        ports = check_linux_serial_ports()
    elif system == "Windows":
        ports = check_windows_serial_ports()
    else:
        print(f"❌ 不支持的操作系统: {system}")
        return
    
    if not ports:
        print("\n⚠️ 未找到串口设备!")
        print("\n🔧 故障排除建议:")
        print("1. 检查USB设备是否已连接")
        print("2. 检查设备是否已通电")
        print("3. 检查驱动程序是否已安装")
        print("4. 尝试重新插拔USB设备")
        return
    
    print(f"\n📊 找到 {len(ports)} 个串口设备")
    print("=" * 60)
    
    # 测试每个端口
    for port in ports:
        test_port_connection(port)
    
    print("\n" + "=" * 60)
    print("💡 使用建议:")
    print("1. 选择权限正常且未被占用的串口")
    print("2. 常用波特率: 9600 (默认) 或 115200 (高速)")
    print("3. 在LaiYu配置中使用找到的串口号")
    print("4. 如果权限不足，使用: sudo chmod 666 <串口号>")


if __name__ == "__main__":
    main()