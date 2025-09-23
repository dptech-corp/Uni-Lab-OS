"""
环境检查模块
用于检查并自动安装 UniLabOS 运行所需的 Python 包
"""

import argparse
import importlib
import subprocess
import sys
from .banner_print import print_status


class EnvironmentChecker:
    """环境检查器"""

    def __init__(self):
        # 定义必需的包及其安装名称的映射
        self.required_packages = {
            # 包导入名 : pip安装名
            # "pymodbus.framer.FramerType": "pymodbus==3.9.2",
            "websockets": "websockets",
            "msgcenterpy": "msgcenterpy",
            "opentrons_shared_data": "opentrons_shared_data",
        }

        # 特殊安装包（需要特殊处理的包）
        self.special_packages = {"pylabrobot": "git+https://github.com/Xuwznln/pylabrobot.git"}

        self.missing_packages = []
        self.failed_installs = []

    def check_package_installed(self, package_name: str) -> bool:
        """检查包是否已安装"""
        try:
            importlib.import_module(package_name)
            return True
        except ImportError:
            return False

    def install_package(self, package_name: str, pip_name: str) -> bool:
        """安装包"""
        try:
            print_status(f"正在安装 {package_name} ({pip_name})...", "info")

            # 构建安装命令
            cmd = [sys.executable, "-m", "pip", "install", pip_name]

            # 执行安装
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)  # 5分钟超时

            if result.returncode == 0:
                print_status(f"✅ {package_name} 安装成功", "success")
                return True
            else:
                print_status(f"❌ {package_name} 安装失败: {result.stderr}", "error")
                return False

        except subprocess.TimeoutExpired:
            print_status(f"❌ {package_name} 安装超时", "error")
            return False
        except Exception as e:
            print_status(f"❌ {package_name} 安装异常: {str(e)}", "error")
            return False

    def check_all_packages(self) -> bool:
        """检查所有必需的包"""
        print_status("开始检查环境依赖...", "info")

        # 检查常规包
        for import_name, pip_name in self.required_packages.items():
            if not self.check_package_installed(import_name):
                self.missing_packages.append((import_name, pip_name))

        # 检查特殊包
        for package_name, install_url in self.special_packages.items():
            if not self.check_package_installed(package_name):
                self.missing_packages.append((package_name, install_url))

        if not self.missing_packages:
            print_status("✅ 所有依赖包检查完成，环境正常", "success")
            return True

        print_status(f"发现 {len(self.missing_packages)} 个缺失的包", "warning")
        return False

    def install_missing_packages(self, auto_install: bool = True) -> bool:
        """安装缺失的包"""
        if not self.missing_packages:
            return True

        if not auto_install:
            print_status("缺失以下包:", "warning")
            for import_name, pip_name in self.missing_packages:
                print_status(f"  - {import_name} (pip install {pip_name})", "warning")
            return False

        print_status(f"开始自动安装 {len(self.missing_packages)} 个缺失的包...", "info")

        success_count = 0
        for import_name, pip_name in self.missing_packages:
            if self.install_package(import_name, pip_name):
                success_count += 1
            else:
                self.failed_installs.append((import_name, pip_name))

        if self.failed_installs:
            print_status(f"有 {len(self.failed_installs)} 个包安装失败:", "error")
            for import_name, pip_name in self.failed_installs:
                print_status(f"  - {import_name} (pip install {pip_name})", "error")
            return False

        print_status(f"✅ 成功安装 {success_count} 个包", "success")
        return True

    def verify_installation(self) -> bool:
        """验证安装结果"""
        if not self.missing_packages:
            return True

        print_status("验证安装结果...", "info")

        failed_verification = []
        for import_name, pip_name in self.missing_packages:
            if not self.check_package_installed(import_name):
                failed_verification.append((import_name, pip_name))

        if failed_verification:
            print_status(f"有 {len(failed_verification)} 个包验证失败:", "error")
            for import_name, pip_name in failed_verification:
                print_status(f"  - {import_name}", "error")
            return False

        print_status("✅ 所有包验证通过", "success")
        return True


def check_environment(auto_install: bool = True, show_details: bool = True) -> bool:
    """
    检查环境并自动安装缺失的包

    Args:
        auto_install: 是否自动安装缺失的包
        show_details: 是否显示详细信息

    Returns:
        bool: 环境检查是否通过
    """
    checker = EnvironmentChecker()

    # 检查包
    if checker.check_all_packages():
        return True

    # 安装缺失的包
    if not checker.install_missing_packages(auto_install):
        if show_details:
            print_status("请手动安装缺失的包后重新启动程序", "error")
        return False

    # 验证安装
    if not checker.verify_installation():
        if show_details:
            print_status("安装验证失败，请检查网络连接或手动安装", "error")
        return False

    return True


if __name__ == "__main__":
    # 命令行参数解析
    parser = argparse.ArgumentParser(description="UniLabOS 环境依赖检查工具")
    parser.add_argument("--no-auto-install", action="store_true", help="仅检查环境，不自动安装缺失的包")
    parser.add_argument("--silent", action="store_true", help="静默模式，不显示详细信息")

    args = parser.parse_args()

    # 执行环境检查
    auto_install = not args.no_auto_install
    show_details = not args.silent

    success = check_environment(auto_install=auto_install, show_details=show_details)

    if not success:
        if show_details:
            print_status("环境检查失败", "error")
        sys.exit(1)
    else:
        if show_details:
            print_status("环境检查完成", "success")
