import logging
import os
import platform
from datetime import datetime
import ctypes
import atexit
import inspect
from typing import Tuple, cast


class CustomRecord:
    custom_stack_info: Tuple[str, int, str, str]


# Windows颜色支持
if platform.system() == "Windows":
    # 尝试启用Windows终端的ANSI支持
    kernel32 = ctypes.windll.kernel32
    # 获取STD_OUTPUT_HANDLE
    STD_OUTPUT_HANDLE = -11
    # 启用ENABLE_VIRTUAL_TERMINAL_PROCESSING
    ENABLE_VIRTUAL_TERMINAL_PROCESSING = 0x0004
    # 获取当前控制台模式
    handle = kernel32.GetStdHandle(STD_OUTPUT_HANDLE)
    mode = ctypes.c_ulong()
    kernel32.GetConsoleMode(handle, ctypes.byref(mode))
    # 启用ANSI处理
    kernel32.SetConsoleMode(handle, mode.value | ENABLE_VIRTUAL_TERMINAL_PROCESSING)

    # 程序退出时恢复控制台设置
    @atexit.register
    def reset_console():
        kernel32.SetConsoleMode(handle, mode.value)


# 定义不同日志级别的颜色
class ColoredFormatter(logging.Formatter):
    """自定义日志格式化器，支持颜色输出"""

    # ANSI 颜色代码
    COLORS = {
        "RESET": "\033[0m",  # 重置
        "BOLD": "\033[1m",  # 加粗
        "GRAY": "\033[37m",  # 灰色
        "WHITE": "\033[97m",  # 白色
        "BLACK": "\033[30m",  # 黑色
        "DEBUG_LEVEL": "\033[1;36m",  # 加粗青色
        "INFO_LEVEL": "\033[1;32m",  # 加粗绿色
        "WARNING_LEVEL": "\033[1;33m",  # 加粗黄色
        "ERROR_LEVEL": "\033[1;31m",  # 加粗红色
        "CRITICAL_LEVEL": "\033[1;35m",  # 加粗紫色
        "DEBUG_TEXT": "\033[37m",  # 灰色
        "INFO_TEXT": "\033[97m",  # 白色
        "WARNING_TEXT": "\033[33m",  # 黄色
        "ERROR_TEXT": "\033[31m",  # 红色
        "CRITICAL_TEXT": "\033[35m",  # 紫色
        "DATE": "\033[37m",  # 日期始终使用灰色
    }

    def __init__(self, use_colors=True):
        super().__init__()
        # 强制启用颜色
        self.use_colors = use_colors

    def format(self, record):
        # 检查是否有自定义堆栈信息
        if hasattr(record, "custom_stack_info") and record.custom_stack_info:  # type: ignore
            r = cast(CustomRecord, record)
            frame_info = r.custom_stack_info
            record.filename = frame_info[0]
            record.lineno = frame_info[1]
            record.funcName = frame_info[2]
            if len(frame_info) > 3:
                record.name = frame_info[3]
        if not self.use_colors:
            return self._format_basic(record)

        level_color = self.COLORS.get(f"{record.levelname}_LEVEL", self.COLORS["WHITE"])
        text_color = self.COLORS.get(f"{record.levelname}_TEXT", self.COLORS["WHITE"])
        date_color = self.COLORS["DATE"]
        reset = self.COLORS["RESET"]

        # 日期格式
        datetime_str = datetime.fromtimestamp(record.created).strftime("%y-%m-%d [%H:%M:%S,%f")[:-3] + "]"

        # 模块和函数信息
        filename = record.filename.replace(".py", "").split("\\")[-1]  # 提取文件名（不含路径和扩展名）
        if "/" in filename:
            filename = filename.split("/")[-1]
        module_path = f"{record.name}.{filename}"
        func_line = f"{record.funcName}:{record.lineno}"
        right_info = f" [{func_line}] [{module_path}]"

        # 主要消息
        main_msg = record.getMessage()

        # 构建基本消息格式
        formatted_message = (
            f"{date_color}{datetime_str}{reset} "
            f"{level_color}[{record.levelname}]{reset} "
            f"{text_color}{main_msg}"
            f"{date_color}{right_info}{reset}"
        )

        # 处理异常信息
        if record.exc_info:
            exc_text = self.formatException(record.exc_info)
            if formatted_message[-1:] != "\n":
                formatted_message = formatted_message + "\n"
            formatted_message = formatted_message + text_color + exc_text + reset
        elif record.stack_info:
            if formatted_message[-1:] != "\n":
                formatted_message = formatted_message + "\n"
            formatted_message = formatted_message + text_color + self.formatStack(record.stack_info) + reset

        return formatted_message

    def _format_basic(self, record):
        """基本格式化，不包含颜色"""
        datetime_str = datetime.fromtimestamp(record.created).strftime("%y-%m-%d [%H:%M:%S,%f")[:-3] + "]"
        filename = os.path.basename(record.filename).rsplit(".", 1)[0]  # 提取文件名（不含路径和扩展名）
        module_path = f"{record.name}.{filename}"
        func_line = f"{record.funcName}:{record.lineno}"

        formatted_message = f"{datetime_str} [{record.levelname}] [{module_path}] [{func_line}]: {record.getMessage()}"

        if record.exc_info:
            exc_text = self.formatException(record.exc_info)
            if formatted_message[-1:] != "\n":
                formatted_message = formatted_message + "\n"
            formatted_message = formatted_message + exc_text
        elif record.stack_info:
            if formatted_message[-1:] != "\n":
                formatted_message = formatted_message + "\n"
            formatted_message = formatted_message + self.formatStack(record.stack_info)

        return formatted_message

    def formatException(self, exc_info):
        """重写异常格式化，确保异常信息保持正确的格式和颜色"""
        # 获取标准的异常格式化文本
        formatted_exc = super().formatException(exc_info)
        return formatted_exc


# 配置日志处理器
def configure_logger():
    """配置日志记录器"""
    # 获取根日志记录器
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)  # 修改为DEBUG以显示所有级别

    # 移除已存在的处理器
    for handler in root_logger.handlers[:]:
        root_logger.removeHandler(handler)

    # 创建控制台处理器
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)  # 修改为DEBUG以显示所有级别

    # 使用自定义的颜色格式化器
    color_formatter = ColoredFormatter()
    console_handler.setFormatter(color_formatter)

    # 添加处理器到根日志记录器
    root_logger.addHandler(console_handler)


# 配置日志系统
configure_logger()

# 获取日志记录器
logger = logging.getLogger(__name__)


# 获取调用栈信息的工具函数
def _get_caller_info(stack_level=0) -> Tuple[str, int, str, str]:
    """
    获取调用者的信息

    Args:
        stack_level: 堆栈回溯的级别，0表示当前函数，1表示调用者，依此类推

    Returns:
        (filename, line_number, function_name, module_name) 元组
    """
    # 堆栈级别需要加3:
    # +1 因为这个函数本身占一层
    # +1 因为日志函数(debug, info等)占一层
    # +1 因为下面调用 inspect.stack() 也占一层
    frame = inspect.currentframe()
    try:
        # 跳过适当的堆栈帧
        for _ in range(stack_level + 3):
            if frame and frame.f_back:
                frame = frame.f_back
            else:
                break

        if frame:
            filename = frame.f_code.co_filename if frame.f_code else "unknown"
            line_number = frame.f_lineno if hasattr(frame, "f_lineno") else 0
            function_name = frame.f_code.co_name if frame.f_code else "unknown"

            # 获取模块名称
            module_name = "unknown"
            if frame.f_globals and "__name__" in frame.f_globals:
                module_name = frame.f_globals["__name__"].rsplit(".", 1)[0]

            return (filename, line_number, function_name, module_name)
        return ("unknown", 0, "unknown", "unknown")
    finally:
        del frame  # 避免循环引用


# 便捷日志记录函数
def debug(msg, *args, stack_level=0, **kwargs):
    """
    记录DEBUG级别日志

    Args:
        msg: 日志消息
        stack_level: 堆栈回溯级别，用于定位日志的实际调用位置
        *args, **kwargs: 传递给logger.debug的其他参数
    """
    # 获取调用者信息
    if stack_level > 0:
        caller_info = _get_caller_info(stack_level)
        extra = kwargs.get("extra", {})
        extra["custom_stack_info"] = caller_info
        kwargs["extra"] = extra
    logger.debug(msg, *args, **kwargs)


def info(msg, *args, stack_level=0, **kwargs):
    """
    记录INFO级别日志

    Args:
        msg: 日志消息
        stack_level: 堆栈回溯级别，用于定位日志的实际调用位置
        *args, **kwargs: 传递给logger.info的其他参数
    """
    if stack_level > 0:
        caller_info = _get_caller_info(stack_level)
        extra = kwargs.get("extra", {})
        extra["custom_stack_info"] = caller_info
        kwargs["extra"] = extra
    logger.info(msg, *args, **kwargs)


def warning(msg, *args, stack_level=0, **kwargs):
    """
    记录WARNING级别日志

    Args:
        msg: 日志消息
        stack_level: 堆栈回溯级别，用于定位日志的实际调用位置
        *args, **kwargs: 传递给logger.warning的其他参数
    """
    if stack_level > 0:
        caller_info = _get_caller_info(stack_level)
        extra = kwargs.get("extra", {})
        extra["custom_stack_info"] = caller_info
        kwargs["extra"] = extra
    logger.warning(msg, *args, **kwargs)


def error(msg, *args, stack_level=0, **kwargs):
    """
    记录ERROR级别日志

    Args:
        msg: 日志消息
        stack_level: 堆栈回溯级别，用于定位日志的实际调用位置
        *args, **kwargs: 传递给logger.error的其他参数
    """
    if stack_level > 0:
        caller_info = _get_caller_info(stack_level)
        extra = kwargs.get("extra", {})
        extra["custom_stack_info"] = caller_info
        kwargs["extra"] = extra
    logger.error(msg, *args, **kwargs)


def critical(msg, *args, stack_level=0, **kwargs):
    """
    记录CRITICAL级别日志

    Args:
        msg: 日志消息
        stack_level: 堆栈回溯级别，用于定位日志的实际调用位置
        *args, **kwargs: 传递给logger.critical的其他参数
    """
    if stack_level > 0:
        caller_info = _get_caller_info(stack_level)
        extra = kwargs.get("extra", {})
        extra["custom_stack_info"] = caller_info
        kwargs["extra"] = extra
    logger.critical(msg, *args, **kwargs)


# 测试日志输出（如果直接运行此文件）
if __name__ == "__main__":
    print("测试不同日志级别的颜色输出:")
    debug("这是一条调试日志 (DEBUG级别显示为蓝色，其他文本为灰色)")
    info("这是一条信息日志 (INFO级别显示为绿色，其他文本为白色)")
    warning("这是一条警告日志 (WARNING级别显示为黄色，其他文本也为黄色)")
    error("这是一条错误日志 (ERROR级别显示为红色，其他文本也为红色)")
    critical("这是一条严重错误日志 (CRITICAL级别显示为紫色，其他文本也为紫色)")
    # 测试异常输出
    try:
        1 / 0
    except Exception as e:
        error(f"发生错误: {e}", exc_info=True)
