import os
import time
import logging

from xyz_stepper_driver import (
    XYZStepperController,
    MotorStatus,
)

# ========== 日志配置 ==========
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("XYZ_Debug")

# 软零点文件与控制器路径（与驱动内部默认一致）
ORIGIN_PATH = "unilabos/devices/laiyu_liquid_test/work_origin.json"


def create_controller(
    port: str = "/dev/ttyUSB1",
    baudrate: int = 115200,
    origin_path: str = ORIGIN_PATH,
) -> XYZStepperController:
    """
    初始化三轴控制器

    说明：
    - XYZStepperController 内部如果 client 为 None，
      会自行创建 ModbusRTUTransport 和 ModbusClient 并打开串口。
    - origin_path 会在 __init__ 时自动尝试加载软零点。
    """
    logger.info(f"初始化控制器: {port} @ {baudrate}bps, origin_path={origin_path}")
    ctrl = XYZStepperController(
        client=None,
        port=port,
        baudrate=baudrate,
        origin_path=origin_path,
    )
    logger.info(f"is_homed={ctrl.is_homed}, work_origin_steps={ctrl.work_origin_steps}")
    return ctrl


def test_enable_axis(ctrl: XYZStepperController):
    """
    依次使能 X / Y / Z 三轴
    """
    logger.info("=== 测试各轴使能 ===")
    for axis in ["X", "Y", "Z"]:
        try:
            result = ctrl.enable(axis, True)
            if result:
                vals = ctrl.get_status(axis)
                st = MotorStatus(vals[3])
                logger.info(f"{axis} 轴使能成功，当前状态: {st.name}")
            else:
                logger.error(f"{axis} 轴使能失败")
        except Exception as e:
            logger.error(f"{axis} 轴使能异常: {e}")
        time.sleep(0.5)


def test_status_read(ctrl: XYZStepperController):
    """
    读取各轴当前状态（调试）
    """
    logger.info("=== 当前各轴状态 ===")
    for axis in ["X", "Y", "Z"]:
        try:
            vals = ctrl.get_status(axis)
            st = MotorStatus(vals[3])
            logger.info(
                f"{axis}: steps={vals[0]}, speed={vals[1]}, "
                f"current={vals[2]}, status={st.name}"
            )
        except Exception as e:
            logger.error(f"获取 {axis} 状态失败: {e}")
        time.sleep(0.2)


def redefine_soft_zero(ctrl: XYZStepperController, path: str = ORIGIN_PATH):
    """
    手动重新定义软零点：
    - 以当前各轴位置为工作原点
    - 写入指定的 JSON 文件
    """
    logger.info("=== 重新定义软零点 ===")
    # 使用驱动内置的保存方法
    ctrl.define_current_as_zero(path)
    logger.info(f"新的软零点已写入: {path}")
    logger.info(f"当前 work_origin_steps = {ctrl.work_origin_steps}")


def test_soft_zero_move(ctrl: XYZStepperController):
    """
    以软零点为基准执行三轴运动测试
    注意：
    - 使用 move_xyz_work，会执行 Z→XY→Z 的安全顺序运动
    """
    logger.info("=== 测试软零点相对运动 ===")
    ctrl.move_xyz_work(x=100.0, y=100.0, z=40.0, speed=100, acc=800)

    for axis in ["X", "Y", "Z"]:
        ctrl.wait_complete(axis)

    test_status_read(ctrl)
    logger.info("软零点运动测试完成")


def main():
    # 如需改串口或波特率，在这里改
    ctrl = create_controller(
        port="/dev/ttyUSB1",
        baudrate=115200,
        origin_path=ORIGIN_PATH,
    )

    try:
        # 1. 使能各轴并读一下状态
        test_enable_axis(ctrl)
        test_status_read(ctrl)

        # 2. 软零点初始化逻辑：
        #    - 如果加载不到，is_homed 会是 False
        if not ctrl.is_homed:
            logger.info("首次运行或未找到软零点文件，将使用当前机械位置定义软零点。")
            redefine_soft_zero(ctrl, ORIGIN_PATH)

        # 3. 回工件软零点（软零点已经在控制器内部）
        logger.info("执行回软零点动作...")
        ctrl.return_to_work_origin()
        logger.info("回软零点完成。")

        # 4. 可选：做一次相对软零点的运动测试
        # test_soft_zero_move(ctrl)

    except KeyboardInterrupt:
        logger.info("手动中断退出")

    except Exception as e:
        logger.exception(f"调试出错: {e}")

    finally:
        # 安全关闭串口
        try:
            if hasattr(ctrl, "client") and hasattr(ctrl.client, "transport"):
                ctrl.client.transport.close()
        except Exception:
            pass
        logger.info("串口已安全关闭")


if __name__ == "__main__":
    main()