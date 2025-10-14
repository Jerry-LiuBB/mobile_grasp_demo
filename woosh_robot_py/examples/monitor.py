import asyncio
import argparse 
import sys

sys.path.append("..")  

from woosh_robot import WooshRobot, CommuSettings, NO_PRINT
from woosh.logger import create_logger
from woosh.proto.robot.robot_pb2 import (
    RobotInfo, 
    RobotState, 
    PoseSpeed, 
    Mode,
    Battery,
    Scene,
    TaskProc,
    DeviceState,   
    HardwareState,
    OperationState, 
    Model
)
from woosh.proto.util.common_pb2 import (
    Pose2D, 
    Twist
) 


def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='Woosh Robot Monitor')
    parser.add_argument(
        '--ip', 
        type=str, 
        default='172.20.128.2', 
        help='Woosh Robot IP address'
    )
    parser.add_argument(
        '--port', 
        type=int, 
        default=5480, 
        help='Woosh Robot port'
    )
    return parser.parse_args()


async def main():
    """主函数"""
    args = parse_args()
    print(f"Starting monitor with ip: {args.ip}, port: {args.port}")
    
    robot = None
    try:
        # 初始化wooshlogger 
        logger = create_logger(name="monitor")

        # 初始化SDK
        settings = CommuSettings(
            addr=args.ip,
            port=args.port,
            identity="monitor",
            logger=logger
        )
        
        # 创建机器人实例
        robot = WooshRobot(settings)
        await robot.run()

        # 1.请求机器人信息
        info, ok, err = await robot.robot_info_req(RobotInfo())
        if not ok:
            logger.error(f"请求机器人信息失败: {err}")
            raise ConnectionError(f"无法获取机器人信息: {err}")
        
        logger.info("机器人信息获取成功")

        # 打印机器人信息
        logger.info(
            f"机器人名称: {info.genral.display_model}, "
            f"ID: {info.genral.serial_number}"
        )
        logger.info(f"机器人状态: {info.state}")
        logger.info(
            f"机器人控制模式: {info.mode.ctrl}, "
            f"工作模式: {info.mode.work}"
        )
        
        pose: Pose2D = info.pose_speed.pose
        twist: Twist = info.pose_speed.twist
        logger.info(
            f"机器人位姿, [x:{pose.x:.2f}, y:{pose.y:.2f}, theta:{pose.theta:.2f}], "
            f"[linear:{twist.linear:.2f}, angular:{twist.angular:.2f}], "
            f"累计里程: {info.pose_speed.mileage:.2f}"
        ) 
        
        logger.info(
            f"机器人充电状态: {info.battery.charge_state}, "
            f"电池电量: {info.battery.power}"
        )
        logger.info(
            f"机器人所在地图: {info.scene.map_name}, "
            f"地图版本: {info.scene.version}"
        )
        logger.info(
            f"机器人任务目的地: {info.task_proc.dest}, "
            f"动作: {info.task_proc.action.type}, "
            f"动作状态: {info.task_proc.action.state}, "
            f"任务状态: {info.task_proc.state}"
        )
        logger.info(
            f"机器人设备状态hardware: {info.device_state.hardware}, "
            f"software: {info.device_state.software}"
        )
        logger.info(
            f"机器人运行状态nav: {info.operation_state.nav}, "
            f"robot: {info.operation_state.robot}"
        )

        # 2.订阅机器人信息
        
        # 2.1 订阅机器人状态
        def print_robot_state(state: RobotState):
            logger.info(f"机器人状态更新: {state}")
        await robot.robot_state_sub(print_robot_state, NO_PRINT)

        # 2.2 订阅模式信息
        def print_mode(mode: Mode):
            logger.info(
                f"机器人模式更新, 控制模式: {mode.ctrl}, "
                f"工作模式: {mode.work}"
            )
        await robot.robot_mode_sub(print_mode, NO_PRINT)

        # 2.3 订阅机器人位姿
        def print_pose_speed(pose_speed: PoseSpeed):
            # pose: Pose2D = pose_speed.pose
            # twist: Twist = pose_speed.twist
            # logger.info(
            #     f"机器人位姿速度更新, [x:{pose.x:.2f}, y:{pose.y:.2f}, theta:{pose.theta:.2f}], "
            #     f"[linear:{twist.linear:.2f}, angular:{twist.angular:.2f}], "
            #     f"累计里程: {pose_speed.mileage:.2f}"
            # )
            pass
        await robot.robot_pose_speed_sub(print_pose_speed, NO_PRINT)
        
        # 2.4 订阅机器人电池信息
        def print_battery(battery: Battery):
            logger.info(
                f"机器人电池更新, 充电状态: {battery.charge_state}, "
                f"电量: {battery.power}"
            )
        await robot.robot_battery_sub(print_battery, NO_PRINT)

        # 2.5 订阅机器人场景信息
        def print_scene(scene: Scene):
            logger.info(
                f"机器人场景更新, 地图名称: {scene.map_name}, "
                f"地图版本: {scene.version}"
            )
        await robot.robot_scene_sub(print_scene, NO_PRINT)

        # 2.6 订阅机器人任务信息
        def print_task_proc(task_proc: TaskProc):
            logger.info(
                f"机器人任务更新, 任务目的地: {task_proc.dest}, "
                f"动作: {task_proc.action.type}, "
                f"动作状态: {task_proc.action.state}, "
                f"任务状态: {task_proc.state}"
            )
        await robot.robot_task_process_sub(print_task_proc, NO_PRINT) 

        # 2.7 订阅机器人设备状态
        def print_device_state(device_state: DeviceState):
            logger.info(
                f"机器人设备状态更新, "
                f"hardware: {device_state.hardware}, "
                f"software: {device_state.software}"
            ) 
            if device_state.hardware & DeviceState.kEmgBtn:
                logger.warning("急停触发!")
            if device_state.software & DeviceState.kLocation:
                logger.info("定位准确.")
        await robot.robot_device_state_sub(print_device_state, NO_PRINT)

        # 2.8 订阅机器人硬件状态
        def print_hardware_state(hardware_state: HardwareState):
            logger.info(f"机器人硬件状态更新, {hardware_state}")
        await robot.robot_hardware_state_sub(print_hardware_state, NO_PRINT)

        # 2.9 订阅机器人运行状态
        def print_operation_state(operation_state: OperationState):
            logger.info(
                f"机器人运行状态更新, "
                f"nav: {operation_state.nav}, "
                f"robot: {operation_state.robot}"
            )
            if operation_state.nav & OperationState.kImpede:
                logger.warning("遇到障碍物!")
            if operation_state.robot & OperationState.kTaskable:
                logger.info("可接任务.")
        await robot.robot_operation_state_sub(print_operation_state, NO_PRINT)
        
        # 2.10 订阅机器人模型
        def print_model(model: Model):
            logger.info(f"机器人模型更新, {model}")
        await robot.robot_model_sub(print_model, NO_PRINT)

        # 3.保持程序一直运行，直到Ctrl+C
        while True:
            await asyncio.sleep(1)
         
    except KeyboardInterrupt:
        logger.info("Monitor stopped by user")
    except ConnectionError as e:
        logger.error(f"连接错误: {e}")
    except TimeoutError as e:
        logger.error(f"连接超时: {e}")
    except asyncio.CancelledError:
        logger.info("任务被取消")
    except Exception as e:
        logger.error(f"未知错误: {e}")
    finally:
        logger.info("Monitor shutdown")
        # 关闭机器人连接
        if robot is not None:
            try:
                await robot.stop()
                logger.info("Robot connection closed")
            except Exception as e:
                logger.error(f"关闭机器人连接失败: {e}")


if __name__ == "__main__":
    asyncio.run(main())
