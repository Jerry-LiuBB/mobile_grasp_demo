import sys
import asyncio

from woosh.proto.robot.robot_pb2 import (
    PoseSpeed,
    TaskProc,
    OperationState,
    ScannerData,
)
from woosh.proto.robot.robot_pack_pb2 import (
    ExecTask,
    ActionOrder,
    Twist,
)
from woosh.proto.util.action_pb2 import kCancel as kActionCancel
from woosh.proto.util.task_pb2 import State as TaskState, Type as TaskType
from woosh.proto.ros.ros_pack_pb2 import (
    CallAction,
    Feedbacks,
)
from woosh.proto.ros.action_pb2 import (
    StepControl,
    ControlAction,
)

from woosh_interface import CommuSettings, NO_PRINT
from woosh_robot import WooshRobot


import sys
import asyncio

from woosh_interface import CommuSettings, NO_PRINT
from woosh_robot import WooshRobot

from woosh.proto.robot.robot_pack_pb2 import (
    ExecTask,
    ActionOrder,
    Twist,
)
from woosh.proto.ros.action_pb2 import (
    StepControl,
    ControlAction,
)

# 定义常量
kActionCancel = 1  # 假设 kActionCancel 的值为 1，若不确定请根据实际 proto 文件确认


async def main():
    # ========== 1. 参数处理：IP 和端口 ==========
    addr = "169.254.128.2"
    port = 5480

    if len(sys.argv) >= 3:
        addr = sys.argv[1]
        port = int(sys.argv[2])

    print(f"连接地址: {addr}:{port}")

    # ========== 2. 初始化连接 ==========
    settings = CommuSettings(
        addr=addr,
        port=port,
        identity="woosdk-demo",
    )
    robot = WooshRobot(settings)
    await robot.run()

    # ========== 3. 用户交互式控制流程 ==========

    input("输入回车执行导航任务\n")

    # ---- [控制1] 执行导航任务（任务请求：ExecTask）----
    exec_task = ExecTask(
        mark_no='A'
    )
    exec_task.pose.x = 1.5
    exec_task.pose.y = 0.5
    exec_task.pose.theta = 1.57  # 约 90 度

    task_result, ok, msg = await robot.exec_task_req(exec_task, NO_PRINT, NO_PRINT)
    if ok:
        print("✅ 执行任务请求成功")
    else:
        print(f"❌ 执行任务请求失败, msg: {msg}")

    await asyncio.sleep(5)  # 等待任务执行一会儿
    input("输入回车取消任务\n")

    # ---- [控制2] 取消任务（动作指令：ActionOrder）----
    action_order = ActionOrder(order=kActionCancel)  # 取消任务
    order_result, ok, msg = await robot.action_order_req(action_order, NO_PRINT, NO_PRINT)
    if ok:
        print("✅ 动作指令（取消任务）请求成功")
    else:
        print(f"❌ 动作指令请求失败, msg: {msg}")

    await asyncio.sleep(1)
    input("输入回车执行步进控制（直行）\n")

    # ---- [控制3] 步进控制（StepControl，直行 0.5 米）----
    step_control = StepControl()
    step = step_control.steps.add()
    step.mode = StepControl.Step.Mode.kStraight  # 直行模式
    step.value = 0.5      # 直行距离 (单位：米？请根据实际协议确认)
    step.speed = 0.25     # 速度 (单位：m/s？请根据实际协议确认)
    step_control.action = ControlAction.kExecute  # 执行

    call_action = CallAction(step_control=step_control)
    action_result, ok, msg = await robot.call_action_req(call_action, NO_PRINT, NO_PRINT)
    if ok:
        print("✅ 步进控制（直行）请求成功")
    else:
        print(f"❌ 步进控制请求失败, msg: {msg}")

    await asyncio.sleep(5)
    input("输入回车进入速度控制（遥控模式）\n")

    # ---- [控制4] 速度控制（Twist：线速度 + 角速度）----
    hertz = 20
    delay = 0.1
    linear = 0.0
    angular = 0.785  # 约 45°/s

    twist = Twist(linear=linear, angular=angular)

    for _ in range(20):
        twist_result, ok, msg = await robot.twist_req(twist, NO_PRINT, NO_PRINT)
        if ok:
            print("✅ 速度控制请求成功")
        else:
            print(f"❌ 速度控制请求失败, msg: {msg}")
        await asyncio.sleep(delay)

    # 平滑减速
    zero_time = 1.5
    num = int(zero_time * hertz)
    linear_reduce = linear / num
    angular_reduce = angular / num
    print(f"🌀 平滑减速：次数={num}, 线速度减量={linear_reduce}, 角速度减量={angular_reduce}")

    twist_reduce = Twist()
    for n in range(num):
        # 线速度递减
        if linear > 0:
            l = linear - linear_reduce * (n + 1)
            twist_reduce.linear = max(l, 0)
        else:
            l = linear + linear_reduce * (n + 1)
            twist_reduce.linear = min(l, 0)
        # 角速度递减
        if angular > 0:
            a = angular - angular_reduce * (n + 1)
            twist_reduce.angular = max(a, 0)
        else:
            a = angular + angular_reduce * (n + 1)
            twist_reduce.angular = min(a, 0)

        print(f"🌀 减速中 -> 线速度: {twist_reduce.linear:.3f}, 角速度: {twist_reduce.angular:.3f}")
        twist_result, ok, msg = await robot.twist_req(twist_reduce, NO_PRINT, NO_PRINT)
        if ok:
            print("✅ 速度控制（减速）请求成功")
        else:
            print(f"❌ 速度控制请求失败, msg: {msg}")
        await asyncio.sleep(delay)

    # 强制归零（保险）
    twist_zero = Twist()
    twist_result, ok, msg = await robot.twist_req(twist_zero, NO_PRINT, NO_PRINT)
    if ok:
        print("✅ 已发送零速度指令，机器人应停止")
    else:
        print(f"❌ 发送零速度失败, msg: {msg}")

    input("输入回车退出程序\n")


if __name__ == "__main__":
    asyncio.run(main())