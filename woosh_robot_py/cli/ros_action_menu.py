import asyncio
import json
from typing import Optional

from woosh_robot import WooshRobot
from woosh_interface import FULL_PRINT
from woosh.proto.ros.ros_pack_pb2 import CallAction as RosCallAction, Feedbacks

from base_menu import BaseMenu


class RosActionMenu(BaseMenu):
    """ROS动作菜单"""

    prompt = "(action) "

    def __init__(
        self, robot: WooshRobot, loop: Optional[asyncio.AbstractEventLoop] = None
    ):
        if loop is not None and not isinstance(loop, asyncio.AbstractEventLoop):
            raise TypeError("loop must be an instance of asyncio.AbstractEventLoop")
        super().__init__(robot, loop)
        self._feedbacks_sub = False

    def _print_help(self):
        """打印帮助信息"""
        sections = {
            "充电控制": [
                (
                    "charge_control",
                    "充电控制 (执行模式 动作)\n"
                    + "    执行模式: 1:充电, 2:退出\n"
                    + "    动作: 0:取消, 1:执行, 2:暂停, 3:继续",
                ),
            ],
            "升降控制": [
                (
                    "lift_control",
                    "升降控制 (执行模式 动作)\n"
                    + "    执行模式: 1:上升, 2:下降\n"
                    + "    动作: 0:取消, 1:执行, 2:暂停, 3:继续",
                ),
                ("lift_control2", "升降控制2 (目标高度)"),
                (
                    "lift_control3",
                    "升降控制3 (执行模式 速度 高度 动作)\n"
                    + "    执行模式: 1:上升, 2:下降\n"
                    + "    动作: 0:取消, 1:执行, 2:暂停, 3:继续",
                ),
            ],
            "步进控制": [
                ("step_forward", "前进步进 (距离 速度 避障)"),
                ("step_rotate", "旋转步进 (角度 速度 避障)"),
                ("step_lateral", "横移步进 (距离 速度 避障)"),
                ("step_diagonal", "斜行步进 (距离 速度 角度 避障)"),
            ],
            "导航控制": [
                (
                    "nav",
                    "导航到目标点 (x y theta 动作)\n"
                    + "    动作: 0:取消, 1:执行, 2:暂停, 3:继续",
                ),
            ],
            "反馈订阅": [
                ("sub_fb", "订阅动作反馈"),
                ("unsub_fb", "取消订阅动作反馈"),
            ],
            "系统命令": [
                ("help", "显示帮助信息"),
                ("exit/back", "返回上级菜单"),
            ],
        }
        self.format_menu_help("ROS动作菜单帮助", sections)

    def do_charge_control(self, arg):
        """充电控制请求: charge_control <exec_mode> <action>
        exec_mode: 执行模式, 1:充电, 2:退出
        action: 任务类型, 0:取消, 1:执行, 2:暂停, 3:继续"""
        args = arg.split()
        if len(args) != 2:
            print("用法: charge_control <exec_mode> <action>")
            print("exec_mode: 执行模式, 1:充电, 2:退出")
            print("action: 任务类型, 0:取消, 1:执行, 2:暂停, 3:继续")
            return

        try:
            exec_mode = int(args[0])
            action = int(args[1])
        except ValueError:
            print("参数必须是整数")
            return

        self.run_async(self._charge_control(exec_mode, action))

    async def _charge_control(self, exec_mode, action):
        ros_action = RosCallAction()
        ros_action.charge_control.execute_mode = exec_mode
        ros_action.charge_control.action = action

        result, ok, msg = await self.robot.call_action_req(
            ros_action, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            print(self.format_success("充电控制请求", result))
        else:
            print(self.format_error("充电控制请求", msg))

    def do_lift_control(self, arg):
        """升降控制请求: lift_control <exec_mode> <action>
        exec_mode: 执行模式, 1:上升, 2:下降
        action: 任务类型, 0:取消, 1:执行, 2:暂停, 3:继续"""
        args = arg.split()
        if len(args) != 2:
            print("用法: lift_control <exec_mode> <action>")
            print("exec_mode: 执行模式, 1:上升, 2:下降")
            print("action: 任务类型, 0:取消, 1:执行, 2:暂停, 3:继续")
            return

        try:
            exec_mode = int(args[0])
            action = int(args[1])
        except ValueError:
            print("参数必须是整数")
            return

        self.run_async(self._lift_control(exec_mode, action))

    async def _lift_control(self, exec_mode, action):
        ros_action = RosCallAction()
        ros_action.lift_control.execute_mode = exec_mode
        ros_action.lift_control.action = action

        result, ok, msg = await self.robot.call_action_req(
            ros_action, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            print(self.format_success("升降控制请求", result))
        else:
            print(self.format_error("升降控制请求", msg))

    def do_lift_control2(self, arg):
        """升降控制2请求: lift_control2 <height>
        height: 高度(米), 正值往上, 负值往下"""
        args = arg.split()
        if len(args) != 1:
            print("用法: lift_control2 <height>")
            print("height: 高度(米), 正值往上, 负值往下")
            return

        try:
            height = float(args[0])
        except ValueError:
            print("高度必须是数字")
            return

        self.run_async(self._lift_control2(height))

    async def _lift_control2(self, height):
        ros_action = RosCallAction()
        ros_action.lift_control2.height = height

        result, ok, msg = await self.robot.call_action_req(
            ros_action, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            print(self.format_success("升降控制2请求", result))
        else:
            print(self.format_error("升降控制2请求", msg))

    def do_lift_control3(self, arg):
        """升降控制3请求: lift_control3 <exec_mode> <speed> <height> [action]
        exec_mode: 执行模式, 0:查询状态, 1:绝对位置, 2:相对位置, 3:位置校准, 4:测试模式
        speed: 速度(米/s)
        height: 高度(米), 正值往上, 负值往下
        action: 任务类型, 0:取消, 1:执行, 2:暂停, 3:继续"""
        args = arg.split()
        if len(args) < 3:
            print("用法: lift_control3 <exec_mode> <speed> <height> [action]")
            print(
                "exec_mode: 执行模式, 0:查询状态, 1:绝对位置, 2:相对位置, 3:位置校准, 4:测试模式"
            )
            print("speed: 速度(米/s)")
            print("height: 高度(米), 正值往上, 负值往下")
            print("action: 任务类型, 0:取消, 1:执行, 2:暂停, 3:继续")
            return

        try:
            exec_mode = int(args[0])
            speed = float(args[1])
            height = float(args[2])
            action = 1

            if len(args) > 3:
                action = int(args[3])
        except ValueError:
            print("参数格式错误")
            return

        self.run_async(self._lift_control3(exec_mode, speed, height, action))

    async def _lift_control3(self, exec_mode, speed, height, action):
        ros_action = RosCallAction()
        ros_action.lift_control3.execute_mode = exec_mode
        ros_action.lift_control3.speed = speed
        ros_action.lift_control3.height = height
        ros_action.lift_control3.action = action

        result, ok, msg = await self.robot.call_action_req(
            ros_action, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            print(self.format_success("升降控制3请求", result))
        else:
            print(self.format_error("升降控制3请求", msg))

    def do_step_forward(self, arg):
        """直走控制请求: step_forward <distance> <speed> [avoid]
        distance: 行驶距离(米), 正值前进, 负值后退
        speed: 线速度(米/s)
        avoid: 避障控制, 0:开启避障(默认), 1:关闭避障"""
        args = arg.split()
        if len(args) < 2:
            print("用法: step_forward <distance> <speed> [avoid]")
            print("distance: 行驶距离(米), 正值前进, 负值后退")
            print("speed: 线速度(米/s)")
            print("avoid: 避障控制, 0:开启避障(默认), 1:关闭避障")
            return

        try:
            value = float(args[0])
            speed = float(args[1])
            avoid = 0
            if len(args) > 2:
                avoid = int(args[2])
        except ValueError:
            print("参数格式错误")
            return

        self.run_async(self._step_control(1, value, speed, 0.0, avoid))

    def do_step_rotate(self, arg):
        """旋转控制请求: step_rotate <angle> <speed> [avoid]
        angle: 旋转弧度, 正值逆时针, 负值顺时针
        speed: 角速度(弧度/s)
        avoid: 避障控制, 0:开启避障(默认), 1:关闭避障"""
        args = arg.split()
        if len(args) < 2:
            print("用法: step_rotate <angle> <speed> [avoid]")
            print("angle: 旋转弧度, 正值逆时针, 负值顺时针")
            print("speed: 角速度(弧度/s)")
            print("avoid: 避障控制, 0:开启避障(默认), 1:关闭避障")
            return

        try:
            value = float(args[0])
            speed = float(args[1])
            avoid = 0
            if len(args) > 2:
                avoid = int(args[2])
        except ValueError:
            print("参数格式错误")
            return

        self.run_async(self._step_control(2, value, speed, 0.0, avoid))

    def do_step_lateral(self, arg):
        """横移控制请求: step_lateral <distance> <speed> [avoid]
        distance: 横移距离(米), 正值向左, 负值向右
        speed: 线速度(米/s)
        avoid: 避障控制, 0:开启避障(默认), 1:关闭避障"""
        args = arg.split()
        if len(args) < 2:
            print("用法: step_lateral <distance> <speed> [avoid]")
            print("distance: 横移距离(米), 正值向左, 负值向右")
            print("speed: 线速度(米/s)")
            print("avoid: 避障控制, 0:开启避障(默认), 1:关闭避障")
            return

        try:
            value = float(args[0])
            speed = float(args[1])
            avoid = 0
            if len(args) > 2:
                avoid = int(args[2])
        except ValueError:
            print("参数格式错误")
            return

        self.run_async(self._step_control(3, value, speed, 0.0, avoid))

    def do_step_diagonal(self, arg):
        """斜移控制请求: step_diagonal <distance> <speed> <angle> [avoid]
        distance: 斜移距离(米), 正值前进, 负值后退
        speed: 线速度(米/s)
        angle: 斜向运动角度, 正值向左, 负值向右
        avoid: 避障控制, 0:开启避障(默认), 1:关闭避障"""
        args = arg.split()
        if len(args) < 3:
            print("用法: step_diagonal <distance> <speed> <angle> [avoid]")
            print("distance: 斜移距离(米), 正值前进, 负值后退")
            print("speed: 线速度(米/s)")
            print("angle: 斜向运动角度, 正值向左, 负值向右")
            print("avoid: 避障控制, 0:开启避障(默认), 1:关闭避障")
            return

        try:
            value = float(args[0])
            speed = float(args[1])
            angle = float(args[2])
            avoid = 0
            if len(args) > 3:
                avoid = int(args[3])
        except ValueError:
            print("参数格式错误")
            return

        self.run_async(self._step_control(4, value, speed, angle, avoid))

    async def _step_control(self, mode, value, speed, angle=0.0, avoid=0):
        """步进控制的内部方法
        mode: 控制模式, 1:直走, 2:旋转, 3:横移, 4:斜移
        value: 旋转弧度/行驶距离
        speed: 角速度/线速度
        angle: 斜向运动角度
        avoid: 避障控制, 0:开启避障, 1:关闭避障"""
        ros_action = RosCallAction()
        step = ros_action.step_control.steps.add()
        step.mode = mode
        step.value = value
        step.speed = speed
        step.angle = angle
        ros_action.step_control.avoid = avoid

        result, ok, msg = await self.robot.call_action_req(
            ros_action, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            print(self.format_success("步进控制请求", result))
        else:
            print(self.format_error("步进控制请求", msg))

    def do_nav(self, arg):
        """基础导航请求: nav <x> <y> <theta> <action>
        x, y, theta: 目标位姿
        action: 任务类型, 0:取消, 1:执行, 2:暂停, 3:继续"""
        args = arg.split()
        if len(args) != 4:
            print("用法: nav <x> <y> <theta> <action>")
            print("x, y, theta: 目标位姿")
            print("action: 任务类型, 0:取消, 1:执行, 2:暂停, 3:继续")
            return

        try:
            x = float(args[0])
            y = float(args[1])
            theta = float(args[2])
            action = int(args[3])
        except ValueError:
            print("参数格式错误")
            return

        self.run_async(self._move_base(x, y, theta, action))

    async def _move_base(self, x, y, theta, action):
        ros_action = RosCallAction()
        ros_action.move_base.target_pose.x = x
        ros_action.move_base.target_pose.y = y
        ros_action.move_base.target_pose.theta = theta
        ros_action.move_base.action = action

        result, ok, msg = await self.robot.call_action_req(
            ros_action, FULL_PRINT, FULL_PRINT
        )

        if ok and result:
            print(self.format_success("基础导航请求", result))
        else:
            print(self.format_error("基础导航请求", msg))

    def do_sub_fb(self, arg):
        """订阅ROS反馈: sub_fb"""
        if self._feedbacks_sub:
            print("已经订阅了ROS反馈")
            return

        self.run_async(self._subscribe_feedbacks())

    def do_unsub_fb(self, arg):
        """取消订阅ROS反馈: unsub_fb"""
        if not self._feedbacks_sub:
            print("未订阅ROS反馈")
            return

        self.run_async(self._unsubscribe_feedbacks())

    async def _subscribe_feedbacks(self):
        """订阅ROS反馈的内部方法"""
        try:

            async def _on_feedbacks(feedbacks: Feedbacks):
                """处理ROS反馈的回调函数"""
                print(f"ROS反馈: {str(feedbacks)}")

            ok = await self.robot.feedbacks_sub(_on_feedbacks, FULL_PRINT)
            if ok:
                self._feedbacks_sub = True
                print("成功订阅ROS反馈")
            else:
                print("订阅ROS反馈失败")
        except Exception as e:
            print(f"订阅ROS反馈时发生错误: {str(e)}")

    async def _unsubscribe_feedbacks(self):
        """取消订阅ROS反馈的内部方法"""
        try:
            ok = await self.robot.feedbacks_unsub(FULL_PRINT)
            if ok:
                self._feedbacks_sub = False
                print("已取消订阅ROS反馈")
            else:
                print("取消订阅ROS反馈失败")
        except Exception as e:
            print(f"取消订阅ROS反馈时发生错误: {str(e)}")

    def do_help(self, arg):
        """显示帮助信息"""
        self._print_help()
