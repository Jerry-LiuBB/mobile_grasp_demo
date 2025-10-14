#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
基于 WOOSH Robotics 真实 ROS 接口的底盘控制器
使用 /exec_task 服务发送导航任务，通过 /robot_status 订阅判断是否到达目标点
"""

import rospy
from woosh_msgs.srv import ExecTask, ExecTaskRequest
from woosh_msgs.msg import RobotStatus

class ChassisController:
    def __init__(self):
        self.arrive_state = False  # 用于记录是否到达目标
        self.navigation_point = None  # ROS Service Proxy: /exec_task
        self.status_sub = None      # ROS Subscriber: /robot_status

        # 初始化 ROS 节点（如果尚未初始化，比如在独立脚本中运行）
        try:
            rospy.init_node('chassis_controller_woosh', anonymous=True, disable_signals=True)
            print("[ChassisController] ROS节点已初始化（如果尚未初始化）")
        except rospy.exceptions.ROSException:
            # 如果节点已经初始化（比如在已有 ROS 节点中被导入调用），则忽略
            pass

        # 创建 Service 客户端：/exec_task，服务类型 ExecTask
        self.navigation_point = rospy.ServiceProxy('/exec_task', ExecTask)
        rospy.loginfo("已创建 /exec_task 服务客户端")

        # 创建 Subscriber：/robot_status，消息类型 RobotStatus
        self.status_sub = rospy.Subscriber('/robot_status', RobotStatus, self._status_callback)
        rospy.loginfo("已订阅 /robot_status 话题")

        # 等待服务可用（可选，如果服务可能启动较慢）
        rospy.loginfo("等待 /exec_task 服务启动...")
        rospy.wait_for_service('/exec_task')
        rospy.loginfo("✅ /exec_task 服务已就绪")

        print("[ChassisController] 底盘控制器（WOOSH）初始化完成")

    def _status_callback(self, msg: RobotStatus):
        """内部回调函数：监听 /robot_status，判断任务是否完成"""
        if msg.task_state == 7:  # 任务完成状态码为 7
            rospy.loginfo("✅ 底盘任务已完成（task_state == 7）")
            self.arrive_state = True

    def move_to_marker(self, marker: str, task_id: int = 1, task_exec: int = 1, task_type: int = 1, direction: int = 0):
        """
        控制底盘移动到指定的标记点（通过 /exec_task 服务）
        :param marker: 目标点位标记名称，如 "A", "1", "marker_01"（根据实际定义）
        :param task_id: 任务ID，一般为 1
        :param task_exec: 任务执行标志，一般为 1
        :param task_type: 任务类型，一般为 1（导航任务）
        :param direction: 方向参数，一般为 0
        :return: 是否成功发送任务 & 最终是否到达（bool）
        """
        self.arrive_state = False  # 重置到达状态

        try:
            # 构造请求
            req = ExecTaskRequest()
            req.task_id = task_id
            req.task_exect = task_exec
            req.task_type = task_type
            req.direction = direction
            req.task_type_no = 0         # 根据文档，通常为 0
            req.mark_no = marker         # 目标点位标记，如 "A"

            rospy.loginfo(f"发送导航任务到点位: {marker} (task_type={task_type}, task_id={task_id})")

            # 调用服务
            res = self.navigation_point(req)

            if not res.success:
                rospy.logerr(f"导航任务发送失败！服务返回 success=False")
                return False

            rospy.loginfo(f"导航任务已提交，等待点位 {marker} 到达...")

            # 等待到达（通过状态回调更新 self.arrive_state）
            rate = rospy.Rate(10)  # 10Hz
            timeout_sec = 60
            waited = 0

            while not rospy.is_shutdown() and not self.arrive_state and waited < timeout_sec:
                rate.sleep()
                waited += 1

            if self.arrive_state:
                rospy.loginfo(f"底盘已成功到达点位: {marker}")
                return True
            else:
                rospy.logwarn(f"等待超时或未到达点位: {marker} (timeout={timeout_sec}s)")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"调用 /exec_task 服务失败: {e}")
            return False
        except Exception as e:
            rospy.logerr(f"发生未知错误: {e}")
            return False


    def disconnect(self):
        """
        清理资源
        """
        if self.status_sub:
            self.status_sub.unregister()
            rospy.loginfo("已取消订阅 /robot_status")
        print("[ChassisController] 底盘控制器已断开连接（ROS节点继续运行）")


def test_chassis():
    """测试函数：控制底盘移动到两个点位"""
    print("\n=== 底盘控制器（WOOSH /exec_task）测试 ===")

    chassis = ChassisController()

    try:
        # 测试移动到点位 "A"
        print("1. 测试移动到点位 A")
        if chassis.move_to_marker("A"):
            print("点位 A 导航成功")
        else:
            print("点位 A 导航失败或超时")


    except KeyboardInterrupt:
        print("\n测试被用户中断")
    finally:
        chassis.disconnect()


if __name__ == "__main__":
    test_chassis()