import json


class EndEffectorController:
    def __init__(self, left_arm_client=None, right_arm_client=None):
        """
        初始化末端工具控制器
        :param left_arm_client: 左臂的socket客户端
        :param right_arm_client: 右臂的socket客户端
        """
        self.left_client = left_arm_client
        self.right_client = right_arm_client

    def control_gripper(self, arm='left', open=True, speed=500, force=200, block=True):
        """
        控制夹爪开合
        :param arm: 'left' 或 'right'
        :param open: True=打开夹爪, False=闭合夹爪
        :param speed: 速度
        :param force: 力度
        :param block: 是否阻塞
        :return: 是否成功发送指令
        """
        client = self.left_client if arm == 'left' else self.right_client
        arm_name = '左' if arm == 'left' else '右'

        if not client:
            print(f"{arm_name}臂未连接，无法控制夹爪")
            return False

        command_type = "set_gripper_release" if open else "set_gripper_pick"

        command = {
            "command": command_type,
            "speed": speed,
            "force": force,
            "block": block
        }

        try:
            command_str = json.dumps(command)
            client.send(command_str.encode('utf-8'))
            print(f"{arm_name}臂夹爪{'打开' if open else '闭合'}指令已发送")
            return True
        except Exception as e:
            print(f"{arm_name}臂夹爪控制失败: {e}")
            return False

    def control_hand(self, arm='left', open=True, posture_num=1, block=True):
        """
        控制灵巧手姿势（示例：1=张开，2=握拳）
        :param arm: 'left' 或 'right'
        :param open: True 可以映射为张开姿势，False为握拳或其他
        :param posture_num: 手势编号，根据实际协议定义
        :param block: 是否阻塞执行
        :return: 是否成功发送指令
        """
        client = self.left_client if arm == 'left' else self.right_client
        arm_name = '左' if arm == 'left' else '右'

        if not client:
            print(f"{arm_name}臂未连接，无法控制灵巧手")
            return False

        # 假设 posture_num: 1 表示张开，2 表示闭合
        actual_posture = 1 if open else 2

        command = {
            "command": "set_hand_posture",
            "posture_num": actual_posture,
            "block": block
        }

        try:
            command_str = json.dumps(command)
            client.send(command_str.encode('utf-8'))
            print(f"{arm_name}臂灵巧手设置为{'张开' if open else '闭合'}姿势（posture={actual_posture}）")
            return True
        except Exception as e:
            print(f"{arm_name}臂灵巧手控制失败: {e}")
            return False