import socket
import json
import time
import os
import numpy as np
import select
from scipy.spatial.transform import Rotation as R
from end_effector import EndEffectorController


class DualArmController:
    def __init__(self, config_file='config.json'):
        # 默认配置文件路径
        self.config_file = config_file
        # 通信参数
        self.socket_timeout = 2.0
        self.buffer_size = 4096
        # 从配置文件加载参数
        self.left_arm_host = None
        self.left_arm_port = None
        self.right_arm_host = None
        self.right_arm_port = None

        self._load_config()

        # 创建socket客户端（初始为None）
        self.left_arm_client = None
        self.right_arm_client = None

    def _load_config(self):
        """从JSON配置文件加载左右臂的IP和端口信息"""
        try:
            if not os.path.exists(self.config_file):
                raise FileNotFoundError(f"配置文件 {self.config_file} 不存在！")

            with open(self.config_file, 'r', encoding='utf-8') as f:
                config = json.load(f)

            # 读取左臂配置
            left_config = config.get('left_arm', {})
            self.left_arm_host = left_config.get('host')
            self.left_arm_port = left_config.get('port')

            # 读取右臂配置
            right_config = config.get('right_arm', {})
            self.right_arm_host = right_config.get('host')
            self.right_arm_port = right_config.get('port')

            # 检查必要字段是否存在
            if not all([self.left_arm_host, self.left_arm_port]):
                raise ValueError("左臂配置缺少 host 或 port")
            if not all([self.right_arm_host, self.right_arm_port]):
                raise ValueError("右臂配置缺少 host 或 port")

            print(f"已从配置文件加载参数:")
            print(f"  左臂 -> IP: {self.left_arm_host}, 端口: {self.left_arm_port}")
            print(f"  右臂 -> IP: {self.right_arm_host}, 端口: {self.right_arm_port}")

        except Exception as e:
            print(f"加载配置文件失败: {e}")
            raise  # 可以选择抛出异常或设置默认值

    def connect(self, arm='both'):
        success = True

        if arm in ['left', 'both']:
            try:
                self.left_arm_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.left_arm_client.connect((self.left_arm_host, self.left_arm_port))
                print("左臂控制器连接成功")
            except Exception as e:
                print(f"左臂控制器连接失败: {e}")
                success = False

        if arm in ['right', 'both']:
            try:
                self.right_arm_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.right_arm_client.connect((self.right_arm_host, self.right_arm_port))
                print("右臂控制器连接成功")
            except Exception as e:
                print(f"右臂控制器连接失败: {e}")
                success = False

        return success

    def _flush_socket(self, sock):
        """清空Socket接收缓冲区"""
        while True:
            ready, _, _ = select.select([sock], [], [], 0.1)
            if not ready:
                break
            sock.recv(self.buffer_size)

    def _send_command(self, client, command_dict, arm_name):
        """发送命令并确保获取完整响应"""
        if not client:
            print(f"{arm_name}臂未连接")
            return False, None

        try:
            # 1. 清空缓冲区
            self._flush_socket(client)
            
            # 2. 发送命令
            command_str = json.dumps(command_dict)
            full_command = command_str + "\n"
            
            print(f"[发送] {arm_name}臂: {command_str}")
            client.sendall(full_command.encode('utf-8'))
            
            # 3. 读取响应
            response = b''
            start_time = time.time()
            
            while time.time() - start_time < self.socket_timeout:
                ready, _, _ = select.select([client], [], [], 0.1)
                if not ready:
                    continue
                    
                chunk = client.recv(self.buffer_size)
                if not chunk:
                    break
                    
                response += chunk
                if b'\n' in response:
                    line, remaining = response.split(b'\n', 1)
                    try:
                        response_data = json.loads(line.decode('utf-8'))
                        print(f"[接收] {arm_name}臂: {json.dumps(response_data, ensure_ascii=False)}")
                        return True, response_data
                    except json.JSONDecodeError:
                        print(f"[接收] {arm_name}臂(原始): {line.decode('utf-8', errors='ignore')}")
                        continue
                        
            print(f"{arm_name}臂响应超时")
            return False, None
            
        except Exception as e:
            print(f"{arm_name}臂通信错误: {e}")
            return False, None

    def move_to_pose(self, pose, velocity=10, blend_radius=0, arm='left'):
        """
        移动机械臂到指定位姿（改进版）
        :return: (success, response)
        """
        command = {
            "command": "movej_p",
            "pose": pose,
            "v": velocity,
            "r": blend_radius
        }
        
        if arm == 'left':
            return self._send_command(self.left_arm_client, command, "左")
        elif arm == 'right':
            return self._send_command(self.right_arm_client, command, "右")
        else:
            print(f"未知机械臂: {arm}")
            return False, None

    def get_current_pose(self, arm='left'):
        """
        获取当前位姿（改进版）
        :return: [x,y,z,rx,ry,rz] 或 None
        """
        command = {"command": "get_current_arm_state"}
        
        if arm == 'left':
            success, response = self._send_command(
                self.left_arm_client, command, "左"
            )
        elif arm == 'right':
            success, response = self._send_command(
                self.right_arm_client, command, "右"
            )
        else:
            return None
            
        if success and 'arm_state' in response and 'pose' in response['arm_state']:
            return response['arm_state']['pose']
        return None


    def wait_for_movement(self, timeout=10, arm='both'):
        """等待机械臂运动完成(模拟实现)"""
        arms = []
        if arm == 'both':
            arms = ['左', '右']
        elif arm == 'left':
            arms = ['左']
        elif arm == 'right':
            arms = ['右']

        for a in arms:
            print(f"等待{a}臂运动完成，预计{timeout}秒...")

        time.sleep(2)

        for a in arms:
            print(f"{a}臂运动完成")

        return True

    def disconnect(self, arm='both'):
        """断开机械臂连接"""
        if arm in ['left', 'both'] and self.left_arm_client:
            self.left_arm_client.close()
            self.left_arm_client = None
            print("左臂连接已断开")

        if arm in ['right', 'both'] and self.right_arm_client:
            self.right_arm_client.close()
            self.right_arm_client = None
            print("右臂连接已断开")


def test_dual_arm():
    """测试双机械臂控制器"""
    print("\n=== 双机械臂控制器测试 ===")
    arm_controller = DualArmController()

    # 1. 连接双臂
    if not arm_controller.connect(arm='both'):
        return

    try:
        # 2. 测试获取当前位姿
        print("\n1. 当前左臂位姿:", arm_controller.get_current_pose(arm='left'))
        print("当前右臂位姿:", arm_controller.get_current_pose(arm='right'))
        
        # 3. 测试左臂移动
        # print("\n2. 测试左臂移动")
        # left_pose = [-379298, 51957, 127214, 1572, 874, -3112]
        # if arm_controller.move_to_pose(left_pose, arm='left'):
        #     arm_controller.wait_for_movement(arm='left')
        """
        # 4. 测试右臂移动
        print("\n3. 测试右臂移动")
        right_pose = [350, -250, 450, 0, 180, 0]
        if arm_controller.move_to_pose(right_pose, arm='right'):
            arm_controller.wait_for_movement(arm='right')

        # 5. 测试双臂协同移动
        print("\n4. 测试双臂协同移动")
        left_pose = [300, 200, 400, 0, 180, 0]
        right_pose = [300, -200, 400, 0, 180, 0]

        arm_controller.move_to_pose(left_pose, arm='left')
        arm_controller.move_to_pose(right_pose, arm='right')
        arm_controller.wait_for_movement(arm='both')
        """
    finally:
        arm_controller.disconnect(arm='both')

def test_dual_arm_with_tools():
    print("\n=== 双机械臂及末端工具控制测试 ===")

    # 1. 创建机械臂控制器并连接
    arm_controller = DualArmController()
    if not arm_controller.connect(arm='both'):
        return
    print("\n1. 当前左臂位姿:", arm_controller.get_current_pose(arm='left'))
    print("当前右臂位姿:", arm_controller.get_current_pose(arm='right'))
    # 2. 创建末端工具控制器，并传入左右臂的 socket 客户端
    end_effector = EndEffectorController(
        left_arm_client=arm_controller.left_arm_client,
        right_arm_client=arm_controller.right_arm_client
    )

    try:
        """
        # 3. 测试运动控制（原 DualArmController 方法）
        left_pose = [350, 250, 450, 0, 180, 0]
        right_pose = [350, -250, 450, 0, 180, 0]

        print("\n移动左臂到位置...")
        arm_controller.move_to_pose(left_pose, arm='left')
        arm_controller.wait_for_movement(arm='left')

        print("移动右臂到位置...")
        arm_controller.move_to_pose(right_pose, arm='right')
        arm_controller.wait_for_movement(arm='right')
        """
        # 4. 测试夹爪控制
        print("\n控制夹爪：")
        print("左臂夹爪闭合...")
        end_effector.control_gripper(arm='left', open=False)
        print("右臂夹爪闭合...")
        end_effector.control_gripper(arm='right', open=False)
        time.sleep(5)
        # print("左臂夹爪打开...")
        end_effector.control_gripper(arm='left', open=True)
        print("右臂夹爪打开...")
        end_effector.control_gripper(arm='right', open=True)
        time.sleep(1)

        # 5. 测试灵巧手控制
        # print("\n控制灵巧手姿势：")
        # print("左臂灵巧手张开（posture=1）...")
        # end_effector.control_hand(arm='left', open=True, posture_num=1)
        # print("右臂灵巧手闭合（posture=2）...")
        # end_effector.control_hand(arm='right', open=False, posture_num=2)

    finally:
        time.sleep(20)
        # 6. 断开连接
        arm_controller.disconnect(arm='both')

if __name__ == "__main__":
    # test_dual_arm()
    test_dual_arm_with_tools()