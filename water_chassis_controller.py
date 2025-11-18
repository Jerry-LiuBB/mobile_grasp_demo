import socket
import time


class ChassisController:
    def __init__(self, host='192.168.10.10', port=31001):
        self.host = host
        self.port = port
        self.client = None

    def connect(self):
        """连接底盘控制器"""
        try:
            self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client.connect((self.host, self.port))
            print("底盘控制器连接成功")
            return True
        except Exception as e:
            print(f"底盘控制器连接失败: {e}")
            return False

    def move_to_marker(self, marker):
        """移动到底盘标记点"""
        if not self.client:
            print("底盘未连接")
            return False

        move_command = f'/api/move?marker={marker}'
        try:
            self.client.send(move_command.encode('utf-8'))
            print(f"已发送移动指令到标记点 {marker}")
            return True
        except Exception as e:
            print(f"发送移动指令失败: {e}")
            return False

    def wait_for_arrival(self, timeout=30):
        """等待底盘到达目标位置(模拟实现)"""
        print(f"等待底盘移动完成，预计{timeout}秒...")
        time.sleep(3)  # 模拟等待时间
        print("底盘已到达目标位置")
        return True

    def disconnect(self):
        """断开底盘连接"""
        if self.client:
            self.client.close()
            self.client = None
            print("底盘连接已断开")


def test_chassis():
    """测试底盘控制器"""
    print("\n=== 底盘控制器测试 ===")
    chassis = ChassisController()

    if chassis.connect():
        print("1. 测试移动到底盘标记点1")
        if chassis.move_to_marker(1):
            chassis.wait_for_arrival()

        print("\n2. 测试移动到底盘标记点2")
        if chassis.move_to_marker(2):
            chassis.wait_for_arrival()

        chassis.disconnect()


if __name__ == "__main__":
    test_chassis()