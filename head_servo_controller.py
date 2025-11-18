import serial
import threading
import time
import json
import os
import logging
import platform
from typing import Optional, Tuple, List, Dict, Any

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class ServoController:
    """
    舵机控制库 - 与DualArmController风格一致的版本
    """
    
    def __init__(self, config_file: str = 'config.json'):
        """
        初始化舵机控制器
        
        Args:
            config_file: 配置文件路径，默认为'config.json'
        """
        self.config_file = config_file
        # 通信参数
        self.serial_timeout = 2.0
        self.serial_write_timeout = 2.0
        self.frequency = 10  # Hz，角度读取频率
        
        # 从配置文件加载参数
        self.serial_port = None
        self.baudrate = 9600
        
        # 舵机参数
        self.servo_ranges = {
            1: {'min': 400, 'max': 600, 'name': '抬头/低头舵机'},
            2: {'min': 200, 'max': 800, 'name': '左转/右转舵机'}
        }
        
        # 目标角度
        self.target_angles = {'servo1': 500, 'servo2': 500}
        
        # 运行时状态
        self.serial_conn = None
        self.lock = threading.Lock()
        self.current_angles = {'servo1': 500, 'servo2': 500}
        self.angle_thread = None
        self.is_running = False
        
        # 从配置文件加载参数
        self._load_config()

        # 创建串口客户端（初始为None）
        # 自动连接
        self._auto_connect()

    def _load_config(self):
        """从JSON配置文件加载舵机参数（与DualArmController风格一致）"""
        try:
            if not os.path.exists(self.config_file):
                raise FileNotFoundError(f"配置文件 {self.config_file} 不存在！")

            with open(self.config_file, 'r', encoding='utf-8') as f:
                config = json.load(f)

            # 读取舵机配置
            if 'head_servo' in config:
                servo_config = config['head_servo']
                
                # 串口配置
                serial_config = servo_config.get('serial', {})
                self.serial_port = serial_config.get('port')
                self.baudrate = serial_config.get('baudrate', 9600)
                self.serial_timeout = serial_config.get('timeout', 2.0)
                self.serial_write_timeout = serial_config.get('write_timeout', 2.0)
                self.frequency = serial_config.get('frequency', 10)

                # 读取目标角度配置
                target_config = servo_config.get('target_angles', {})
                self.target_angles['servo1'] = target_config.get('servo1', 500)
                self.target_angles['servo2'] = target_config.get('servo2', 500)

                # 舵机范围配置
                ranges_config = servo_config.get('servo_ranges', {})
                for servo_id, range_config in ranges_config.items():
                    servo_id = int(servo_id)
                    if servo_id in self.servo_ranges:
                        self.servo_ranges[servo_id].update(range_config)

                print(f"已从配置文件加载参数:")
                print(f"  串口 -> 端口: {self.serial_port}, 波特率: {self.baudrate}")
                print(f"  舵机1范围: [{self.servo_ranges[1]['min']}, {self.servo_ranges[1]['max']}]")
                print(f"  舵机2范围: [{self.servo_ranges[2]['min']}, {self.servo_ranges[2]['max']}]")
                print(f"  目标角度: 舵机1={self.target_angles['servo1']}, 舵机2={self.target_angles['servo2']}")
            else:
                raise ValueError("配置文件中未找到'head_servo' section")

        except Exception as e:
            print(f"加载配置文件失败: {e}")
            # 根据系统设置默认值
            system = platform.system()
            if system == "Windows":
                self.serial_port = "COM3"
                self.baudrate = 115200
            else:
                self.serial_port = "/dev/ttyUSB0"
                self.baudrate = 9600
            print(f"使用默认参数: {self.serial_port}, {self.baudrate}bps")

    def _auto_connect(self) -> bool:
        """自动连接串口"""
        return self.connect()

    def connect(self) -> bool:
        """
        连接串口（与机械臂connect方法风格一致）
        
        Returns:
            bool: 连接成功返回True，失败返回False
        """
        success = True

        try:
            if self.serial_conn and self.serial_conn.is_open:
                print("串口已经连接")
                return True

            print(f"尝试连接串口: {self.serial_port}, {self.baudrate}bps")
            
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=self.serial_timeout,
                write_timeout=self.serial_write_timeout
            )
            
            # 清空缓冲区
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            print(f"✓ 串口连接成功: {self.serial_port}, {self.baudrate}bps")
            
            # 启动角度读取线程
            self._start_angle_monitoring()
            
            return True
            
        except Exception as e:
            print(f"✗ 串口连接失败: {e}")
            success = False

        return success

    def disconnect(self, arm='both'):
        """
        断开串口连接（与机械臂disconnect方法风格一致）
        """
        self.is_running = False
        
        if self.angle_thread and self.angle_thread.is_alive():
            self.angle_thread.join(timeout=2)
        
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("串口连接已断开")

    def _start_angle_monitoring(self) -> None:
        """启动角度监控线程"""
        self.is_running = True
        self.angle_thread = threading.Thread(target=self._angle_monitor_loop, daemon=True)
        self.angle_thread.start()
        print("角度监控线程已启动")

    def _angle_monitor_loop(self) -> None:
        """角度监控循环 - 与ROS代码兼容"""
        # 与ROS代码完全一致的读取指令
        read_command = [0x55, 0x55, 0x05, 0x15, 0x02, 0x01, 0x02]
        
        while self.is_running and self.serial_conn and self.serial_conn.is_open:
            try:
                with self.lock:
                    self.serial_conn.write(bytes(read_command))
                
                # 读取数据
                if self.serial_conn.in_waiting > 0:
                    buffer = self.serial_conn.read(self.serial_conn.in_waiting)
                    
                    if len(buffer) >= 11 and buffer[0] == 0x55 and buffer[1] == 0x55:
                        # 解析舵机角度（与ROS代码一致）
                        servo_id_1 = buffer[5]
                        angle_1 = (buffer[7] << 8) | buffer[6]
                        servo_id_2 = buffer[8]
                        angle_2 = (buffer[10] << 8) | buffer[9]
                        
                        with self.lock:
                            self.current_angles['servo1'] = angle_1
                            self.current_angles['servo2'] = angle_2
                        
                        logger.debug(f"舵机1(ID:{servo_id_1}): {angle_1}°, 舵机2(ID:{servo_id_2}): {angle_2}°")
                
                time.sleep(1.0 / self.frequency)
                
            except Exception as e:
                logger.debug(f"角度读取错误: {e}")
                time.sleep(0.1)

    def _send_servo_command(self, servo_id: int, angle: int) -> bool:
        """
        发送舵机控制指令 - 与ROS代码完全兼容
        """
        try:
            # 验证角度范围
            if servo_id not in self.servo_ranges:
                print(f"✗ 无效的舵机ID: {servo_id}")
                return False
            
            range_info = self.servo_ranges[servo_id]
            min_angle = range_info['min']
            max_angle = range_info['max']
            
            if angle < min_angle or angle > max_angle:
                print(f"✗ 舵机{servo_id}({range_info['name']})角度 {angle} 超出范围 [{min_angle}, {max_angle}]")
                return False

            # 与ROS代码完全一致的指令格式
            s_buffer = [0x00] * 10
            s_buffer[0] = 0x55
            s_buffer[1] = 0x55
            s_buffer[2] = 0x08
            s_buffer[3] = 0x03
            s_buffer[4] = 0x01
            s_buffer[5] = 0xe8      # 与ROS代码一致
            s_buffer[6] = 0x03      # 与ROS代码一致
            s_buffer[7] = servo_id
            s_buffer[8] = angle & 0xFF        # 低八位
            s_buffer[9] = (angle >> 8) & 0xFF  # 高八位
            
            with self.lock:
                sent_bytes = self.serial_conn.write(bytes(s_buffer))
            
            if sent_bytes == 10:
                print(f"✓ 舵机{servo_id}({range_info['name']}) -> {angle}° (发送{sent_bytes}字节)")
                return True
            else:
                print(f"✗ 发送字节数不匹配: 期望10, 实际{sent_bytes}")
                return False
                
        except Exception as e:
            print(f"✗ 发送舵机指令失败: {e}")
            return False

    # ==================== 公共API接口（与机械臂风格一致）====================

    def set_servo_angle(self, servo_id: int, angle: int) -> bool:
        """
        设置单个舵机角度
        
        Args:
            servo_id: 舵机ID (1或2)
            angle: 目标角度
            
        Returns:
            bool: 设置成功返回True
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            print("✗ 串口未连接")
            return False
        
        success = self._send_servo_command(servo_id, angle)
        if success:
            self.target_angles[f'servo{servo_id}'] = angle
        return success

    def set_head_angles(self, servo1_angle: Optional[int] = None, servo2_angle: Optional[int] = None) -> bool:
        """
        同时设置两个舵机的角度
        
        Args:
            servo1_angle: 舵机1角度（抬头/低头），None表示不控制
            servo2_angle: 舵机2角度（左转/右转），None表示不控制
            
        Returns:
            bool: 至少一个舵机设置成功返回True
        """
        success_count = 0
        total_commands = 0
        
        if servo1_angle is not None:
            total_commands += 1
            if self.set_servo_angle(1, servo1_angle):
                success_count += 1
        
        if servo2_angle is not None:
            total_commands += 1
            if self.set_servo_angle(2, servo2_angle):
                success_count += 1
        
        if total_commands == 0:
            print("✗ 没有指定要控制的舵机")
            return False
        
        print(f"舵机角度设置完成: {success_count}/{total_commands} 个舵机成功")
        return success_count > 0

    def move_to_target_angles(self) -> bool:
        """
        移动到配置文件中的目标角度
        """
        print(f"移动到目标角度: 舵机1={self.target_angles['servo1']}, 舵机2={self.target_angles['servo2']}")
        return self.set_head_angles(
            servo1_angle=self.target_angles['servo1'],
            servo2_angle=self.target_angles['servo2']
        )

    def get_current_angles(self) -> Dict[str, int]:
        """
        获取当前舵机角度
        
        Returns:
            Dict[str, int]: 包含'servo1'和'servo2'角度的字典
        """
        return self.current_angles.copy()

    def get_target_angles(self) -> Dict[str, int]:
        """
        获取目标舵机角度
        """
        return self.target_angles.copy()

    def reset_head(self) -> bool:
        """
        重置头部到中心位置
        """
        print("重置头部到中心位置")
        return self.set_head_angles(servo1_angle=500, servo2_angle=500)


# 便捷函数 - 与机械臂风格一致
def create_controller(config_file: str = 'config.json') -> ServoController:
    """
    创建舵机控制器的便捷函数
    
    Args:
        config_file: 配置文件路径
        
    Returns:
        ServoController: 舵机控制器实例
    """
    return ServoController(config_file)


# 测试函数（与机械臂测试风格一致）
def test_servo_controller():
    """测试舵机控制器（与test_dual_arm风格一致）"""
    print("\n=== 舵机控制器测试 ===")
    arm_controller = create_controller()

    # 1. 连接
    if not arm_controller.connect():
        return

    try:
        # # 2. 测试获取当前角度
        # print("\n1. 当前舵机角度:", arm_controller.get_current_angles())
        # print("目标角度:", arm_controller.get_target_angles())
        
        # # 3. 测试设置舵机角度
        # print("\n2. 测试设置舵机角度")
        # arm_controller.set_head_angles(servo1_angle=550, servo2_angle=600)
        # time.sleep(2)
        
        # 4. 测试移动到目标角度
        print("\n3. 测试移动到目标角度")
        arm_controller.move_to_target_angles()
        time.sleep(2)
        
        # # 5. 测试重置
        # print("\n4. 重置头部")
        # arm_controller.reset_head()
        # time.sleep(1)

    finally:
        arm_controller.disconnect()


if __name__ == "__main__":
    test_servo_controller()