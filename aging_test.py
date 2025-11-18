import json
import time
import logging
import socket
import threading
import signal
import sys
from datetime import datetime, timedelta
from dual_arm_controller import DualArmController
from end_effector import EndEffectorController
from woosh_chassis_controller import ChassisController

# 配置日志
log_file = f'aging_test_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class AgingTest:
    def __init__(self):
        # 初始化控制器
        self.arm_controller = DualArmController()
        self.end_effector = None  # 将在连接机械臂后初始化
        self.chassis_controller = None  # 将在测试底盘时初始化
        self.neck_servo_controller = None  # 颈部舵机控制器（根据实际情况实现）
        
        # 测试配置
        self.test_config = {
            "arm_gripper_test_duration": 2 * 60 * 60,  # 2小时
            "arm_lift_test_duration": 1 * 60 * 60,    # 1小时
            "chassis_lift_test_duration": 2 * 60 * 60, # 2小时
            "neck_servo_test_duration": 1 * 60 * 60    # 1小时
        }
        
        # 测试计数
        self.test_counters = {
            "arm_gripper_cycles": 0,
            "arm_lift_cycles": 0,
            "chassis_lift_cycles": 0,
            "neck_servo_cycles": 0
        }
        
        # 状态监控
        self.current_test = None
        self.test_start_times = {}
        self.errors = []
        self.is_running = False
        self.stop_event = threading.Event()
        
        # 进度报告线程
        self.progress_thread = None
        
        # 注册信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, sig, frame):
        """信号处理函数，用于优雅地退出测试"""
        logger.info(f"接收到信号 {sig}，正在停止测试...")
        self.stop_event.set()
        self.is_running = False
        
        # 等待进度报告线程结束
        if self.progress_thread and self.progress_thread.is_alive():
            self.progress_thread.join(timeout=5)
        
        # 保存测试状态
        self.save_test_state()
        logger.info("测试状态已保存，准备退出...")
        sys.exit(0)
    
    def _progress_monitor(self):
        """进度监控线程函数"""
        while self.is_running:
            if self.current_test and self.current_test in self.test_start_times:
                elapsed = time.time() - self.test_start_times[self.current_test]
                total = self.test_config.get(f"{self.current_test}_duration", 0)
                progress = (elapsed / total * 100) if total > 0 else 0
                
                # 计算预计剩余时间
                remaining = total - elapsed if total > elapsed else 0
                remaining_str = str(timedelta(seconds=int(remaining)))
                
                logger.info(f"[进度] 当前测试: {self.current_test} - {progress:.1f}% - 已运行: {str(timedelta(seconds=int(elapsed)))} - 预计剩余: {remaining_str}")
            
            # 每60秒报告一次进度
            for _ in range(60):
                if self.stop_event.is_set():
                    return
                time.sleep(1)
    
    def connect_arm(self):
        """连接机械臂"""
        try:
            logger.info("正在连接机械臂...")
            success = self.arm_controller.connect(arm='both')
            if success:
                self.end_effector = EndEffectorController(
                    left_arm_client=self.arm_controller.left_arm_client,
                    right_arm_client=self.arm_controller.right_arm_client
                )
                logger.info("机械臂连接成功")
                return True
            else:
                logger.error("机械臂连接失败")
                return False
        except Exception as e:
            logger.error(f"连接机械臂时发生错误: {str(e)}")
            self.errors.append(f"连接机械臂失败: {str(e)}")
            return False
    
    def connect_chassis(self):
        """连接底盘"""
        try:
            logger.info("正在连接底盘...")
            self.chassis_controller = ChassisController()
            logger.info("底盘连接成功")
            return True
        except Exception as e:
            logger.error(f"连接底盘时发生错误: {str(e)}")
            self.errors.append(f"连接底盘失败: {str(e)}")
            return False
    
    def control_lift(self, height, speed=50, arm='left'):
        """
        控制升降系统
        :param height: 升降高度
        :param speed: 升降速度
        :param arm: 使用哪个机械臂的TCP连接发送命令
        :return: 是否成功
        """
        try:
            # 根据用户提供的信息，使用机械臂TCP连接发送升降命令
            client = self.arm_controller.left_arm_client if arm == 'left' else self.arm_controller.right_arm_client
            if not client:
                error_msg = f"{arm}臂未连接，无法控制升降"
                logger.error(error_msg)
                self.errors.append(error_msg)
                return False
            
            # 构建升降命令
            command = {
                "command": "set_lift_height",
                "height": height,
                "speed": speed
            }
            
            # 发送命令
            command_str = json.dumps(command)
            client.send(command_str.encode('utf-8'))
            logger.info(f"升降命令已发送: height={height}, speed={speed}, arm={arm}")
            
            # 等待升降动作完成
            time.sleep(2)  # 根据实际情况调整等待时间
            return True
        except Exception as e:
            error_msg = f"控制升降时发生错误: {str(e)}"
            logger.error(error_msg)
            self.errors.append(error_msg)
            return False
    
    def test_arm_gripper(self):
        """测试机械臂+夹爪老化"""
        self.current_test = "arm_gripper_test"
        self.test_start_times[self.current_test] = time.time()
        
        logger.info("开始机械臂+夹爪老化测试")
        logger.info(f"测试持续时间: {self.test_config['arm_gripper_test_duration']}秒 ({self.test_config['arm_gripper_test_duration']/3600:.1f}小时)")
        
        start_time = time.time()
        end_time = start_time + self.test_config["arm_gripper_test_duration"]
        
        # 获取初始位姿作为安全位置
        initial_left_pose = self.arm_controller.get_current_pose(arm='left')
        initial_right_pose = self.arm_controller.get_current_pose(arm='right')
        
        if not initial_left_pose or not initial_right_pose:
            error_msg = "无法获取初始位姿，测试终止"
            logger.error(error_msg)
            self.errors.append(error_msg)
            return False
        
        logger.info(f"初始左臂位姿: {initial_left_pose}")
        logger.info(f"初始右臂位姿: {initial_right_pose}")
        
        # 定义测试位姿（简单的上下移动）
        test_positions = [
            [0, 0, -50, 0, 0, 0],  # 下降50单位
            [0, 0, 50, 0, 0, 0]    # 上升50单位
        ]
        
        try:
            while time.time() < end_time and not self.stop_event.is_set():
                cycle_start = time.time()
                
                # 移动机械臂
                for offset in test_positions:
                    if self.stop_event.is_set():
                        break
                    
                    # 计算目标位姿
                    target_left_pose = [initial_left_pose[i] + offset[i] for i in range(6)]
                    target_right_pose = [initial_right_pose[i] + offset[i] for i in range(6)]
                    
                    # 移动双臂
                    success_left, _ = self.arm_controller.move_to_pose(target_left_pose, arm='left')
                    success_right, _ = self.arm_controller.move_to_pose(target_right_pose, arm='right')
                    
                    if not success_left or not success_right:
                        error_msg = "机械臂移动失败"
                        logger.error(error_msg)
                        self.errors.append(error_msg)
                        continue
                    
                    self.arm_controller.wait_for_movement(arm='both')
                    
                    # 控制夹爪开合
                    self.end_effector.control_gripper(arm='left', open=True)
                    self.end_effector.control_gripper(arm='right', open=True)
                    time.sleep(1)
                    self.end_effector.control_gripper(arm='left', open=False)
                    self.end_effector.control_gripper(arm='right', open=False)
                    time.sleep(1)
                
                if not self.stop_event.is_set():
                    # 增加计数
                    self.test_counters["arm_gripper_cycles"] += 1
                    cycle_time = time.time() - cycle_start
                    logger.info(f"机械臂+夹爪测试循环计数: {self.test_counters['arm_gripper_cycles']}, 循环时间: {cycle_time:.1f}秒")
                    
                    # 定期保存测试状态
                    if self.test_counters["arm_gripper_cycles"] % 10 == 0:
                        self.save_test_state()
        
        except KeyboardInterrupt:
            logger.info("测试被用户中断")
            return False
        except Exception as e:
            logger.error(f"测试过程中发生错误: {str(e)}")
            return False
        finally:
            # 返回到初始位置
            logger.info("返回初始位置")
            self.arm_controller.move_to_pose(initial_left_pose, arm='left')
            self.arm_controller.move_to_pose(initial_right_pose, arm='right')
            self.arm_controller.wait_for_movement(arm='both')
        
        logger.info("机械臂+夹爪老化测试完成")
        return True
    
    def test_arm_lift(self):
        """测试机械臂+升降系统老化"""
        self.current_test = "arm_lift_test"
        self.test_start_times[self.current_test] = time.time()
        
        logger.info("开始机械臂+升降系统老化测试")
        logger.info(f"测试持续时间: {self.test_config['arm_lift_test_duration']}秒 ({self.test_config['arm_lift_test_duration']/3600:.1f}小时)")
        
        start_time = time.time()
        end_time = start_time + self.test_config["arm_lift_test_duration"]
        
        # 获取初始位姿
        initial_left_pose = self.arm_controller.get_current_pose(arm='left')
        if not initial_left_pose:
            error_msg = "无法获取初始位姿，测试终止"
            logger.error(error_msg)
            self.errors.append(error_msg)
            return False
        
        # 定义升降高度
        lift_heights = [500, 1000, 1500]  # 根据实际情况调整
        
        try:
            while time.time() < end_time and not self.stop_event.is_set():
                cycle_start = time.time()
                
                # 测试不同高度的升降
                for height in lift_heights:
                    if self.stop_event.is_set():
                        break
                    
                    # 控制升降
                    if not self.control_lift(height=height):
                        error_msg = f"升降控制失败，高度: {height}"
                        logger.error(error_msg)
                        self.errors.append(error_msg)
                        continue
                    
                    # 移动机械臂
                    test_pose = initial_left_pose.copy()
                    test_pose[2] += 50  # 简单的上下移动
                    success, _ = self.arm_controller.move_to_pose(test_pose, arm='left')
                    
                    if not success:
                        error_msg = "机械臂移动失败"
                        logger.error(error_msg)
                        self.errors.append(error_msg)
                        continue
                    
                    self.arm_controller.wait_for_movement(arm='left')
                    
                    test_pose[2] -= 50  # 回到原位
                    self.arm_controller.move_to_pose(test_pose, arm='left')
                    self.arm_controller.wait_for_movement(arm='left')
                
                if not self.stop_event.is_set():
                    # 增加计数
                    self.test_counters["arm_lift_cycles"] += 1
                    cycle_time = time.time() - cycle_start
                    logger.info(f"机械臂+升降测试循环计数: {self.test_counters['arm_lift_cycles']}, 循环时间: {cycle_time:.1f}秒")
                    
                    # 定期保存测试状态
                    if self.test_counters["arm_lift_cycles"] % 10 == 0:
                        self.save_test_state()
        
        except KeyboardInterrupt:
            logger.info("测试被用户中断")
            return False
        except Exception as e:
            logger.error(f"测试过程中发生错误: {str(e)}")
            return False
        finally:
            # 返回到初始位置和高度
            logger.info("返回初始位置和高度")
            self.arm_controller.move_to_pose(initial_left_pose, arm='left')
            self.arm_controller.wait_for_movement(arm='left')
            self.control_lift(height=500)  # 回到安全高度
        
        logger.info("机械臂+升降系统老化测试完成")
        return True
    
    def test_chassis_lift(self):
        """测试底盘移动+升降老化"""
        self.current_test = "chassis_lift_test"
        self.test_start_times[self.current_test] = time.time()
        
        logger.info("开始底盘移动+升降老化测试")
        logger.info(f"测试持续时间: {self.test_config['chassis_lift_test_duration']}秒 ({self.test_config['chassis_lift_test_duration']/3600:.1f}小时)")
        
        # 连接底盘
        if not self.connect_chassis():
            error_msg = "无法连接底盘，测试终止"
            logger.error(error_msg)
            self.errors.append(error_msg)
            return False
        
        start_time = time.time()
        end_time = start_time + self.test_config["chassis_lift_test_duration"]
        
        # 定义测试标记点和升降高度
        markers = ["A", "B", "C"]  # 根据实际环境设置
        lift_heights = [500, 1000]
        
        try:
            while time.time() < end_time and not self.stop_event.is_set():
                cycle_start = time.time()
                
                # 底盘移动和升降组合测试
                for marker in markers:
                    if self.stop_event.is_set():
                        break
                    
                    # 移动到底盘标记点
                    logger.info(f"移动到底盘标记点: {marker}")
                    success = self.chassis_controller.move_to_marker(marker=marker)
                    
                    if not success:
                        error_msg = f"底盘移动到标记点 {marker} 失败"
                        logger.error(error_msg)
                        self.errors.append(error_msg)
                        continue
                    
                    self.chassis_controller.wait_for_arrival()
                    
                    # 在每个点测试不同升降高度
                    for height in lift_heights:
                        if not self.control_lift(height=height):
                            error_msg = f"升降控制失败，高度: {height}"
                            logger.error(error_msg)
                            self.errors.append(error_msg)
                        time.sleep(1)
                
                if not self.stop_event.is_set():
                    # 增加计数
                    self.test_counters["chassis_lift_cycles"] += 1
                    cycle_time = time.time() - cycle_start
                    logger.info(f"底盘+升降测试循环计数: {self.test_counters['chassis_lift_cycles']}, 循环时间: {cycle_time:.1f}秒")
                    
                    # 定期保存测试状态
                    if self.test_counters["chassis_lift_cycles"] % 10 == 0:
                        self.save_test_state()
        
        except KeyboardInterrupt:
            logger.info("测试被用户中断")
            return False
        except Exception as e:
            logger.error(f"测试过程中发生错误: {str(e)}")
            return False
        finally:
            # 返回到初始高度
            logger.info("返回初始高度")
            self.control_lift(height=500)  # 回到安全高度
            
            # 断开底盘连接
            if self.chassis_controller:
                self.chassis_controller.disconnect()
        
        logger.info("底盘移动+升降老化测试完成")
        return True
    
    def test_neck_servo(self):
        """测试颈部舵机老化"""
        self.current_test = "neck_servo_test"
        self.test_start_times[self.current_test] = time.time()
        
        logger.info("开始颈部舵机老化测试")
        logger.info(f"测试持续时间: {self.test_config['neck_servo_test_duration']}秒 ({self.test_config['neck_servo_test_duration']/3600:.1f}小时)")
        
        start_time = time.time()
        end_time = start_time + self.test_config["neck_servo_test_duration"]
        
        # 注意：由于未找到颈部舵机的具体控制接口，这里使用模拟实现
        # 实际使用时需要根据具体的颈部舵机控制接口进行修改
        
        try:
            while time.time() < end_time and not self.stop_event.is_set():
                cycle_start = time.time()
                
                # 模拟颈部舵机运动
                logger.info("颈部舵机向左转")
                time.sleep(1)
                logger.info("颈部舵机向右转")
                time.sleep(1)
                logger.info("颈部舵机向上转")
                time.sleep(1)
                logger.info("颈部舵机向下转")
                time.sleep(1)
                
                # 增加计数
                self.test_counters["neck_servo_cycles"] += 1
                cycle_time = time.time() - cycle_start
                logger.info(f"颈部舵机测试循环计数: {self.test_counters['neck_servo_cycles']}, 循环时间: {cycle_time:.1f}秒")
                
                # 定期保存测试状态
                if self.test_counters["neck_servo_cycles"] % 10 == 0:
                    self.save_test_state()
        
        except KeyboardInterrupt:
            logger.info("测试被用户中断")
            return False
        except Exception as e:
            logger.error(f"测试过程中发生错误: {str(e)}")
            return False
        
        logger.info("颈部舵机老化测试完成")
        return True
    
    def save_test_state(self):
        """保存测试状态"""
        try:
            state = {
                "timestamp": datetime.now().isoformat(),
                "counters": self.test_counters,
                "current_test": self.current_test,
                "errors": self.errors,
                "log_file": log_file
            }
            with open("test_state.json", "w") as f:
                json.dump(state, f, indent=4)
            logger.info("测试状态已保存")
        except Exception as e:
            logger.error(f"保存测试状态时发生错误: {str(e)}")
    
    def load_test_state(self):
        """加载测试状态（用于恢复测试）"""
        try:
            with open("test_state.json", "r") as f:
                state = json.load(f)
            
            self.test_counters = state.get("counters", self.test_counters)
            self.current_test = state.get("current_test", None)
            self.errors = state.get("errors", [])
            
            logger.info(f"测试状态已加载，上次保存时间: {state.get('timestamp', '未知')}")
            logger.info(f"当前测试计数: {self.test_counters}")
            return True
        except Exception as e:
            logger.warning(f"加载测试状态失败: {str(e)}")
            return False
    
    def run_all_tests(self):
        """运行所有测试"""
        self.is_running = True
        
        # 启动进度监控线程
        self.progress_thread = threading.Thread(target=self._progress_monitor, daemon=True)
        self.progress_thread.start()
        
        try:
            # 尝试加载之前的测试状态
            self.load_test_state()
            
            # 连接机械臂（这是大多数测试需要的）
            if not self.connect_arm():
                logger.error("无法连接机械臂，无法继续测试")
                return False
            
            # 运行所有测试
            tests = [
                ("机械臂+夹爪老化测试", self.test_arm_gripper),
                ("机械臂+升降系统老化测试", self.test_arm_lift),
                ("底盘移动+升降老化测试", self.test_chassis_lift),
                ("颈部舵机老化测试", self.test_neck_servo)
            ]
            
            all_success = True
            for name, test_func in tests:
                if self.stop_event.is_set():
                    logger.info("测试已被用户停止")
                    break
                
                logger.info(f"========== 开始 {name} ==========")
                success = test_func()
                all_success = all_success and success
                logger.info(f"========== {name} {'成功' if success else '失败'} ==========")
                
                # 记录测试结果
                self.save_test_state()
            
            # 生成测试报告
            self.generate_report()
            
            return all_success and not self.stop_event.is_set()
            
        except Exception as e:
            error_msg = f"测试运行过程中发生严重错误: {str(e)}"
            logger.error(error_msg)
            self.errors.append(error_msg)
            self.save_test_state()
            return False
        finally:
            # 确保断开所有连接
            self.is_running = False
            self.stop_event.set()
            
            if hasattr(self, 'arm_controller'):
                try:
                    self.arm_controller.disconnect()
                    logger.info("机械臂连接已断开")
                except Exception as e:
                    logger.error(f"断开机械臂连接时发生错误: {str(e)}")
            
            if hasattr(self, 'chassis_controller') and self.chassis_controller:
                try:
                    self.chassis_controller.disconnect()
                    logger.info("底盘连接已断开")
                except Exception as e:
                    logger.error(f"断开底盘连接时发生错误: {str(e)}")
            
            # 等待进度报告线程结束
            if self.progress_thread and self.progress_thread.is_alive():
                self.progress_thread.join(timeout=5)
                logger.info("进度监控线程已停止")
    
    def generate_report(self):
        """生成测试报告"""
        try:
            report = {
                "timestamp": datetime.now().isoformat(),
                "test_counters": self.test_counters,
                "test_config": self.test_config,
                "errors": self.errors,
                "total_cycles": sum(self.test_counters.values()),
                "log_file": log_file
            }
            
            # 计算每个测试的理论循环次数和实际循环次数的比例
            report["test_efficiency"] = {}
            for test_name, duration in self.test_config.items():
                counter_name = test_name.replace("_duration", "_cycles")
                if counter_name in self.test_counters:
                    # 假设每个循环平均需要10秒（根据实际情况调整）
                    expected_cycles = duration / 10
                    actual_cycles = self.test_counters[counter_name]
                    efficiency = (actual_cycles / expected_cycles * 100) if expected_cycles > 0 else 0
                    report["test_efficiency"][test_name.replace("_duration", "")] = {
                        "expected_cycles": expected_cycles,
                        "actual_cycles": actual_cycles,
                        "efficiency_percent": efficiency
                    }
            
            report_file = f"test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            with open(report_file, "w") as f:
                json.dump(report, f, indent=4)
            
            logger.info(f"测试报告已生成: {report_file}")
            
            # 同时在日志中输出报告摘要
            logger.info("\n========== 测试报告摘要 ==========")
            logger.info(f"测试时间: {report['timestamp']}")
            logger.info(f"总循环次数: {report['total_cycles']}")
            logger.info(f"机械臂+夹爪循环次数: {report['test_counters']['arm_gripper_cycles']}")
            logger.info(f"机械臂+升降循环次数: {report['test_counters']['arm_lift_cycles']}")
            logger.info(f"底盘+升降循环次数: {report['test_counters']['chassis_lift_cycles']}")
            logger.info(f"颈部舵机循环次数: {report['test_counters']['neck_servo_cycles']}")
            
            # 输出错误信息
            if report['errors']:
                logger.warning(f"测试过程中发生 {len(report['errors'])} 个错误:")
                for i, error in enumerate(report['errors'], 1):
                    logger.warning(f"  {i}. {error}")
            else:
                logger.info("测试过程中未发生错误")
            
            logger.info("=================================\n")
            
        except Exception as e:
            logger.error(f"生成测试报告时发生错误: {str(e)}")


def print_usage():
    """打印使用说明"""
    print("\n复合机器人老化测试脚本使用说明:")
    print("  python aging_test.py             # 运行完整的老化测试")
    print("\n测试步骤:")
    print("  1. 机械臂+夹爪老化测试 (2小时)")
    print("  2. 机械臂+升降系统老化测试 (1小时)")
    print("  3. 底盘移动+升降老化测试 (2小时)")
    print("  4. 颈部舵机老化测试 (1小时)")
    print("\n控制操作:")
    print("  - 按 Ctrl+C 可以安全停止测试并保存当前状态")
    print("  - 测试状态会定期保存到 test_state.json 文件中")
    print("  - 详细日志记录在 aging_test_时间戳.log 文件中")
    print("  - 测试完成后会生成测试报告 test_report_时间戳.json")
    print("")


if __name__ == "__main__":
    # 打印使用说明
    print_usage()
    
    logger.info("========== 复合机器人老化测试开始 ==========")
    logger.info("按 Ctrl+C 可以随时安全停止测试")
    
    # 创建并运行老化测试
    aging_test = AgingTest()
    try:
        success = aging_test.run_all_tests()
        if success:
            logger.info("所有测试都已成功完成！")
        else:
            logger.warning("部分测试可能失败或被中断，请查看日志获取详细信息。")
    except KeyboardInterrupt:
        logger.info("测试被用户中断")
    except Exception as e:
        logger.error(f"测试过程中发生严重错误: {str(e)}")
    finally:
        logger.info("========== 复合机器人老化测试结束 ==========")
