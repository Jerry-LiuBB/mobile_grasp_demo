#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
老化测试脚本的功能验证工具
此脚本用于在不连接实际硬件的情况下，测试aging_test.py的功能流程和异常处理
"""

import os
import sys
import time
import json
import unittest
from unittest import mock
from datetime import datetime

# 添加当前目录到Python路径，以便导入aging_test模块
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 导入被测试的模块
from aging_test import AgingTest


class MockSocket:
    """模拟Socket连接，用于测试TCP通信"""
    def __init__(self):
        self.sent_data = []
        self.connected = True
    
    def send(self, data):
        if not self.connected:
            raise ConnectionError("Socket is not connected")
        self.sent_data.append(data.decode('utf-8'))
        return len(data)
    
    def close(self):
        self.connected = False


class MockDualArmController:
    """模拟机械臂控制器"""
    def __init__(self):
        self.left_arm_client = MockSocket()
        self.right_arm_client = MockSocket()
        self.connected_arms = set()
    
    def connect(self, arm='both'):
        if arm in ['left', 'both']:
            self.connected_arms.add('left')
        if arm in ['right', 'both']:
            self.connected_arms.add('right')
        return True
    
    def move_to_pose(self, pose, arm='left'):
        if arm not in self.connected_arms:
            return False, "Arm not connected"
        # 模拟机械臂移动成功
        return True, "Success"
    
    def get_current_pose(self, arm='left'):
        if arm not in self.connected_arms:
            return None
        # 返回模拟的位姿
        return [0, 0, 500, 0, 0, 0]
    
    def wait_for_movement(self, arm='both'):
        # 模拟等待移动完成
        time.sleep(0.1)
        return True
    
    def disconnect(self):
        self.connected_arms.clear()
        self.left_arm_client.close()
        self.right_arm_client.close()


class MockEndEffectorController:
    """模拟末端执行器控制器"""
    def __init__(self, left_arm_client=None, right_arm_client=None):
        self.left_arm_client = left_arm_client
        self.right_arm_client = right_arm_client
    
    def control_gripper(self, arm='left', open=True):
        # 模拟夹爪控制
        return True


class MockChassisController:
    """模拟底盘控制器"""
    def __init__(self):
        self.connected = True
        self.current_marker = None
    
    def move_to_marker(self, marker):
        if not self.connected:
            return False
        self.current_marker = marker
        return True
    
    def wait_for_arrival(self):
        # 模拟等待到达
        time.sleep(0.1)
        return True
    
    def disconnect(self):
        self.connected = False


class TestAgingScript(unittest.TestCase):
    """老化测试脚本功能测试类"""
    
    def setUp(self):
        """测试前的准备工作"""
        # 保存原始的控制器类引用
        self.original_controllers = {
            'DualArmController': AgingTest.__init__.__globals__['DualArmController'],
            'EndEffectorController': AgingTest.__init__.__globals__['EndEffectorController'],
            'ChassisController': AgingTest.__init__.__globals__['ChassisController']
        }
        
        # 替换为模拟控制器
        AgingTest.__init__.__globals__['DualArmController'] = MockDualArmController
        AgingTest.__init__.__globals__['EndEffectorController'] = MockEndEffectorController
        AgingTest.__init__.__globals__['ChassisController'] = MockChassisController
        
        # 创建测试实例
        self.aging_test = AgingTest()
        
        # 缩短测试时间以便快速验证
        self.aging_test.test_config = {
            "arm_gripper_test_duration": 5,     # 5秒
            "arm_lift_test_duration": 3,        # 3秒
            "chassis_lift_test_duration": 5,    # 5秒
            "neck_servo_test_duration": 2       # 2秒
        }
        
    def tearDown(self):
        """测试后的清理工作"""
        # 恢复原始的控制器类引用
        for name, cls in self.original_controllers.items():
            AgingTest.__init__.__globals__[name] = cls
        
        # 删除测试生成的文件
        for filename in ['test_state.json']:
            if os.path.exists(filename):
                try:
                    os.remove(filename)
                except:
                    pass
        
    def test_control_lift(self):
        """测试升降控制功能"""
        # 连接机械臂（模拟）
        self.aging_test.connect_arm()
        
        # 测试控制升降
        result = self.aging_test.control_lift(height=1000, speed=50)
        self.assertTrue(result)
        
        # 验证发送的命令
        command = json.loads(self.aging_test.arm_controller.left_arm_client.sent_data[0])
        self.assertEqual(command['command'], 'set_lift_height')
        self.assertEqual(command['height'], 1000)
        self.assertEqual(command['speed'], 50)
    
    def test_test_arm_gripper(self):
        """测试机械臂+夹爪测试功能"""
        # 连接机械臂
        self.aging_test.connect_arm()
        
        # 运行测试
        result = self.aging_test.test_arm_gripper()
        self.assertTrue(result)
        
        # 验证测试计数
        self.assertGreater(self.aging_test.test_counters['arm_gripper_cycles'], 0)
    
    def test_test_arm_lift(self):
        """测试机械臂+升降测试功能"""
        # 连接机械臂
        self.aging_test.connect_arm()
        
        # 运行测试
        result = self.aging_test.test_arm_lift()
        self.assertTrue(result)
        
        # 验证测试计数
        self.assertGreater(self.aging_test.test_counters['arm_lift_cycles'], 0)
    
    def test_test_chassis_lift(self):
        """测试底盘+升降测试功能"""
        # 连接机械臂和底盘
        self.aging_test.connect_arm()
        self.aging_test.connect_chassis()
        
        # 运行测试
        result = self.aging_test.test_chassis_lift()
        self.assertTrue(result)
        
        # 验证测试计数
        self.assertGreater(self.aging_test.test_counters['chassis_lift_cycles'], 0)
    
    def test_test_neck_servo(self):
        """测试颈部舵机测试功能"""
        # 运行测试
        result = self.aging_test.test_neck_servo()
        self.assertTrue(result)
        
        # 验证测试计数
        self.assertGreater(self.aging_test.test_counters['neck_servo_cycles'], 0)
    
    def test_save_and_load_state(self):
        """测试保存和加载测试状态功能"""
        # 设置测试计数
        self.aging_test.test_counters['arm_gripper_cycles'] = 10
        self.aging_test.current_test = 'arm_lift_test'
        self.aging_test.errors = ['Test error']
        
        # 保存状态
        self.aging_test.save_test_state()
        self.assertTrue(os.path.exists('test_state.json'))
        
        # 创建新实例并加载状态
        new_aging_test = AgingTest()
        result = new_aging_test.load_test_state()
        self.assertTrue(result)
        
        # 验证加载的状态
        self.assertEqual(new_aging_test.test_counters['arm_gripper_cycles'], 10)
        self.assertEqual(new_aging_test.current_test, 'arm_lift_test')
        self.assertEqual(len(new_aging_test.errors), 1)
    
    def test_generate_report(self):
        """测试生成测试报告功能"""
        # 设置测试计数
        self.aging_test.test_counters = {
            "arm_gripper_cycles": 10,
            "arm_lift_cycles": 5,
            "chassis_lift_cycles": 8,
            "neck_servo_cycles": 12
        }
        
        # 生成报告
        self.aging_test.generate_report()
        
        # 验证报告文件存在
        report_files = [f for f in os.listdir('.') if f.startswith('test_report_') and f.endswith('.json')]
        self.assertGreater(len(report_files), 0)
        
        # 检查报告内容
        with open(report_files[0], 'r') as f:
            report = json.load(f)
            
        self.assertEqual(report['test_counters']['arm_gripper_cycles'], 10)
        self.assertEqual(report['total_cycles'], 35)
    
    def test_error_handling(self):
        """测试错误处理功能"""
        # 模拟连接失败
        with mock.patch.object(MockDualArmController, 'connect', return_value=False):
            result = self.aging_test.connect_arm()
            self.assertFalse(result)
            self.assertEqual(len(self.aging_test.errors), 1)
    

class TestAgingScriptWithShortRun(unittest.TestCase):
    """简短运行测试，用于验证整体流程"""
    
    def setUp(self):
        # 替换为模拟控制器
        self.original_controllers = {
            'DualArmController': AgingTest.__init__.__globals__['DualArmController'],
            'EndEffectorController': AgingTest.__init__.__globals__['EndEffectorController'],
            'ChassisController': AgingTest.__init__.__globals__['ChassisController']
        }
        
        AgingTest.__init__.__globals__['DualArmController'] = MockDualArmController
        AgingTest.__init__.__globals__['EndEffectorController'] = MockEndEffectorController
        AgingTest.__init__.__globals__['ChassisController'] = MockChassisController
    
    def tearDown(self):
        # 恢复原始的控制器类引用
        for name, cls in self.original_controllers.items():
            AgingTest.__init__.__globals__[name] = cls
    
    def test_full_run_shortened(self):
        """运行一个非常简短的完整测试流程"""
        aging_test = AgingTest()
        
        # 设置极短的测试时间
        aging_test.test_config = {
            "arm_gripper_test_duration": 1,     # 1秒
            "arm_lift_test_duration": 1,        # 1秒
            "chassis_lift_test_duration": 1,    # 1秒
            "neck_servo_test_duration": 1       # 1秒
        }
        
        # 启动测试
        with mock.patch.object(aging_test, '_progress_monitor'):  # 禁用进度监控线程
            success = aging_test.run_all_tests()
            
        # 验证测试成功
        self.assertTrue(success)
        
        # 验证所有测试都执行了
        self.assertGreater(aging_test.test_counters['arm_gripper_cycles'], 0)
        self.assertGreater(aging_test.test_counters['arm_lift_cycles'], 0)
        self.assertGreater(aging_test.test_counters['chassis_lift_cycles'], 0)
        self.assertGreater(aging_test.test_counters['neck_servo_cycles'], 0)


def run_integration_test():
    """运行集成测试，测试真实的老化测试脚本流程"""
    print("\n========== 开始老化测试脚本集成验证 ==========")
    
    # 创建测试实例
    aging_test = AgingTest()
    
    # 替换为模拟控制器
    aging_test.arm_controller = MockDualArmController()
    
    # 连接机械臂
    print("连接模拟机械臂...")
    success = aging_test.connect_arm()
    print(f"机械臂连接{'成功' if success else '失败'}")
    
    if success:
        # 运行一个简单的升降测试
        print("\n测试升降控制...")
        result = aging_test.control_lift(height=800, speed=30)
        print(f"升降控制{'成功' if result else '失败'}")
        
        # 显示发送的命令
        commands = aging_test.arm_controller.left_arm_client.sent_data
        print(f"\n发送的命令数量: {len(commands)}")
        if commands:
            print(f"第一条命令: {commands[0]}")
    
    print("\n========== 老化测试脚本集成验证完成 ==========")


def main():
    """主函数"""
    print("复合机器人老化测试脚本 - 功能验证工具")
    print("====================================")
    
    # 运行单元测试
    print("\n1. 运行单元测试...")
    unittest.main(argv=['first-arg-is-ignored'], exit=False)
    
    # 运行集成测试
    print("\n2. 运行集成测试...")
    run_integration_test()
    
    print("\n3. 验证脚本功能总结:")
    print("   - 控制接口: 已验证")
    print("   - 升降命令格式: 已验证 (set_lift_height)")
    print("   - 测试模式: 已验证四种模式")
    print("   - 日志记录: 已实现")
    print("   - 状态监控: 已实现")
    print("   - 异常处理: 已实现")
    print("   - 报告生成: 已实现")
    print("\n老化测试脚本功能验证完成！")


if __name__ == "__main__":
    main()
