#!/usr/bin/env python
import rospy
import math
import threading
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from dual_arm_msgs.msg import MoveJ, Lift_Height

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        
        # 创建发布器
        self.left_arm_pub = rospy.Publisher('/l_arm/rm_driver/MoveJ_Cmd', MoveJ, queue_size=10)
        self.right_arm_pub = rospy.Publisher('/r_arm/rm_driver/MoveJ_Cmd', MoveJ, queue_size=10)
        self.base_pub = rospy.Publisher('/smooth_cmd_vel', Twist, queue_size=10)
        self.lift_pub = rospy.Publisher('/l_arm/rm_driver/Lift_SetHeight', Lift_Height, queue_size=10)
        self.r_gripper_pub = rospy.Publisher('/r_arm/rm_driver/Gripper_Set', Gripper_Set, queue_size=10)
        self.l_gripper_pub = rospy.Publisher('/l_arm/rm_driver/Gripper_Set', Gripper_Set, queue_size=10)
        
        # 底盘控制相关变量
        self.base_cmd = Twist()
        self.base_active = False
        self.base_rate = 10  # 发布频率 (Hz)
        self.base_thread = None
        
        # 等待发布器建立连接
        rospy.sleep(1)
        
    def start_base_control(self):
        """启动底盘控制线程"""
        if self.base_thread is None or not self.base_thread.is_alive():
            self.base_active = True
            self.base_thread = threading.Thread(target=self._base_control_loop)
            self.base_thread.daemon = True
            self.base_thread.start()
            rospy.loginfo("Base control thread started")
    
    def stop_base_control(self):
        """停止底盘控制线程"""
        self.base_active = False
        if self.base_thread and self.base_thread.is_alive():
            self.base_thread.join(timeout=1.0)
        # 发送停止命令
        stop_cmd = Twist()
        self.base_pub.publish(stop_cmd)
        rospy.loginfo("Base control thread stopped")
    
    def _base_control_loop(self):
        """底盘控制循环 - 持续发布速度命令"""
        rate = rospy.Rate(self.base_rate)
        while self.base_active and not rospy.is_shutdown():
            try:
                self.base_pub.publish(self.base_cmd)
                rate.sleep()
            except rospy.ROSInterruptException:
                break
    
    def control_base_continuous(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, 
                              angular_x=0.0, angular_y=0.0, angular_z=0.0, duration=None):
        """
        持续控制底盘运动
        linear_x: 前进速度 (m/s)
        angular_z: 旋转速度 (rad/s)
        duration: 持续时间(秒)，如果为None则持续直到调用stop
        """
        # 更新速度命令
        self.base_cmd.linear.x = linear_x
        self.base_cmd.linear.y = linear_y
        self.base_cmd.linear.z = linear_z
        self.base_cmd.angular.x = angular_x
        self.base_cmd.angular.y = angular_y
        self.base_cmd.angular.z = angular_z
        
        # 确保控制线程在运行
        self.start_base_control()
        
        rospy.loginfo(f"Base continuous command: linear.x={linear_x}, angular.z={angular_z}")
        
        # 如果指定了持续时间，在时间到达后自动停止
        if duration is not None:
            def stop_after_delay():
                rospy.sleep(duration)
                if self.base_active:
                    self.control_base_stop()
            
            stop_thread = threading.Thread(target=stop_after_delay)
            stop_thread.daemon = True
            stop_thread.start()
    
    def control_base_stop(self):
        """停止底盘运动"""
        self.base_cmd = Twist()  # 重置所有速度为0
        # 线程继续运行但发布零速度
    
    def control_base_timed(self, linear_x=0.0, angular_z=0.0, duration=2.0):
        """
        定时控制底盘运动
        duration: 运动持续时间(秒)
        """
        rospy.loginfo(f"Base timed move: linear.x={linear_x}, angular.z={angular_z}, duration={duration}s")
        
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        
        # 持续发布指定时间
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz
        
        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            self.base_pub.publish(cmd)
            rate.sleep()
        
        # 发布停止命令
        stop_cmd = Twist()
        self.base_pub.publish(stop_cmd)
        rospy.loginfo("Base timed move completed")
    
    def control_left_arm(self, joint_positions, speed=0.5, trajectory_connect=0):
        """控制左机械臂"""
        msg = MoveJ()
        msg.joint = joint_positions
        msg.speed = speed
        msg.trajectory_connect = trajectory_connect
        
        self.left_arm_pub.publish(msg)
        rospy.loginfo(f"Left arm command: joints={joint_positions}, speed={speed}")
    
    def control_right_arm(self, joint_positions, speed=0.5, trajectory_connect=0):
        """控制右机械臂"""
        msg = MoveJ()
        msg.joint = joint_positions
        msg.speed = speed
        msg.trajectory_connect = trajectory_connect
        
        self.right_arm_pub.publish(msg)
        rospy.loginfo(f"Right arm command: joints={joint_positions}, speed={speed}")
    
    def control_lift(self, height=0, speed=0.5):
        """控制升降系统"""
        msg = Lift_Height()
        msg.height = height
        msg.speed = speed
        
        self.lift_pub.publish(msg)
        rospy.loginfo(f"Lift command: height={height}, speed={speed}")
        
        
    def control_r_gripper(self, position=550):
        """
        控制右夹爪
        position: 夹爪位置0-1000
        """
        msg = Gripper_Set()
        msg.position = position
        self.r_gripper_pub.publish(msg)
        
    def control_l_gripper(self, position=550):
        """
        控制左夹爪
        position: 夹爪位置0-1000
        """
        msg = Gripper_Set()
        msg.position = position
        self.l_gripper_pub.publish(msg)
    
    def demo_sequence(self):
        """演示序列：执行一系列动作"""
        rospy.loginfo("Starting demo sequence...")
        
        try:
            # 1. 控制底盘前进3秒
            rospy.loginfo("Moving base forward for 3 seconds...")
            self.control_base_timed(linear_x=0.5, angular_z=0.0, duration=1.0)
            rospy.sleep(1.0)
            
            # 2. 控制升降系统
            rospy.loginfo("Lifting up...")
            self.control_lift(height=500, speed=10)
            rospy.sleep(3.0)
            
            # 3. 控制左机械臂 (假设6关节)
            rospy.loginfo("Moving left arm...")
            left_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.control_left_arm(left_joints, speed=0.1)
            rospy.sleep(2.0)
            
            # 4. 持续旋转底盘（使用持续控制）
            rospy.loginfo("Rotating base continuously...")
            self.control_base_continuous(linear_x=0.0, angular_z=0.3, duration=4.0)
            rospy.sleep(5.0)  # 等待旋转完成
            
            # 5. 控制右机械臂
            rospy.loginfo("Moving right arm...")
            right_joints = [0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
            self.control_right_arm(right_joints, speed=0.1)
            rospy.sleep(2.0)
            
            # 6. 复杂移动：同时前进和旋转
            #rospy.loginfo("Moving forward while rotating...")
            #self.control_base_continuous(linear_x=0.1, angular_z=0.2, duration=3.0)
            #rospy.sleep(4.0)
            # 7. 夹爪打开（释放物体）
            
            rospy.loginfo("Closing right gripper ...")
            self.control_r_gripper(position=0)  # 打开夹爪
            rospy.sleep(2.0)
            
            rospy.loginfo("Closing left gripper ...")
            self.control_l_gripper(position=0)  # 打开夹爪
            rospy.sleep(2.0)
            
            rospy.loginfo("Demo sequence completed!")
            
        except Exception as e:
            rospy.logerr(f"Demo sequence error: {e}")
        finally:
            # 确保停止所有运动
            self.stop_base_control()

def main():
    try:
        controller = RobotController()
        
        # 等待节点初始化完成
        rospy.sleep(1)
        
        # 运行演示序列
        controller.demo_sequence()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Robot controller node terminated.")
    except Exception as e:
        rospy.logerr(f"Main error: {e}")
    finally:
        # 确保停止底盘控制
        try:
            controller.stop_base_control()
        except:
            pass

if __name__ == '__main__':
    main()