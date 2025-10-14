# -*- coding: utf-8 -*-
import cv2
import numpy as np
import json
import time
import threading
from concurrent.futures import ThreadPoolExecutor
import rospy
from scipy.spatial.transform import Rotation as R

from woosh_chassis_controller import ChassisController
from dual_arm_controller import DualArmController, EndEffectorController
from vision_system import VisionSystem


import asyncio
import sys
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


class VisualGraspingDemo:
    def __init__(self, config_file='config.json'):
        self.config_file = config_file
        self.load_config()
        self.load_calibration_params()       
        # 新增稳定性检查相关变量
        self.stable_detection_time = 3.0  # 需要稳定识别3秒
        self.target_candidate = None  # 候选目标
        self.candidate_first_seen = None  # 候选目标首次出现时间
        self.last_stable_check_time = 0  # 上次稳定性检查时间
        self.stable_check_interval = 0.1  # 稳定性检查间隔       
        # 初始化各系统
        self.chassis = ChassisController()
        self.arm_controller = DualArmController()
        self.vision_system = None
        self.end_effector = None
        
        # 线程池用于异步任务
        self.thread_pool = ThreadPoolExecutor(max_workers=3)
        
        # 状态变量
        self.is_initialized = False
        self.target_found = False
        self.target_3d_position = None
        self.latest_detections = None
        self.detection_lock = threading.Lock()  # 检测结果锁
        
        # 配置参数
        self.scan_timeout = self.config.get('scan_timeout', 60)
        self.detection_interval = self.config.get('detection_interval', 0.1)
        self.grasp_height_offset = self.config.get('grasp_height_offset', 0.05)  # 50mm      
            
    def load_config(self):
        """加载配置文件"""
        try:
            with open(self.config_file, 'r', encoding='utf-8') as f:
                self.config = json.load(f)
            print("✅ 配置文件加载成功")
        except Exception as e:
            print(f"❌ 加载配置文件失败: {e}")
            # 使用默认配置
            self.config = {}
            print("⚠️ 使用默认配置")

    def load_calibration_params(self):
        """从配置文件加载相机-机械臂标定参数"""
        try:
            if 'camera_calibration' in self.config:
                calib = self.config['camera_calibration']
                # 加载旋转矩阵
                if 'rotation_matrix' in calib:
                    self.rotation_matrix = np.array(calib['rotation_matrix'])
                    print(f'加载旋转矩阵：{self.rotation_matrix}')
                else:
                    print('未从配置文件中找到参数')
                # 加载平移向量
                if 'translation_vector' in calib:
                    self.translation_vector = np.array(calib['translation_vector'])
                    print(f'加载平移矩阵：{self.translation_vector}')
                else:
                    print('未从配置文件中找到参数')   
                print("✅ 相机标定参数加载成功")
        except Exception as e:
            print(f"❌ 加载标定参数失败: {e}")
            
    def initialize_systems(self):
        """初始化所有系统（增加重试机制）"""
        print("\n=== 初始化系统 ===")
        
        max_retries = 3
        retry_delay = 2  # 秒
        
        for attempt in range(max_retries):
            try:
                print(f"尝试 {attempt + 1}/{max_retries}")
                
                # 1. 初始化底盘
                print("1. 初始化底盘...")
                # 底盘在ChassisController构造函数中已初始化
                
                # 2. 初始化机械臂
                print("2. 初始化机械臂...")
                if not self.arm_controller.connect(arm='both'):
                    raise Exception("机械臂连接失败")
                
                # 3. 初始化末端工具
                print("3. 初始化末端工具...")
                self.end_effector = EndEffectorController(
                    left_arm_client=self.arm_controller.left_arm_client,
                    right_arm_client=self.arm_controller.right_arm_client
                )
                
                # 4. 初始化视觉系统
                print("4. 初始化视觉系统...")
                self.vision_system = VisionSystem(
                    config_file=self.config_file, 
                    arm_type='left_arm'
                )
                if not self.vision_system.initialize_camera():
                    raise Exception("相机初始化失败")
                
                self.is_initialized = True
                print("✅ 所有系统初始化完成")
                return True
                
            except Exception as e:
                print(f"❌ 初始化尝试 {attempt + 1} 失败: {e}")
                if attempt < max_retries - 1:
                    print(f"等待 {retry_delay} 秒后重试...")
                    time.sleep(retry_delay)
                    self.cleanup_partial()  # 清理部分初始化的资源
                else:
                    print("❌ 所有初始化尝试均失败")
                    return False
    
    def cleanup_partial(self):
        """清理部分初始化的资源"""
        if hasattr(self, 'arm_controller') and self.arm_controller:
            try:
                self.arm_controller.disconnect()
            except:
                pass
        
        if hasattr(self, 'vision_system') and self.vision_system:
            try:
                self.vision_system.shutdown()
            except:
                pass
    
    def move_to_scan_position(self,marker):
        """移动到扫描位置（增加超时和重试）"""
        print("\n=== 移动到扫描位置 ===")
        
        max_attempts = 3
        timeout = 60  # 秒
        
        for attempt in range(max_attempts):
            print(f"尝试移动到扫描点位 {marker} (尝试 {attempt + 1}/{max_attempts})")
            
            try:
                # 使用线程执行移动任务，避免阻塞
                move_future = self.thread_pool.submit(
                    self.chassis.move_to_marker,marker)
                
                # 等待完成，带超时
                result = move_future.result(timeout=timeout)
                
                if result:
                    print("✅ 到达扫描位置")
                    return True
                else:
                    print(f"❌ 移动失败 (尝试 {attempt + 1})")
                    
            except TimeoutError:
                print(f"❌ 移动超时 (尝试 {attempt + 1})")
            except Exception as e:
                print(f"❌ 移动异常: {e} (尝试 {attempt + 1})")
            
            if attempt < max_attempts - 1:
                retry_delay = 5
                print(f"等待 {retry_delay} 秒后重试...")
                time.sleep(retry_delay)
        
        print("❌ 所有移动尝试均失败")
        return False
    
    def async_detect_objects(self, color_image):
        """异步执行目标检测（带锁保护）"""
        try:
            detections = self.vision_system.detect_objects(color_image)
            with self.detection_lock:
                self.latest_detections = detections
        except Exception as e:
            print(f"目标检测出错: {e}")
            with self.detection_lock:
                self.latest_detections = None
    
    def get_valid_detections(self):
        """安全获取检测结果"""
        with self.detection_lock:
            return self.latest_detections if self.latest_detections else []

    def scan_for_target(self, timeout=None):
        """优化后的扫描目标物体方法，选择稳定识别3秒以上的最近水瓶"""
        if timeout is None:
            timeout = self.scan_timeout
            
        print(f"\n=== 开始扫描目标物体 (需要稳定识别{self.stable_detection_time}秒) ===")
        
        start_time = time.time()
        last_detection_time = 0
        frames_processed = 0
        detection_count = 0
        
        # 重置稳定性检查变量
        self.target_candidate = None
        self.candidate_first_seen = None
        self.last_stable_check_time = time.time()
        
        # 预分配图像显示窗口
        cv2.namedWindow("Target Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Target Detection", 800, 600)
        
        try:
            while time.time() - start_time < timeout:
                current_time = time.time()
                
                # 1. 获取图像
                depth_image, color_image = self.vision_system.get_frames()
                if depth_image is None or color_image is None:
                    time.sleep(0.05)  # 降低CPU占用
                    continue
                
                frames_processed += 1
                
                # 2. 按固定间隔启动异步检测
                if current_time - last_detection_time >= self.detection_interval:
                    self.thread_pool.submit(
                        self.async_detect_objects, 
                        color_image.copy()  # 避免数据竞争
                    )
                    last_detection_time = current_time
                
                # 3. 处理检测结果
                detections = self.get_valid_detections()
                
                # 4. 稳定性检查和目标选择
                stable_target = self._check_stability_and_select(detections, depth_image, current_time)
                
                if stable_target is not None:
                    target_det, target_3d, stable_duration = stable_target
                    
                    self.target_3d_position = target_3d
                    print(f"✅ 找到稳定目标: 稳定识别{stable_duration:.1f}秒, 距离={np.linalg.norm(target_3d):.3f}m")
                    print(f"相机坐标系3D坐标: X={target_3d[0]:.3f}m, Y={target_3d[1]:.3f}m, Z={target_3d[2]:.3f}m")
                    
                    self.target_found = True
                    cv2.destroyAllWindows()
                    
                    # 输出扫描统计信息
                    scan_duration = time.time() - start_time
                    print(f"扫描统计: {frames_processed}帧处理, 耗时{scan_duration:.2f}秒")
                    return True
                
                # 5. 显示实时画面（降低显示频率）
                if frames_processed % 3 == 0:  # 每3帧显示一次
                    display_image = self._draw_detections(
                        color_image.copy(), detections, depth_image, current_time
                    )
                    cv2.imshow("Target Detection", display_image)
                    cv2.waitKey(1)
                
                # 6. 降低CPU占用
                time.sleep(0.02)
            
            print(f"❌ 扫描超时 ({timeout}秒)，未找到稳定目标")
            return False
            
        except KeyboardInterrupt:
            print("\n扫描被用户中断")
            return False
        finally:
            cv2.destroyAllWindows()

    def _find_closest_bottle(self, bottle_detections, depth_image):
        """
        从水瓶检测结果中找到距离最近的一个
        :param bottle_detections: 水瓶检测结果列表
        :param depth_image: 深度图像
        :return: (最近的检测结果, 3D坐标) 或 None
        """
        closest_distance = float('inf')
        closest_detection = None
        closest_3d = None
        
        for det in bottle_detections:
            # 计算目标中心点
            x1, y1, x2, y2 = det['bbox']
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            
            # 获取3D坐标
            target_3d = self.vision_system.get_3d_point(depth_image, center_x, center_y)
            
            if (target_3d is not None and 
                not np.any(np.isnan(target_3d)) and
                np.linalg.norm(target_3d) > 0.1):  # 过滤无效点
                
                distance = np.linalg.norm(target_3d)  # 计算到相机的距离
                
                if distance < closest_distance:
                    closest_distance = distance
                    closest_detection = det
                    closest_3d = target_3d
        
        if closest_detection is not None:
            print(f"最近水瓶距离: {closest_distance:.3f}m, 置信度: {closest_detection['confidence']:.2f}")
            return closest_detection, closest_3d
        
        return None
    
    def _check_stability_and_select(self, detections, depth_image, current_time):
        """
        检查目标稳定性并选择符合条件的最近水瓶
        :param detections: 检测结果
        :param depth_image: 深度图像
        :param current_time: 当前时间
        :return: 稳定目标 (检测结果, 3D坐标, 稳定时长) 或 None
        """
        # 按固定间隔检查稳定性，避免频繁计算
        if current_time - self.last_stable_check_time < self.stable_check_interval:
            return None
        
        self.last_stable_check_time = current_time
        
        # 筛选出水瓶类别的检测结果
        bottle_detections = [det for det in detections if det['class'] == 'bottle'] if detections else []
        
        if not bottle_detections:
            # 没有检测到水瓶，重置候选目标
            if self.target_candidate is not None:
                print("❌ 候选目标丢失，重新搜索...")
                self.target_candidate = None
                self.candidate_first_seen = None
            return None
        
        # 找到当前帧中距离最近的水瓶
        current_closest = self._find_closest_bottle(bottle_detections, depth_image)
        if current_closest is None:
            return None
        
        current_det, current_3d = current_closest
        current_distance = np.linalg.norm(current_3d)
        
        # 检查是否与候选目标匹配
        if self._is_same_target(current_det, current_3d):
            # 是同一个目标，更新稳定性时间
            stable_duration = current_time - self.candidate_first_seen
            print(f" 目标持续识别: {stable_duration:.1f}s, 距离: {current_distance:.3f}m")
            
            # 检查是否达到稳定时间要求
            if stable_duration >= self.stable_detection_time:
                print(f" 目标稳定识别{stable_duration:.1f}秒，准备抓取！")
                return current_det, current_3d, stable_duration
            
        else:
            # 新的候选目标或目标发生变化
            self.target_candidate = current_det
            self.candidate_first_seen = current_time
            self.candidate_3d = current_3d
            print(f" 新候选目标: 距离{current_distance:.3f}m, 开始稳定性检查...")
        
        return None

    def _is_same_target(self, current_det, current_3d):
        """
        判断当前检测目标是否与候选目标是同一个目标
        :param current_det: 当前检测结果
        :param current_3d: 当前3D坐标
        :return: 是否是同一个目标
        """
        if self.target_candidate is None or self.candidate_3d is None:
            return False
        
        # 计算位置变化
        if hasattr(self, 'candidate_3d'):
            position_change = np.linalg.norm(np.array(current_3d) - np.array(self.candidate_3d))
            
            # 位置变化阈值（50mm）
            position_threshold = 0.05
            
            #  bounding box重叠度检查（简单IOU计算）
            overlap_ratio = self._calculate_bbox_overlap(current_det['bbox'], self.target_candidate['bbox'])
            
            # 综合判断：位置变化小且 bounding box 有重叠
            if position_change < position_threshold and overlap_ratio > 0.3:
                # 更新候选目标的3D坐标
                self.candidate_3d = current_3d
                return True
        
        return False

    def _calculate_bbox_overlap(self, bbox1, bbox2):
        """
        计算两个bounding box的重叠比例
        :param bbox1: [x1, y1, x2, y2]
        :param bbox2: [x1, y1, x2, y2]
        :return: 重叠比例 (0-1)
        """
        x1_1, y1_1, x2_1, y2_1 = bbox1
        x1_2, y1_2, x2_2, y2_2 = bbox2
        
        # 计算交集区域
        inter_x1 = max(x1_1, x1_2)
        inter_y1 = max(y1_1, y1_2)
        inter_x2 = min(x2_1, x2_2)
        inter_y2 = min(y2_1, y2_2)
        
        # 检查是否有交集
        if inter_x1 < inter_x2 and inter_y1 < inter_y2:
            inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1)
            area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
            area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
            
            # 返回与较小box的重叠比例
            min_area = min(area1, area2)
            return inter_area / min_area if min_area > 0 else 0
        
        return 0

    def _draw_detections(self, image, detections, depth_image, current_time=None):
        """在图像上绘制检测结果，显示稳定性信息"""
        # 添加帧率显示
        cv2.putText(image, "Visual Grasping Demo - Stability Check", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if not detections:
            cv2.putText(image, "Scanning for bottles...", (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            return image
        
        # 筛选水瓶检测结果
        bottle_detections = [det for det in detections if det['class'] == 'bottle']
        other_detections = [det for det in detections if det['class'] != 'bottle']
        
        # 显示稳定性信息
        stability_info = "No candidate"
        if self.target_candidate is not None and self.candidate_first_seen is not None and current_time is not None:
            stable_duration = current_time - self.candidate_first_seen
            stability_info = f"Stability: {stable_duration:.1f}s/{self.stable_detection_time}s"
            color = (0, 255, 0) if stable_duration >= self.stable_detection_time else (0, 165, 255)  # 绿色或橙色
        
        cv2.putText(image, f"Bottles: {len(bottle_detections)} | {stability_info}", 
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # 绘制非水瓶目标
        for det in other_detections[:2]:
            x1, y1, x2, y2 = det['bbox']
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 255), 1)  # 黄色细框
        
        # 绘制水瓶目标
        candidate_bbox = self.target_candidate['bbox'] if self.target_candidate else None
        
        for i, det in enumerate(bottle_detections[:4]):  # 最多显示4个水瓶
            x1, y1, x2, y2 = det['bbox']
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            
            # 获取距离信息
            target_3d = self.vision_system.get_3d_point(depth_image, center_x, center_y)
            distance = np.linalg.norm(target_3d) if target_3d is not None else 0
            
            # 判断是否是候选目标
            is_candidate = (candidate_bbox is not None and 
                        self._calculate_bbox_overlap(det['bbox'], candidate_bbox) > 0.3)
            
            if is_candidate:
                # 候选目标：根据稳定性显示不同颜色
                if current_time is not None and self.candidate_first_seen is not None:
                    stable_duration = current_time - self.candidate_first_seen
                    if stable_duration >= self.stable_detection_time:
                        color = (0, 255, 0)  # 绿色：稳定完成
                        thickness = 3
                    else:
                        color = (0, 165, 255)  # 橙色：稳定中
                        thickness = 2
                else:
                    color = (255, 255, 0)  # 青色：新候选
                    thickness = 2
                
                # 显示稳定性进度条
                progress_width = int((x2 - x1) * min(1.0, stable_duration / self.stable_detection_time))
                cv2.rectangle(image, (x1, y2 + 5), (x1 + progress_width, y2 + 10), color, -1)
                cv2.rectangle(image, (x1, y2 + 5), (x2, y2 + 10), (255, 255, 255), 1)
                
            else:
                # 非候选目标：蓝色
                color = (255, 0, 0)
                thickness = 1
            
            cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)
            label = f"Bottle {distance:.2f}m" if target_3d is not None else "Bottle"
            cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
            if is_candidate and current_time is not None and self.candidate_first_seen is not None:
                stable_duration = current_time - self.candidate_first_seen
                cv2.putText(image, f"{stable_duration:.1f}s", (x1, y1 - 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        return image
    
    def camera_to_arm_transform(self, camera_point, current_arm_pose):
        """
        使用convert函数将相机坐标系中的点转换到机械臂基座坐标系,convert里的参数单位是米和弧度
        :param camera_point: 相机坐标系中的点 [x, y, z] (米)
        :param current_arm_pose: 当前机械臂末端位姿 [x, y, z, rx, ry, rz] (微米和毫弧度)
        :return: 机械臂基座坐标系中的点 [x, y, z, rx, ry, rz] (微米和毫弧度)
        """
        try:
            # 相机坐标系中的物体坐标（米）
            x, y, z = camera_point
            
            # 当前机械臂末端位姿
            x1, y1, z1 = current_arm_pose[0:3]  # 位置（微米）
            rx, ry, rz = current_arm_pose[3:6]  # 姿态（弧度）
            
            # 将位置从微米转换为毫米
            x1_mm, y1_mm, z1_mm = x1 / 1000000.0, y1 / 1000000.0, z1 / 1000000.0
            rx_, ry_, rz_ = rx / 1000.0, ry / 1000.0, rz / 1000.0
            
            # 调用convert函数进行坐标转换
            obj_base_pose = self.convert(x, y, z, x1_mm, y1_mm, z1_mm, rx_, ry_, rz_)
            
            # 提取位置并转换为微米
            obj_base_pos_mm = obj_base_pose[0:3] * 1000000  
            orientation = obj_base_pose[3:6] * 1000  # 保持原有姿态
            
            # 组合结果 [x, y, z, rx, ry, rz] (微米和毫弧度)
            result_pose = np.hstack((obj_base_pos_mm, orientation))
            
            print(f"坐标转换结果:")
            print(f"相机坐标: ({x:.3f}, {y:.3f}, {z:.3f}) m")
            print(f"基座坐标: ({result_pose[0]} um, {result_pose[1]} um, {result_pose[2]} um)")
            
            return result_pose.tolist()
            
        except Exception as e:
            print(f"❌ 坐标转换错误: {e}")
            return None

    def convert(self, x, y, z, x1, y1, z1, rx, ry, rz):
        """
        坐标转换函数
        输入：物体相机坐标(x,y,z)米，机械臂末端位姿(x1,y1,z1)毫米，(rx,ry,rz)弧度
        输出：物体基座坐标(x,y,z,rx,ry,rz)毫米和弧度
        """
        try:
            # 这里需要你提供rotation_matrix和translation_vector的定义
            # 如果没有定义，需要先定义这些参数
            if not hasattr(self, 'rotation_matrix') or not hasattr(self, 'translation_vector'):
                self.load_calibration_params()
            
            obj_camera_coordinates = np.array([x, y, z])

            # 机械臂末端的位姿
            end_effector_pose = np.array([x1, y1, z1, rx, ry, rz])
            
            # 将旋转矩阵和平移向量转换为齐次变换矩阵
            T_camera_to_end_effector = np.eye(4)
            T_camera_to_end_effector[:3, :3] = self.rotation_matrix
            T_camera_to_end_effector[:3, 3] = self.translation_vector
            
            # 机械臂末端的位姿转换为齐次变换矩阵
            position = end_effector_pose[:3]
            orientation = R.from_euler('xyz', end_effector_pose[3:], degrees=False).as_matrix()
            T_base_to_end_effector = np.eye(4)
            T_base_to_end_effector[:3, :3] = orientation
            T_base_to_end_effector[:3, 3] = position
            
            # 计算物体相对于机械臂基座的位姿
            obj_camera_coordinates_homo = np.append(obj_camera_coordinates, 1)
            obj_end_effector_coordinates_homo = T_camera_to_end_effector.dot(obj_camera_coordinates_homo)
            obj_base_coordinates_homo = T_base_to_end_effector.dot(obj_end_effector_coordinates_homo)
            obj_base_coordinates = obj_base_coordinates_homo[:3]
            
            # 计算物体的旋转
            obj_orientation_matrix = T_base_to_end_effector[:3, :3].dot(self.rotation_matrix)
            obj_orientation_euler = R.from_matrix(obj_orientation_matrix).as_euler('xyz', degrees=False)
            
            # 组合结果
            obj_base_pose = np.hstack((obj_base_coordinates, obj_orientation_euler))
            obj_base_pose[3:] = rx, ry, rz  # 保持原有姿态
            
            return obj_base_pose
            
        except Exception as e:
            print(f"❌ convert函数错误: {e}")
            return None
    
    def calculate_grasp_pose(self, object_3d_pos_camera):
        """使用convert函数计算抓取位姿"""
        print("\n=== 计算抓取位姿 ===")
        
        if object_3d_pos_camera is None:
            print("❌ 无效的3D坐标")
            return None
        
        try:
            # 获取当前机械臂位姿
            current_pose = self.arm_controller.get_current_pose(arm='left')
            if current_pose is None:
                print("❌ 无法获取当前机械臂位姿")
                return None
            
            # 使用convert函数进行坐标转换
            grasp_pose_base = self.camera_to_arm_transform(object_3d_pos_camera, current_pose)
            if grasp_pose_base is None:
                return None
            
            print(f"抓取位姿(基座坐标系): {grasp_pose_base}")
            return grasp_pose_base
            
        except Exception as e:
            print(f"❌ 抓取位姿计算失败: {e}")
            return None
    
    def execute_grasp_sequence(self, grasp_pose):
        """优化后的抓取序列（增加状态检查）"""
        print("\n=== 执行抓取序列 ===")
        
        if grasp_pose is None:
            print("❌ 无效的抓取位姿")
            return False
        
        try:
            # 预抓取位置
            pre_grasp_pose = [int(x) for x in grasp_pose]  
            # pre_grasp_pose[2] += int(self.grasp_height_offset * 1000 * 1000)  # 转换为微米
            
            # 2. 移动到预抓取位置
            print("移动到预抓取位置...")
            if not self.arm_controller.move_to_pose(pre_grasp_pose, arm='left'):
                return False
            
            # 3. 打开夹爪
            print("打开夹爪...")
            if not self.end_effector.control_gripper(arm='left', open=True):
                print("❌ 夹爪打开失败")
                return False
            time.sleep(0.3)
            
            # 4. 移动到抓取位置
            print("移动到抓取位置...")
            if not self.arm_controller.move_to_pose(grasp_pose, arm='left'):
                return False
            time.sleep(10)
            # 5. 闭合夹爪
            print("闭合夹爪...")
            grasp_success = self.end_effector.control_gripper(
                arm='left', open=False, force=50
            )
            
            if not grasp_success:
                print("❌ 抓取失败，夹爪未闭合")
                return False
            
            # time.sleep(0.5)  # 稳定时间
            
            # # 6. 提升物体
            # print("提升物体...")
            # if not self.arm_controller.move_to_pose(pre_grasp_pose, arm='left'):
            #     return False
            
            print("✅ 抓取完成")
            return True
            
        except Exception as e:
            print(f"❌ 抓取序列执行失败: {e}")
            return False
    
    def run_demo(self):
        """优化后的主运行逻辑"""
        print("\n" + "="*50)
        print("       高效视觉抓取 Demo (优化版)")
        print("="*50)
        scan_marker = str(self.config.get('chassis', {}).get('scan_marker', ''))
        drop_marker = str(self.config.get('chassis', {}).get('drop_marker', ''))
        if not self.initialize_systems():
            return
        
        try:
            # 阶段1: 移动到扫描位置
            # if not self.move_to_scan_position(scan_marker):
            #     return
            # 机械臂运动到识别姿态
            self.arm_controller.move_to_pose([-328304, 209414, -106505, -1768, 1483, -588], arm='left')
            print("移动到识别姿态")
            
            # 扫描目标
            if not self.scan_for_target():
                return
            
            # 计算抓取位姿
            grasp_pose = self.calculate_grasp_pose(self.target_3d_position)
            if not grasp_pose:
                return
                
            # 执行抓取
            if not self.execute_grasp_sequence(grasp_pose):
                return
            
            print("\n Demo执行完成！")
            
        except KeyboardInterrupt:
            print("\n用户中断Demo")
        except Exception as e:
            print(f"\n❌ Demo运行错误: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """资源清理（优化版）"""
        print("\n=== 释放资源 ===")
        
        try:
            if hasattr(self, 'thread_pool'):
                self.thread_pool.shutdown(wait=True)
            
            if hasattr(self, 'arm_controller') and self.arm_controller:
                self.arm_controller.disconnect()
            
            if hasattr(self, 'vision_system') and self.vision_system:
                self.vision_system.shutdown()
                
        except Exception as e:
            print(f"资源清理过程中出错: {e}")
        
        cv2.destroyAllWindows()
        print("✅ 资源已释放")


if __name__ == "__main__":
    asyncio.run(main())
    try:
        demo = VisualGraspingDemo(config_file='config.json')
        demo.run_demo()
    except Exception as e:
        print(f"❌ 程序异常: {e}")
    finally:
        print("程序结束")