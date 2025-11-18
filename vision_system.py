# -*- coding: utf-8 -*-
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import json
import os


class VisionSystem:
    def __init__(self, config_file='config.json', arm_type='right_arm', model_path='yolov8n.pt'):
        """
        初始化视觉系统
        
        Args:
            config_file: 配置文件路径
            arm_type: 机械臂类型 ('left_arm' 或 'right_arm')
            model_path: YOLO模型路径
        """
        self.model = YOLO(model_path)
        self.class_names = self.model.names
        
        # 从配置文件读取设置
        self.config = self.load_config(config_file)
        self.arm_type = arm_type
        
        if arm_type in self.config:
            arm_config = self.config[arm_type]
            self.camera_sn = arm_config.get('camera_sn')
            # 从config文件中读取target_class，如果没有则使用默认值
            self.target_class = arm_config.get('target_class', 'bottle')
            # 可以读取其他配置项
            self.host = arm_config.get('host')
            self.port = arm_config.get('port')
        else:
            raise ValueError(f"配置文件中未找到 {arm_type} 的配置")
        
        self.pipeline = None
        self.align = None
        self.depth_scale = None
        self.intrinsics = None

    def load_config(self, config_file):
        """加载配置文件"""
        try:
            if not os.path.exists(config_file):
                raise FileNotFoundError(f"配置文件 {config_file} 不存在")
            
            with open(config_file, 'r', encoding='utf-8') as f:
                config = json.load(f)
            
            print("配置文件加载成功")
            return config
        except Exception as e:
            print(f"加载配置文件失败: {e}")
            return {}

    def set_target_class(self, target_class):
        """设置目标检测类别"""
        if target_class in self.class_names.values():
            self.target_class = target_class
            print(f"目标检测类别已设置为: {target_class}")
        else:
            print(f"警告: {target_class} 不在可检测类别中")
            # 显示可用的类别
            available_classes = list(set(self.class_names.values()))
            print(f"可用类别: {available_classes}")

    def get_available_classes(self):
        """获取YOLO模型支持的所有类别"""
        return list(set(self.class_names.values()))

    def initialize_camera(self):
        """初始化指定序列号的 RealSense 相机"""
        try:
            ctx = rs.context()
            devices = ctx.query_devices()

            # 如果指定了序列号，则寻找对应的设备
            selected_device = None
            if self.camera_sn:
                print(f"尝试连接指定序列号的相机: {self.camera_sn}")
                for dev in devices:
                    sn = dev.get_info(rs.camera_info.serial_number)
                    if sn == self.camera_sn:
                        selected_device = dev
                        print(f"找到相机: {sn}")
                        break
                if not selected_device:
                    print(f"未找到序列号为 {self.camera_sn} 的相机")
                    return False
            else:
                if not devices:
                    print("未检测到任何 RealSense 相机")
                    return False
                selected_device = devices[0]  # 默认第一个
                sn = selected_device.get_info(rs.camera_info.serial_number)
                print(f"未指定序列号，使用默认相机: {sn}")

            # 创建 pipeline 和 config
            self.pipeline = rs.pipeline()
            config = rs.config()

            # ✅ 关键：只启用我们想要的那个设备的数据流
            if self.camera_sn:
                config.enable_device(self.camera_sn)

            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

            profile = self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)

            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()

            # 获取相机内参
            color_profile = profile.get_stream(rs.stream.color)
            intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = intrinsics.width
            self.intrinsics.height = intrinsics.height
            self.intrinsics.ppx = intrinsics.ppx
            self.intrinsics.ppy = intrinsics.ppy
            self.intrinsics.fx = intrinsics.fx
            self.intrinsics.fy = intrinsics.fy
            self.intrinsics.model = intrinsics.model
            self.intrinsics.coeffs = intrinsics.coeffs

            print(f"RealSense相机初始化成功 (序列号: {self.camera_sn})")
            print(f"目标检测类别: {self.target_class}")
            return True

        except Exception as e:
            print(f"相机初始化失败: {e}")
            return False

    def get_frames(self):
        """获取对齐的深度和彩色帧"""
        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                return None, None

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            return depth_image, color_image
        except Exception as e:
            print(f"获取帧数据失败: {e}")
            return None, None

    def detect_objects(self, image, target_class=None):
        """检测图像中的物体"""
        if target_class is None:
            target_class = self.target_class
            
        results = self.model(image, verbose=False)
        detections = []

        for result in results:
            boxes = result.boxes
            for box in boxes:
                class_id = int(box.cls[0])
                class_name = self.class_names[class_id]
                if class_name == target_class:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    detections.append({
                        'class': class_name,
                        'bbox': [x1, y1, x2, y2],
                        'confidence': conf
                    })

        return detections

    def detect_all_objects(self, image):
        """检测图像中的所有物体（不限于目标类别）"""
        results = self.model(image, verbose=False)
        detections = []

        for result in results:
            boxes = result.boxes
            for box in boxes:
                class_id = int(box.cls[0])
                class_name = self.class_names[class_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                detections.append({
                    'class': class_name,
                    'bbox': [x1, y1, x2, y2],
                    'confidence': conf
                })

        return detections

    def get_3d_point(self, depth_image, x, y):
        """获取图像中某点的3D坐标(米)"""
        # 添加边界检查
        if x < 0 or x >= depth_image.shape[1] or y < 0 or y >= depth_image.shape[0]:
            return None
            
        depth = depth_image[y, x] * self.depth_scale
        if depth <= 0:  # 无效深度值
            return None
            
        point = rs.rs2_deproject_pixel_to_point(
            self.intrinsics, [x, y], depth)
        return np.array(point)

    def shutdown(self):
        """关闭相机"""
        if self.pipeline:
            self.pipeline.stop()
            print("相机已关闭")


def test_vision():
    """测试视觉系统 - 持续显示检测画面"""
    print("\n=== 视觉系统测试 ===")
    print("按 'q' 键退出测试")
    print("按 's' 键保存当前帧")
    print("按 'c' 键切换检测类别")
    print("按 'a' 键显示所有检测到的物体")

    # 从配置文件初始化视觉系统
    vision = VisionSystem(config_file='config.json', arm_type='right_arm')
    
    # 显示从config文件中读取的target_class
    print(f"从配置文件中读取的目标类别: {vision.target_class}")

    if not vision.initialize_camera():
        return

    try:
        frame_count = 0
        show_all_objects = False  # 是否显示所有物体
        
        # 获取可用的类别
        available_classes = vision.get_available_classes()
        print(f"模型支持的类别: {available_classes}")

        while True:
            # 获取图像帧
            depth_image, color_image = vision.get_frames()
            if depth_image is None or color_image is None:
                continue

            # 创建显示图像的副本
            display_image = color_image.copy()
            
            # 检测物体
            if show_all_objects:
                detections = vision.detect_all_objects(color_image)
                mode_text = "模式: 显示所有物体"
            else:
                detections = vision.detect_objects(color_image)
                mode_text = f"模式: 仅显示 {vision.target_class}"
            
            # 在图像顶部显示检测信息
            info_text = f"Arm: {vision.arm_type} | Camera: {vision.camera_sn}"
            cv2.putText(display_image, info_text, 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.putText(display_image, mode_text, 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.putText(display_image, f"Detected: {len(detections)} object(s)", 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.putText(display_image, "Press 'q':quit, 's':save, 'c':change class, 'a':toggle all", 
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

            # 在图像上绘制检测结果
            for i, det in enumerate(detections):
                x1, y1, x2, y2 = det['bbox']
                
                # 根据是否是目标类别选择颜色
                if det['class'] == vision.target_class:
                    color = (0, 255, 0)  # 绿色：目标类别
                else:
                    color = (255, 0, 0)  # 蓝色：其他类别
                
                # 绘制检测框
                cv2.rectangle(display_image, (x1, y1), (x2, y2), color, 2)
                
                # 绘制中心点
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                cv2.circle(display_image, (center_x, center_y), 5, (0, 0, 255), -1)
                
                # 计算3D坐标
                point_3d = vision.get_3d_point(depth_image, center_x, center_y)
                
                # 显示类别和置信度
                label = f"{det['class']} {det['confidence']:.2f}"
                cv2.putText(display_image, label,
                           (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                           0.6, color, 2)

                # 显示3D坐标（如果有效）
                if point_3d is not None:
                    coord_text = f"3D: ({point_3d[0]:.2f}, {point_3d[1]:.2f}, {point_3d[2]:.2f})m"
                    cv2.putText(display_image, coord_text,
                               (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX,
                               0.5, (0, 255, 255), 2)
                else:
                    cv2.putText(display_image, "3D: Invalid depth",
                               (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX,
                               0.5, (0, 0, 255), 2)

            # 显示图像
            cv2.imshow("Object Detection - Press 'q' to quit", display_image)
            
            frame_count += 1

            # 检测按键
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                # 保存当前帧
                filename = f"detection_{vision.arm_type}_{vision.target_class}_{frame_count}.jpg"
                cv2.imwrite(filename, display_image)
                print(f"帧已保存为: {filename}")
            elif key == ord('c'):
                # 切换检测类别
                if available_classes:
                    current_index = available_classes.index(vision.target_class) if vision.target_class in available_classes else 0
                    next_index = (current_index + 1) % len(available_classes)
                    vision.set_target_class(available_classes[next_index])
            elif key == ord('a'):
                # 切换显示所有物体模式
                show_all_objects = not show_all_objects
                mode = "所有物体" if show_all_objects else f"仅 {vision.target_class}"
                print(f"切换显示模式: {mode}")

    except Exception as e:
        print(f"测试过程中发生错误: {e}")
    finally:
        vision.shutdown()
        cv2.destroyAllWindows()


# 新增函数：可以选择不同的机械臂进行测试
def test_specific_arm(arm_type='right_arm'):
    """测试特定机械臂的视觉系统"""
    print(f"\n=== 测试 {arm_type} 视觉系统 ===")
    
    vision = VisionSystem(config_file='config.json', arm_type=arm_type)
    
    print(f"相机序列号: {vision.camera_sn}")
    print(f"目标检测类别: {vision.target_class}")
    print(f"主机地址: {vision.host}")
    print(f"端口号: {vision.port}")

    if not vision.initialize_camera():
        return False
    
    # 这里可以添加具体的测试逻辑
    print(f"成功初始化 {arm_type} 相机")
    vision.shutdown()
    return True


if __name__ == "__main__":
    # 测试右臂
    test_vision()
    
    # 也可以测试左臂
    # test_specific_arm('left_arm')