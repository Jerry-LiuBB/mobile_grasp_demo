## 移动抓取grasp_demo说明


### 代码包结构

使用单元模块结构，将相机、底盘、机械臂、末端工具解耦，方便每个模块的独立测试与修改，注意！！！！测试的时候请保证机器人一米范围内空旷，避免造成碰撞及伤害

├── config.json                     #参数文件，通过修改其中的参数连接机器人各个模块
├── demo.py                         #抓取demo
├── dual_arm_controller.py          #机械臂控制脚本，可以运行此脚本验证机械臂是否正常
├── end_effector.py                 #末端控制脚本
├── get_camera_sn.py                #快速获取机器人当前相机序列号脚本
├── readme.md                       
├── vision_system.py                #相机驱动脚本，带视觉识别
├── water_chassis_controller.py     #云迹water2底盘控制脚本
├── woosh_chassis_controller.py     #悟时底盘驱动脚本
└── yolov8n.pt                      #yolov8识别模型权重

### 运行步骤

1.单元测试

若对每个模块进行单独测试，则使用python3 ****.py即可，例如测试相机模块
`python3 vision_system.py`

2.移动抓取demo运行
 首先更改config文件中的参数
 
{
    "left_arm": {
        "host": "169.254.128.18",
        "port": 8080,
        "camera_sn": "425122070186"   #左臂相机序列号
    },
    "right_arm": {
        "host": "169.254.128.19",
        "port": 8080,
        "camera_sn": "425122070185"
    },
    "chassis": {
        "scan_marker": "123",
        "drop_marker": "456"
    },
    "vision": {
        "model_path": "yolov8n.pt",
        "timeout": 30
    },
    "camera_calibration": {
        "rotation_matrix": [
            [0, 1, 0],
            [-1, 0, 0],
            [0, 0, 1]
        ],
        "translation_vector": [-0.085, 0.03, -0.12]
    }
}

首先根据底盘使用说明建立好当前环境地图，设置好地图点，然后在config.json文件中的chassis字段下，替换地图点名称为你设置的名称，然后确认left_arm字段下的target_class为你想要抓取的物体（demo中默认左臂进行抓取水瓶），最后在camera_calibration字段下替换为你的手眼标定结果。
运行`python3 demo.py`开始移动抓取