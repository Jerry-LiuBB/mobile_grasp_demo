import pyrealsense2 as rs

def list_realsense_cameras():
    ctx = rs.context()
    devices = ctx.query_devices()
    if not devices:
        print("未检测到任何 RealSense 相机")
        return

    print("检测到的 RealSense 相机：")
    for i, dev in enumerate(devices):
        sn = dev.get_info(rs.camera_info.serial_number)
        name = dev.get_info(rs.camera_info.name)
        print(f"[{i}] 相机名称: {name}, 序列号: {sn}")

if __name__ == "__main__":
    list_realsense_cameras()