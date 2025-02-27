import cv2
import os
from datetime import timedelta

def extract_frames(video_path, output_folder, start_time, frame_rate):
    # 打开视频文件
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print('Error: Failed to open video file.')
        return

    # 创建输出文件夹（如果不存在）
    os.makedirs(output_folder, exist_ok=True)

    frame_count = 0
    while True:
        ret, frame = cap.read()

        if not ret:
            break

        # 计算当前帧的时间戳（精确到毫秒）
        timestamp_seconds = frame_count / frame_rate
        timestamp = start_time + timedelta(seconds=timestamp_seconds)

        # 格式化时间戳为时:分:秒.毫秒，保留3位毫秒
        timestamp_str = str(timestamp)
        
        if '.' in timestamp_str:
            time_str, milliseconds = timestamp_str.split('.')
            milliseconds = milliseconds[:3]  # 取前三位毫秒
        else:
            time_str = timestamp_str
            milliseconds = '000'  # 如果没有毫秒部分，默认给 000

        formatted_time = f"{time_str}.{milliseconds}"

        # 生成帧的文件名
        frame_filename = f'{formatted_time}_{frame_count:04d}.png'
        frame_path = os.path.join(output_folder, frame_filename)

        # 保存帧
        cv2.imwrite(frame_path, frame)

        frame_count += 1

    cap.release()


# 视频文件路径
video_path = './camera/2025-01-10/2025-01-10_17-18-00_racobit@12@192.168.1.81.avi'
# 输出文件夹路径
output_folder = '/Users/austin/Downloads/beijing-2025-01-10/camera/2025-01-10/images'

# 视频起始时间：17:18:00.000
start_time = timedelta(hours=17, minutes=18, seconds=0)

# 帧率：23.926587112996742 FPS
frame_rate = 23.926587112996742

# 调用函数提取帧
extract_frames(video_path, output_folder, start_time, frame_rate)