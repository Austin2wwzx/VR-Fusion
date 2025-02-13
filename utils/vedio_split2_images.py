import cv2

def extract_frames(video_path, output_folder):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print('Error: Failed to open video file.')
        return

    frame_count = 0
    while True:
        ret, frame = cap.read()

        if not ret:
            break

        frame_path = f'{output_folder}/frame_{frame_count:04d}.png'
        cv2.imwrite(frame_path, frame)

        frame_count += 1

    cap.release()


# 视频文件路径
video_path = './camera/2025-01-10/2025-01-10_17-18-00_racobit@12@192.168.1.81.avi'
# 输出文件夹路径
output_folder = '/Users/austin/Downloads/beijing-2025-01-10/camera/2025-01-10/images'

# 调用函数提取帧
extract_frames(video_path, output_folder)
