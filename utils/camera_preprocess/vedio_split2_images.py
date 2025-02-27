import os
import os.path as osp
import subprocess
import datetime
import cv2

from tqdm import tqdm


def get_vedio_info(file_path: str,
                   verbose: bool) -> int:

    command_get_vedio_info = [
        'ffprobe', 
        '-v', 'error', 
        '-select_streams', 'v:0',
        '-show_entries', 'stream=width,height,r_frame_rate,duration,nb_frames',
        '-of', 'default=noprint_wrappers=1',
        file_path
    ]

    vedio_info_metadata = subprocess.run(command_get_vedio_info, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    vedio_info = vedio_info_metadata.stdout.decode().strip()

    vedio_info_dict = dict()

    for line in vedio_info.splitlines():
        key, value = line.split('=')
        vedio_info_dict[key] = value

    width = int(vedio_info_dict.get('width', 0))
    height = int(vedio_info_dict.get('height', 0))
    r_frame_rate = vedio_info_dict.get('r_frame_rate', '0/1')
    duration = float(vedio_info_dict.get('duration', 0.0))
    nb_frames = int(vedio_info_dict.get('nb_frames', 0))

    numerator, denominator = map(int, r_frame_rate.split('/'))
    floating_fps = numerator / denominator  

    if verbose:
        print('Vedio - {} - infos:'.format(file_path))
        print('Resolution: {}x{} pixel; \nFloating Frame Rate: {} fps ({}); \nDuration: {} s; \nTotal Frames: {} frames.\n'.format(width, height, 
                                                                                                                                   floating_fps, r_frame_rate, 
                                                                                                                                   duration, 
                                                                                                                                   nb_frames))
    return floating_fps

# FIXME: Why the num of key frames extracted by ffmpeg is not equal to the num of frames read??? fxxk ffmpeg!
def get_images_ffmpeg(vedio_file_path: str = './camera/chengdu-2025-02-26/2025-02-26_13-04-01_racobit@12@192.168.1.81.avi', 
                      timestamp_file_path: str = None,
                      output_floder_path: str = './camera/chengdu-2025-02-26/images',
                      verbose: bool = True):
    
    if not osp.exists(output_floder_path):
        os.makedirs(output_floder_path)

    fps = get_vedio_info(file_path=vedio_file_path, verbose=verbose)

    command = [
        'ffmpeg',
        '-i', vedio_file_path,
        # '-vf', 'fps={}'.format(fps),
        '-vf', 'select=gt(scene\,0)',
        '-vsync', '0',
        # '-vframes', '1',
        osp.join(output_floder_path, 'frame_%04d.png')
    ]

    subprocess.run(command)

    # with open(timestamp_file_path, 'r') as file:
    #     single_frame_time_stamp_list = file.readlines()

    # for idx, time_stamp in enumerate(single_frame_time_stamp_list):
    #     time_stamp = int(time_stamp.strip())
    #     dt = datetime.datetime.fromtimestamp(time_stamp / 1000.0)
    #     date_time = dt.strftime('%Y-%m-%d_%H:%M:%S') + '.' + str(time_stamp)[-3:]

    #     old_filename = osp.join(output_floder_path, 'frame_{:04d}.png'.format(idx+1))
    #     new_filename = osp.join(output_floder_path, date_time + '_{}.png'.format(idx))

    #     os.rename(old_filename, new_filename)
    

def get_images_cv(vedio_file_path: str = './camera/chengdu-2025-02-26/2025-02-26_13-04-01_racobit@12@192.168.1.81.avi', 
                  timestamp_file_path: str = None,
                  output_floder_path: str = './camera/chengdu-2025-02-26/images'):
    
    cap = cv2.VideoCapture(vedio_file_path)
    if not cap.isOpened():
        print('Error: Failed to open video file.')
        return
    
    if not osp.exists(output_floder_path):
        os.makedirs(output_floder_path)

    with open(timestamp_file_path, 'r') as file:
        single_frame_time_stamp_list = file.readlines()

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    num_timestamps = len(single_frame_time_stamp_list)

    assert total_frames == num_timestamps, 'The num of key frames in vedio is not equal to the num of timestamps, with {} and {}'.format(total_frames, num_timestamps)

    frame_count = 0
    for idx in tqdm(range(len(single_frame_time_stamp_list))):
        ret, frame = cap.read()

        if not ret:
            break

        time_stamp = int(single_frame_time_stamp_list[frame_count].strip())
        dt = datetime.datetime.fromtimestamp(time_stamp / 1000.0)
        date_time = dt.strftime('%Y-%m-%d_%H:%M:%S') + '.' + str(time_stamp)[-3:]
        frame_filename = date_time + '_{}.png'.format(frame_count)

        frame_path = osp.join(output_floder_path, frame_filename)
        cv2.imwrite(frame_path, frame)

        frame_count += 1

    cap.release()


if __name__ == '__main__':
    VIDEO_FILE_PATH = './camera/chengdu-2025-02-26/2025-02-26_13-04-01_racobit@12@192.168.1.81.avi'
    TIMESTAMP_FILE_PATH = './camera/chengdu-2025-02-26/parameters/2025-02-26_13-04-03timesteamp.txt'
    OUT_FOLDER_PATH = './camera/chengdu-2025-02-26/images'

    get_images_cv(timestamp_file_path=TIMESTAMP_FILE_PATH)
