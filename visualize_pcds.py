import open3d as o3d
import numpy as np
import os
import os.path as osp
import multiprocessing
import time
import joblib

# from utils import RadarData
from natsort import natsorted


def visualize_PCs(queue: multiprocessing.Queue):
    
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    lidar_pcd = o3d.geometry.PointCloud()
    # radar_pcd = o3d.geometry.PointCloud()
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])
    
    first_add = True

    while True:
        if not queue.empty():
            lidar_file_path, radar_file_idx = queue.get()
            
            lidar_point_clouds = o3d.io.read_point_cloud(lidar_file_path)
            lidar_point_clouds = np.asarray(lidar_point_clouds.points).reshape((-1, 3))
            
            lidar_pcd.points = o3d.utility.Vector3dVector(lidar_point_clouds)
            lidar_pcd.paint_uniform_color([0, 0, 1])

            # radar_point_clouds = radar_dets_list[radar_file_idx]
            # radar_point_clouds = np.asarray([[np.asarray([item.add_pos_x, -item.add_pos_y+24, 0]) for item in radar_point_clouds]]).reshape((-1, 3))
            # radar_pcd.points = o3d.utility.Vector3dVector(radar_point_clouds)
            # radar_pcd.paint_uniform_color([1, 0, 0])

            if first_add:
                vis.add_geometry(coordinate_frame)
                vis.add_geometry(lidar_pcd)
                # vis.add_geometry(radar_pcd)
                first_add = False

            opt = vis.get_render_option()
            opt.point_size = 4
            
            vis.update_geometry(coordinate_frame)
            vis.update_geometry(lidar_pcd)
            # vis.update_geometry(radar_pcd)

        vis.poll_events()
        vis.update_renderer()

    # TODO: 添加窗口终止逻辑
    vis.destroy_window()

def update_queue(lidar_file_path_list: list, 
                 radar_file_path_list: list,
                 queue: multiprocessing.Queue):
    frame = 0

    # 现在的逻辑是按照 frame 计数进行播放，可更改为按键播放

    while True:
        lidar_file_path = lidar_file_path_list[frame]
        # radar_file_path = radar_file_path_list[frame]
        # print('LiDAR file path: {}, RADAR file path: {}'.format(lidar_file_path.split('/')[-1], radar_file_path.split('/')[-1]))

        if queue.empty():
            queue.put([lidar_file_path, frame])

        frame += 1
        frame %= len(lidar_file_path_list)

        time.sleep(0.2)
    

if __name__ == '__main__':
    DATA_FILE_DIR = '/Users/austin/Downloads/VRFusion/lidar/beijing-2025-02-16/PCD'
    file_path_list = [osp.join(DATA_FILE_DIR, file_path) for file_path in natsorted(os.listdir(DATA_FILE_DIR)) if file_path.endswith('.pcd')]

    # lidar_file_path_list, radar_file_path_list = joblib.load('/Users/austin/Downloads/beijing-2025-01-10/utils/time_aligned_list.list')

    # radar_frames_dict = RadarData(specified_path_list=radar_file_path_list, )
    # radar_frames_list = [radar_frames_dict.frame_dict[keys] for keys in radar_frames_dict.frame_dict]
    # radar_dets_list = [radar_frame.dets.det_list for radar_frame in radar_frames_list]


    queue = multiprocessing.Queue(maxsize=1)

    load_process = multiprocessing.Process(target=update_queue, args=(file_path_list, None, queue))
    load_process.daemon = True
    load_process.start()
    
    visualize_PCs(queue)
