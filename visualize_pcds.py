import open3d as o3d
import numpy as np
import os
import os.path as osp
import multiprocessing
import time
import joblib
import pandas as pd

from utils import RadarData
from natsort import natsorted


def create_bounding_box(center, size, rotation_angle, do_init=False):

    if do_init:
        return o3d.geometry.TriangleMesh.create_box()

    box = o3d.geometry.TriangleMesh.create_box(width=size[0], height=size[1], depth=size[2])
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_xyz([0, 0, rotation_angle])

    box.rotate(rotation_matrix, center=(0, 0, 0))
    box.translate(center)
    
    return box

def create_bounding_box_lines(center, size, rotation_angle):

    half_size = np.array(size) / 2
    corners = np.array([
        [-half_size[0], -half_size[1], -half_size[2]],
        [ half_size[0], -half_size[1], -half_size[2]],
        [ half_size[0],  half_size[1], -half_size[2]],
        [-half_size[0],  half_size[1], -half_size[2]],
        [-half_size[0], -half_size[1],  half_size[2]],
        [ half_size[0], -half_size[1],  half_size[2]],
        [ half_size[0],  half_size[1],  half_size[2]],
        [-half_size[0],  half_size[1],  half_size[2]]
    ])

    rotation_matrix = o3d.geometry.get_rotation_matrix_from_xyz([0, 0, (rotation_angle - 90) * np.pi / 180])
    corners = np.dot(corners, rotation_matrix.T) + center

    lines = [
        [0, 1], [1, 2], [2, 3], [3, 0],
        [4, 5], [5, 6], [6, 7], [7, 4],
        [0, 4], [1, 5], [2, 6], [3, 7]
    ]

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(corners)
    line_set.lines = o3d.utility.Vector2iVector(lines)

    return line_set

def visualize_PCs(queue: multiprocessing.Queue):

    lidar_annotation_df = pd.read_feather('/Users/austin/Downloads/VRFusion/result/annotation/chengdu-2025-02-26:lidar_annotation.feather')
    
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    lidar_pcd = o3d.geometry.PointCloud()
    radar_pcd = o3d.geometry.PointCloud()
    d3_bbox_dict = dict()

    for _id in list(lidar_annotation_df['ID'].value_counts().keys()):
        d3_bbox_dict[_id] = create_bounding_box(None, None, None, do_init=True)



    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])
    
    first_add = True

    while True:
        if not queue.empty():
            lidar_file_path, radar_file_idx, lidar_timestamp = queue.get()
            lidar_annotation_single_frame_df = lidar_annotation_df[lidar_annotation_df['Timestamp'] == lidar_timestamp].reset_index(drop=True)

            lidar_point_clouds = o3d.io.read_point_cloud(lidar_file_path)
            lidar_point_clouds = np.asarray(lidar_point_clouds.points).reshape((-1, 3))
            
            lidar_pcd.points = o3d.utility.Vector3dVector(lidar_point_clouds)
            lidar_pcd.paint_uniform_color([0, 0, 1])

            radar_point_clouds = radar_dets_list[radar_file_idx]

            theta = 2 * np.pi / 180
            rotation_array = np.array([[ np.cos(theta), np.sin(theta), 0],
                                       [-np.sin(theta), np.cos(theta), 0],
                                       [             0,             0, 0]])

            radar_point_clouds = np.asarray([[np.asarray([item.add_pos_x, -item.add_pos_y, 0]) for item in radar_point_clouds]]).reshape((-1, 3)) @ \
                                 rotation_array + np.asarray([0, 21.5, 0])
            radar_pcd.points = o3d.utility.Vector3dVector(radar_point_clouds)
            radar_pcd.paint_uniform_color([1, 0, 0])

            

            if first_add:
                vis.add_geometry(coordinate_frame)
                vis.add_geometry(lidar_pcd)
                vis.add_geometry(radar_pcd)









                first_add = False

            opt = vis.get_render_option()
            opt.point_size = 3
            
            vis.update_geometry(coordinate_frame)
            vis.update_geometry(lidar_pcd)
            vis.update_geometry(radar_pcd)








        vis.poll_events()
        vis.update_renderer()

    # TODO: 添加窗口终止逻辑
    vis.destroy_window()

def update_queue(lidar_file_path_list: list, 
                 radar_file_path_list: list,
                 queue: multiprocessing.Queue):
    frame = 0

    # 现在的逻辑是按照 frame 计数进行播放，可更改为按键播放

    time.sleep(2)

    while True:
        lidar_file_path = lidar_file_path_list[frame]
        radar_file_path = radar_file_path_list[frame]
        print('LiDAR file path: {}, RADAR file path: {}'.format(lidar_file_path.split('/')[-1], radar_file_path.split('/')[-1]))

        lidar_timestamp = lidar_file_path.split('/')[-1].split('.')[0] + lidar_file_path.split('/')[-1].split('.')[1][:-3]

        if queue.empty():
            queue.put([lidar_file_path, frame, lidar_timestamp])

        frame += 1
        frame %= len(lidar_file_path_list)

        time.sleep(0.2)
    

if __name__ == '__main__':
    # DATA_FILE_DIR = '/Users/austin/Downloads/chengdu-2025-01-10/lidar/data'
    # file_path_list = [osp.join(DATA_FILE_DIR, file_path) for file_path in natsorted(os.listdir(DATA_FILE_DIR)) if file_path.endswith('.pcd')]

    # lidar_file_path_list, radar_file_path_list = joblib.load('./result/aligned_data/chengdu-2025-02-26:lidar_radar_temploral_aligned_data.list')[0][:10], joblib.load('./result/aligned_data/chengdu-2025-02-26:lidar_radar_temploral_aligned_data.list')[1][:10]
    
    lidar_file_path_list, radar_file_path_list = joblib.load('./result/aligned_data/chengdu-2025-02-26:lidar_radar_temploral_aligned_data.list')

    radar_frames_dict = RadarData(specified_path_list=radar_file_path_list, )
    radar_frames_list = [radar_frames_dict.frame_dict[keys] for keys in radar_frames_dict.frame_dict]
    radar_dets_list = [radar_frame.dets.det_list for radar_frame in radar_frames_list]


    queue = multiprocessing.Queue(maxsize=1)

    load_process = multiprocessing.Process(target=update_queue, args=(lidar_file_path_list, radar_file_path_list, queue))
    load_process.daemon = True
    load_process.start()
    
    visualize_PCs(queue)
