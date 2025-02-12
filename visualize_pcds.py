import open3d as o3d
import numpy as np
import os
import os.path as osp
import multiprocessing
import time


def visualize_PCs(queue: multiprocessing.Queue):
    
    vis = o3d.visualization.Visualizer()
    # 要更改地图参数就在下面修改
    vis.create_window()

    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)

    first_add = True

    while True:
        # 如果队列不为空，获取下一个点云文件的路径
        if not queue.empty():
            file_path = queue.get()
            point_clouds = o3d.io.read_point_cloud(file_path)
            point_clouds = np.asarray(point_clouds.points).reshape((-1, 3))
            pcd.points = o3d.utility.Vector3dVector(point_clouds)

            if first_add:
                vis.add_geometry(pcd)
                first_add = False

            vis.update_geometry(pcd)

        vis.poll_events()
        vis.update_renderer()

    # TODO: 添加窗口终止逻辑
    # vis.destroy_window()

def update_queue(file_path_list: list, queue: multiprocessing.Queue):
    frame = 0

    # 现在的逻辑是按照 frame 计数进行播放，可更改为按键播放

    while True:
        print(frame)
        file_path = file_path_list[frame]

        if queue.empty():
            queue.put(file_path)

        frame += 1
        frame %= len(file_path_list)

        time.sleep(0.5)
    

if __name__ == '__main__':
    DATA_FILE_DIR = '/Users/austin/Downloads/beijing-2025-01-10/lidar/data'
    file_path_list = [osp.join(DATA_FILE_DIR, file_path) for file_path in os.listdir(DATA_FILE_DIR) if file_path.endswith('.pcd')]

    queue = multiprocessing.Queue(maxsize=1)

    load_process = multiprocessing.Process(target=update_queue, args=(file_path_list, queue))
    load_process.daemon = True
    load_process.start()
    
    visualize_PCs(queue)
