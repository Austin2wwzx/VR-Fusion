import open3d as o3d
import os
import os.path as osp
import numpy as np
import time


def vis_pcd(pcd_path_list: list, view_point_json_psth: str):
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # opt = vis.get_render_option()
    # opt.background_color = np.asarray([0, 0, 0])
    # opt.point_size = 2
    # opt.show_coordinate_frame = True

    # param = o3d.io.read_pinhole_camera_parameters(view_point_json_psth)
    # ctr = vis.get_view_control()
    # ctr.convert_from_pinhole_camera_parameters(param)

    pointcloud = o3d.geometry.PointCloud()
    vis.add_geometry(pointcloud)

    # road_pc = o3d.geometry.PointCloud()
    # road = o3d.io.read_point_cloud(road_pcd)
    # road = np.asarray(road.points).reshape((-1, 3))
    # road_pc.points = o3d.utility.Vector3dVector(road)
    # road_pc.paint_uniform_color([0,1,0])
    # vis.add_geometry(road_pc)

    to_reset = True

    # file_path_list = os.listdir(pcd_path_list)

    for file_path in pcd_path_list:
        pcd = o3d.io.read_point_cloud(file_path)

        pcd = np.asarray(pcd.points).reshape((-1, 3))
        pointcloud.points = o3d.utility.Vector3dVector(pcd)

        # pointcloud.paint_uniform_color([1, 0, 0])

        vis.update_geometry(pointcloud)
        vis.add_geometry(pointcloud)
        # ctr.convert_from_pinhole_camera_parameters(param)

        # if to_reset:
        #     vis.reset_view_point(True)
        #     to_reset = False

        vis.poll_events()
        vis.update_renderer()

        time.sleep(1)

    vis.destroy_window()

if __name__ == '__main__':

    DATA_FILE_DIR = '/Users/austin/Downloads/25.1.10西博城采集数据——二分钟/Beijing_LiDAR_data'
    file_path_list = [osp.join(DATA_FILE_DIR, file_path) for file_path in os.listdir(DATA_FILE_DIR) if file_path.endswith('.pcd')]


    vis_pcd(file_path_list, 'viewpoint.json')

# def show_pointcloud_dir(file_path):
#     g_idx = 0
#     vis = o3d.visualization.VisualizerWithKeyCallback()
#     filelist = [os.path.join(file_path, f) for f in os.listdir(file_path)]

#     def show_pointcloud(vis):
#         nonlocal g_idx
#         points = np.fromfile(filelist[g_idx], np.float32).reshape(-1, 4) # xyzi
#         pcd = o3d.geometry.PointCloud()
#         pcd.points = o3d.utility.Vector3dVector(points[:, :3])

#         # point_color = (0.5, 0.5, 0.5)
#         # points_colors = np.tile(np.array(point_color), (points.shape[0], 1))
#         # pcd.colors = o3d.utility.Vector3dVector(points_colors)

#         vis.clear_geometries()
#         vis.add_geometry(pcd) # fuck bug, vis.update_geometry(pcd)没有用！
#         vis.update_renderer()
#         vis.poll_events()

#     def key_forward_callback(vis):
#         nonlocal g_idx
#         g_idx += 1
#         if g_idx >= len(filelist):
#             g_idx = len(filelist) - 1
#         show_pointcloud(vis)
#         return True

#     def key_back_callback(vis):
#         nonlocal g_idx
#         g_idx -= 1
#         if g_idx < 0:
#             g_idx = 0
#         show_pointcloud(vis)
#         return True

#     vis.create_window()
#     vis.get_render_option().point_size = 2  # set points size

#     vis.register_key_callback(ord(' '), key_forward_callback)  # space
#     vis.register_key_callback(ord('B'), key_back_callback)  # fuck bug, 字母必须是大写!
#     vis.run()


# if __name__ == "__main__":
#     DATA_FILE_DIR = '/Users/austin/Downloads/25.1.10西博城采集数据——二分钟/Beijing_LiDAR_data'
#     show_pointcloud_dir(DATA_FILE_DIR) 


# import open3d as o3d
# import numpy as np
# import copy
 
# if __name__ == "__main__":
#     point_cloud = o3d.io.read_point_cloud('Beijing_LiDAR_data/1736500681.467958000.pcd')
 
#     vis = o3d.visualization.Visualizer()
#     vis.create_window()
#     vis.add_geometry(point_cloud)
#     # vis.add_geometry(target)
#     threshold = 0.05
#     icp_iteration = 1000000000000
#     save_image = False
 
#     ctr = vis.get_view_control()
#     ctr.set_lookat(np.array([0.0, 0.0, 55.0]))
#     ctr.set_up((0, -1, 0))  # set the positive direction of the x-axis as the up direction
#     ctr.set_front((-1, 0, 0))  # set the positive direction of the x-axis toward you
 
 
#     for i in range(icp_iteration):
 
#         R = point_cloud.get_rotation_matrix_from_xyz((np.pi / 180 * 1, 0, 0 * np.pi / 2))
#         point_cloud.rotate(R, center=(0.0,0,55.0))
#         vis.update_geometry(point_cloud)
#         vis.poll_events()
#         vis.update_renderer()
 
 
#         if save_image:
#             vis.capture_screen_image("temp_%04d.jpg" % i)
