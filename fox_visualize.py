import struct
import numpy as np
import open3d as o3d
import os
import os.path as osp
import joblib
import cv2

from utils import RadarData, Radar, detections

from io import BytesIO
from tqdm import tqdm
from typing import List
from datetime import datetime
from foxglove_schemas_protobuf.PackedElementField_pb2 import PackedElementField
from foxglove_schemas_protobuf.Pose_pb2 import Pose
from foxglove_schemas_protobuf.Vector3_pb2 import Vector3
from foxglove_schemas_protobuf.Quaternion_pb2 import Quaternion
from foxglove_schemas_protobuf.PointCloud_pb2 import PointCloud
from foxglove_schemas_protobuf.CompressedImage_pb2 import CompressedImage
from mcap_protobuf.writer import Writer
from google.protobuf.timestamp_pb2 import Timestamp


def write_mcap(lidar_radar_temporal_align_file_path: str = 
               '/Users/austin/Downloads/VRFusion/result/aligned_data/chengdu-2025-02-26:lidar_radar_temploral_aligned_data.list',
               mcap_out_path: str = 
               '/Users/austin/Downloads/VRFusion/result/mcap/') -> None:

    if not osp.exists(mcap_out_path):
        os.makedirs(mcap_out_path)

    LiDAR_ROOT_PATH = '/Users/austin/Downloads/VRFusion/lidar/chengdu-2025-02-26/data'
    lidar_pcds_file_list = [osp.join(LiDAR_ROOT_PATH, file_path) for file_path in os.listdir(LiDAR_ROOT_PATH)]

    RADAR_ROOT_PATH = '/Users/austin/Downloads/VRFusion/radar/chengdu-2025-02-26/json'
    radar_frames_file_list = [osp.join(RADAR_ROOT_PATH, file_path) for file_path in os.listdir(RADAR_ROOT_PATH)]
    radar_frames_dict = RadarData(specified_path_list=radar_frames_file_list, )
    radar_frames_list: List[Radar] = [radar_frames_dict.frame_dict[keys] for keys in radar_frames_dict.frame_dict]

    Camera_ROOT_PATH = '/Users/austin/Downloads/VRFusion/camera/chengdu-2025-02-26/2025-02-26_13-04-01_racobit@12@192.168.1.81.avi'
    camera_video_stream = cv2.VideoCapture(Camera_ROOT_PATH)
    camera_video_num_frames = int(camera_video_stream.get(cv2.CAP_PROP_FRAME_COUNT))
    Camera_Timestamp_ROOT_PATH = '/Users/austin/Downloads/VRFusion/camera/chengdu-2025-02-26/parameters/2025-02-26_13-04-03timesteamp.txt'
    with open(Camera_Timestamp_ROOT_PATH, 'r') as file:
        frame_time_stamp_list = file.readlines()

    with open(osp.join(mcap_out_path, 'chengdu-2025-02-26:lidar_radar_pcds.mcap'), 'wb') as f, Writer(f) as writer:

        for idx in tqdm(range(len(lidar_pcds_file_list))):
            lidar_timestamp = lidar_pcds_file_list[idx].split('/')[-1].split('.')[0] + \
                              lidar_pcds_file_list[idx].split('/')[-1].split('.')[1]

            lidar_pcds_path = lidar_pcds_file_list[idx]
            write_LiDAR_pcds(writer, 
                             lidar_pcds_path=lidar_pcds_path, 
                             now=int(lidar_timestamp))
        
        for idx in tqdm(range(len(radar_frames_file_list))):
            radar_timestamp = int(radar_frames_file_list[idx].split('/')[-1].split('_')[0] + \
                                  radar_frames_file_list[idx].split('/')[-1].split('_')[1].replace('.json', '')) * 1e6
            
            radar_frame_obj: Radar = radar_frames_list[idx]
            write_RADAR_pcds(writer,
                             radar_frame=radar_frame_obj,
                             now=int(radar_timestamp))

        for idx in tqdm(range(camera_video_num_frames)):
            camera_timestamp = int(frame_time_stamp_list[idx].strip()) * 1e6
            _, single_frame = camera_video_stream.read()

            write_Camera_frame(writer=writer,
                               single_frame=single_frame,
                               now=int(camera_timestamp))


def write_LiDAR_pcds(writer: Writer, 
                     lidar_pcds_path: str, 
                     now: int) -> None:

    fields = [PackedElementField(name='x', type=PackedElementField.FLOAT32, offset=0),
              PackedElementField(name='y', type=PackedElementField.FLOAT32, offset=4),
              PackedElementField(name='z', type=PackedElementField.FLOAT32, offset=8),
    ]

    pose = Pose(
        position=Vector3(x=0, y=0, z=0),
        orientation=Quaternion(w=1, x=0, y=0, z=0),
    )

    data_stream = BytesIO()
    lidar_pcds: o3d.cpu.pybind.t.geometry.PointCloud = o3d.io.read_point_cloud(lidar_pcds_path)
    lidar_pcds_array = np.array(lidar_pcds.points)
    for array in lidar_pcds_array:
        data_stream.write(
            struct.pack(
                '<fff',
                float(array[0]),
                float(array[1]),
                float(array[2]),
            )
        )

    message = PointCloud(
        frame_id='pcds',
        pose=pose,
        timestamp=timestamp(now),
        point_stride=12,
        fields=fields,
        data=data_stream.getvalue(),
    )
    writer.write_message(
        topic='/lidar_PCDs',
        log_time=now,
        message=message,
        publish_time=now,
    )


def write_RADAR_pcds(writer: Writer, 
                     radar_frame: Radar, 
                     now: int,
                     theta: float = 2,
                     trans_x: float = 0,
                     trans_y: float = 21.5) -> None:

    fields = [PackedElementField(name='x', type=PackedElementField.FLOAT32, offset=0),
              PackedElementField(name='y', type=PackedElementField.FLOAT32, offset=4),
              PackedElementField(name='z', type=PackedElementField.FLOAT32, offset=8),

              PackedElementField(name='doppler', type=PackedElementField.FLOAT32, offset=12),
              PackedElementField(name='dopplerIdx', type=PackedElementField.INT32, offset=16),

              PackedElementField(name='peakVal', type=PackedElementField.FLOAT32, offset=20),
              PackedElementField(name='snr', type=PackedElementField.FLOAT32, offset=24),
    ]

    pose = Pose(
        position=Vector3(x=0, y=0, z=0),
        orientation=Quaternion(w=1, x=0, y=0, z=0),
    )

    data_stream = BytesIO()
    radar_pcds_list: List[detections] = radar_frame.dets.det_list

    radar_dets_array = np.array([[radar_frame.add_pos_x, -radar_frame.add_pos_y] for radar_frame in radar_pcds_list])
    
    theta = theta * np.pi / 180
    rotation_array = np.array([[ np.cos(theta), np.sin(theta)],
                               [-np.sin(theta), np.cos(theta)]])
    radar_dets_array = radar_dets_array @ rotation_array + np.array([trans_x, trans_y]).reshape((1, 2))

    for idx, dets in enumerate(radar_pcds_list):
        data_stream.write(
            struct.pack(
                '<fffffff',
                float(radar_dets_array[idx][0]),
                float(radar_dets_array[idx][1]),
                float(0),

                float(dets.doppler),
                float(dets.dopplerIdx),

                float(dets.peakVal),
                float(dets.snr),
            )
        )

    message = PointCloud(
        frame_id='pcds',
        pose=pose,
        timestamp=timestamp(now),
        point_stride=28,
        fields=fields,
        data=data_stream.getvalue(),
    )
    writer.write_message(
        topic='/radar_PCDs',
        log_time=now,
        message=message,
        publish_time=now,
    )


def write_Camera_frame(writer: Writer,
                       single_frame: np.array,
                       now: int) -> None:
    
    data_stream = BytesIO()
    # data_stream.write(cv2.imencode('.png', single_frame, [cv2.IMWRITE_PNG_COMPRESSION, 1])[1].tobytes())
    data_stream.write(cv2.imencode('.jpg', single_frame, [cv2.IMWRITE_JPEG_QUALITY, 20])[1].tobytes())

    img = CompressedImage(
        timestamp=timestamp(now),
        frame_id='camera',
        data=data_stream.getvalue(),
        # format='png'
        format='jpeg'
    )

    writer.write_message(
        topic='/camera_frames',
        log_time=now,
        message=img,
        publish_time=now,
    )


def timestamp(time_ns: int) -> Timestamp:
    return Timestamp(seconds=time_ns // 1_000_000_000, nanos=time_ns % 1_000_000_000)


if __name__ == '__main__':

    write_mcap()
