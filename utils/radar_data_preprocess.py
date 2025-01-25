import os
import os.path as osp
import json

from radar_dataclass import *
from natsort import natsorted
from typing import List



class RadarData(object):

    def __init__(self, root_path: str,
                 start_frame_idx: int = 0,
                 duration_frames: int = 0, ):
        
        radar_file_path_list = [osp.join(root_path, path) for path in natsorted(os.listdir(root_path)) if path.endswith('json')]
        self.total_frames = len(radar_file_path_list)

        assert start_frame_idx < self.total_frames and (start_frame_idx + duration_frames) < self.total_frames
        if duration_frames == 0:
            self.radar_file_path_list = radar_file_path_list[start_frame_idx:]
        else:
            self.radar_file_path_list = radar_file_path_list[start_frame_idx: start_frame_idx+duration_frames]

        self.num_frames = len(self.radar_file_path_list)
        self.frame_dict = self.read_frames(file_path_list=self.radar_file_path_list)

    def read_single_frame(self, data: dict = None,
                          data_file_path: str = None,
                          dropped_field: List[str] = ['adc', 'bv', 'hrrp', 'rd', 'ra']) -> Radar:
        single_frame = Radar()

        if data == None:
            with open(data_file_path, 'r') as file:
                data = json.load(file)

        valid_fields = {'adc', 'bv', 'frame', 'hrrp', 'rd', 'ra', 'dets', 'trks'}
        assert all(field in valid_fields for field in dropped_field), 'Invalid fields found: {}.'.format(', '.join([field for field in dropped_field if field not in valid_fields]))

        if 'adc' not in dropped_field:
            adc_data = adcStruct(**{keys: data['adcStruct'][keys] for keys in data['adcStruct']})
            single_frame.adc = adc_data
        if 'bv' not in dropped_field:
            bv_data = beamVectorStruct(**{keys: data['beamVectorStruct'][keys] for keys in data['beamVectorStruct']})
            single_frame.bv = bv_data
        if 'frame' not in dropped_field:
            tLVHeaders = [TLVHeader(**tlv) for tlv in data['frameHeader']['tLVHeaderlsit']]
            frame_data = frameHeader(**{key: data['frameHeader'][key] for key in data['frameHeader'] if key != 'tLVHeaderlsit'}, tLVHeaderlist=tLVHeaders)
            single_frame.frameInfo = frame_data
        if 'hrrp' not in dropped_field:
            hrrp_data = hrrpStruct(data['hrrp'])
            single_frame.hrrp = hrrp_data
        if 'rd' not in dropped_field:
            rd_data = rangeDopplerStruct(data['rangeDoppler'])
            single_frame.rd = rd_data
        if 'ra' not in dropped_field:
            ra_data = rangeAngleStruct(**{keys: data['rangeAngle'][keys] for keys in data['rangeAngle']})
            single_frame.ra = ra_data
        if 'dets' not in dropped_field:
            det_data = detPointsStruct([detections(**item) for item in data['detPoints']])
            single_frame.dets = det_data
        if 'trks' not in dropped_field:
            trk_data = trackStruct([tracks(**item) for item in data['track']])
            single_frame.trks = trk_data

        return single_frame
    
    def read_frames(self, file_path_list: list):
        frame_dict = dict()

        for file in file_path_list:
            with open(file, 'r') as f:
                data = json.load(f)

            single_frame = self.read_single_frame(data=data)
            frame_idx = single_frame.frameInfo.FrameNumber
            frame_dict[frame_idx] = single_frame

        return frame_dict



if __name__ == '__main__':

    RADAR_DATA_DIR = './radar/json/'

    data = RadarData(root_path=RADAR_DATA_DIR, duration_frames=10)

    pass