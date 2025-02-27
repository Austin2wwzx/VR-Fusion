import os
import os.path as osp
import json
import time
import numpy as np
import multiprocessing

from radar_dataclass import *
from natsort import natsorted
from typing import List
from tqdm import tqdm


class RadarData(object):

    def __init__(self, root_path: str = None,
                 specified_path_list: str = None,
                 start_frame_idx: int = 0,
                 duration_frames: int = 0, ):
        
        if specified_path_list == None:
            radar_file_path_list = [osp.join(root_path, path) for path in natsorted(os.listdir(root_path)) if path.endswith('json')]
        else:
            radar_file_path_list = specified_path_list
        self.total_frames = len(radar_file_path_list)

        assert start_frame_idx < self.total_frames and (start_frame_idx + duration_frames) < self.total_frames
        if duration_frames == 0:
            self.radar_file_path_list = radar_file_path_list[start_frame_idx:]
        else:
            self.radar_file_path_list = radar_file_path_list[start_frame_idx: start_frame_idx+duration_frames]

        self.num_frames = len(self.radar_file_path_list)
        if self.num_frames < 200:
            self.frame_dict = self.read_frames(file_path_list=self.radar_file_path_list)
        else:
            self.frame_dict = self.multiprocess_read_frames(file_path_list=self.radar_file_path_list)

    def read_single_frame(self, data: dict = None,
                          data_file_path: str = None,
                          dropped_field: List[str] = ['adc', 'bv', 'hrrp', 'rd', 'ra', 'trks']) -> Radar:
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
            def cal_abs_time_ms(frameRealTime_ms: int, frameRealTime_s: int):
                return frameRealTime_s * 1000 + frameRealTime_ms

            tLVHeaders = [TLVHeader(**tlv) for tlv in data['frameHeader']['tLVHeaderlsit']]
            frame_data = frameHeader(**{key: data['frameHeader'][key] for key in data['frameHeader'] if key != 'tLVHeaderlsit'}, 
                                     tLVHeaderlist=tLVHeaders, 
                                     add_frameTime_ms=cal_abs_time_ms(frameRealTime_ms=data['frameHeader']['frameRealTime_ms'],
                                                                      frameRealTime_s=data['frameHeader']['frameRealTime_s']))
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
            def polar_to_cartesian(r: float, theta: float):
                theta_rad = np.radians(theta)
                
                x = r * np.cos(theta_rad)
                y = r * np.sin(theta_rad)
                
                return x, y
            
            det_data = detPointsStruct(
                    [detections(**item, 
                                add_pos_x=polar_to_cartesian(item['range'], item['angle'])[0], 
                                add_pos_y=polar_to_cartesian(item['range'], item['angle'])[1]) for item in data['detPoints']]
                )
            single_frame.dets = det_data
        if 'trks' not in dropped_field:
            trk_data = trackStruct([tracks(**item) for item in data['track']])
            single_frame.trks = trk_data

        return single_frame
    
    def read_frames(self, file_path_list: list, verbose: bool = True):
        frame_dict = dict()

        if verbose:
            for file in tqdm(file_path_list, desc='Loading raw RADAR frames...'):
                with open(file, 'r') as f:
                    data = json.load(f)

                single_frame = self.read_single_frame(data=data)
                frame_idx = single_frame.frameInfo.FrameNumber
                frame_dict[frame_idx] = single_frame
        else:
            for file in file_path_list:
                with open(file, 'r') as f:
                    data = json.load(f)

                single_frame = self.read_single_frame(data=data)
                frame_idx = single_frame.frameInfo.FrameNumber
                frame_dict[frame_idx] = single_frame

        return frame_dict
    
    def multiprocess_read_frames(self, file_path_list: list, num_processor: int = 8):
        start_time = time.time()

        sub_file_list = np.array_split(file_path_list, num_processor)
        sub_file_list = [list(sub_list) for sub_list in sub_file_list]

        print('Loading raw RADAR frames by multi-processors...')
        pool = multiprocessing.Pool(processes=num_processor)
        results = [pool.apply_async(self.read_frames, args=(sub_list, False)) for sub_list in sub_file_list]
        pool.close()
        pool.join()

        results = [res.get() for res in results]        
        final_frame_dict = dict()
        for sub_dict in results:
            final_frame_dict.update(sub_dict)

        print('Times usage: {}s.'.format(time.time() - start_time))

        return final_frame_dict


if __name__ == '__main__':

    RADAR_DATA_DIR = './radar/json/'
    data = RadarData(root_path=RADAR_DATA_DIR, duration_frames=0)

    pass