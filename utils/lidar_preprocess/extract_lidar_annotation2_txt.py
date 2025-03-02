import numpy as np
import os
import os.path as osp


if __name__ == '__main__':
    CSV_PATH = './lidar/chengdu-2025-02-26/raw_annotation/output_Mec_20250226130405_002.csv'
    ROOT_SAVE_PATH = './lidar/chengdu-2025-02-26/annotation'

    NUM_INFO = 126  # 每个信息块的长度

    data = open(CSV_PATH)
    lines = data.readlines()[1:]
    final_objs = list()
    
    for line in lines:
        infos = line.split(',')

        n_objs = int((len(infos) - 1) / NUM_INFO)
        print(n_objs)

        for idx in range(n_objs):
            cur_obj_info = infos[1 + idx*NUM_INFO: 1 + (idx+1)*NUM_INFO]

            cur_obj_info_time = cur_obj_info[1][:16]
            cur_obj_info_id = cur_obj_info[2]
            cur_obj_info_lon = cur_obj_info[23]
            cur_obj_info_lat = cur_obj_info[24]
            cur_obj_info_dim_x = cur_obj_info[48]
            cur_obj_info_dim_y = cur_obj_info[49]
            cur_obj_info_dim_z = cur_obj_info[50]
            cur_obj_info_yaw = cur_obj_info[40]
            cur_obj_info_x = cur_obj_info[13]
            cur_obj_info_y = cur_obj_info[14]
            cur_obj_info_z = cur_obj_info[15]

            cur_obj = [cur_obj_info_time, cur_obj_info_id, cur_obj_info_lon, cur_obj_info_lat,
                       cur_obj_info_dim_x, cur_obj_info_dim_y, cur_obj_info_dim_z, cur_obj_info_yaw,
                       cur_obj_info_x, cur_obj_info_y, cur_obj_info_z]

            final_objs.append(cur_obj)

    file_name = CSV_PATH.split('/')[-1].split('.')[0]
    file_format = '.txt'
    np.savetxt(osp.join(ROOT_SAVE_PATH, file_name) + file_format, final_objs, fmt='%s', delimiter=',')
