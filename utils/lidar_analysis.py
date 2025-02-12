import numpy as np
import os
import os.path as osp


CSV_PATH = './lidar/raw_annotation/output_20250110171801_002.csv'
SAVE_ROOT_PATH = './lidar/annotation'

NUM_INFO = 126  # 每个目标信息的长度


def read_csv_data(csv_path):
    with open(csv_path, 'r') as file:
        return file.readlines()

def parse_line_to_object(line):
    '''将每一行数据解析为目标对象'''
    infos = line.split(',')
    n_objs = len(infos) // NUM_INFO
    objects = list()

    for i in range(n_objs):
        obj_info = infos[1 + i * NUM_INFO: 1 + (i + 1) * NUM_INFO]
        obj = {
            'time': obj_info[0:13],
            'obj_id': obj_info[13],
            'velocity': obj_info[2],
            'latitude': obj_info[23],
            'longitude': obj_info[24],
            'dim_x': obj_info[49],
            'dim_y': obj_info[50],
            'yaw': obj_info[51],
            'position_x': obj_info[14],
            'position_y': obj_info[15],
            'position_z': obj_info[16]
        }
        objects.append(obj)
    
    return objects

def save_to_txt(save_path, data):
    '''将目标数据保存到txt文件'''
    np.savetxt(save_path, data, fmt='%s', delimiter=',')

def main():
    # 读取数据
    lines = read_csv_data(CSV_PATH)
    
    final_objs = []
    for line in lines:
        objects = parse_line_to_object(line)
        final_objs.extend(objects)

    # 保存结果
    file_name = CSV_PATH.split('/')[-1].split('.')[0]
    file_format = '.txt'
    save_to_txt(osp.join(SAVE_ROOT_PATH, file_name) + file_format, final_objs)

if __name__ == '__main__':
    main()
    