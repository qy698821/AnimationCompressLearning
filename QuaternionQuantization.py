import numpy as np
import struct
import math
from scipy.spatial.transform import Rotation as R
from Lab1_FK_answers import *
from AnimationCompress import *

bvh_file_path = "data/walk60.bvh"

bits15_min = 0
bits15_max = 32767

range_quat_min = -1.415
range_quat_max = 1.415

def loat_motion_data_as_vector3(path):
    motion_data = load_motion_data(path)
    motion_data_vector3 = []
    step = 3
    for frame_data in motion_data:
        frame_data_3 = [frame_data[i:i+step] for i in range(0, len(frame_data), step)]
        motion_data_vector3.append(frame_data_3)

    return motion_data_vector3

def convert_rotation_data_to_quat(vector3_data):
    quat_data_all = []
    for frame_data in vector3_data:
        quat_data_one_frame = []
        for i in range(len(frame_data)):
            if i == 0:
                continue
            quat_data = R.from_euler("XYZ", [frame_data[i][0], frame_data[i][1], frame_data[i][2]], degrees=True).as_quat()
            quat_data_one_frame.append(quat_data)
        quat_data_all.append(quat_data_one_frame)
    return quat_data_all

# The bch file has root motion
def get_root_joint_transform_data(vector3_data):
    root_data = []
    for frame_data in vector3_data:
        root_data.append(frame_data[0])

    return root_data

def quaternion_quantization(quat):
    print(quat)
    max_value = 0
    max_index = 0
    for i in range(len(quat)):
        if quat[i] * quat[i] > max_value:
            max_value = quat[i] * quat[i]
            max_index = i

    uint_list = []
    for i in range(len(quat)):
        if i != max_index:
            print(np.uint16(translate(quat[i], range_quat_min, range_quat_max, bits15_min, bits15_max)))
            uint_list.append(np.uint16(translate(quat[i], range_quat_min, range_quat_max, bits15_min, bits15_max)))

    bin1 = bin(uint_list[0])[2:].zfill(16)
    bin2 = bin(uint_list[1])[2:].zfill(16)
    bin3 = bin(uint_list[2])[2:].zfill(16)

    bin1 = bin1[-15:]
    bin2 = bin2[-15:]
    bin3 = bin3[-15:]


    print(bin1)
    print(bin2)
    print(bin3)


def main():
    motion_data_vector3 = loat_motion_data_as_vector3(bvh_file_path)
    quat_rotation_data = convert_rotation_data_to_quat(motion_data_vector3)
    root_move_data = get_root_joint_transform_data(motion_data_vector3)

    quaternion_quantization(quat_rotation_data[0][0])

    # max_num = 0
    # max_index = 0
    # for i in range(len(quat_rotation_data[0][0])):
    #     if quat_rotation_data[0][0][i] * quat_rotation_data[0][0][i] > max_num * max_num:
    #         max_num = quat_rotation_data[0][0][i]
    #         max_index = i
    #
    # sum = 0
    # for i in range(len(quat_rotation_data[0][0])):
    #     if i != max_index:
    #         sum = sum + quat_rotation_data[0][0][i] * quat_rotation_data[0][0][i]
    # max_num_calculate = math.sqrt(1 - sum)
    # print(max_num_calculate)
    # print(max_num)

if __name__ == "__main__":
    main()