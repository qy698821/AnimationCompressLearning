import numpy as np
import struct
import math
from scipy.spatial.transform import Rotation as R
from Lab1_FK_answers import *
from AnimationCompress import *

bvh_file_path = "data/walk60.bvh"
motion_file_path_compress_quat = "data/compress/walk60_compress_quat.txt"
uncompress_bvh_file_quat = "data/walk60_uint16_uncompress_quat.bvh"

bits15_min = 0
bits15_max = 32767

range_quat_min = -0.71
range_quat_max = 0.71

data_range_min_location = -54
data_range_max_location = 70

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
            quat_data_one_frame.append(quaternion_quantization(quat_data))
        quat_data_all.append(quat_data_one_frame)
    return quat_data_all

def convert_rotation_data_to_quat_no_quantization(vector3_data):
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
        root_data.append(translate_location(frame_data[0]))

    return root_data

def translate_location(vector3_data):
    uint_list = []
    for value in vector3_data:
        uint_list.append(np.uint16(translate(value, data_range_min_location, data_range_max_location, 0, 65535)))
    return uint_list

def quaternion_quantization(quat):
    max_value = 0
    max_index = 0
    for i in range(len(quat)):
        if quat[i] * quat[i] > max_value:
            max_value = quat[i] * quat[i]
            max_index = i

    uint_list = []
    for i in range(len(quat)):
        if i != max_index:
            uint_list.append(np.uint16(translate(quat[i], range_quat_min, range_quat_max, bits15_min, bits15_max)))

    bin1 = bin(uint_list[0])[2:].zfill(16)
    bin2 = bin(uint_list[1])[2:].zfill(16)
    bin3 = bin(uint_list[2])[2:].zfill(16)

    # 0 - 32767 only need 15 bits
    bin1 = bin1[-15:]
    bin2 = bin2[-15:]
    bin3 = bin3[-15:]

    bin_data_quat = bin1 + bin2 + bin3

    uint_data = int(bin_data_quat, 2)

    if quat[max_index] > 0:
        uint_data |= (1 << 47)

    # Mark the position of the value with largest absolute value
    if max_index == 1:
        uint_data |= (1 << 45)
    elif max_index == 2:
        uint_data |= (1 << 46)
    elif max_index == 3:
        uint_data |= (1 << 46) | (1 << 45)

    uint16_1 = np.uint16(0)
    uint16_2 = np.uint16(0)
    uint16_3 = np.uint16(0)

    uint16_1 |= ((uint_data >> 32) & 65535)
    uint16_2 |= ((uint_data >> 16) & 65535)
    uint16_3 |= (uint_data & 65535)

    uint_list = []
    uint_list.append(uint16_1)
    uint_list.append(uint16_2)
    uint_list.append(uint16_3)

    return uint_list

def uncompress_to_quat(uint_data):

    data_48bit = bin(uint_data[0])[2:].zfill(16) + bin(uint_data[1])[2:].zfill(16) + bin(uint_data[2])[2:].zfill(16)

    uint_data = int(data_48bit, 2)

    max_value_location_num = (uint_data >> 45) & ( (1 << 1) | 1 )

    # Get the 15 bits quat value
    quat_value_3 = translate(int(data_48bit[-15:], 2), bits15_min, bits15_max, range_quat_min, range_quat_max)
    quat_value_2 = translate(int(data_48bit[-30:-15], 2), bits15_min, bits15_max, range_quat_min, range_quat_max)
    quat_value_1 = translate(int(data_48bit[-45:-30], 2), bits15_min, bits15_max, range_quat_min, range_quat_max)

    quat_data = []
    quat_data.append(quat_value_1)
    quat_data.append(quat_value_2)
    quat_data.append(quat_value_3)

    max_quat_value = math.sqrt(1 - pow(quat_value_1, 2) - pow(quat_value_2, 2) - pow(quat_value_3, 2))

    # check if the max value < 0
    if (uint_data >> 47) == 0:
        max_quat_value = 1 - max_quat_value

    quat_data.insert(max_value_location_num, max_quat_value)

    return quat_data

def write_quat_compress_data_to_file(motion_data_vector3):
    quat_rotation_data = convert_rotation_data_to_quat(motion_data_vector3)
    root_move_data = get_root_joint_transform_data(motion_data_vector3)

    num_frame_data = len(quat_rotation_data[0]) * 3 + len(root_move_data[0])

    with open(motion_file_path_compress_quat, 'wb') as w:
        w.write(struct.pack("H", num_frame_data))
        for i in range(len(quat_rotation_data)):
            w.write(struct.pack(("%dH" % len(root_move_data[i])), *root_move_data[i]))
            for value in quat_rotation_data[i]:
                w.write(struct.pack(("%dH" % len(value)), *value))

def read_quat_compress_data(path):
    motion_data_from_float = []
    num_frame_data = 0
    with open(path, 'rb') as r:
        data = r.read(2)
        num_frame_data = struct.unpack('H', data)[0]
        print(num_frame_data)
        while True:
            data = r.read(2)
            if not data:
                break
            uint16_item = struct.unpack('H', data)[0]
            motion_data_from_float.append(uint16_item)

    new_array_data = [motion_data_from_float[i:i + num_frame_data] for i in
                      range(0, len(motion_data_from_float), num_frame_data)]

    motion_data = []
    for frame_data in new_array_data:
        frame_data_float = []
        current_index = 0
        for i in range(len(frame_data)):
            if i < 3:
                float_item = translate(frame_data[i], 0, 65535, data_range_min_location, data_range_max_location)
                frame_data_float.append(round(float_item, 6))
            else:
                if i < current_index:
                    continue
                if i + 2 >= len(frame_data):
                    break
                vector3_data = []
                vector3_data.append(frame_data[i])
                vector3_data.append(frame_data[i + 1])
                vector3_data.append(frame_data[i + 2])
                rotation_data = R.from_quat(uncompress_to_quat(vector3_data)).as_euler('XYZ', degrees=True)
                frame_data_float.append(round(rotation_data[0], 6))
                frame_data_float.append(round(rotation_data[1], 6))
                frame_data_float.append(round(rotation_data[2], 6))
                current_index = i + 3
        motion_data.append(frame_data_float)

    return motion_data

def main():
    motion_data_vector3 = loat_motion_data_as_vector3(bvh_file_path)
    write_quat_compress_data_to_file(motion_data_vector3)

    motion_data = read_quat_compress_data(motion_file_path_compress_quat)

    write_uncompress_data(motion_data, uncompress_bvh_file_quat)

    # uint_quat = quaternion_quantization(quat_rotation_data[20][0])
    # uint_quat = struct.pack(("%dH" % len(uint_quat)), *uint_quat)
    # uncompress_to_quat(uint_quat)

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