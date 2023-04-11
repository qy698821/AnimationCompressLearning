import numpy as np
from scipy.spatial.transform import Rotation as R
from QuaternionQuantization import *

# # 定义起始四元数和目标四元数
# q_start = np.array([1, 0, 0, 0])  # 等同于旋转矩阵的单位矩阵
# q_end = np.array([1, 0, 0, 0])  # 45度绕 [1, 1, 1] 向量旋转得到的四元数
#
# # 定义插值数量
# num_interpolation = 3
#
# # 创建四元数对象
# r_start = R.from_quat(q_start)
# r_end = R.from_quat(q_end)
#
# # 线性插值
# interpolated_quaternions = np.empty((num_interpolation, 4))
# for i, t in enumerate(np.linspace(0, 1, num_interpolation)):
#     interpolated_quaternion = R.from_quat(np.array(q_start) + t * (np.array(q_end) - np.array(q_start)))
#     interpolated_quaternions[i] = interpolated_quaternion.as_quat()
#
# print(interpolated_quaternions)
#
# print(interpolated_quaternions[0].dot(interpolated_quaternions[1]))

motion_file_path_compress_line_interp = "data/compress/walk60_compress_line_interp.txt"
uncompress_bvh_file_line_interp = "data/walk60_uint16_uncompress_line_interp.bvh"

error_num = 0.0005

def convert_to_joint_frame_data(quat_data):
    joint_frame_data = []
    for i in range(len(quat_data)):
        for j in range(len(quat_data[i])):
            if i == 0:
                joint_frame_data.append([])
            joint_frame_data[j].append(quat_data[i][j])
    return joint_frame_data

def line_interp(quat_start, quat_end, percent):
    interpolated_quaternion = R.from_quat(np.array(quat_start) + percent * (np.array(quat_end) - np.array(quat_start))).as_quat()
    return interpolated_quaternion

def calculate_key_frame(joint_data):
    key_frame = []
    key_frame_index = []
    for i in range(len(joint_data)):
        if i == 0:
            key_frame.append(quaternion_quantization(joint_data[i]))
            key_frame_index.append(i)
            continue
        if i == len(joint_data) - 1:
            key_frame.append(quaternion_quantization(joint_data[i]))
            key_frame_index.append(i)
            continue
        if i < len(joint_data) - 1:
            quat_calc = line_interp(joint_data[i - 1], joint_data[i + 1], 0.5)
            if not is_quaternion_close(quat_calc, joint_data[i]):
                key_frame.append(quaternion_quantization(joint_data[i]))
                key_frame_index.append(i)
    print(key_frame_index)
    print(key_frame)
    return key_frame, key_frame_index


def processe_data(joint_frame_data):
    key_frame_list = []
    key_frame_index_list = []
    for joint_data in joint_frame_data:
        key_frame, key_frame_index = calculate_key_frame(joint_data)
        key_frame_list.append(key_frame)
        key_frame_index_list.append(key_frame_index)
    return key_frame_list, key_frame_index_list


def write_line_interp_data_to_file(file_path, joint_frame_data):
    key_frame_list, key_frame_index_list = processe_data(joint_frame_data)
    max_frame = key_frame_index_list[0][len(key_frame_index_list[0]) - 1]

    with open(file_path, 'wb') as w:
        w.write(struct.pack("H", max_frame))
        for i in range(len(key_frame_index_list)):
            w.write(struct.pack(("%dH" % len(key_frame_index_list[i])), *key_frame_index_list[i]))
        w.write(struct.pack("H", 65535))

        for i in range(len(key_frame_list)):
            for j in range(len(key_frame_list[i])):
                w.write(struct.pack(("%dH" % len(key_frame_list[i][j])), *key_frame_list[i][j]))

def read_line_interp_data_to_file(file_path):
    key_frame_index_list = []
    key_frame_list = []

    with open(file_path, 'rb') as r:
        data = r.read(2)
        max_frame = struct.unpack('H', data)[0]

        key_fram_index = []
        last_index = 0
        # get index
        while True:
            data = r.read(2)
            if not data:
                break
            uint16_item = struct.unpack('H', data)[0]
            if uint16_item == 65535 and last_index == max_frame:
                break
            key_fram_index.append(uint16_item)
            if uint16_item == max_frame:
                key_frame_index_list.append(key_fram_index)
                key_fram_index = []

            last_index = uint16_item
        # get keyframe data
        all_keyframe_data = []
        while True:
            data = r.read(2)
            if not data:
                break
            uint16_item = struct.unpack('H', data)[0]
            all_keyframe_data.append(uint16_item)

        index = 0
        for i in range(len(key_frame_index_list)):
            key_frame = []
            for j in range(len(key_frame_index_list[i])):
                vector_item = []
                vector_item.append(all_keyframe_data[index])
                vector_item.append(all_keyframe_data[index + 1])
                vector_item.append(all_keyframe_data[index + 2])
                key_frame.append(uncompress_to_quat(vector_item))
                index += 3
            key_frame_list.append(key_frame)
        print(len(key_frame_list))
        return spell_motion_data(key_frame_index_list, key_frame_list)

def get_root_joint_data(vector3_data):
    root_data = []
    for frame_data in vector3_data:
        root_data.append(frame_data[0])

    return root_data

def spell_motion_data(key_frame_index_list, key_frame_list):
    motion_data = []
    joint_data = []
    motion_data_vector3 = loat_motion_data_as_vector3(bvh_file_path)
    root_move_data = get_root_joint_data(motion_data_vector3)
    for i in range(len(key_frame_index_list)):
        joint_data_item = []
        for j in range(len(key_frame_index_list[i])):
            if j == len(key_frame_index_list[i]) - 1:
                joint_data_item.append(R.from_quat(key_frame_list[i][j]).as_euler('XYZ', degrees=True))
                break
            joint_data_item.append(R.from_quat(key_frame_list[i][j]).as_euler('XYZ', degrees=True))
            if key_frame_index_list[i][j + 1] - key_frame_index_list[i][j] == 1:
                continue
            for k, t in enumerate(np.linspace(0, 1, key_frame_index_list[i][j + 1] - key_frame_index_list[i][j] - 1)):
                interpolated_quaternion = R.from_quat(np.array(key_frame_list[i][j]) + t * (np.array(key_frame_list[i][j + 1]) - np.array(key_frame_list[i][j])))
                joint_data_item.append(interpolated_quaternion.as_euler('XYZ', degrees=True))
        joint_data.append(joint_data_item)

    for i in range(len(root_move_data)):
        motion_data_item = []
        for j in range(len(joint_data)):
            if j == 0:
                motion_data_item.append(root_move_data[i][0])
                motion_data_item.append(root_move_data[i][1])
                motion_data_item.append(root_move_data[i][2])
            motion_data_item.append(joint_data[j][i][0])
            motion_data_item.append(joint_data[j][i][1])
            motion_data_item.append(joint_data[j][i][2])
        motion_data.append(motion_data_item)

    return motion_data

def quaternion_distance(q1, q2):
    dot_product = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]
    angle = 2 * math.acos(abs(dot_product))
    return angle

def is_quaternion_close(q1, q2, threshold=error_num):
    angle = quaternion_distance(q1, q2)
    return angle < threshold

def main():
    motion_data = loat_motion_data_as_vector3(bvh_file_path)
    quat_data = convert_rotation_data_to_quat_no_quantization(motion_data)
    joint_frame_data = convert_to_joint_frame_data(quat_data)
    # quat_test = line_interp(joint_frame_data[0][1], joint_frame_data[0][3], 0.5)
    # print(joint_frame_data[0][2].dot(quat_test))
    #
    # print(R.from_quat(quat_test).as_euler('XYZ', degrees=True))
    # print(R.from_quat(joint_frame_data[0][2]).as_euler('XYZ', degrees=True))
    write_line_interp_data_to_file(motion_file_path_compress_line_interp, joint_frame_data)

    motion_data = read_line_interp_data_to_file(motion_file_path_compress_line_interp)
    write_uncompress_data(motion_data, uncompress_bvh_file_line_interp)

if __name__ == "__main__":
    main()
