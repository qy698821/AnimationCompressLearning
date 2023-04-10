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

def convert_to_joint_frame_data(quat_data):
    joint_frame_data = []
    for i in range(len(quat_data)):
        for j in range(len(quat_data[i])):
            if i == 0:
                joint_frame_data.append([])
            joint_frame_data[j-1].append(quat_data[i][j])
    print(joint_frame_data)

motion_data = loat_motion_data_as_vector3(bvh_file_path)
quat_data = convert_rotation_data_to_quat_no_quantization(motion_data)
convert_to_joint_frame_data(quat_data)
