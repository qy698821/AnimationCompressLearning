import numpy as np
from scipy.spatial.transform import Rotation as R

def load_motion_data(bvh_file_path):
    """part2 辅助函数，读取bvh文件"""
    with open(bvh_file_path, 'r') as f:
        lines = f.readlines()
        for i in range(len(lines)):
            if lines[i].startswith('Frame Time'):
                break
        motion_data = []
        for line in lines[i+1:]:
            data = [float(x) for x in line.split()]
            if len(data) == 0:
                break
            motion_data.append(np.array(data).reshape(1,-1))
        motion_data = np.concatenate(motion_data, axis=0)
    return motion_data



def part1_calculate_T_pose(bvh_file_path):
    """请填写以下内容
    输入： bvh 文件路径
    输出:
        joint_name: List[str]，字符串列表，包含着所有关节的名字
        joint_parent: List[int]，整数列表，包含着所有关节的父关节的索引,根节点的父关节索引为-1
        joint_offset: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的偏移量

    Tips:
        joint_name顺序应该和bvh一致
    """
    joint_name = []
    joint_parent = []
    joint_offset_list = []
    joint_stack = []

    current_name = ""
    current_parent = -1
    max_parent = -1
    joint_stack.append(current_parent)
    with open(bvh_file_path, 'r') as f:
        lines = f.readlines()
        for i in range(len(lines)):
            if "RootJoint" in lines[i]:
                current_name = "RootJoint"
                joint_name.append(current_name)
                joint_parent.append(current_parent)
            if 'JOINT' in lines[i]:
                current_name = lines[i].split(" ")[-1].replace("\n", "")
                joint_name.append(current_name)
                joint_parent.append(current_parent)
            if "End Site" in lines[i]:
                current_name = current_name + '_end'
                joint_name.append(current_name)
                joint_parent.append(current_parent)
            if "OFFSET" in lines[i]:
                data_line = lines[i].split("OFFSET")[-1]
                data = [float(x) for x in data_line.split()]
                joint_offset_list.append(data)
            if "{" in lines[i]:
                max_parent = max_parent + 1
                current_parent = max_parent
                joint_stack.append(max_parent)
            if "}" in lines[i]:
                joint_stack.pop()
                current_parent = joint_stack[-1]
            if lines[i].startswith('MOTION'):
                break

    joint_offset = np.array(joint_offset_list)
    return joint_name, joint_parent, joint_offset


def part2_forward_kinematics(joint_name, joint_parent, joint_offset, motion_data, frame_id):
    """请填写以下内容
    输入: part1 获得的关节名字，父节点列表，偏移量列表
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数
        frame_id: int，需要返回的帧的索引
    输出:
        joint_positions: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的全局位置
        joint_orientations: np.ndarray，形状为(M, 4)的numpy数组，包含着所有关节的全局旋转(四元数)
    Tips:
        1. joint_orientations的四元数顺序为(x, y, z, w)
        2. from_euler时注意使用大写的XYZ
    """
    joint_positions = []
    joint_orientations = []
    frame_data = motion_data[frame_id]
    step = 3
    channel_num = 0
    frame_data_3 = [frame_data[i:i+step] for i in range(0, len(frame_data), step)]
    for i in range(len(joint_name)):
        if joint_name[i] == "RootJoint":
            joint_positions.append(frame_data_3[channel_num])
            channel_num += 1
            joint_orientations.append(R.from_euler('XYZ', [frame_data_3[channel_num][0], frame_data_3[channel_num][1], frame_data_3[channel_num][2]], degrees=True).as_quat())
        else:
            if not "_end" in joint_name[i]:
                channel_num += 1
            local_rotation = R.from_euler('XYZ', [frame_data_3[channel_num][0], frame_data_3[channel_num][1], frame_data_3[channel_num][2]], degrees=True)
            parent_rotation = R.from_quat(joint_orientations[joint_parent[i]])
            joint_orientations.append((parent_rotation * local_rotation).as_quat())
            joint_location = joint_positions[joint_parent[i]] + R.from_quat(joint_orientations[joint_parent[i]]).apply(joint_offset[i])
            joint_positions.append(joint_location)

    joint_positions = np.array(joint_positions)
    joint_orientations = np.array(joint_orientations)
    return joint_positions, joint_orientations


def part3_retarget_func(T_pose_bvh_path, A_pose_bvh_path):
    """
    将 A-pose的bvh重定向到T-pose上
    输入: 两个bvh文件的路径
    输出: 
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数。retarget后的运动数据
    Tips:
        两个bvh的joint name顺序可能不一致哦(
        as_euler时也需要大写的XYZ
    """
    motion_data = None
    return motion_data
