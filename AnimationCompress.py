import struct
import numpy as np
from Lab1_FK_answers import *

bvh_file_path = "data/walk60.bvh"
motion_file_path_compress_float = "data/compress/walk60_compress_float.txt"
motion_file_path_compress_unit16 = "data/compress/walk60_compress_unit16.txt"

motion_file_path_uncompress_unit16 = "data/compress/walk60_uncompress_unit16.txt"

uncompress_bvh_file = "data/walk60_uint16_uncompress.bvh"

# data_range_min = -54
# data_range_max = 70

data_range_min = -360
data_range_max = 360

# translate the value from (left_min, left_max) to (right_min, right_max)
def translate(value, left_min, left_max, right_min, right_max):
    left_span = left_max - left_min
    right_span = right_max - right_min

    value_scaled = float(value - left_min) / float(left_span)
    return right_min + (value_scaled * right_span)

def save_as_float(motion_data):
    len_frame_data = len(motion_data[0])
    with open(motion_file_path_compress_float, 'wb') as w:
        w.write(struct.pack("H", len_frame_data))
        for i in range(len(motion_data)):
            w.write(struct.pack(("%df" % len(motion_data[i])), *motion_data[i]))

def read_from_float():
    motion_data_from_float = []
    num_frame_data = 0
    with open(motion_file_path_compress_float, 'rb') as r:
        data = r.read(2)
        num_frame_data = struct.unpack('H', data)[0]
        print(num_frame_data)
        while True:
            data = r.read(4)
            if not data:
                break
            float_item = struct.unpack('f', data)[0]
            motion_data_from_float.append(round(float_item, 6))

    new_array_data = [motion_data_from_float[i:i+num_frame_data] for i in range(0, len(motion_data_from_float), num_frame_data)]
    return new_array_data

def save_as_uint(motion_data):
    len_frame_data = len(motion_data[0])
    motion_data_uint = []
    for frame_motion in motion_data:
        frame_motion_uint = []
        for item in frame_motion:
            frame_motion_uint.append(np.uint16(translate(item, data_range_min, data_range_max, 0, 65535)))
        motion_data_uint.append(frame_motion_uint)

    with open(motion_file_path_compress_unit16, 'wb') as w:
        w.write(struct.pack("H", len_frame_data))
        for i in range(len(motion_data_uint)):
            w.write(struct.pack(("%dH" % len(motion_data_uint[i])), *motion_data_uint[i]))

def read_from_uint():
    motion_data_from_float = []
    num_frame_data = 0
    with open(motion_file_path_compress_unit16, 'rb') as r:
        data = r.read(2)
        num_frame_data = struct.unpack('H', data)[0]
        print(num_frame_data)
        while True:
            data = r.read(2)
            if not data:
                break
            uint16_item = struct.unpack('H', data)[0]
            float_item = translate(uint16_item, 0, 65535, data_range_min, data_range_max)
            motion_data_from_float.append(round(float_item, 6))

    new_array_data = [motion_data_from_float[i:i + num_frame_data] for i in
                      range(0, len(motion_data_from_float), num_frame_data)]
    return new_array_data

def write_the_motion_data(motion_data):
    save_as_float(motion_data)
    save_as_uint(motion_data)
    print(motion_data)

def write_uncompress_data(motion_data):
    # with open(motion_file_path_uncompress_unit16, "w") as w:
    #     for frame_data in motion_data:
    #         for joint_data in frame_data:
    #             w.write(str(joint_data))
    #             w.write("   ")
    #         w.write("\n")
    file_data = []
    with open(uncompress_bvh_file, "r") as r:
        lines = r.readlines()
        for i in range(len(lines)):
            file_data.append(lines[i])
            if lines[i].startswith('Frame Time'):
                break
    for frame_data in motion_data:
        frame_string = ""
        for joint_data in frame_data:
            frame_string = frame_string + str(joint_data) + "   "
        file_data.append(frame_string + "\n")

    with open(uncompress_bvh_file, "w") as w:
        for line in file_data:
            w.write(line)



def main():
    # compress the animation data by quantization
    motion_data = load_motion_data(bvh_file_path)
    write_the_motion_data(motion_data)

    # motion_data = read_from_float()

    motion_data = read_from_uint()
    write_uncompress_data(motion_data)
    print(motion_data)

    # max_value = 0
    # min_value = 0
    #
    # for frame_data in motion_data:
    #     for joint_data in frame_data:
    #         if joint_data > max_value:
    #             max_value = joint_data
    #         if joint_data < min_value:
    #             min_value = joint_data
    #
    # print(max_value)
    # print(min_value)

if __name__ == "__main__":
    main()
