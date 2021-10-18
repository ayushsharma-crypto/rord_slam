import numpy as np
import os

data_dir_rgb = "../Files/Eight_shaped_data_6DOF/Eight_shaped_data/rgb"
data_dir_depth = "../Files/Eight_shaped_data_6DOF/Eight_shaped_data/depth"
data_new_H = "./intermediate_data/Eight_shaped_data/new_Trans"
data_new_ortho = "./intermediate_data/Eight_shaped_data/new_ortho"
data_new_H_txt = "./intermediate_data/Eight_shaped_data/new_H.txt"

def call_GROG(rgb_file, depth_file, camera, save_path, save_ortho_path):
    cmd = "python getRealOneGazeboSE3.py --rgb "
    cmd = cmd + rgb_file 
    cmd = cmd + " --depth "
    cmd = cmd + depth_file
    cmd = cmd +  " --camera_file "
    cmd = cmd + camera
    cmd = cmd + " --save "
    cmd = cmd + save_path
    cmd = cmd + " --save_ortho "
    cmd = cmd + save_ortho_path
    with open(data_new_H_txt, "a") as F:
        F.write("\n\n")
        F.write(cmd)
        F.write("\n\n")
    os.system(cmd)

rgb_file_list = sorted(os.listdir(data_dir_rgb))
depth_file_list = sorted(os.listdir(data_dir_depth))
assert len(rgb_file_list)==len(depth_file_list)
print(len(depth_file_list))
for idx in range(len(rgb_file_list)):
    rgb_file = data_dir_rgb+"/"+rgb_file_list[idx]
    depth_file = data_dir_depth+"/"+depth_file_list[idx]
    camera = "./configs/camera_gazebo_drone.txt"
    save_path = data_new_H+"/"+rgb_file_list[idx][3:-4]+".npy"
    save_ortho_path = data_new_ortho+"/"+rgb_file_list[idx][3:-4]+".png"
    call_GROG(rgb_file, depth_file, camera, save_path, save_ortho_path)
