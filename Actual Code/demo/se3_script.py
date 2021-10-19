import numpy as np
import os

root_dir = "/home/ayushsharma/Documents/College_IIIT-H_Files/RRC/RoRD\ SLAM/My\ RoRD\ Version/"
data_dir_rgb = root_dir+"Files/Eight_shaped_data_6DOF/Eight_shaped_data/rgb"
data_dir_depth = root_dir+"Files/Eight_shaped_data_6DOF/Eight_shaped_data/depth"
data_new_H = root_dir+"Actual\ Code/intermediate_data/Eight_shaped_data/new_Trans"
transLC =  root_dir+'Actual\ Code/intermediate_data/Eight_shaped_data/transLC_loop_pairs/'
odom_edge= root_dir+'Actual\ Code/intermediate_data/Eight_shaped_data/odom_transLC_loop_pairs.txt'
model_rord=root_dir+"Files/models/rord.pth"
camWrtBase= root_dir+'Actual\ Code/configs/camWrtBaseDrone.txt'
cam =root_dir+"Actual\ Code/configs/camera_gazebo_drone.txt"

LP1 = [853 ]
LP2 = [1516 ]

def call_GROG(rgb_file, depth_file, camera, save_path):
    cmd = "python ../getRealOneGazeboSE3.py --rgb "
    cmd = cmd + rgb_file 
    cmd = cmd + " --depth "
    cmd = cmd + depth_file
    cmd = cmd +  " --camera_file "
    cmd = cmd + camera
    cmd = cmd + " --save "
    cmd = cmd + save_path
    os.system(cmd)
    print(cmd)

def call_register(rgb_file1, depth_file1, rgb_file2, depth_file2, camera, h1, h2, model_rord, save_path):
    cmd = "python ./register.py --rgb1 "
    cmd = cmd + rgb_file1 
    cmd = cmd + " --depth1 "
    cmd = cmd + depth_file1
    cmd = cmd + " --rgb2 "
    cmd = cmd + rgb_file2
    cmd = cmd + " --depth2 "
    cmd = cmd + depth_file2
    cmd = cmd +  " --camera_file "
    cmd = cmd + camera
    cmd = cmd +  " --H "
    cmd = cmd + h1
    cmd = cmd +  " --H2 "
    cmd = cmd + h2
    cmd = cmd +  " --model_rord "
    cmd = cmd + model_rord
    cmd = cmd + " --save_trans "
    cmd = cmd + save_path
    os.system(cmd)
    print(cmd)

def call_cood(static_trans, rord_trans, save_edge, p1, p2):
    cmd = "python ./cordTrans.py --static_trans "
    cmd = cmd + static_trans
    cmd = cmd + " --rord_trans "
    cmd = cmd + rord_trans
    cmd = cmd + " --save_edge "
    cmd = cmd + save_edge
    cmd = cmd + " --p1 "
    cmd = cmd + str(p1)
    cmd = cmd + " --p2 "
    cmd = cmd + str(p2)
    cmd = cmd + " --se3 "
    os.system(cmd)
    print(cmd)

def add_zero(val):
    if (val//10)==0:
        val="00000"+str(val)
    elif (val//100)==0:
        val="0000"+str(val)
    elif (val//1000)==0:
        val="000"+str(val)
    elif (val//10000)==0:
        val="00"+str(val)
    elif (val//100000)==0:
        val="0"+str(val)
    return val


for idx in range(len(LP1)):
    print("\nLP1 = ", LP1[idx], " LP2 = ", LP2[idx])
    I1 = add_zero(LP1[idx])
    I2 = add_zero(LP2[idx])
    rgb_file1 = data_dir_rgb+"/rgb"+I1+".jpg"
    depth_file1 = data_dir_depth+"/depth"+I1+".npy"
    rgb_file2 = data_dir_rgb+"/rgb"+I2+".jpg"
    depth_file2 = data_dir_depth+"/depth"+I2+".npy"
    camera = cam
    save_path1 = data_new_H+"/"+I1+".npy"
    save_path2 = data_new_H+"/"+I2+".npy"

    # run getRealOneGazebo.py
    call_GROG(rgb_file1, depth_file1, camera, save_path1)
    call_GROG(rgb_file2, depth_file2, camera, save_path2)

    # run register.py
    transLC_save = transLC+I1+"-"+I2+".npy"
    call_register(rgb_file1, depth_file1, rgb_file2, depth_file2, camera, save_path1, save_path2, model_rord, transLC_save)

    # run co-ord.py
    static_trans = camWrtBase
    rord_trans = transLC_save
    save_edge = odom_edge
    p1 = LP1[idx]
    p2 = LP2[idx]
    call_cood(static_trans, rord_trans, save_edge, p1, p2)
    print("\n")