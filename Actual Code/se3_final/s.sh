#!/bin/bash

PYTHON="/usr/bin/python3"


# $PYTHON genG2o.py ./bigdata/poses.txt 
# $PYTHON optimizePose.py ./bigdata/noise.g2o ./bigdata/loop_pairs.txt
# $PYTHON evo.py ../final_testing_code/bigdataset/gt.g2o ./bigdata_evo/gt.kitti
# $PYTHON evo.py ./bigdata/noise_lc.g2o ./bigdata_evo/gt_noise_lc.kitti
# $PYTHON evo.py ./bigdata/opt.g2o ./bigdata_evo/final.kitti

# evo_traj kitti ./bigdata_evo/gt_noise_lc.kitti --ref=./bigdata_evo/gt.kitti -p --plot_mode=xy
# evo_traj kitti ./bigdata_evo/final.kitti --ref=./bigdata_evo/gt.kitti -p --plot_mode=xy
# evo_ape kitti ./bigdata_evo/gt.kitti ./bigdata_evo/gt_noise_lc.kitti -va --plot --plot_mode xy
# evo_ape kitti ./bigdata_evo/gt.kitti ./bigdata_evo/final.kitti -va --plot --plot_mode xy

$PYTHON add_noise.py ../../Files/Eight_shaped_data_6DOF/Eight_shaped_data/poses.txt ./eight_shape_traj/gt.g2o ./eight_shape_traj/noise.g2o
$PYTHON opt.py ./eight_shape_traj/noise.g2o ../intermediate_data/Eight_shaped_data/odom_transLC_loop_pairs.txt ./eight_shape_traj/gt.g2o
$PYTHON se3_evo.py ./eight_shape_traj/gt.g2o ./eight_shape_traj/evo_res/gt.kitti
$PYTHON se3_evo.py ./eight_shape_traj/noise_lc.g2o ./eight_shape_traj/evo_res/noise_lc.kitti
$PYTHON se3_evo.py ./eight_shape_traj/opt.g2o ./eight_shape_traj/evo_res/opt.kitti

evo_traj kitti ./eight_shape_traj/evo_res/noise_lc.kitti --ref=./eight_shape_traj/evo_res/gt.kitti -p --plot_mode=xyz
evo_traj kitti ./eight_shape_traj/evo_res/opt.kitti --ref=./eight_shape_traj/evo_res/gt.kitti -p --plot_mode=xyz
evo_ape kitti ./eight_shape_traj/evo_res/gt.kitti ./eight_shape_traj/evo_res/noise_lc.kitti -va --plot --plot_mode xyz
evo_ape kitti ./eight_shape_traj/evo_res/gt.kitti ./eight_shape_traj/evo_res/opt.kitti -va --plot --plot_mode xyz