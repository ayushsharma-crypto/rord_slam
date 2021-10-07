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

$PYTHON genG2o.py ./data5/poses.txt 
$PYTHON optimizePose.py ./data5/noise.g2o ./data5/loop_pairs.txt
$PYTHON evo.py ../final_testing_code/data5/gt.g2o ./data5_evo/gt.kitti
$PYTHON evo.py ./data5/noise_lc.g2o ./data5_evo/gt_noise_lc.kitti
$PYTHON evo.py ./data5/opt.g2o ./data5_evo/final.kitti

evo_traj kitti ./data5_evo/gt_noise_lc.kitti --ref=./data5_evo/gt.kitti -p --plot_mode=xy
evo_traj kitti ./data5_evo/final.kitti --ref=./data5_evo/gt.kitti -p --plot_mode=xy
evo_ape kitti ./data5_evo/gt.kitti ./data5_evo/gt_noise_lc.kitti -va --plot --plot_mode xy
evo_ape kitti ./data5_evo/gt.kitti ./data5_evo/final.kitti -va --plot --plot_mode xy