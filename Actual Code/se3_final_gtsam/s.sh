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

# optimisation output
# converged
# errorThreshold: 0.0257556 <? 0
# absoluteDecrease: 2.37009633098e-06 <? 1e-05
# relativeDecrease: 9.2013962349e-05 <? 1e-05
# iterations: 4 >? 100
# Optimization complete
# initial error =  287.830905446377
# final error =  0.025755637389373705
# Writing results to file:  ./eight_shape_traj/opt.g2o
# Done!

# noise_lc:


#        max      1.450049
#       mean      0.858750
#     median      0.868405
#        min      0.557652
#       rmse      0.886641
#        sse      1332.493274
#        std      0.220634


# opt:

#        max      0.313310
#       mean      0.152140
#     median      0.144835
#        min      0.033296
#       rmse      0.165047
#        sse      46.172641
#        std      0.063983
