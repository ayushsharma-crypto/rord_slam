#!/bin/bash

PYTHON="/usr/bin/python3"

jpg=".jpg"
npy=".npy"
rgb="rgb/rgb"
depth="depth/depth"

data_path='../../Files/data_gazebo/data5/'
topH='../intermediate_data/data5/topH.npy'
camera_file='../configs/camera_gazebo.txt'
model_rord="../../Files/models/rord.pth"
camWrtBase='../configs/camWrtBase.txt'


transLC='../intermediate_data/data5/few_transLC_loop_pairs/'
odom_edge='../intermediate_data/data5/few_odom_transLC_loop_pairs.txt'


loop_point_1=(156 261 410 562 660 1015 1088 353 450 894 1389 1)
loop_point_2=(1657 1535 1427 783 698 1284 1208 450 894 1389 1423 1754)

for idx in `seq 0 11`;
    do
        j=${loop_point_1[idx]}
        i=${loop_point_2[idx]}
        pj=""
        pi=""

        if [ ${#j} == 1 ]
        then 
            pj="00000"
        elif [ ${#j} == 2 ]
        then
            pj="0000"
        elif [ ${#j} == 3 ]
        then
            pj="000"
        elif [ ${#j} == 4 ]
        then
            pj="00"
        elif [ ${#j} == 5 ]
        then
            pj="0"
        fi

        if [ ${#i} == 1 ]
        then 
            pi="00000"
        elif [ ${#i} == 2 ]
        then
            pi="0000"
        elif [ ${#i} == 3 ]
        then
            pi="000"
        elif [ ${#i} == 4 ]
        then
            pi="00"
        elif [ ${#i} == 5 ]
        then
            pi="0"
        fi

        rgb1="$data_path$rgb$pj$j$jpg"
        rgb2="$data_path$rgb$pi$i$jpg"
        d1="$data_path$depth$pj$j$npy"
        d2="$data_path$depth$pi$i$npy"
        curr_transLC="$transLC$pj$j-$pi$i$npy"

        if [ $1 -gt 0 ]
        then
            echo $d1
            echo $d2
            echo $rgb1
            echo $rgb2
            echo $curr_transLC
        fi
        
        $PYTHON  ./register.py --rgb1 $rgb1 --rgb2 $rgb2 --depth1 $d1 --depth2 $d2 --camera_file $camera_file  --H $topH --model_rord $model_rord --save_trans $curr_transLC
        echo
        echo "Saved transLC..."
        echo 
        $PYTHON ./cordTrans.py --static_trans $camWrtBase --rord_trans $curr_transLC --save_edge $odom_edge --p1 ${loop_point_1[idx]} --p2 ${loop_point_2[idx]}
        echo
        echo "$i edge done..."
        echo 
    done