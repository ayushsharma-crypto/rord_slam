#!/bin/bash

data_path='../../Files/data_gazebo/data5/'
jpg=".jpg"
npy=".npy"
rgb="rgb/rgb"
depth="depth/depth"
transLC='../intermediate_data/data5/all_transLC/'
odom_edge='../intermediate_data/data5/all_odom_transLC.txt'
camera_file='../configs/camera_gazebo.txt'
topH='../intermediate_data/data5/topH.npy'
model_rord="../../Files/models/rord.pth"
camWrtBase='../configs/camWrtBase.txt'

PYTHON="/usr/bin/python3"

for i in `seq 1 1829`;
    do
        pj=""
        pi=""
        j=$((i-1))

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
        $PYTHON ./cordTrans.py --static_trans $camWrtBase --rord_trans $curr_transLC --save_edge $odom_edge
        echo
        echo "$i edge done..."
        echo 
    done