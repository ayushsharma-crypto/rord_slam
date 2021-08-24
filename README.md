# RoRD SLAM  

### Pretrained Models and dataset  

1. Download gazebo_dataset (mainly data5 and data6) [Google Drive](https://drive.google.com/file/d/1Yr7PW7nS03MatvqubV1iX18so6F6ieKW/view) (382 MB)
2. Download big_dataset [Google drive](https://iiitaphyd-my.sharepoint.com/personal/sudhansh_yelishetty_students_iiit_ac_in/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fsudhansh%5Fyelishetty%5Fstudents%5Fiiit%5Fac%5Fin%2FDocuments%2Fbig%5Fdataset&originalPath=aHR0cHM6Ly9paWl0YXBoeWQtbXkuc2hhcmVwb2ludC5jb20vOmY6L2cvcGVyc29uYWwvc3VkaGFuc2hfeWVsaXNoZXR0eV9zdHVkZW50c19paWl0X2FjX2luL0VucVJISXQ2UU9CQmpIVXdxWmZZYWxrQkZMUUVzUDE2c012Rnp5dEFJblJ3eXc%5FcnRpbWU9SXN1TzRSbG4yVWc) (0.5KB)
3. Download models from [Google Drive](https://drive.google.com/file/d/1-5aLHyZ_qlHFNfRnDpXUh5egtf_XtoiA/view?usp=sharing) (73.9 MB) in the base directory.    
4. Download dataset from [Google Drive](https://drive.google.com/file/d/1BkhcHBKwcjNHgbLZ1XKurpcP7v4hFD_b/view?usp=sharing) (97.8 MB)  

`Ignored through .gitignore file`

### Generating top view  
1. Selects four points in the image in the Region of interest, whose top view is required:  
	1. `python getRealOneGazebo.py --rgb 'data/drone1/rgb/rgb000540.jpg' --depth 'data/drone1/depth/depth000540.npy' --camera_file configs/camera_gazebo_drone.txt`   
	2. It generates Homography matrics for orthographic view of the provided image.

### Caculating transformation  

In last part, we got Homography matrics for orthographic view for each and every images input. Now, we perform two steps:-
1. Calculate the transformation from one orthographic view to another. By default, this is in camera frame(also left handed system).
2. This need to be in odom frame(also right handed system). Hence, the step `2`.

The steps are:-

1. Inferring on gazebo dataset in orthographic view:    
		1. `cd demo`  
		2. `python register.py --rgb1 <img1.jpg>  --rgb2 <img2.jpg>  --depth1 <depth1.npy>  --depth2 <depth2.npy>  --camera_file ../configs/camera_gazebo.txt  --H ../configs/topH.npy  --model_rord ../models/rord.pth --viz3d --save_trans`  
		3. If homography is different for the two images, then use `--H` and `--H2` flags:  
		`python register.py --H <first_homography.npy> --H2 <second_homography.npy>`     
		4. Saves the Transformation matrix from one image to another.

2. Converting RoRD transformations (in camera frame and in left handed system) to loop closure transformations (in odom frame and in right handed system):  
	1. Getting static transform from ros, `Tbase_camera` or `Camera wrt Base link`
		1. `rosrun tf tf_echo base_link camera_link`    
	2. Transformations for SE2 and SE3:  
		1. SE2 Optimization:  
			1. `python cordTrans.py --static_trans ../configs/camWrtBase.txt --rord_trans ../demo/transLC.npy`  
		2. SE3 Optimization:  
			1. `python cordTrans.py --static_trans ../configs/camWrtBaseDrone.txt --rord_trans ../data/drone1/transLC212_2275.npy --se3`   

	3. Derivation of how transformations in [`cordTrans.py`](pose_graph/cordTrans.py) are dervied can be found [here](https://drive.google.com/file/d/1UfLmfj4JtnokyQDI0k9mx3KbO0Xsvegk/view?usp=sharing).  

### Optimizing pose graph  
1. `cd pose_graph`  
2. Generating odometry edges `noise.g2o` using gazebo's odometer output `poses.txt`.  
	1. SE2 Optimization:  
		1. `python genG2o.py data5/poses.txt`  
	2. SE3 Optimization:  
		1. `python genG2oSE3.py drone1/poses.txt`    
3. Adding loop closure edges `loop_pairs.txt` to generated odometry edges `noise.g2o` to output `noise_lc.g2o`. Also optimizing odometry and loop closure edges stored in `noise_lc.g2o` to output `opt.g2o`.  
	1. SE2 Optimization:  
		1. `python optimizePose.py data5/noise.g2o data5/loop_pairs.txt`  
	2. SE3 Optimization:  
		1. `python optimizePoseSE3.py drone1/noise.g2o drone1/loop_pairs.txt drone1/gt.g2o`  
