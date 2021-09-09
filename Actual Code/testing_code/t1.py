
# Ideally, do the following:
# 1. Solve the theory question first (will help you during coding)
# 2. Read global poses (data5 ka poses.txt for example)
# 3. Build the ground truth pose-graph using the relations you get from your theory
# 4. Try to visualise it and check if it works

# You know that data5 has that T shape, you should be able to recreate the T shape accurately

# Kuch aur datasets ke liye results produce karo khudke code se

# setup ROS


import matplotlib.pyplot as plt
from sys import argv
import math
import numpy as np


def read_poses(filepath):
    x_val = []
    y_val = []
    yaw_val = []
    with open(filepath,'r') as pose_file:
        line_list =  pose_file.readlines()
    for line in line_list:
        X, Y, YAW = line.split()
        X = float(X)
        Y = float(Y)
        YAW = math.radians(float(YAW.rstrip('\n')))
        x_val.append(X)
        y_val.append(Y)
        yaw_val.append(YAW)
    return x_val, y_val, yaw_val


def build_pose_graph(X,Y,YAW, pose_graph_file):
	with open(pose_graph_file, 'w+') as g2o:
            total_pose = len(X)
            info_mat = "500.0 0.0 0.0 500.0 0.0 500.0"

            # Nodes in world frame (x, y, yaw in radian)
            for idx in range(total_pose):
                g2o.write("VERTEX_SE2 " + str(idx) + " " + str(X[idx]) + " " + str(Y[idx]) + " " + str(YAW[idx])+"\n")

            # Odometry Constraint
            # Ti : i with respect to world
            # Ti_j : i with respect to j
            g2o.write("# Odometry constraints\n")
            for idx in range(1,total_pose):
                x1, x2  = X[idx-1], X[idx]
                y1, y2  = Y[idx-1], Y[idx]
                yaw1, yaw2  = YAW[idx-1], YAW[idx]
                T1 = np.asarray([[np.cos(yaw1), -np.sin(yaw1), x1],[np.sin(yaw1), np.cos(yaw1), y1],[0, 0, 1]])
                T2 = np.asarray([[np.cos(yaw2), -np.sin(yaw2), x2],[np.sin(yaw2), np.cos(yaw2), y2],[0, 0, 1]])
                T2_1 = np.linalg.inv(T1) @ T2
                dx, dy, dyaw = T2_1[0][2], T2_1[1][2], np.arctan2(T2_1[1][0], T2_1[0][0])
                line = "EDGE_SE2 "+str(idx-1)+" "+str(idx)+" "+str(dx)+" "+str(dy)+" "+str(dyaw)+" "+info_mat+"\n"
                g2o.write(line)
            g2o.write("FIX 0\n")


def read_pose_graph_vertices(filepath):
    x_val = []
    y_val = []
    yaw_val = []
    with open(filepath,'r') as pose_file:
        line_list =  pose_file.readlines()
    for line in line_list:
        if "VERTEX_SE2" in line:
            (ver, ind, X, Y, YAW) = line.split()
            x_val.append(float(X))
            y_val.append(float(Y))
            yaw_val.append(float(YAW.rstrip('\n')))
    return x_val, y_val, yaw_val


def draw(X, Y):
	ax = plt.subplot(111)
	ax.plot(X, Y, 'ro')
	ax.plot(X, Y, 'k-')
	ax.set_aspect('equal', 'datalim')
	ax.margins(0.1)
	plt.show()


def addNoise(X, Y, THETA):
	xN = np.zeros(len(X)); yN = np.zeros(len(Y)); tN = np.zeros(len(THETA))
	xN[0] = X[0]; yN[0] = Y[0]; tN[0] = THETA[0]

	for i in range(1, len(X)):
		# Get T2_1
		p1 = (X[i-1], Y[i-1], THETA[i-1])
		p2 = (X[i], Y[i], THETA[i])
		T1_w = np.array([[math.cos(p1[2]), -math.sin(p1[2]), p1[0]], [math.sin(p1[2]), math.cos(p1[2]), p1[1]], [0, 0, 1]])
		T2_w = np.array([[math.cos(p2[2]), -math.sin(p2[2]), p2[0]], [math.sin(p2[2]), math.cos(p2[2]), p2[1]], [0, 0, 1]])
		T2_1 = np.dot(np.linalg.inv(T1_w), T2_w)
		del_x = T2_1[0][2]
		del_y = T2_1[1][2]
		del_theta = math.atan2(T2_1[1, 0], T2_1[0, 0])
		
		# Add noise
		if(i<5):
			xNoise = 0; yNoise = 0; tNoise = 0
		else:
			# np.random.seed(42)
			# xNoise = np.random.normal(0, 0.08); yNoise = np.random.normal(0, 0.08); tNoise = np.random.normal(0, 0.002)
			# xNoise = 0.005; yNoise = 0.005; tNoise = -0.0005
			# xNoise = 0.01; yNoise = 0.01; tNoise = 0.0007
			xNoise = 0; yNoise = 0; tNoise = 0.00015


		del_xN = del_x + xNoise; del_yN = del_y + yNoise; del_thetaN = del_theta + tNoise

		# Convert to T2_1'
		T2_1N = np.array([[math.cos(del_thetaN), -math.sin(del_thetaN), del_xN], [math.sin(del_thetaN), math.cos(del_thetaN), del_yN], [0, 0, 1]])

		# Get T2_w' = T1_w' . T2_1'
		p1 = (xN[i-1], yN[i-1], tN[i-1])
		T1_wN = np.array([[math.cos(p1[2]), -math.sin(p1[2]), p1[0]], [math.sin(p1[2]), math.cos(p1[2]), p1[1]], [0, 0, 1]])
		T2_wN = np.dot(T1_wN, T2_1N)
		
		# Get x2', y2', theta2'
		x2N = T2_wN[0][2]
		y2N = T2_wN[1][2]
		theta2N = math.atan2(T2_wN[1, 0], T2_wN[0, 0])

		xN[i] = x2N; yN[i] = y2N; tN[i] = theta2N

	# tN = getTheta(xN, yN)

	return (xN, yN, tN)


if __name__ == '__main__':
	X, Y, YAW = read_poses(argv[1])
	pose_graph_name = argv[2]
	draw(X, Y)
	# (xN, yN, tN) = addNoise(X, Y, THETA)
	# draw(xN, yN, tN)
	build_pose_graph(X, Y, YAW, pose_graph_name)
	(xOpt, yOpt, tOpt) = read_pose_graph_vertices(pose_graph_name)
	draw(xOpt, yOpt)
