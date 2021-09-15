import matplotlib.pyplot as plt
from sys import argv
import math
import numpy as np
import copy

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


def draw(X, Y, name):
	ax = plt.subplot(111)
	ax.plot(X, Y, 'ro')
	ax.plot(X, Y, 'k-')
	ax.set_aspect('equal', 'datalim')
	ax.margins(0.1)
	plt.title(name)
	plt.show()


def drawTheta(X, Y, THETA):
	ax = plt.subplot(111)

	for i in range(len(THETA)):
		x2 = math.cos(THETA[i]) + X[i]
		y2 = math.sin(THETA[i]) + Y[i]
		plt.plot([X[i], x2], [Y[i], y2], 'm->')

	ax.plot(X, Y, 'ro')
	# ax.plot(X, Y, 'k-')

	plt.xlim(-12, 12)
	plt.ylim(-12, 12)
	
	plt.show()



def addNoise(X, Y, YAW, noise = [0,0,0]):
    noisyX = copy.deepcopy(X)    
    noisyY = copy.deepcopy(Y)    
    noisyYaw = copy.deepcopy(YAW)
    
    for idx in range(1,len(X)):
        x1, x2  = X[idx-1], X[idx]
        y1, y2  = Y[idx-1], Y[idx]
        yaw1, yaw2  = YAW[idx-1], YAW[idx]
        T1 = np.asarray([[np.cos(yaw1), -np.sin(yaw1), x1],[np.sin(yaw1), np.cos(yaw1), y1],[0, 0, 1]])
        T2 = np.asarray([[np.cos(yaw2), -np.sin(yaw2), x2],[np.sin(yaw2), np.cos(yaw2), y2],[0, 0, 1]])
        T2_1 = np.linalg.inv(T1) @ T2
        dx, dy, dyaw = T2_1[0][2], T2_1[1][2], np.arctan2(T2_1[1][0], T2_1[0][0]) # property w.r.t frame 1

        if idx<5:
            nx, ny, nyaw = 0, 0, 0
        else:
            nx, ny, nyaw = noise[0], noise[1], noise[2]
        
        dx, dy, dyaw = dx+nx, dy+ny, dyaw+nyaw
        T2_1 = np.asarray([
            [np.cos(dyaw), -np.sin(dyaw), dx],
            [np.sin(dyaw), np.cos(dyaw), dy],
            [0, 0, 1]
        ])
        x1, y1, yaw1 = noisyX[idx-1], noisyY[idx-1], noisyYaw[idx-1]
        T1 = np.asarray([
            [np.cos(yaw1), -np.sin(yaw1), x1],
            [np.sin(yaw1), np.cos(yaw1), y1],
            [0, 0, 1],
        ])
        T2 = T1 @ T2_1
        noisyX[idx], noisyY[idx], noisyYaw[idx] = T2[0,2], T2[1, 2], math.atan2(T2[1, 0], T2[0, 0])
    return noisyX, noisyY, noisyYaw






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
                dx, dy, dyaw = T2_1[0][2], T2_1[1][2], np.arctan2(T2_1[1][0], T2_1[0][0]) # property w.r.t frame 1
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


if __name__ == '__main__':

	X, Y, YAW = read_poses(argv[1])
	build_pose_graph(X, Y, YAW, argv[2])
	(xOpt, yOpt, tOpt) = read_pose_graph_vertices(argv[2])
	draw(xOpt, yOpt, "Ground Truth")

	(xN, yN, tN) = addNoise(X, Y, YAW, [0,0,0.00015])
	build_pose_graph(xN, yN, tN, argv[3])
	(xOpt, yOpt, tOpt) = read_pose_graph_vertices(argv[3])
	draw(xOpt, yOpt, "Noisy Poses")
