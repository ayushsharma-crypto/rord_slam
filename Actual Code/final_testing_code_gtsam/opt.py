import gtsam
import matplotlib.pyplot as plt
from sys import argv, exit
import math
import numpy as np
import os


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



def readLC(filename):
	f = open(filename, 'r')
	A = f.readlines()
	f.close()

	src, trg, trans = [], [], []

	for i, line in enumerate(A):
		if(i%2 == 0):
			st, end = line.split(' ')
			src.append(int(st)); trg.append(int(end.rstrip('\n')))
		else:
			tran = line.split(' ')
			theta = math.radians(float(tran[2].rstrip('\n')))
			trans.append((float(tran[0]), float(tran[1]), theta))

	return src, trg, trans


def combine_lc_odometry(X,Y,YAW,src, trg, trans ,pose_graph_file):
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
			

            # Loop Constraints
            g2o.write("# Loop Closure constraints\n")
            total_lc = len(src)
            info_mat = "700.0 0.0 0.0 700.0 0.0 700.0\n"
            for i in range(total_lc):
                del_x, del_y, del_theta = str(trans[i][0]), str(trans[i][1]), str(trans[i][2])
                line = "EDGE_SE2 "+str(src[i])+" "+str(trg[i])+" "+del_x+" "+del_y+" "+del_theta+" "+info_mat
                g2o.write(line)

            g2o.write("FIX 0\n")




def draw(X, Y, name):
	ax = plt.subplot(111)
	ax.plot(X, Y, 'ro')
	ax.plot(X, Y, 'k-')
	ax.set_aspect('equal', 'datalim')
	ax.margins(0.1)
	plt.title(name)
	plt.show()



def drawLC(X, Y, THETA, srcs, trgs, name):
	ax = plt.subplot(111)
	ax.plot(X, Y, 'ro')
	ax.plot(X, Y, 'k-')
	for src, trg in zip(srcs, trgs):
		ax.plot([X[src], X[trg]], [Y[src], Y[trg]], 'b--', markersize=10)
		ax.plot([X[src], X[trg]], [Y[src], Y[trg]], 'bo', markersize=5)
	ax.set_aspect('equal', 'datalim')
	ax.margins(0.1)
	plt.title(name)
	plt.show()


def optimize(save_file, noiselc, x0, y0, t0):
	graph, initial = gtsam.readG2o(noiselc, is3D=False)

	# Add prior on the pose having index (key) = 0
	# x0, y0, t0 = -8.11295545835, -0.00335385260495, 0.009590975430197747
	priorModel = gtsam.noiseModel.Diagonal.Variances(gtsam.Point3(1e-6, 1e-6, 1e-8))
	graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(x0, y0, t0), priorModel))
	# print("pppp =  ",gtsam.Pose2(x0, y0, t0), " initial = ", type(initial))
	max_iterations=300
	# params = gtsam.GaussNewtonParams()
	# params.setVerbosity("Termination")
	# params.setMaxIterations(max_iterations)
	# # params.setRelativeErrorTol(1e-5)
	# # Create the optimizer ...
	# optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)
	# # ... and optimize
	params = gtsam.LevenbergMarquardtParams()
	params.setVerbosity("Termination")  # this will show info about stopping conds
	optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)

	result = optimizer.optimize()
	print("Optimization complete")
	print("initial error = ", graph.error(initial))
	print("final error = ", graph.error(result))

	print("Writing results to file: ", save_file)
	graphNoKernel, _ = gtsam.readG2o(noiselc, is3D=False)
	gtsam.writeG2o(graphNoKernel, result, save_file)
	print("Done!")


if __name__ == '__main__':
	X, Y, THETA = read_pose_graph_vertices(argv[1])
	draw(X, Y, "Noisy Odometry Pose Graph")

	src, trg, trans = readLC(argv[2])
	drawLC(X, Y, THETA, src, trg, "Noisy + LC (Before Optimisation)")

	combine_lc_odometry(X, Y, THETA, src, trg, trans,argv[3])
	optimize(argv[4],argv[3], X[0], Y[0], THETA[0])
	(xOpt, yOpt, tOpt) = read_pose_graph_vertices(argv[4])
	drawLC(xOpt, yOpt, tOpt, src, trg, "Optimised")