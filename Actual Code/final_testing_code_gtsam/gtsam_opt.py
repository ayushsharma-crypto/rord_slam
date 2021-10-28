"""
GTSAM Copyright 2010-2018, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

A 2D Pose SLAM example that reads input from g2o, converts it to a factor graph
and does the optimization. Output is written on a file, in g2o format
"""
# pylint: disable=invalid-name, E1101

from __future__ import print_function

import argparse
import math
import gtsam
import matplotlib.pyplot as plt
from gtsam.utils import plot


def main():
    """Main runner."""

    parser = argparse.ArgumentParser(
        description="A 2D Pose SLAM example that reads input from g2o, "
        "converts it to a factor graph and does the optimization. "
        "Output is written on a file, in g2o format")
    parser.add_argument('-i', '--input', help='input file g2o format')
    parser.add_argument(
        '-o',
        '--output',
        help="the path to the output file with optimized graph")
    parser.add_argument('-m',
                        '--maxiter',
                        type=int,
                        help="maximum number of iterations for optimizer")
    parser.add_argument('-k',
                        '--kernel',
                        choices=['none', 'huber', 'tukey'],
                        default="none",
                        help="Type of kernel used")
    parser.add_argument("-p",
                        "--plot",
                        action="store_true",
                        help="Flag to plot results")
    parser.add_argument("-lp",
                        "--loop_pairs",
                        help="path to loop pairs")
    args = parser.parse_args()

    g2oFile = gtsam.findExampleDataFile("noisyToyGraph.txt") if args.input is None\
        else args.input

    maxIterations = 100 if args.maxiter is None else args.maxiter

    is3D = False

    graph, initial = gtsam.readG2o(g2oFile, is3D)

    assert args.kernel == "none", "Supplied kernel type is not yet implemented"

    # Add prior on the pose having index (key) = 0
    priorModel = gtsam.noiseModel.Diagonal.Variances(gtsam.Point3(1e-6, 1e-6, 1e-8))
    graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(), priorModel))

    params = gtsam.GaussNewtonParams()
    params.setVerbosity("Termination")
    params.setMaxIterations(maxIterations)
    # parameters.setRelativeErrorTol(1e-5)
    # Create the optimizer ...
    optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)
    # ... and optimize
    result = optimizer.optimize()

    print("Optimization complete")
    print("initial error = ", graph.error(initial))
    print("final error = ", graph.error(result))

    if args.output is None:
        print("\nFactor Graph:\n{}".format(graph))
        print("\nInitial Estimate:\n{}".format(initial))
        print("Final Result:\n{}".format(result))
    else:
        outputFile = args.output
        print("Writing results to file: ", outputFile)
        graphNoKernel, _ = gtsam.readG2o(g2oFile, is3D)
        gtsam.writeG2o(graphNoKernel, result, outputFile)
        print("Done!")

    if args.plot:
        resultPoses = gtsam.utilities.extractPose2(result)
        for i in range(resultPoses.shape[0]):
            plot.plot_pose2(1, gtsam.Pose2(resultPoses[i, :]))
        plt.show()
    
    (xOpt, yOpt, tOpt) = read_pose_graph_vertices(args.output)
    src, trg, trans = readLC(args.loop_pairs)
    drawLC(xOpt, yOpt, tOpt, src, trg, "Optimised")
    
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



if __name__ == "__main__":
    main()


# python gtsam_opt.py -i ./data5/gt_noise_lc.g2o -o ./data5/final.g2o -lp ../intermediate_data/data5/loop_pairs_udit_sir.txt -p