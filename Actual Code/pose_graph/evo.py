from sys import argv, exit
import math
import numpy as np
import os

def readG2o(fileName):
    f = open(fileName, 'r')
    A = f.readlines()
    f.close()
    
    X = []
    Y = []
    THETA = []

    for line in A:
        if "VERTEX_SE2" in line:
            (ver, ind, x, y, theta) = line.split()
            X.append(float(x))
            Y.append(float(y))
            THETA.append(float(theta.rstrip('\n')))

    # X_temp = X
    # Y_temp = Y
    # X = [y for y in Y_temp]
    # Y = [-x for x in X_temp]

    return (X, Y, THETA)


def convert(X, Y, THETA):
    A = np.zeros((len(X), 12))

    for i in range(len(X)):
        T = np.identity(4)
        T[0, 3] = X[i]
        T[1, 3] = Y[i]
        R = np.array([[math.cos(THETA[i]), -math.sin(THETA[i]), 0], [math.sin(THETA[i]), math.cos(THETA[i]), 0], [0, 0, 1]])
        T[0:3, 0:3] = R

        A[i] = T[0:3, :].reshape(1, 12)

    return A

def getRadians(degrees):
    return degrees * np.pi/180

def getPoses(filename):
    poses = np.loadtxt(filename)
    poses[:,2] = getRadians(poses[:,2])
    return poses

def writeOdom(poses):
    g2o = open("gt.g2o", "w")
    for i, [x, y, theta] in enumerate(poses):
        line = "VERTEX_SE2 " + str(i) + " " + str(x) + " " + str(y) + " " + str(theta)
        g2o.write(line)
        g2o.write("\n")

    info_mat = "500.0 0.0 0.0 500.0 0.0 500.0"
    for i in range(1, len(poses)):
        p1 = (poses[i-1,0], poses[i-1,1], poses[i-1,2])
        p2 = (poses[i,0], poses[i,1], poses[i,2])
        T1_w = np.array([[math.cos(p1[2]), -math.sin(p1[2]), p1[0]], [math.sin(p1[2]), math.cos(p1[2]), p1[1]], [0, 0, 1]])
        T2_w = np.array([[math.cos(p2[2]), -math.sin(p2[2]), p2[0]], [math.sin(p2[2]), math.cos(p2[2]), p2[1]], [0, 0, 1]])
        T2_1 = np.dot(np.linalg.inv(T1_w), T2_w)
        del_x = str(T2_1[0][2])
        del_y = str(T2_1[1][2])
        del_theta = str(math.atan2(T2_1[1, 0], T2_1[0, 0]))

        line = "EDGE_SE2 "+str(i-1)+" "+str(i)+" "+del_x+" "+del_y+" "+del_theta+" "+info_mat+"\n"
        g2o.write(line)
        # g2o.write("\n")
    return g2o

if __name__=='__main__':
    gtX, gtY, gtTheta = readG2o(argv[1])
    gtKitti = convert(gtX, gtY, gtTheta)
    np.savetxt(argv[2], gtKitti, delimiter=' ')

# fullBatchOptX, fullBatchOptY, fullBatchOptTheta = readG2o("full_opt.g2o")
# incBatchOptX, incBatchOptY, incBatchOptTheta = readG2o("inc_opt.g2o")
# noiseX, noiseY, noiseTheta = readG2o("noise.g2o")
# posesFile = "/home/ys/rrc/official_rord_slam/rord_slam/pose_graph/data5/poses.txt"
# poses = getPoses(posesFile)
# gt = writeOdom(poses)
# gt.close()
# os.system("cp gt.g2o /home/ys/rrc/official_rord_slam/rord_slam/pose_graph/data5/gt.g2o")

# gtX, gtY, gtTheta = readG2o("/home/ys/rrc/official_rord_slam/rord_slam/pose_graph/data5/opt.g2o")
# gtKitti = convert(gtX, gtY, gtTheta)

# noiseKitti = convert(noiseX, noiseY, noiseTheta)
# fullKitti = convert(fullBatchOptX, fullBatchOptY, fullBatchOptTheta)
# incKitti = convert(incBatchOptX, incBatchOptY, incBatchOptTheta)
# print("debug", gtKitti.shape, noiseKitti.shape, fullKitti.shape, incKitti.shape)

# np.savetxt("/home/ys/rrc/official_rord_slam/rord_slam/pose_graph/data5/opt.kitti", gtKitti, delimiter=' ')

# np.savetxt("noise.kitti", noiseKitti, delimiter=' ')
# np.savetxt("full.kitti", fullKitti, delimiter=' ')
# np.savetxt("inc.kitti", incKitti, delimiter=' ')

# gtX, gtY, gtTheta = readG2o("/home/ys/rrc/official_rord_slam/rord_slam/pose_graph/data5/opt_rord.g2o")
# gtKitti = convert(gtX, gtY, gtTheta)
# np.savetxt("/home/ys/rrc/official_rord_slam/rord_slam/pose_graph/data5/opt_rord.kitti", gtKitti, delimiter=' ')

#os.system("evo_traj kitti full.kitti --ref gt.kitti --plot --plot_mode xy --save_as_kitti")
# os.system("evo_traj kitti inc.kitti --ref gt.kitti --plot --plot_mode xy --save_as_kitti")
#os.system("evo_traj kitti noise.kitti --ref gt.kitti --plot --plot_mode xy --save_as_kitti")
# os.system("evo_traj kitti gt.kitti --plot --plot_mode xy")

#os.system("evo_rpe kitti gt.kitti full.kitti -v --plot --plot_mode xy")
# os.system("evo_rpe kitti gt.kitti inc.kitti -v --plot --plot_mode xy")
#os.system("evo_rpe kitti gt.kitti noise.kitti -v --plot --plot_mode xy")

# os.system("evo_ape kitti gt.kitti full.kitti -v")
# os.system("evo_ape kitti gt.kitti inc.kitti -v --plot --plot_mode xy")
# os.system("evo_ape kitti gt.kitti noise.kitti -v")
