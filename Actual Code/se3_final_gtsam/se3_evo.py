from sys import argv, exit
import math
import numpy as np
import os
from scipy.spatial.transform import Rotation as R

def readG2o(fileName):
	f = open(fileName, 'r')
	A = f.readlines()
	f.close()

	X = []
	Y = []
	Z = []
	Qx = []
	Qy = []
	Qz = []
	Qw = []

	for line in A:
		if "VERTEX_SE3:QUAT" in line:			
			if(len(line.split(' ')) == 10):
				(ver, ind, x, y, z, qx, qy, qz, qw, newline) = line.split(' ')
			elif(len(line.split(' ')) == 9):
				(ver, ind, x, y, z, qx, qy, qz, qw) = line.split(' ')

			X.append(float(x))
			Y.append(float(y))
			Z.append(float(z))
			Qx.append(float(qx))
			Qy.append(float(qy))
			Qz.append(float(qz))

			if(len(line.split(' ')) == 10):
				Qw.append(float(qw))
			elif(len(line.split(' ')) == 9):
				Qw.append(float(qw.rstrip('\n')))

	return (X, Y, Z, Qx, Qy, Qz, Qw)




def convert(X, Y, Z, Qx, Qy, Qz, Qw):
    A = np.zeros((len(X), 12))
    for i in range(len(X)):
        T = np.identity(4)
        T[0, 3] = X[i]
        T[1, 3] = Y[i]
        T[2, 3] = Z[i]
        Rot = R.from_quat([Qx[i], Qy[i], Qz[i], Qw[i]]).as_dcm()
        T[0:3, 0:3] = Rot
        A[i] = T[0:3, :].reshape(1, 12)
    return A


if __name__=='__main__':
    X, Y, Z, Qx, Qy, Qz, Qw = readG2o(argv[1])
    gtKitti = convert(X, Y, Z, Qx, Qy, Qz, Qw)
    np.savetxt(argv[2], gtKitti, delimiter=' ')
