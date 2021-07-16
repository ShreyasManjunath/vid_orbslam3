#!/usr/bin/env python

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from decimal import Decimal

filename = "/home/manshr/converted_SLAM2ENU.txt"

def main():
	with open(filename) as f:
    		lines = f.read().splitlines()
	x = []
	y = []
	z = []
	for line in lines:
		content = line.split(',')
		x.append(float(content[0]))
		y.append(float(content[1]))
		z.append(float(content[2]))
		
	
	fig = plt.figure()
	ax = Axes3D(fig)
	ax.scatter(x, y, z, zdir='z', s=20, c='r', depthshade=True)
	ax.set_xlim3d(-150,150)
	ax.set_ylim3d(-150,150)
	ax.set_zlim3d(-40,40)
	plt.show()	
	
if __name__ == '__main__':
    main()
