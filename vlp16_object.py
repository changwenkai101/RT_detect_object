#!/usr/bin/env python
import rospy
import numpy as np
import math
import random
import matplotlib
import matplotlib.pyplot as plt
import sys
import std_msgs.msg
import time
import serial
from mpl_toolkits.mplot3d import Axes3D
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

limt_pub = rospy.Publisher("/limt_cloud",PointCloud2,queue_size=1)
obj_pub = rospy.Publisher("/obj_cloud",PointCloud2,queue_size=1)
road_pub = rospy.Publisher("/road_cloud",PointCloud2,queue_size=1)
control_pub = rospy.Publisher("/obj_block",String,queue_size=1)

serial_to_con = serial.Serial('/dev/ttyS1',115200)

left = 6.0
right = -2.5
near = 0.0
far = 30.0
ali = 0.5

Y = np.arange(left,right-ali,-ali)
Y_obj = [0]*len(Y)
X = np.arange(near,far+ali,ali)

def callback(data):
    time_start = time.time()

    cloud_points = list(point_cloud2.read_points(data, skip_nans=True, field_names=("x","y","z")))
    newCloud = []
    P = 0
    VessPlane = []
    VessObj = []
    Center = []
    Plane = []

    ryth = -15/180.0*3.1415926
    ry=[[math.cos(ryth),0,math.sin(ryth)],[0,1,0],[-math.sin(ryth),0,math.cos(ryth)]]
    rxth = -1/180.0*3.1415926
    rx=[[1,0,0],[0,math.cos(rxth),math.sin(rxth)],[0,-math.sin(rxth),math.cos(rxth)]]
    for point in cloud_points:
	if (point[1]<right)|(point[1]>left):continue
	if (point[0]<near)|(point[0]>far):continue
	transpoint1 = ry[0][0]*point[0]+ry[1][0]*point[1]+ry[2][0]*point[2]
	transpoint2 = ry[0][1]*point[0]+ry[1][1]*point[1]+ry[2][1]*point[2]
	transpoint3 = ry[0][2]*point[0]+ry[1][2]*point[1]+ry[2][2]*point[2]
	if transpoint3>-1:continue
	transpoint = [transpoint1,transpoint2,transpoint3]

	transpoint1 = rx[0][0]*transpoint[0]+rx[1][0]*transpoint[1]+rx[2][0]*transpoint[2]
	transpoint2 = rx[0][1]*transpoint[0]+rx[1][1]*transpoint[1]+rx[2][1]*transpoint[2]
	transpoint3 = rx[0][2]*transpoint[0]+rx[1][2]*transpoint[1]+rx[2][2]*transpoint[2]

	transpoint = [transpoint1,transpoint2,transpoint3]
	newCloud.append(transpoint)
    #np.savetxt("data.csv",newCloud,delimiter=",")

    '''header1 = std_msgs.msg.Header()
    header1.stamp = rospy.Time.now()
    header1.frame_id = 'map'

    limt_pcl = point_cloud2.create_cloud_xyz32(header1, newCloud)
    limt_pub.publish(limt_pcl)'''

    N = len(newCloud)


    Vess_count = np.zeros([len(Y),len(X)])
    Vess_max  = np.ones([len(Y),len(X)])*(-100000.0)
    Vess_min  = np.zeros([len(Y),len(X)])

    #print(N)

    for i in range(N):

	x = newCloud[i][0]
	y = newCloud[i][1]
	z = newCloud[i][2]

	index_x=0
	index_y=0

	for ii in range(len(X)-1,-1,-1):
		if x>X[ii]:
			index_x=ii
			break
	for ii in range(len(Y)):
		if y>Y[ii]:
			index_y=ii
			break

	Vess_count[index_y][index_x] = Vess_count[index_y][index_x] + 1
	if Vess_max[index_y][index_x]<z:
		Vess_max[index_y][index_x]=z
	if Vess_min[index_y][index_x]>z:
		Vess_min[index_y][index_x]=z

    index_obj = abs(Vess_max-Vess_min)>0.3
    index_val = Vess_count>2
    index_obj = index_obj & index_val

    Y_obj = [-1.0]*len(Y)
    for iii in range(len(Y)):

	for jjj in range(len(X)):
		if index_obj[iii][jjj]==True:
			Y_obj[iii] = jjj*ali
			break

    s2='@'+str(left)+','+','.join(str(s) for s in Y_obj)+'$\n'
    s1='@'+str(left)+','+','.join(str(s) for s in Y_obj)+'$'
    #s='@'+str(-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0)+'$\n'	
    control_pub.publish(s1)
    #serial_to_con.write(s1)
    time_end = time.time()
    print('N:',N,'s:',s2,'t:',time_end-time_start)

'''
    #show points 
    obj_point = []
    road_point = []

    for i in range(0,N,1):

	x = newCloud[i][0]
	y = newCloud[i][1]
	z = newCloud[i][2]

	index_x=0
	index_y=0

	for ii in range(len(X)-1,-1,-1):
		if x>X[ii]:
			index_x=ii
			break
	for ii in range(len(Y)):
		if y>Y[ii]:
			index_y=ii
			break
	
	if index_obj[index_y,index_x]==True:
		obj_point.append(newCloud[i])
	else:
		road_point.append(newCloud[i])

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'

    obj_pcl = point_cloud2.create_cloud_xyz32(header, obj_point)
    obj_pub.publish(obj_pcl)
    road_pcl = point_cloud2.create_cloud_xyz32(header, road_point)
    road_pub.publish(road_pcl)

    #time_end2 = time.time()
    #print('Time1:',time_end2-time_end)

'''
def objectDetecter():

    rospy.init_node('object_detect', anonymous=True)
    rospy.Subscriber('velodyne_points', PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    objectDetecter()
