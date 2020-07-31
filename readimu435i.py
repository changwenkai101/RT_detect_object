import pyrealsense2 as rs
import numpy as np
import cv2
import os

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3


cov_orientation = [-1, -1, -1, -1, -1, -1, -1, -1, -1]
cov_angular_velocity, cov_linear_acceleration = [0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0]


pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

profile = pipeline.start(config)

pub = rospy.Publisher('imu', Imu, queue_size=10)

def gyro_data(gyro):
    return np.asarray([gyro.x, gyro.y, gyro.z])

def accel_data(accel):
    return np.asarray([accel.x, accel.y, accel.z])

def imu_cb():

    while True:
        
        frames = pipeline.wait_for_frames()
        accel = accel_data(frames[0].as_motion_frame().get_motion_data())
        gyro = gyro_data(frames[1].as_motion_frame().get_motion_data())

        stamp = rospy.get_rostime()

        imuMsg = Imu()
        imuMsg.header.stamp, imuMsg.header.frame_id = stamp, "base_link"
        imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w= 0,0,0,0
        imuMsg.orientation_covariance = cov_orientation
        imuMsg.angular_velocity.x, imuMsg.angular_velocity.y, imuMsg.angular_velocity.z = gyro[0], gyro[1], gyro[2]
        imuMsg.angular_velocity_covariance = cov_angular_velocity
        imuMsg.linear_acceleration.x, imuMsg.linear_acceleration.y, imuMsg.linear_acceleration.z = accel[0], accel[1], accel[2]
        imuMsg.linear_acceleration_covariance = cov_linear_acceleration

        pub.publish(imuMsg)

if __name__ == '__main__':

    rospy.init_node('d435toimu', anonymous=True)
    #rate = rospy.Rate(200) # 10hz
    imu_cb()
    rospy.spin()
