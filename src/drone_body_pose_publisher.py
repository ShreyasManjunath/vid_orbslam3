#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped
from sensor_msgs.msg import Imu
import tf2_ros
import numpy as np
from tf.transformations import *

class PosePublish:
    def __init__(self):
        self.pub = tf2_ros.TransformBroadcaster()
        self.pub_body = rospy.Publisher('/body_pose', PoseStamped, queue_size=1)
        self.pub_cam = rospy.Publisher('/cam_pose', PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber('/dji_osdk_ros/imu', Imu, self.temp, queue_size=1)
        print('initiated')
        
    
    def callback(self, msg):
        current_time_stamp = rospy.Time.now()
        t_body = TransformStamped()
        t_cam = TransformStamped()
        t_body.header.frame_id = "map"
        t_body.child_frame_id = "body_pose"

        # Orientation with respect to MAP frame (FLU coordinate system)
        # Imu provides orientation in MAP frame.
       # orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        orientation = [0, 0, 0, 1]
        Q_Rbm = [0.0,0.0,0.0,1]

        q_new = quaternion_multiply(orientation, Q_Rbm)

        t_body.header.stamp = current_time_stamp
        t_body.transform.translation.x = 0
        t_body.transform.translation.y = 0
        t_body.transform.translation.z = 0.6
        t_body.transform.rotation.x = q_new[0]
        t_body.transform.rotation.y = q_new[1]
        t_body.transform.rotation.z = q_new[2]
        t_body.transform.rotation.w = q_new[3]

        self.pub.sendTransform(t_body)

        t_cam.header.frame_id = "body_pose"
        t_cam.child_frame_id = "main_camera"
        Q_Rbc = [0.5, -0.5, 0.5, -0.5]
        q_new = quaternion_multiply(orientation, Q_Rbc) 

        print(q_new)

        t_cam.header.stamp = current_time_stamp
        t_cam.transform.translation.x = 0.101
        t_cam.transform.translation.y = 0.0426
        t_cam.transform.translation.z = -0.179
        t_cam.transform.rotation.x = q_new[0]
        t_cam.transform.rotation.y = q_new[1]
        t_cam.transform.rotation.z = q_new[2]
        t_cam.transform.rotation.w = q_new[3]

        self.pub.sendTransform(t_cam)
        

    def temp(self, msg):
        current_time_stamp = rospy.Time.now()
        t_body = PoseStamped()
        t_cam = PoseStamped()

        t_body.header.frame_id = "/map"

        t_body.header.stamp = current_time_stamp
        t_body.pose.position.x = 0
        t_body.pose.position.y = 0
        t_body.pose.position.z = 2

        t_body.pose.orientation = msg.orientation

        #self.publish_body_pose_tf(msg, current_time_stamp)
        self.pub_body.publish(t_body)

        orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        Q_Rbc = [0.5, -0.5, 0.5, -0.5]
        q_new = quaternion_multiply(orientation, Q_Rbc)

        t_cam.header.frame_id = "/map"

        t_cam.pose.position.x = 0.101
        t_cam.pose.position.y = 0.0426
        t_cam.pose.position.z = 1.88

        t_cam.header.stamp = current_time_stamp

        t_cam.pose.orientation.x = q_new[0]
        t_cam.pose.orientation.y = q_new[1]
        t_cam.pose.orientation.z = q_new[2]
        t_cam.pose.orientation.w = q_new[3]

        self.pub_cam.publish(t_cam)


    def publish_body_pose_tf(self, msg, current_time_stamp):
        t_body = TransformStamped()
        t_body.header.frame_id = "map"
        t_body.child_frame_id = "body_pose"

        # Orientation with respect to MAP frame (FLU coordinate system)
        # Imu provides orientation in MAP frame.
        orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        Q_Rbm = [0.0,0.0,0.0,1]

        q_new = quaternion_multiply(orientation, Q_Rbm)

        t_body.header.stamp = current_time_stamp
        t_body.transform.translation.x = 0
        t_body.transform.translation.y = 0
        t_body.transform.translation.z = 0.5
        t_body.transform.rotation.x = q_new[0]
        t_body.transform.rotation.y = q_new[1]
        t_body.transform.rotation.z = q_new[2]
        t_body.transform.rotation.w = q_new[3]
        self.pub.sendTransform(t_body)





if __name__ == '__main__':
    rospy.init_node('pose_publisher')
    pose_publish = PosePublish()
    try:
        while not rospy.is_shutdown():
            continue
    except KeyboardInterrupt:
        print('Bye!')