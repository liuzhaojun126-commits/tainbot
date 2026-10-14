#!/usr/bin/env python3

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped, Vector3Stamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf
import tf2_ros
# RoboStack 环境无 PyKDL/sip, 移除 tf2_geometry_msgs; 下方 twist 旋转改用 tf.transformations 显式计算

robot_name = rospy.get_param('robot_name', default="tianracer")
class OdometryNode:
    # Set publishers
    pub_odom = rospy.Publisher('odom', Odometry, queue_size=1)

    def __init__(self):
        # init internals
        self.last_received_pose = Pose()
        self.last_received_pose.orientation.w = 1
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None

        # Set the update rate
        rospy.Timer(rospy.Duration(.05), self.timer_callback) # 20hz

        self.tf_pub = tf2_ros.TransformBroadcaster()

        # Set subscribers
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
        # Find the index of the racecar
        try:
            arrayIndex = msg.name.index(f'{robot_name}::{robot_name}/base_footprint')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract our current position information
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_received_twist = msg.twist[arrayIndex]
            
            # get a Vector3Stamped from the twist_in_gazebo
            twist_linear = Vector3Stamped()
            twist_linear.vector = self.last_received_twist.linear
            
            # get a TransformStamped from the last_received_pose
            twist_transform = TransformStamped()
            
            # last received pose is base_footprint in gazebo. twist is in gazebo frame.
            # we want to transform the twist into the base_footprint frame. requires a inverse
            q=tf.transformations.quaternion_inverse([self.last_received_pose.orientation.x, self.last_received_pose.orientation.y, self.last_received_pose.orientation.z, self.last_received_pose.orientation.w])
            
            twist_transform.transform.rotation.x = q[0]
            twist_transform.transform.rotation.y = q[1]
            twist_transform.transform.rotation.z = q[2]
            twist_transform.transform.rotation.w = q[3]
                
            # transform the Vector3Stamped into the child frame, base_link
            # (纯旋转, 平移为0): 用逆四元数矩阵显式旋转, 等价 do_transform_vector3
            _v=np.array([twist_linear.vector.x,twist_linear.vector.y,twist_linear.vector.z])
            _vr=tf.transformations.quaternion_matrix(q)[:3,:3].dot(_v)
            twist_linear.vector.x,twist_linear.vector.y,twist_linear.vector.z=_vr
            
            ######### the result is not proper ###########
            #print(twist_linear)

            self.last_received_twist.linear = twist_linear.vector
        
        self.last_recieved_stamp = rospy.Time.now()
          
            
    def timer_callback(self, event):
        if self.last_recieved_stamp is None:
            return

        odom = Odometry()
        odom.header.stamp = self.last_recieved_stamp
        odom.header.frame_id = f'{robot_name}/odom'
        odom.child_frame_id = f'{robot_name}/base_footprint'
        odom.pose.pose = self.last_received_pose
        odom.twist.twist = self.last_received_twist
        odom.pose.covariance =[1e-3, 0, 0, 0, 0, 0,
						0, 1e-3, 0, 0, 0, 0,
						0, 0, 1e6, 0, 0, 0,
						0, 0, 0, 1e6, 0, 0,
						0, 0, 0, 0, 1e6, 0,
						0, 0, 0, 0, 0, 1e3]

        odom.twist.covariance = [1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9]


        self.pub_odom.publish(odom)

        tf = TransformStamped(
            header=Header(
                frame_id=odom.header.frame_id,
                stamp=odom.header.stamp
            ),
            child_frame_id=odom.child_frame_id,
            transform=Transform(
                translation=odom.pose.pose.position,
                rotation=odom.pose.pose.orientation
            )
        )
        self.tf_pub.sendTransform(tf)

# Start the node
if __name__ == '__main__':
    rospy.init_node("gazebo_odometry_node")
    node = OdometryNode()
    rospy.spin()
