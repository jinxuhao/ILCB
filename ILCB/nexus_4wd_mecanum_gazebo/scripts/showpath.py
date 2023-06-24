#!/usr/bin/env python

import rospy
import nav_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import Pose, Twist, PoseStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu




class PathNode:
    # Set publishers
    pub_path = rospy.Publisher('/path', Path, queue_size=1)
    
    
    def __init__(self):

        self.path = Path()
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "odom"
        # init internals
        self.last_received_p = np.zeros(3)
        self.last_received_q = np.zeros(4)
        self.last_recieved_stamp = None
        print('init')
        # Set the update rate
        rospy.Timer(rospy.Duration(.05), self.timer_callback) # 20hz

        #self.path_pushback = 
        
        # Set subscribers
        rospy.Subscriber('/odom', nav_msgs.msg.Odometry, self.gps_callback)
        #rospy.Subscriber('uav_imu', Imu, self.imu_callback)
        
    def gps_callback(self, msg):
       

        self.last_received_p[0] = msg.pose.pose.position.x
        self.last_received_p[1] = msg.pose.pose.position.y
        self.last_received_p[2] = msg.pose.pose.position.z
        self.last_received_q[0] = msg.pose.pose.orientation.w
        self.last_received_q[1] = msg.pose.pose.orientation.x
        self.last_received_q[2] = msg.pose.pose.orientation.y
        self.last_received_q[3] = msg.pose.pose.orientation.z
        print('1')
        self.last_recieved_stamp = rospy.Time.now()
    

    def timer_callback(self, event):
        if self.last_recieved_stamp is None:
            return
        pose = PoseStamped()
        print('2')
        #while not rospy.is_shutdown():
            
        #cmd.child_frame_id = 'base_footprint'Odometry
        pose.header.frame_id = "odom"
        pose.header.stamp = self.last_recieved_stamp
        pose.pose.position.x = self.last_received_p[0]
        pose.pose.position.y = self.last_received_p[1]
        pose.pose.position.z = self.last_received_p[2]
        pose.pose.orientation.x = self.last_received_q[0]
        pose.pose.orientation.y = self.last_received_q[1]
        pose.pose.orientation.z = self.last_received_q[2]
        pose.pose.orientation.w = self.last_received_q[3]
        self.path.poses.append(pose)
        self.pub_path.publish(self.path)
        rospy.loginfo("Published {} waypoints.".format(len(self.path.poses)))


# Start the node
if __name__ == '__main__':
    rospy.init_node("showpath_node")
    print('3')
    node = PathNode()
    print('4')
    rospy.spin()


