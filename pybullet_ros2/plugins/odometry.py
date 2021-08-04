#!/usr/bin/env python3

"""
Query robot base pose and speed from pybullet and publish to /odom topic
This component does not add any noise to it
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class simpleOdometry:
    def __init__(self, pybullet, node: Node, robot, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        self.node = node
        self.node.declare_parameter('odom_frame', 'odom')
        self.node.declare_parameter('robot_base_frame', 'base_link')
        # get robot from parent class
        self.robot = robot
        # register this node as a /odom publisher
        # self.pub_odometry = rospy.Publisher('odom', Odometry, queue_size=1)
        self.pub_odometry = node.create_publisher(Odometry, 'odom', 10)

        # save some overhead by setting some information only once
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = node.get_parameter('odom_frame').value  # , 'odom')
        self.odom_msg.child_frame_id = node.get_parameter('robot_base_frame').value  # , 'base_link')
        # self.br = tf.TransformBroadcaster()

        self.br = StaticTransformBroadcaster(node)

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        # set msg timestamp based on current time
        self.odom_msg.header.stamp = self.node.get_clock().now().to_msg()
        # query base pose from pybullet and store in odom msg
        position, orientation = self.pb.getBasePositionAndOrientation(self.robot)
        [self.odom_msg.pose.pose.position.x,
         self.odom_msg.pose.pose.position.y,
         self.odom_msg.pose.pose.position.z] = position
        [self.odom_msg.pose.pose.orientation.x,
         self.odom_msg.pose.pose.orientation.y,
         self.odom_msg.pose.pose.orientation.z,
         self.odom_msg.pose.pose.orientation.w] = orientation
        # query base velocity from pybullet and store it in msg
        linear, angular = self.pb.getBaseVelocity(self.robot)
 
        self.odom_msg.twist.twist.linear.x = linear[0]
        self.odom_msg.twist.twist.linear.y = linear[1]
        self.odom_msg.twist.twist.linear.z = linear[2]
       
        self.odom_msg.twist.twist.angular.x = angular[0]
        self.odom_msg.twist.twist.angular.y = angular[1]
        self.odom_msg.twist.twist.angular.z = angular[2]
        
        self.pub_odometry.publish(self.odom_msg)
        # tf broadcast (odom to base_link)
        self.br.sendTransform(position, orientation, self.node.get_clock().now(),
                              self.odom_msg.child_frame_id, self.odom_msg.header.frame_id)
