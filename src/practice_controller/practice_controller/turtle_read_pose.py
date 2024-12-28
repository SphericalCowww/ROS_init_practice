#!/usr/bin/env python3
import os, sys, pathlib, time, re, glob, math, copy
import numpy as np
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
###############################################################################################
class readPoseNode(Node):
    def __init__(self):
        super().__init__('turtle_read_pose')
        self.read_pose_sub = self.create_subscription(Pose, '/turtle1/pose',\
                                                      self.read_pose_callback, 10)
        self.get_logger().info('Starting turtle_read_pose...')
    def read_pose_callback(self, msg:Pose):
        self.get_logger().info('Running read_pose_callback...')
        self.get_logger().info(str(msg))
        self.get_logger().info('(x, y) = ('+str(msg.x)+', '+str(msg.y)+')')
###############################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = readPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()




###############################################################################################
if __name__ == '__main__': main()








