#!/usr/bin/env python3
import os, sys, pathlib, time, re, glob, math, copy
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
###############################################################################################
class moveCycleNode(Node):
    def __init__(self):
        super().__init__('turtle_move_cycle')
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_timer(0.5, self.sent_vel_command)
        self.get_logger().info('Starting turtle_move_cycle...')
    def sent_vel_command(self):
        cmd_msg = Twist()
        cmd_msg.linear.x  = 5.0
        cmd_msg.angular.z = 1.0
        self.cmd_vel_pub.publish(cmd_msg)
        self.get_logger().info('Running sent_vel_command...')

###############################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = moveCycleNode()
    rclpy.spin(node)
    rclpy.shutdown()




###############################################################################################
if __name__ == '__main__': main()








