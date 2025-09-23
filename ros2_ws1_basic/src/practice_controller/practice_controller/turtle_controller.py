#!/usr/bin/env python3
import os, sys, pathlib, time, re, glob, math, copy
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from functools import partial
###############################################################################################
class controllerNode(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.cmd_vel_pub = self.create_publisher(\
            Twist, '/turtle1/cmd_vel', 10)
        self.read_pose_sub = self.create_subscription(\
            Pose, '/turtle1/pose', self.pose_callback, 10)
        self.get_logger().info('Starting turtle_controller...')
        self.turtle_pointup = True
    # ros2 service list
    # ros2 service type /turtle1/set_pen 
    # ros2 interface show turtlesim/srv/SetPen
    def call_set_pen_service(self, r, g, b, width, off):
        client = self.create_client(SetPen, '/turtle1/set_pen') 
        while not client.wait_for_service(1.0):
            self.get_logger().info('Waiting for call_set_pen_service()...')
        request = SetPen.Request()
        request.r     = r
        request.g     = g
        request.b     = b
        request.width = width
        request.off   = off
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))    #partial in case needed
    def callback_set_pen(self, future):
        try:
            response = future.result()      #no need to use response in this case
        except Exception as errMsg: 
            self.get_logger().error('ERROR: callback_set_pen():\n'+str(errMsg))
    def pose_callback(self, msg:Pose):
        cmd_msg = Twist()
        box_width  = 4.0
        box_buffer = 1.0
        epsilon = 0.3
        cmd_msg.linear.x = 0.0
        if box_width <= (msg.x-5.0):
            if abs(msg.theta - np.pi) < epsilon: 
                cmd_msg.angular.z = 0.0
                cmd_msg.linear.x  = 5.0
            else:
                cmd_msg.angular.z = 6.0 
        elif (msg.x-5.0) < -box_width:
            if abs(msg.theta - 0) < epsilon:  
                cmd_msg.angular.z = 0.0
                cmd_msg.linear.x  = 5.0
            else:
                cmd_msg.angular.z = 6.0
        elif box_width <= (msg.y-5.0):
            if abs(msg.theta + np.pi/2.0) < epsilon:  
                cmd_msg.angular.z = 0.0
                cmd_msg.linear.x  = 5.0
            else:
                cmd_msg.angular.z = 6.0
        elif (msg.y-5.0) < -box_width:
            if abs(msg.theta - np.pi/2.0) < epsilon:   
                cmd_msg.angular.z = 0.0
                cmd_msg.linear.x  = 5.0
            else:
                cmd_msg.angular.z = 6.0
        else:
            cmd_msg.linear.x  = 10.0
            cmd_msg.angular.z = 3.0

        if (0 <= msg.theta) and (self.turtle_pointup == False):
            self.call_set_pen_service(255, 0, 0, 3, 0)
            self.turtle_pointup = True
        elif (msg.theta < 0) and (self.turtle_pointup == True):
            self.call_set_pen_service(0, 255, 0, 3, 0)
            self.turtle_pointup = False        
    
        self.cmd_vel_pub.publish(cmd_msg)
        self.get_logger().info('Running pose_callback()...')
        self.get_logger().info(str(msg))
        #print('(x, y) = ('+str(msg.x-5.0)+', '+str(msg.y-5.0)+')')
###############################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = controllerNode()
    rclpy.spin(node)
    rclpy.shutdown()




###############################################################################################
if __name__ == '__main__': main()








