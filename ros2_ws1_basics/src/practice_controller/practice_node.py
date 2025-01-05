#!/usr/bin/env python3
import os, sys, pathlib, time, re, glob, math, copy
import numpy as np
import rclpy
from rclpy.node import Node
###############################################################################################
class practiceNode(Node):
    def __init__(self):
        super().__init__('practice_node')
        self.create_timer(1.0, self.timer_callback)
    def timer_callback(self):
        self.get_logger().info('Hello there!!!!')

###############################################################################################
def main(args=None):
    rclpy.init(args=args)

    node = practiceNode()
    rclpy.spin(node)

    rclpy.shutdown()




###############################################################################################
if __name__ == '__main__': main()








