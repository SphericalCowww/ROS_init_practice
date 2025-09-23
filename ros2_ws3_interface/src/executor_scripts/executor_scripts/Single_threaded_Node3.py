#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
#######################################################################################################################
class Node3(Node):
    def __init__(self):
        super().__init__("node3")
        self.timer1_ = self.create_timer(3.0, self.callback_timer1)
        self.timer2_ = self.create_timer(3.0, self.callback_timer2)
        self.timer3_ = self.create_timer(3.0, self.callback_timer3)
    def callback_timer1(self):
        time.sleep(2.0)
        self.get_logger().info("cb 3-1")
    def callback_timer2(self):
        time.sleep(2.0)
        self.get_logger().info("cb 3-2")
    def callback_timer3(self):
        time.sleep(2.0)
        self.get_logger().info("cb 3-3")

#######################################################################################################################
if __name__ == "__main__": pass
