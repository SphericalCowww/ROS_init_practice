#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
#######################################################################################################################
class Node1(Node):
    def __init__(self):
        super().__init__("node1")
        self.callback_group1_ = MutuallyExclusiveCallbackGroup()
        self.callback_group2_ = MutuallyExclusiveCallbackGroup()   #ReentrantCallbackGroup()
        self.timer1_ = self.create_timer(1.0, self.callback_timer1, callback_group=self.callback_group1_)
        self.timer2_ = self.create_timer(1.0, self.callback_timer2, callback_group=self.callback_group2_)
        self.timer3_ = self.create_timer(1.0, self.callback_timer3, callback_group=self.callback_group2_)
    def callback_timer1(self):
        time.sleep(2.0)
        self.get_logger().info("cb 1-1")
    def callback_timer2(self):
        time.sleep(2.0)
        self.get_logger().info("cb 1-2")
    def callback_timer3(self):
        time.sleep(2.0)
        self.get_logger().info("cb 1-3")

class Node2(Node):
    def __init__(self):
        super().__init__("node2")
        self.callback_group1_ = MutuallyExclusiveCallbackGroup()
        self.callback_group2_ = ReentrantCallbackGroup()
        self.timer1_ = self.create_timer(1.0, self.callback_timer1, callback_group=self.callback_group2_)
        self.timer2_ = self.create_timer(1.0, self.callback_timer2, callback_group=self.callback_group2_)
    def callback_timer1(self):
        time.sleep(2.0)
        self.get_logger().info("cb 2-1")
    def callback_timer2(self):
        time.sleep(2.0)
        self.get_logger().info("cb 2-2")

#######################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    node1 = Node1()
    node2 = Node2()
    executor = MultiThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)
    executor.spin()
    rclpy.shutdown()

#######################################################################################################################
if __name__ == "__main__": main()
