#!/usr/bin/env python3
import rclpy
import time
import numpy as np
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from turtlesim.srv import Spawn, Kill

#######################################################################################################################
class MoveTurtle_manager(Node):
    def __init__(self):
        super().__init__("MoveTurtle_manager")
        self.callback_group_ = ReentrantCallbackGroup()
        self.spawn_client_ = self.create_client(Spawn, "spawn", callback_group=self.callback_group_)
        self.kill_client_  = self.create_client(Kill,  "kill",  callback_group=self.callback_group_)
        while not self.spawn_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("MoveTurtle_manager: waiting for spawn service...")
        self.spawn_turtle('this_turtle')
        time.sleep(5.0)
        self.kill_turtle('this_turtle')
    def spawn_turtle(self, turtle_name):
        request = Spawn.Request()
        request.x     = 4.0
        request.y     = 6.0
        request.theta = np.pi/2.0
        request.name = turtle_name

        future = self.spawn_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("MoveTurtle_manager: spawning turtle "+turtle_name)
        else:
            self.get_logger().error("MoveTurtle_manager: spawning fails")
    def kill_turtle(self, turtle_name):
        request = Kill.Request()    
        request.name = turtle_name
        
        future = self.kill_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info("MoveTurtle_manager: killing turtle "+turtle_name)
        else:
            self.get_logger().error("MoveTurtle_manager: killing fails")

#######################################################################################################################
def main(args=None):
    rclpy.init(args=args)    
    node = MoveTurtle_manager()
    rclpy.spin(node)
    rclpy.shutdown()
#######################################################################################################################
if __name__ == "__main__": main()






