#!/usr/bin/env python3
import time
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from example_interfaces.msg import Int64

#######################################################################################################################
class MoveDist_serverNode(LifecycleNode):
    def __init__(self):
        super().__init__("MoveDist_lifecycle")
        self.get_logger().info("MoveDist_serverNode: initializing")
        self.current_position_ = 50
        self.MoveDist_server_  = None
    def on_configure(self, statePre: LifecycleState):
        self.get_logger().info("MoveDist_serverNode: configuring")
        self.MoveDist_server_ = self.create_service(Int64, 
                                                    "MoveDist_server", 
                                                    self.execute_callback,\
                                                    callback_group = ReentrantCallbackGroup())
        return TransitionCallbackReturn.SUCCESS
    def on_cleanup(self, statePre: LifecycleState):
        self.get_logger().info("MoveDist_serverNode: cleaning up")
        self.cleanup_()
        return TransitionCallbackReturn.SUCCESS
    def on_activate(self, statePre: LifecycleState):
        self.get_logger().info("MoveDist_serverNode: activating")
        return super().on_activate(statePre)
    def on_deactivate(self, statePre: LifecycleState):
        self.get_logger().info("MoveDist_serverNode: deactivating")
        return super().on_deactivate(statePre)
    def on_shutdown(self, statePre: LifecycleState):
        self.get_logger().info("MoveDist_serverNode: shutting down")
        self.destroy_service(self.MoveDist_server_)
        return TransitionCallbackReturn.SUCCESS
    def on_error(self, statePre: LifecycleState):
        self.get_logger().info("MoveDist_serverNode: deadly error occured, shutting down")
        self.destroy_service(self.MoveDist_server_)
        return TransitionCallbackReturn.FAILURE
    def execute_callback(self):
        self.current_position_ += 1
        time.sleep(1)
        return 
#######################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = MoveDist_serverNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

#######################################################################################################################
if __name__ == "__main__": main()

class ActionServerNode(Node):
...
class lifecycleNode(Node):
    def __init__(self):
        ...
    def on_configure():
        self.action_node_ = ActionServerNode()
        ...
    def on_activate():
        rclpy.spin(self.action_node_)
    ...
def main(args=None):
    rclpy.init(args=args)
    node = lifecycleNode()
    rclpy.spin(node)
    rclpy.shutdown()

