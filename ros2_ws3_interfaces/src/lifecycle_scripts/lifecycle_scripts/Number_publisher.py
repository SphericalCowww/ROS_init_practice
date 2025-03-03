#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from example_interfaces.msg import Int64

#######################################################################################################################
class Number_publisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("Number_publisher")
        self.get_logger().info("Number_publisherNode: initializing")
        self.number_            = 1
        self.publish_frequency_ = 1.0
        self.number_publisher_ = None
        self.number_timer_     = None
    def on_configure(self, statePre: LifecycleState):
        self.get_logger().info("Number_publisherNode: configuring")
        self.number_publisher_ = self.create_lifecycle_publisher(Int64, "number", 10)
        self.number_timer_     = self.create_timer(1.0/self.publish_frequency_, self.publish_number)
        self.number_timer_.cancel()
        return TransitionCallbackReturn.SUCCESS
    def on_cleanup(self, statePre: LifecycleState):
        self.get_logger().info("Number_publisherNode: cleaning up")
        self.cleanup_()
        return TransitionCallbackReturn.SUCCESS
    def on_activate(self, statePre: LifecycleState):
        self.get_logger().info("Number_publisherNode: activating")
        self.number_timer_.reset()
        return super().on_activate(statePre)
    def on_deactivate(self, statePre: LifecycleState):
        self.get_logger().info("Number_publisherNode: deactivating")
        return super().on_deactivate(statePre)
    def on_shutdown(self, statePre: LifecycleState):
        self.get_logger().info("Number_publisherNode: shutting down")
        self.cleanup_()
        return TransitionCallbackReturn.SUCCESS
    def on_error(self, statePre: LifecycleState):
        ### sent here by TransitionCallbackReturn.ERROR/.FAILURE, or raise Exeception()
        self.get_logger().info("Number_publisherNode: deadly error occured, shutting down")
        self.cleanup_()
        return TransitionCallbackReturn.SUCCESS
    def cleanup_(self):
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_lifecycle_timer(    self.number_timer_)
    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)
        self.number_ += 1
#######################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = Number_publisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

#######################################################################################################################
if __name__ == "__main__": main()



