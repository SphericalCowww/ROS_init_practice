#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

#######################################################################################################################
class Number_client(Node):
    def __init__(self):
        super().__init__("Number_client")
        self.declare_parameter("managed_node_name", rclpy.Parameter.Type.STRING)
        node_name = self.get_parameter("managed_node_name").value
        self.Number_client_ = self.create_client(ChangeState, "/"+node_name+"/change_state") # check: ros2 service list
        self.get_logger().info("Number_client: lifecycle client has been started")
    def change_state(self, transition:Transition):
        self.get_logger().info("Number_client: changing state")
        self.Number_client_.wait_for_service()
        request = ChangeState.Request()
        request.transition = transition
        future = self.Number_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
    def initialization_sequence(self):
        self.get_logger().info("Number_client: configuring")
        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = "configure"
        self.change_state(transition)
        self.get_logger().info("Number_client: configuring complete, now at inactive")

        time.sleep(3)

        self.get_logger().info("Number_client: activating")
        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        transition.label = "activate"
        self.change_state(transition)
        self.get_logger().info("Number_client: activating complete, now at active")
#######################################################################################################################
def main(args=None):
    rclpy.init(args=args)    
    node = Number_client()
    node.initialization_sequence()
    rclpy.shutdown()
#######################################################################################################################
if __name__ == "__main__": main()






