#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

#######################################################################################################################
class Number_manager(Node):
    def __init__(self):
        super().__init__("Number_manager")
        self.declare_parameter("managed_node_name", rclpy.Parameter.Type.STRING)
        node_name = self.get_parameter("managed_node_name").value
        self.Number_manager_ = self.create_client(ChangeState, "/"+node_name+"/change_state") #check: ros2 service list
        self.get_logger().info("Number_manager: lifecycle manager has been started")
    def change_state(self, transition:Transition):
        self.get_logger().info("Number_manager: changing state")
        self.Number_manager_.wait_for_service()
        request = ChangeState.Request()
        request.transition = transition
        future = self.Number_manager_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
    def initialization_sequence(self):
        self.get_logger().info("Number_manager: configuring")
        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = "configure"
        self.change_state(transition)
        self.get_logger().info("Number_manager: configuring complete, now at inactive")

        time.sleep(3)

        self.get_logger().info("Number_manager: activating")
        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        transition.label = "activate"
        self.change_state(transition)
        self.get_logger().info("Number_manager: activating complete, now at active")
#######################################################################################################################
def main(args=None):
    rclpy.init(args=args)    
    node = Number_manager()
    node.initialization_sequence()
    rclpy.shutdown()
#######################################################################################################################
if __name__ == "__main__": main()






