#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

#######################################################################################################################
class MoveTurtle_manager(Node):
    def __init__(self):
        super().__init__("MoveTurtle_manager")
        self.declare_parameter("managed_node_names", rclpy.Parameter.Type.STRING_ARRAY)
        self.managed_node_names_ = self.get_parameter("managed_node_names").value
        self.get_logger().info("MoveTurtle_manager: starting lifecycle manager")
        self.MoveTurtle_managers_ = {}
        for nodeName in self.managed_node_names_:
            self.get_logger().info("MoveTurtle_manager: loading "+str(nodeName))
            self.MoveTurtle_managers_[nodeName] = self.create_client(ChangeState, "/"+nodeName+"/change_state")
    def change_state(self, transition:Transition):
        for nodeName in self.managed_node_names_:
            self.get_logger().info("MoveTurtle_manager: changing state")
            self.MoveTurtle_managers_[nodeName].wait_for_service()
            request = ChangeState.Request()
            request.transition = transition
            future = self.MoveTurtle_managers_[nodeName].call_async(request)
            rclpy.spin_until_future_complete(self, future)
    def initialization_sequence(self):
        self.get_logger().info("MoveTurtle_manager: configuring")
        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = "configure"
        self.change_state(transition)
        self.get_logger().info("MoveTurtle_manager: configuring complete, now at inactive")

        self.get_logger().info("MoveTurtle_manager: activating")
        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        transition.label = "activate"
        self.change_state(transition)
        self.get_logger().info("MoveTurtle_manager: activating complete, now at active")
#######################################################################################################################
def main(args=None):
    rclpy.init(args=args)    
    node = MoveTurtle_manager()
    node.initialization_sequence()
    rclpy.shutdown()
#######################################################################################################################
if __name__ == "__main__": main()






