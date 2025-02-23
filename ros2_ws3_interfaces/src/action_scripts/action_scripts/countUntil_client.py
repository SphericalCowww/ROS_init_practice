#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from practice_robot_interfaces.action import countUntil

#####################################################################################################
class countUntil_serverNode(Node):
    def __init__(self):
        super().__init__("countUntil_client")
        self.count_until_client_ = ActionClient(\
            self,\
            CountUntil,\
            "count_until") #NOTE: must be the same name as server
        self.get_logger().info("countUntil_clientNode: action client has been started")
    def send_goal(self, target_number, wait_time_per_count):
        self.count_until_client_.wait_for_server()
        goal = countUntil.Goal()
        goal.target_number = target_number
        goal.wait_time_per_count = wait_time_per_count

        self.get_logger().info("countUntil_clientNode: sending goal")
        # without async, only send after spin finishes; async is a python future object
        self.count_until_client_.send_goal_async(goal)\
                                .add_done_callback(self.goal_response_callback)
    def goal_response_callback(self, futureObj):
        self.goal_handle_ : ClientGoalHandle = futureObj.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("countUntil_clientNode: goal accepted")
            self.goal_handle_.get_result_async()\
                             .add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("countUntil_clientNode: goal rejected")
    def goal_result_callback(self, futureObj):
        result = futureObj.result().result
        self.get_logger().info("countUntil_clientNode: receiving result: "+str(result.reached_number))
        
#####################################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = countUntil_serverNode()
    node.send_goal(10, 1.0)             # client sending goal request to server
    rclpy.spin(node)                    # basically while 1==1
    rclpy.shutdown()

#####################################################################################################
if __name__ == "__main__": main()






