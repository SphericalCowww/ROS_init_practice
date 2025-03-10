#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from robot_interfaces.action import MoveDist

#######################################################################################################################
class MoveDist_clientNode(Node):
    def __init__(self, actionName):
        rclpy.init()
        super().__init__("MoveDist_client")
        self.goal_        = None
        self.goal_handle_ = None
        self.MoveDist_client_ = ActionClient(self, MoveDist, actionName) 
        self.get_logger().info("MoveDist_clientNode: action client has been started")
    def send_goal(self, target_position, target_speed):
        self.MoveDist_client_.wait_for_server()
        self.goal_                 = MoveDist.Goal()
        self.goal_.target_position = target_position
        self.goal_.target_speed    = target_speed
        
        self.get_logger().info("MoveDist_clientNode: sending goal")
        self.MoveDist_client_.send_goal_async(self.goal_, feedback_callback=self.goal_feedback_callback)\
                             .add_done_callback(self.goal_response_callback)
    def goal_feedback_callback(self, feedbackMsg):
        self.get_logger().info("MoveDist_clientNode: receiving feedback: "+str(feedbackMsg.feedback.current_position))
    def goal_response_callback(self, futureObj):
        self.goal_handle_:ClientGoalHandle = futureObj.result()
        self.get_logger().info("goal id: "+str("".join([str(hex(val)).replace("0x", "")\
                                                        for val in self.goal_handle_.goal_id.uuid])))
        
        if (self.goal_.target_position == 0) & (self.goal_.target_speed == 0): self.cancel_goal()
        if self.goal_handle_.accepted:
            self.get_logger().info("MoveDist_clientNode: goal accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("MoveDist_clientNode: goal rejected")
    def goal_result_callback(self, futureObj):
        status = futureObj.result().status
        result = futureObj.result().result
        if   status == GoalStatus.STATUS_SUCCEEDED: self.get_logger().info( "MoveDist_clientNode: goal succeeded")
        elif status == GoalStatus.STATUS_ABORTED:   self.get_logger().error("MoveDist_clientNode: goal aborted")
        elif status == GoalStatus.STATUS_CANCELED:  self.get_logger().error("MoveDist_clientNode: goal canceled")
        self.get_logger().info("MoveDist_clientNode: receiving result: "+str(result.reached_position))
        rclpy.shutdown()
    def cancel_goal(self):
        self.get_logger().info("MoveDist_clientNode: sending cancel request")
        self.goal_handle_.cancel_goal_async()
        rclpy.shutdown()
#######################################################################################################################
def main(args=None):
    if len(sys.argv) != 4:
        print("MoveDist client requires arguments target_position, target_speed")
        print("In case both are 0 will cancel the current goal")
        sys.exit(0)
    node = MoveDist_clientNode(sys.argv[1])
    node.send_goal(int(sys.argv[2]), int(sys.argv[3]))
    rclpy.spin(node)
#######################################################################################################################
if __name__ == "__main__": main()






