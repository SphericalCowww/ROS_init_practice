#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from practice_robot_interfaces.action import CountUntil

#####################################################################################################
class CountUntil_serverNode(Node):
    def __init__(self):
        super().__init__("CountUntil_client")
        self.count_until_client_ = ActionClient(\
            self,\
            CountUntil,\
            "count_until") #NOTE: must be the same name as server
        self.goal_handle_ = None
        self.get_logger().info("CountUntil_clientNode: action client has been started")
    def send_goal(self, target_number, wait_time_per_count):
        self.count_until_client_.wait_for_server()
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.wait_time_per_count = wait_time_per_count

        self.get_logger().info("CountUntil_clientNode: sending goal")
        # without async, only send after spin finishes; async is a python future object
        self.count_until_client_.send_goal_async(goal,feedback_callback=self.goal_feedback_callback)\
                                .add_done_callback(self.goal_response_callback)

        self.timer_ = self.create_timer(2.0, self.cancel_goal)
    def cancel_goal(self):
        self.get_logger().info("CountUntil_clientNode: sending cancel request")
        self.goal_handle_.cancel_goal_async()
        self.timer_.cancel()

    def goal_response_callback(self, futureObj):
        self.goal_handle_: ClientGoalHandle = futureObj.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("CountUntil_clientNode: goal accepted")
            self.goal_handle_.get_result_async()\
                             .add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("CountUntil_clientNode: goal rejected")
    def goal_result_callback(self, futureObj):
        status = futureObj.result().status
        result = futureObj.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("CountUntil_clientNode: goal succeeded")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("CountUntil_clientNode: goal aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error("CountUntil_clientNode: goal canceled")
        self.get_logger().info("CountUntil_clientNode: receiving result: "+\
                               str(result.reached_number))
    def goal_feedback_callback(self, feedback_msg):
        self.get_logger().info("CountUntil_clientNode: receiving feedback: "+\
                               str(feedback_msg.feedback.current_number)) 
#####################################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = CountUntil_serverNode()
    node.send_goal(10, 1.0)             # client sending goal request to server
    rclpy.spin(node)                    # basically while 1==1
    rclpy.shutdown()

#####################################################################################################
if __name__ == "__main__": main()






