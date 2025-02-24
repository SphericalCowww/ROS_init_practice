#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from practice_robot_interfaces.action import CountUntil

#####################################################################################################
class CountUntil_serverNode(Node):
    def __init__(self):
        super().__init__("CountUntil_server")
        self.count_until_server_ = ActionServer(\
            self,\
            CountUntil,\
            "count_until",\
            goal_callback=self.goal_callback,                                    
            execute_callback=self.execute_callback)
        self.get_logger().info("CountUntil_serverNode: action server has been started")
    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info("CountUntil_serverNode: goal received")
        if goal_request.target_number <= 0:
            self.get_logger().info("CountUntil_serverNode: rejecting goal")
            return GoalResponse.REJECT
        self.get_logger().info("CountUntil_serverNode: accepting goal") 
        return GoalResponse.ACCEPT
    def execute_callback(self, goal_handle: ServerGoalHandle):
        target_number       = goal_handle.request.target_number
        wait_time_per_count = goal_handle.request.wait_time_per_count
        self.get_logger().info("CountUntil_serverNode: executing the goal")
        feedback = CountUntil.Feedback()
        local_counter = 0
        for counterIdx in range(target_number):
            local_counter += 1
            feedback.current_number = local_counter
            goal_handle.publish_feedback(feedback)
            self.get_logger().info("CountUntil_serverNode: counting: "+str(local_counter))
            time.sleep(wait_time_per_count)
        goal_handle.succeed()
        result = CountUntil.Result()
        result.reached_number = local_counter
        return result
#####################################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = CountUntil_serverNode()
    rclpy.spin(node)
    rclpy.shutdown()






#####################################################################################################
if __name__ == "__main__": main()

