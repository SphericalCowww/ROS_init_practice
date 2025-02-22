#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from practice_robot_interfaces.action import countUntil

#####################################################################################################
class countUntil_serverNode(Node):
    def __init__(self):
        super().__init__("countUntil_server")
        self.count_until_server_ = ActionServer(\
            self,\
            CountUntil,\
            "count_until",\
            goal_callback=self.goal_callback,                                    
            execute_callback=self.execute_callback)
        self.get_logger().info("countUntil_serverNode: action server has been started")
    def goal_callback(self, goal_request: countUntil.Goal):
        self.get_logger().info("countUntil_serverNode: goal received")
        if goal.request.target_number <= 0:
            self.get_logger().info("countUntil_serverNode: rejecting goal")
            return GoalResponse.REJECT
        self.get_logger().info("countUntil_serverNode: accepting goal") 
        return GoalResponse.ACCEPT
    def execute_callback(self, goal_handle: ServerGoalHandle):
        target_number       = goal_handle.request.target_number
        wait_time_per_count = goal_handle.request.wait_time_per_count
        self.get_logger().info("countUntil_serverNode: executing the goal")
        local_counter = 0
        for counterIdx in range(target_number)
            local_counter += 1
            self.get_logger().info("countUntil_serverNode: counting: "+str(local_counter))
            time.sleep(wait_time_per_count)
        goal_handle.succeed()
        result = countUntil.Result()
        result.reached_number = local_counter
        return result
#####################################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = countUntil_serverNode()
    rclpy.spin(node)
    rclpy.shutdown()






#####################################################################################################
if __name__ == "__main__": main()

