#!/usr/bin/env python3
import rclpy
import time
import threading
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from practice_robot_interfaces.action import CountUntil

#######################################################################################################################
class CountUntil_serverNode(Node):
    def __init__(self):
        super().__init__("CountUntil_server")
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_  = threading.Lock()
        self.goal_queue_ = []
        self.count_until_server_ = ActionServer(\
            self,\
            CountUntil,\
            "count_until",\
            goal_callback            = self.goal_callback,\
            handle_accepted_callback = self.handle_accepted_callback,\
            execute_callback         = self.execute_callback,\
            cancel_callback          = self.cancel_callback,\
            callback_group           = ReentrantCallbackGroup())
        self.get_logger().info("CountUntil_serverNode: action server has been started")
    def goal_callback(self, goal:CountUntil.Goal):
        self.get_logger().info("CountUntil_serverNode: goal received")
        if goal.target_number <= 0:
            self.get_logger().info("CountUntil_serverNode: rejecting goal")
            return GoalResponse.REJECT        

        '''
        ### for test cancel
        with self.goal_lock_:
            if self.goal_handle_ is not None:
                if self.goal_handle_.is_active == True:
                    ### cancel new goal if multiple goal
                    #self.get_logger().info("CountUntil_serverNode: a goal is already active, rejecting new goal")
                    #return GoalResponse.REJECT
                    ### abort current goal if multiple goal
                    self.get_logger().info("CountUntil_serverNode: abort current goal, accepting new goal")
                    self.goal_handle_.abort()
        '''
        
        self.get_logger().info("CountUntil_serverNode: accepting goal") 
        return GoalResponse.ACCEPT
    def handle_accepted_callback(self, goal_handle:ServerGoalHandle):
        with self.goal_lock_:
            if self.goal_handle_ is not None:
                self.get_logger().info("CountUntil_serverNode: goal queue append")
                self.goal_queue_.append(goal_handle)
            else:
                self.get_logger().info("CountUntil_serverNode: goal queue first execute")
                goal_handle.execute()   
    def execute_callback(self, goal_handle:ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle
        goalVars     = goal_handle.request                          ### object of Goal() varaiables
        feedbackVars = CountUntil.Feedback()                        ### object of Feedback() varaiables
        resultVars   = CountUntil.Result()                          ### object of Result() varaiables

        self.get_logger().info("CountUntil_serverNode: executing the goal")
        self.get_logger().info("goal id: "+str("".join([str(hex(val)).replace("0x", "")\
                                               for val in goal_handle.goal_id.uuid])))
        local_counter = 0
        for counterIdx in range(goalVars.target_number):
            if goal_handle.is_active == False:
                self.get_logger().info("CountUntil_serverNode: aborting the goal")
                resultVars.reached_number = local_counter
                self.process_next_goal_in_queue()
                return resultVars
            if goal_handle.is_cancel_requested == True:
                self.get_logger().info("CountUntil_serverNode: canceling the goal")
                goal_handle.canceled()
                resultVars.reached_number = local_counter
                self.process_next_goal_in_queue()
                return resultVars
            local_counter += 1
            self.get_logger().info("CountUntil_serverNode: counting: "+str(local_counter))
            feedbackVars.current_number = local_counter
            goal_handle.publish_feedback(feedbackVars)
            time.sleep(goalVars.wait_time_per_count)
        self.get_logger().info("CountUntil_serverNode: goal succeeded")
        goal_handle.succeed()
        resultVars.reached_number = local_counter
        self.process_next_goal_in_queue()
        return resultVars
    def process_next_goal_in_queue(self):
        ### honestly sequentialize goals, more or less defeat the purpose of ReentrantCallbackGroup?
        with self.goal_lock_:
            if len(self.goal_queue_) > 0:
                self.get_logger().info("CountUntil_serverNode: goal queue pop")
                self.goal_queue_.pop(0).execute()
            else:
                self.goal_handle_: ServerGoalHandle = None
    def cancel_callback(self, goal_handle:ServerGoalHandle):
        self.get_logger().info("CountUntil_serverNode: receiving cancel request")
        return CancelResponse.ACCEPT
#######################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = CountUntil_serverNode()
    rclpy.spin(node, MultiThreadedExecutor())       # basically while 1==1
    rclpy.shutdown()






#######################################################################################################################
if __name__ == "__main__": main()

