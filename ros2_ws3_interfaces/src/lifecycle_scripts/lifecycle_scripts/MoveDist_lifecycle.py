#!/usr/bin/env python3
import time
import threading
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from robot_interfaces.action import MoveDist

#######################################################################################################################
class MoveDist_lifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__("MoveDist_lifecycleNode")
        self.get_logger().info("MoveDist_lifecycleNode: initializing")
        self.goal_lock_ = threading.Lock()
        self.activated = False
        
        self.MoveDist_server_               = None
        self.goal_handle_: ServerGoalHandle = None
        self.current_position_              = 50  
    def on_configure(self, statePre: LifecycleState):
        self.get_logger().info("MoveDist_lifecycleNode: configuring")
        self.MoveDist_server_ = ActionServer(\
            self,\
            MoveDist,\
            "MoveDist",\
            goal_callback            = self.goal_callback,\
            handle_accepted_callback = self.handle_accepted_callback,\
            execute_callback         = self.execute_callback,\
            cancel_callback          = self.cancel_callback,\
            callback_group           = ReentrantCallbackGroup())
        return TransitionCallbackReturn.SUCCESS
    def on_cleanup(self, statePre: LifecycleState):
        self.get_logger().info("MoveDist_lifecycleNode: cleaning up")
        self.cleanup_()
        return TransitionCallbackReturn.SUCCESS
    def on_activate(self, statePre: LifecycleState):
        self.get_logger().info("MoveDist_lifecycleNode: activating")
        self.activated = True
        return super().on_activate(statePre)
    def on_deactivate(self, statePre: LifecycleState):
        self.get_logger().info("MoveDist_lifecycleNode: deactivating")
        self.activated = False
        return super().on_deactivate(statePre)
    def on_shutdown(self, statePre: LifecycleState):
        self.get_logger().info("MoveDist_lifecycleNode: shutting down")
        self.cleanup_()
        return TransitionCallbackReturn.SUCCESS
    def on_error(self, statePre: LifecycleState):
        self.get_logger().info("MoveDist_lifecycleNode: deadly error occured, shutting down")
        self.cleanup_()
        return TransitionCallbackReturn.FAILURE
    def cleanup_(self):
        self.activated = False
        if self.MoveDist_server_ is not None: self.MoveDist_server_.destroy()
        self.MoveDist_server_               = None
        self.goal_handle_: ServerGoalHandle = None
    ###################################################################################################################
    def goal_callback(self, goal:MoveDist.Goal):
        self.get_logger().info("MoveDist_serverNode: goal received")
        if self.activated == False:
            self.get_logger().info("MoveDist_serverNode: "+self.get_name()+" is not activated")
            return GoalResponse.REJECT
        if (goal.target_position < 0) or (100 < goal.target_position):
            self.get_logger().info("MoveDist_serverNode: rejecting goal for target_position")
            return GoalResponse.REJECT
        if (goal.target_speed < 0) or (100 < goal.target_speed):
            self.get_logger().info("MoveDist_serverNode: rejecting goal for target_speed")
            return GoalResponse.REJECT
        
        with self.goal_lock_:
            if self.goal_handle_ is not None:
                if self.goal_handle_.is_active == True:
                    self.get_logger().info("MoveDist_serverNode: abort current goal, accepting new goal")
                    self.goal_handle_.abort()

        self.get_logger().info("MoveDist_serverNode: accepting goal") 
        return GoalResponse.ACCEPT
    def handle_accepted_callback(self, goal_handle:ServerGoalHandle):
        goal_handle.execute()
    def execute_callback(self, goal_handle:ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle
        goalVars     = goal_handle.request
        feedbackVars = MoveDist.Feedback() 
        resultVars   = MoveDist.Result()    

        self.get_logger().info("MoveDist_serverNode: executing the goal")
        self.get_logger().info("goal id: "+str("".join([str(hex(val)).replace("0x", "")\
                                               for val in goal_handle.goal_id.uuid])))
        self.get_logger().info("MoveDist_serverNode: current_position: "+str(self.current_position_))
        while self.current_position_ != goalVars.target_position:
            if goal_handle.is_active == False:
                self.get_logger().info("MoveDist_serverNode: aborting the goal")
                resultVars.reached_position = self.current_position_
                return resultVars
            if goal_handle.is_cancel_requested == True:
                self.get_logger().info("MoveDist_serverNode: canceling the goal")
                goal_handle.canceled()
                resultVars.reached_position = self.current_position_
                return resultVars
            if abs(goalVars.target_position - self.current_position_) > goalVars.target_speed:
                self.current_position_ += int(np.sign(goalVars.target_position - self.current_position_)*\
                                              goalVars.target_speed)
            else:
                self.current_position_ = goalVars.target_position
            self.get_logger().info("MoveDist_serverNode: current_position: "+str(self.current_position_))
            feedbackVars.current_position = self.current_position_
            goal_handle.publish_feedback(feedbackVars)
            time.sleep(1)
        resultVars.reached_position = self.current_position_
        goal_handle.succeed()
        self.get_logger().info("MoveDist_serverNode: goal succeeded")
        return resultVars
    def cancel_callback(self, goal_handle:ServerGoalHandle):
        self.get_logger().info("MoveDist_serverNode: receiving cancel request")
        return CancelResponse.ACCEPT
#######################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = MoveDist_lifecycleNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

#######################################################################################################################
if __name__ == "__main__": main()





