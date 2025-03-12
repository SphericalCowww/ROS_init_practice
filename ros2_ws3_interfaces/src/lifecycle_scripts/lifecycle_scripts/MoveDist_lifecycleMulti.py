#!/usr/bin/env python3
import time
import threading
import numpy as np
from functools import partial
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
class MoveDist_lifecycleMultiNode(LifecycleNode):
    def __init__(self):
        super().__init__("MoveDist_lifecycleMultiNode")
        self.get_logger().info(self.get_name()+": initializing")

        self.goal_lock_ = threading.Lock()
        self.activated = False
        self.legNumber_, self.actionNames_ = 4, []
        self.MoveDist_servers_, self.goal_handles_, self.current_positions_ = [], [], []
        for legIdx in range(self.legNumber_):
            self.MoveDist_servers_ .append(None)
            self.goal_handles_     .append(None)
            self.current_positions_.append(50)
    def on_configure(self, statePre: LifecycleState) -> TransitionCallbackReturn:
        self.MoveDist_servers_ = []
        for legIdx in range(self.legNumber_):
            self.actionNames_.append(self.get_name()+"_ACleg"+str(legIdx))
            self.get_logger().info(self.get_name()+": configuring "+self.actionNames_[-1])
            goal_callback_idx            = partial(self.goal_callback,            legIdx=legIdx)
            handle_accepted_callback_idx = partial(self.handle_accepted_callback, legIdx=legIdx)
            execute_callback_idx         = partial(self.execute_callback,         legIdx=legIdx)
            cancel_callback_idx          = partial(self.cancel_callback,          legIdx=legIdx)
            self.MoveDist_servers_.append(ActionServer(\
                                              self,\
                                              MoveDist,\
                                              self.actionNames_[legIdx],\
                                              goal_callback            = goal_callback_idx,\
                                              handle_accepted_callback = handle_accepted_callback_idx,\
                                              execute_callback         = execute_callback_idx,\
                                              cancel_callback          = cancel_callback_idx,\
                                              callback_group           = ReentrantCallbackGroup()))
            self.get_logger().info("MoveDist_serverNode "+self.actionNames_[legIdx]+": current_position: "+\
                                   str(self.current_positions_[legIdx]))
 
        return TransitionCallbackReturn.SUCCESS
    def on_cleanup(self, statePre: LifecycleState):
        self.get_logger().info(self.get_name()+": cleaning up")
        self.cleanup_()
        return TransitionCallbackReturn.SUCCESS
    def on_activate(self, statePre: LifecycleState):
        self.get_logger().info(self.get_name()+": activating")
        self.activated = True
        return super().on_activate(statePre)
    def on_deactivate(self, statePre: LifecycleState):
        self.get_logger().info(self.get_name()+": deactivating")
        self.activated = False
        return super().on_deactivate(statePre)
    def on_shutdown(self, statePre: LifecycleState):
        self.get_logger().info(self.get_name()+": shutting down")
        self.cleanup_()
        return TransitionCallbackReturn.SUCCESS
    def on_error(self, statePre: LifecycleState):
        self.get_logger().info(self.get_name()+": deadly error occured, shutting down")
        self.cleanup_()
        return TransitionCallbackReturn.FAILURE
    def cleanup_(self):
        self.activated = False
        for legIdx in range(self.legNumber_):
            if self.MoveDist_servers_[legIdx] is not None: self.MoveDist_servers_[legIdx].destroy()
            self.MoveDist_servers_[legIdx] = None
            self.goal_handles_[legIdx]     = None
    ###################################################################################################################
    def goal_callback(self, goal:MoveDist.Goal, legIdx):
        self.get_logger().info("MoveDist_serverNode "+self.actionNames_[legIdx]+": goal received")
        if self.activated == False:
            self.get_logger().info("MoveDist_serverNode "+self.actionNames_[legIdx]+\
                                   ": "+self.get_name()+" is not activated")
            return GoalResponse.REJECT
        if (goal.target_position < 0) or (100 < goal.target_position):
            self.get_logger().info("MoveDist_serverNode "+self.actionNames_[legIdx]+\
                                   ": rejecting goal for target_position")
            return GoalResponse.REJECT
        if (goal.target_speed < 0) or (100 < goal.target_speed):
            self.get_logger().info("MoveDist_serverNode "+self.actionNames_[legIdx]+\
                                   ": rejecting goal for target_speed")
            return GoalResponse.REJECT       
 
        with self.goal_lock_:
            if self.goal_handles_[legIdx] is not None:
                if self.goal_handles_[legIdx].is_active == True:
                    self.get_logger().info("MoveDist_serverNode "+self.actionNames_[legIdx]+\
                                           ": abort current goal, accepting new goal")
                    self.goal_handles_[legIdx].abort()

        self.get_logger().info("MoveDist_serverNode "+self.actionNames_[legIdx]+": accepting goal") 
        return GoalResponse.ACCEPT
    def handle_accepted_callback(self, goal_handle:ServerGoalHandle, legIdx):
        goal_handle.execute()
    def execute_callback(self, goal_handle:ServerGoalHandle, legIdx):
        with self.goal_lock_:
            self.goal_handles_[legIdx] = goal_handle
        goalVars     = goal_handle.request
        feedbackVars = MoveDist.Feedback() 
        resultVars   = MoveDist.Result()    

        self.get_logger().info("MoveDist_serverNode "+self.actionNames_[legIdx]+": executing the goal")
        self.get_logger().info("goal id: "+str("".join([str(hex(val)).replace("0x", "")\
                                               for val in goal_handle.goal_id.uuid])))
        self.get_logger().info("MoveDist_serverNode "+self.actionNames_[legIdx]+": current_position: "+\
                               str(self.current_positions_[legIdx]))
        while self.current_positions_[legIdx] != goalVars.target_position:
            if goal_handle.is_active == False:
                self.get_logger().info("MoveDist_serverNode "+self.actionNames_[legIdx]+": aborting the goal")
                resultVars.reached_position = self.current_positions_[legIdx]
                return resultVars
            if goal_handle.is_cancel_requested == True:
                self.get_logger().info("MoveDist_serverNode "+self.actionNames_[legIdx]+": canceling the goal")
                goal_handle.canceled()
                resultVars.reached_position = self.current_positions_[legIdx]
                return resultVars
            if abs(goalVars.target_position - self.current_positions_[legIdx]) > goalVars.target_speed:
                self.current_positions_[legIdx] += int(np.sign(goalVars.target_position - \
                                                       self.current_positions_[legIdx])*goalVars.target_speed)
            else:
                self.current_positions_[legIdx] = goalVars.target_position
            self.get_logger().info("MoveDist_serverNode "+self.actionNames_[legIdx]+": current_position: "+\
                                   str(self.current_positions_[legIdx]))
            feedbackVars.current_position = self.current_positions_[legIdx]
            goal_handle.publish_feedback(feedbackVars)
            time.sleep(1)
        self.get_logger().info("MoveDist_serverNode "+self.actionNames_[legIdx]+": goal succeeded")
        goal_handle.succeed()
        resultVars.reached_position = self.current_positions_[legIdx]
        return resultVars
    def cancel_callback(self, goal_handle:ServerGoalHandle, legIdx):
        self.get_logger().info("MoveDist_serverNode "+self.actionNames_[legIdx]+": receiving cancel request")
        return CancelResponse.ACCEPT
#######################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = MoveDist_lifecycleMultiNode()
#    node.trigger_configure()
#    node.trigger_activate()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

#######################################################################################################################
if __name__ == "__main__": main()





