#!/usr/bin/env python3
import time
import threading, asyncio
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from turtlesim.srv import Spawn, Kill
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from robot_interfaces.action import MoveTurtle

#######################################################################################################################
class MoveTurtle_lifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__("MoveTurtle_lifecycleNode")
        self.get_logger().info("MoveTurtle_lifecycleNode: initializing")
        self.callback_group_ = ReentrantCallbackGroup()
        self.goal_lock_      = threading.Lock()
        self.activated = False

        self.spawn_client_ = None
        self.kill_client_  = None
        self.MoveTurtle_server_                        = None
        self.MoveTurtle_goal_handle_: ServerGoalHandle = None
        self.MoveTurtle_vel_pub_                       = None
        self.MoveTurtle_pose_sub_                      = None   

        self.turtleName_       = 'ninja_turtle'
        self.spawn_position_   = (4.0, 6.0, np.pi/2.0)
        self.current_position_ = None
    def on_configure(self, statePre: LifecycleState):
        self.get_logger().info("MoveTurtle_lifecycleNode: configuring")
        self.spawn_client_ = self.create_client(Spawn, "spawn", callback_group=self.callback_group_)
        self.kill_client_  = self.create_client(Kill,  "kill",  callback_group=self.callback_group_)        
        self.MoveTurtle_server_ = ActionServer(self,\
                                               MoveTurtle,\
                                               "MoveTurtle",\
                                               goal_callback            = self.goal_callback,\
                                               handle_accepted_callback = self.handle_accepted_callback,\
                                               execute_callback         = self.execute_callback,\
                                               cancel_callback          = self.cancel_callback,\
                                               callback_group           = ReentrantCallbackGroup())
        self.MoveTurtle_vel_pub_  = self.create_publisher(Twist, '/'+self.turtleName_+'/cmd_vel', 10,\
                                                          callback_group=self.callback_group_)
        self.MoveTurtle_pose_sub_ = self.create_subscription(Pose, '/turtle1/pose', self.turtle_pose_callback, 10,\
                                                             callback_group=self.callback_group_)
        self.get_logger().info("MoveTurtle_lifecycleNode: action server now online")
        return TransitionCallbackReturn.SUCCESS
    def on_cleanup(self, statePre: LifecycleState):
        self.get_logger().info("MoveTurtle_lifecycleNode: cleaning up")
        self.cleanup_()
        return TransitionCallbackReturn.SUCCESS
    def on_activate(self, statePre: LifecycleState):
        self.get_logger().info("MoveTurtle_lifecycleNode: activating")
        self.activated = True
        return super().on_activate(statePre)
    def on_deactivate(self, statePre: LifecycleState):
        self.get_logger().info("MoveTurtle_lifecycleNode: deactivating")
        self.activated = False
        return super().on_deactivate(statePre)
    def on_shutdown(self, statePre: LifecycleState):
        self.get_logger().info("MoveTurtle_lifecycleNode: shutting down")
        self.cleanup_()
        return TransitionCallbackReturn.SUCCESS
    def on_error(self, statePre: LifecycleState):
        self.get_logger().info("MoveTurtle_lifecycleNode: test in __init__ first, otherwise no compilation error")
        self.cleanup_()
        return TransitionCallbackReturn.FAILURE
    def cleanup_(self):
        self.activated = False
        self.spawn_client_ = None
        self.kill_client_  = None
        if self.MoveTurtle_server_ is not None: self.MoveTurtle_server_.destroy()
        self.MoveTurtle_server_                        = None
        self.MoveTurtle_goal_handle_: ServerGoalHandle = None
        self.MoveTurtle_vel_pub_                       = None
        self.MoveTurtle_pose_sub_                      = None
        self.current_position_ = None
    ###################################################################################################################
    def spawn_turtle(self, turtle_name):
        request = Spawn.Request()
        request.x, request.y, request.theta = self.spawn_position_
        request.name = turtle_name

        future = self.spawn_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("MoveTurtle_SpawnNode: spawning turtle "+turtle_name)
        else:
            self.get_logger().error("MoveTurtle_SpawnNode: spawning fails")
    def kill_turtle(self, turtle_name):
        request = Kill.Request()    
        request.name = turtle_name
        
        future = self.kill_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info("MoveTurtle_KillNode: killing turtle "+turtle_name)
        else:
            self.get_logger().error("MoveTurtle_KileNode: killing fails")
    def turtle_pose_callback(self, msg:Pose):
        self.current_position_ = (msg.x, msg.y, msg.theta)
        #self.get_logger().info("MoveTurtle_PoseNode: loading current position")
    ###################################################################################################################
    def goal_callback(self, goal:MoveTurtle.Goal):
        if self.activated == False:
            self.get_logger().info("MoveTurtle_serverNode: "+self.get_name()+" is not activated")
            return GoalResponse.REJECT   
     
        with self.goal_lock_:
            while not self.spawn_client_.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("MoveTurtle_manager: waiting for spawn service...")
            if self.MoveTurtle_goal_handle_ is not None:
                if self.MoveTurtle_goal_handle_.is_active == True:
                    self.get_logger().info("MoveTurtle_serverNode: abort current goal, accepting new goal")
                    self.MoveTurtle_goal_handle_.abort()

        self.get_logger().info("MoveTurtle_serverNode: accepting goal") 
        return GoalResponse.ACCEPT
    def handle_accepted_callback(self, goal_handle:ServerGoalHandle):
        goal_handle.execute()
    def execute_callback(self, goal_handle:ServerGoalHandle):
        with self.goal_lock_:
            self.MoveTurtle_goal_handle_ = goal_handle
        goalVars     = goal_handle.request
        feedbackVars = MoveTurtle.Feedback() 
        resultVars   = MoveTurtle.Result()    

        self.get_logger().info("MoveTurtle_serverNode: executing the goal")
        self.get_logger().info("goal id: "+str("".join([str(hex(val)).replace("0x", "")\
                                               for val in goal_handle.goal_id.uuid])))
        self.spawn_turtle(self.turtleName_)

        cmd_msg = Twist()
        cmd_msg.linear.x  = goalVars.linear_vel_x
        cmd_msg.angular.z = goalVars.angular_vel_z
        self.MoveTurtle_vel_pub_.publish(cmd_msg) 
        timeElapses, timeGap = 0, 1.0
        while timeElapses < goalVars.duration:
            if goal_handle.is_active == False:
                self.get_logger().info("MoveTurtle_serverNode: aborting the goal")
                resultVars.final_pos_x, resultVars.final_pos_y, resultVars.final_pos_theta = self.current_position_
                return resultVars
            if goal_handle.is_cancel_requested == True:
                self.get_logger().info("MoveTurtle_serverNode: canceling the goal")
                goal_handle.canceled()
                resultVars.final_pos_x, resultVars.final_pos_y, resultVars.final_pos_theta = self.current_position_
                return resultVars
            self.get_logger().info("MoveTurtle_serverNode: current_position: "+str(self.current_position_))
            feedbackVars.current_pos_x, feedbackVars.current_pos_y,feedbackVars.current_pos_theta=self.current_position_
            feedbackVars.current_linear_vel_x  = cmd_msg.linear.x
            feedbackVars.current_angular_vel_z = cmd_msg.angular.z
            feedbackVars.leftover_duration     = goalVars.duration - timeElapses
            goal_handle.publish_feedback(feedbackVars)
            time.sleep(timeGap)
            #await asyncio.sleep(timeGap) => no way at the moment
            timeElapses += timeGap
        resultVars.final_pos_x, resultVars.final_pos_y, resultVars.final_pos_theta = self.current_position_
        goal_handle.succeed()
        self.kill_turtle(self.turtleName_)
        self.get_logger().info("MoveTurtle_serverNode: goal succeeded")
        return resultVars
    def cancel_callback(self, goal_handle:ServerGoalHandle):
        self.get_logger().info("MoveTurtle_serverNode: receiving cancel request")
        return CancelResponse.ACCEPT
#######################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtle_lifecycleNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

#######################################################################################################################
if __name__ == "__main__": main()





