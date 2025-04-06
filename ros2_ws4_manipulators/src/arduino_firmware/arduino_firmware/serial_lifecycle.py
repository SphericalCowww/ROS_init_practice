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
from example_interfaces.msg import String
import serial

#######################################################################################################################
class serial_lifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__("serial_lifecycleNode")
        self.get_logger().info(self.get_name()+": initializing")
        self.declare_parameter("port",      "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.aduino_port_        = self.get_parameter("port").value
        self.aduino_baud_rate_   = self.get_parameter("baud_rate").value
        self.receiver_frequency_ = 0.01    #s
        self.arduinoSerial_, self.transmitter_, self.receiver_, self.publisher_ = None, None, None, None

        self.activated = False
        self.servoNumber_ = 1
        self.servo_positions_ = [0 for servoIdx in range(self.servoNumber_)]
    def on_configure(self, statePre: LifecycleState) -> TransitionCallbackReturn: 
        self.arduinoSerial_ = serial.Serial(port=self.aduino_port_, baudrate=self.aduino_baud_rate_, timeout=0.1)
        self.transmitter_   = self.create_subscription(String, "serial_lifecycle_transmitter",\
                                                       self.transmitterCallback, 10)
        self.receiver_      = self.create_timer(self.receiver_frequency_, self.receiverCallback)
        self.publisher_     = self.create_publisher(String, "serial_lifecycle_receiver", 10)
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
        self.get_logger().info(self.get_name()+": test in __init__ first, otherwise no compilation error")
        self.cleanup_()
        return TransitionCallbackReturn.FAILURE
    def cleanup_(self):
        self.activated = False
        if self.arduinoSerial_ is not None:
            self.arduinoSerial_.close()
            self.transmitter_  .destroy()
            self.receiver_     .destroy()
            self.publisher_    .destroy()
        self.arduinoSerial_, self.transmitter_, self.receiver_, self.publisher_ = None, None, None, None
    ###################################################################################################################
    def transmitterCallback(self, msg):
        if self.activated == False: return
        self.get_logger().info("transmitterCallback(): receiving message on ros2 publication "+str(msg.data))
        self.get_logger().info("transmitterCallback(): sending message to arduino serial")
        self.arduinoSerial_.write(msg.data.encode("utf-8"))
    def receiverCallback(self):
        if self.activated == False: return
        if (rclpy.ok() == True) and (self.arduinoSerial_.is_open == True):
            msg = self.arduinoSerial_.readline()
            try:
                msg.decode("utf-8")
            except:
                self.get_logger().info("receiverCallback(): msg decoding failed")
                return
            msgObj = String()
            msgObj.data = str(msg)
            if len(msgObj.data) > 3: self.publisher_.publish(msgObj)
#######################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = serial_lifecycleNode()
    node.trigger_configure()
    node.trigger_activate()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("main(): keyboard interrupt detected. Shutting down the lifecycle.")
    finally:
        node.cleanup_()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


#######################################################################################################################
if __name__ == "__main__": main()





