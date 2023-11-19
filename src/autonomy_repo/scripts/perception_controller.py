#!/usr/bin/env python3
import numpy
import rclpy

#import asl tb3 related librarys
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl,TurtleBotState
from std_msgs.msg import Bool


class PerceptionController(BaseHeadingController):
    def __init__(self) -> None:
        super().__init__("perception_controller")

        self.declare_parameter("kp", 4)
        self.declare_parameter("active",True)
        self.omega_default = 0.2

        self.image_detected = False
        self.img_sub = self.create_subscription(Bool, "/detector_bool", self.detect_callback, 10)

    def compute_control_with_goal(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        if(not self.image_detected):
            w = self.omega_default
            self.get_logger().info('spin')
        else:
            w = 0.0
            self.get_logger().info('stop')
        new_msg = TurtleBotControl()
        new_msg.omega = w
        return new_msg
    
    def detect_callback(self,msg: Bool):
        if(msg.data):
            self.image_detected = True

    
    @property
    def kp(self) -> float:
        return self.get_parameter("kp").value
    
    @property
    def active(self) -> bool:
        return self.get_parameter("active").value


if __name__ == "__main__":
    #initialize the ROS2 system
    rclpy.init()

    #create instance of our controller
    perceptionController = PerceptionController()

    #Spin the node
    rclpy.spin(perceptionController)

    #shut down the ROS2 system
    rclpy.shutdown()