#!/usr/bin/env python3
import numpy
import rclpy

#import asl tb3 related librarys
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl,TurtleBotState


class HeadingController(BaseHeadingController):
    def __init__(self) -> None:
        super().__init__()

        self.declare_parameter("kp", 4)

    def compute_control_with_goal(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        heading_error = wrap_angle(goal.theta - state.theta)
        self.get_logger().info(f'goal: {goal.theta}, state: {state.theta}')
        self.get_logger().info(f'Heading error is {heading_error}')
        w = self.kp * heading_error
        new_msg = TurtleBotControl()
        new_msg.omega = w
        return new_msg
    
    @property
    def kp(self) -> float:
        return self.get_parameter("kp").value


if __name__ == "__main__":
    #initialize the ROS2 system
    rclpy.init()

    #create instance of our controller
    headingController = HeadingController()

    #Spin the node
    rclpy.spin(headingController)

    #shut down the ROS2 system
    rclpy.shutdown()