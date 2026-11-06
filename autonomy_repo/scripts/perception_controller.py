#!/usr/bin/env python3

import numpy as np
import rclpy

from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState
from std_msgs.msg import Int64, Bool, String


class PerceptionController (BaseHeadingController):
    def __init__(self):
        super().__init__("perception_controller")
        self.declare_parameter("kp",2.0)
        self.declare_parameter("active", True)
        self.image_detected = False
        self.motor_sub = self.create_subscription(Bool, "/detector_bool", self.detector_callback, 10)

    def detector_callback(self, msg: Bool) -> None:
            # This handles incoming messages from the topic
        self.image_detected = msg.data

    def compute_control_with_goal(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        error = wrap_angle(goal.theta - state.theta)
        control = TurtleBotControl()
        if self.image_detected: 
            control.omega = 0.0
        else:
            control.omega = 0.2
        return control
    
    @property
    def kp(self) -> float:
        return self.get_parameter("kp").value

    @property
    def active(self) -> bool:
        return self.get_parameter("active").value
        


if __name__ == "__main__":
    rclpy.init()
    node = PerceptionController(
    )
    rclpy.spin(node)
    rclpy.shutdown()
