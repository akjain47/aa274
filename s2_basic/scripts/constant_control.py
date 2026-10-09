#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Bool, String
from geometry_msgs.msg import Twist

class basicNode(Node):
    def __init__(self) -> None:
        super().__init__("basicNode")
        self.hb_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.hb_counter = 0
        
        # self.motor_sub = self.create_subscription(Twist, "/cmd_vel", self.sub_callback, 10)
        self.motor_sub = self.create_subscription(Bool, "/kill", self.sub_callback, 10)

        
        self.hb_timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x=0.5
        msg.angular.z=0.1
        self.hb_pub.publish(msg)
        self.hb_counter += 1


    def sub_callback(self, msg: Bool) -> None:
        # This handles incoming messages from the topic
        if bool(msg.data):
            self.get_logger().fatal("kill the process")
            self.hb_timer.cancel()
            killmsg = Twist()
            killmsg.linear.x=0.0
            killmsg.angular.z=0.0
            self.hb_pub.publish(killmsg)
            self.hb_counter +=1
        # self.get_logger().info(f"Received message from subscription: {msg.linear.x}")


if __name__ == "__main__":
    rclpy.init()
    node = basicNode()
    rclpy.spin(node)
    rclpy.shutdown()
