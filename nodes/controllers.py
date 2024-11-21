#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from random import random, randint
import PyKDL
from os import popen

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, '/controllers', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_trays)
        self.i = 0

    def publish_trays(self):
        msg = String()
        msg.data = "test"
        self.get_logger().info(popen("ros2 control list_controllers").read())

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()