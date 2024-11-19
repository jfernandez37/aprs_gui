#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from random import random, randint

from aprs_interfaces.msg import Trays, Tray


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Trays, '/fanuc/table_vision/trays_info', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_trays)
        self.i = 0

    def publish_trays(self):
        msg = Trays()

        first_tray = Tray()
        # first_tray.identifier = randint(13, 17)
        first_tray.identifier = Tray.M2L1_KIT_TRAY
        first_tray.name = "mt_gear_tray_ 01"
        first_tray.transform_stamped.transform.translation.x = 0.25
        first_tray.transform_stamped.transform.translation.y = 0.25

        first_tray.transform_stamped.transform.rotation.x = random()
        first_tray.transform_stamped.transform.rotation.y = random()
        first_tray.transform_stamped.transform.rotation.z = random()
        first_tray.transform_stamped.transform.rotation.w = random()

        msg.part_trays.append(first_tray)
        self.publisher_.publish(msg)
        self.i += 1


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