#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from random import random, randint

from aprs_interfaces.msg import Trays, Tray, SlotInfo


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Trays, '/fanuc/table_vision/trays_info', 10)
        self.publisher_2 = self.create_publisher(Trays, '/motoman/table_vision/trays_info', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_trays)
        self.i = 0

    def publish_trays(self):
        msg = Trays()

        first_tray = Tray()
        # first_tray.identifier = randint(13, 17)
        first_tray.identifier = Tray.LARGE_GEAR_TRAY
        first_tray.name = "mt_gear_tray_ 01"
        first_tray.transform_stamped.transform.translation.x = 0.25
        first_tray.transform_stamped.transform.translation.y = 0.15

        first_tray.transform_stamped.transform.rotation.x = 0.0
        first_tray.transform_stamped.transform.rotation.y = 0.0
        first_tray.transform_stamped.transform.rotation.z = 0.0
        first_tray.transform_stamped.transform.rotation.w = 1.0

        second_tray = Tray()
        second_tray.identifier = Tray.MEDIUM_GEAR_TRAY
        second_tray.name = "mt_gear_tray_ 02"
        second_tray.transform_stamped.transform.translation.x = 0.25
        second_tray.transform_stamped.transform.translation.y = 0.3
        second_tray.transform_stamped.transform.rotation.x = 0.0
        second_tray.transform_stamped.transform.rotation.y = 0.0
        second_tray.transform_stamped.transform.rotation.z = 0.0
        second_tray.transform_stamped.transform.rotation.w = 1.0

        # for i in range(4):
        #     slot = SlotInfo()
        #     slot.occupied = random() > 0.6
        #     slot.size = SlotInfo.LARGE if i < 2 else SlotInfo.SMALL
        #     slot.name = f"lg_{i%2+1}" if i < 2 else f"sg_{i%2+1}"
        #     first_tray.slots.append(slot)

        msg.part_trays.append(first_tray)
        msg.part_trays.append(second_tray)
        self.publisher_.publish(msg)
        self.publisher_2.publish(msg)
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