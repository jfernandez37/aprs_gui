#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from random import random, randint
import PyKDL

from aprs_interfaces.msg import Trays, Tray, SlotInfo


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Trays, '/fanuc/table_trays_info', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_trays)
        self.i = 0

    def publish_trays(self):
        msg = Trays()

        first_tray = Tray()
        # first_tray.identifier = randint(13, 17)
        first_tray.identifier = Tray.LARGE_GEAR_TRAY
        first_tray.name = "mt_gear_tray_ 01"
        first_tray.tray_pose.pose.position.x = 0.25
        first_tray.tray_pose.pose.position.y = 0.15

        x, y, z, w = PyKDL.Rotation.RPY(0.0, 0.0, 1.571).GetQuaternion()

        first_tray.tray_pose.pose.orientation.x = x
        first_tray.tray_pose.pose.orientation.y = y
        first_tray.tray_pose.pose.orientation.z = z
        first_tray.tray_pose.pose.orientation.w = w

        for i in range(2):
            slot = SlotInfo()
            slot.occupied = True
            slot.size = SlotInfo.LARGE
            slot.name = f"slot_{i+1}"
            first_tray.slots.append(slot)

        second_tray = Tray()
        second_tray.identifier = Tray.MEDIUM_GEAR_TRAY
        second_tray.name = "mt_gear_tray_ 02"
        second_tray.tray_pose.pose.position.x = 0.25
        second_tray.tray_pose.pose.position.y = 0.3
        
        x, y, z, w = PyKDL.Rotation.RPY(0.0, 0.0, 0.75).GetQuaternion()

        second_tray.tray_pose.pose.orientation.x = x
        second_tray.tray_pose.pose.orientation.y = y
        second_tray.tray_pose.pose.orientation.z = z
        second_tray.tray_pose.pose.orientation.w = w

        for i in range(4):
            slot = SlotInfo()
            slot.occupied = True
            slot.size = SlotInfo.MEDIUM
            slot.name = f"slot_{i+1}"
            second_tray.slots.append(slot)

        msg.part_trays.append(first_tray)
        msg.part_trays.append(second_tray)
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