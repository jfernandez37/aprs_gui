import os
import xacro

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):

    gui_node = Node(
        package="aprs_gui",
        executable="gui_node.py",
        output="both"
    )

    vision_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('aprs_vision'),'launch', 'aprs_vision.launch.py')]
        )
    )
    
    fanuc_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('fanuc_description'),'launch', 'fanuc_bringup.launch.py')]
        )
    )

    fanuc_robot_commander = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('robot_commander'), 'launch', 'fanuc_robot_commander.launch.py')]
        )
    )
   
    motoman_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('motoman_description'),'launch', 'motoman_bringup.launch.py')]
        )
    )

    motoman_robot_commander = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('robot_commander'),'launch', 'motoman_robot_commander.launch.py')]
        )
    )

    nodes_to_start = [
        gui_node,
        vision_system,
        # fanuc_bringup,
        # fanuc_robot_commander,
        # motoman_bringup,
        # motoman_robot_commander
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])