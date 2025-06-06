import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    movement_node = Node(
        name="acm_r5_controller_node",
        package='act1_7',
        executable='acm_r5_controller',
        emulate_tty=True,
        output='screen'
    )

    patterns_node = Node(
        name="pattern_sender_node",
        package='act1_7',
        executable='pattern_sender',
        emulate_tty=True,
        output='screen'
    )

    return LaunchDescription([movement_node, patterns_node])
