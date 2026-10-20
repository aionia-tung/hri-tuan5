"""Bai 05 - Phan 3 + 4: node nhan dien YOLO va node dieu huong qua cac goal.

Chay sau khi tb4_week5_sim.launch.py da khoi dong xong (AMCL + Nav2 san sang).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_tb4_maps = get_package_share_directory('tb4_maps')

    arguments = [
        DeclareLaunchArgument('goals', default_value=os.path.join(pkg_tb4_maps, 'config', 'goals.yaml')),
        DeclareLaunchArgument('yolo_params', default_value=os.path.join(pkg_tb4_maps, 'config', 'yolo.yaml')),
        DeclareLaunchArgument('navigate', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('yolo', default_value='true', choices=['true', 'false']),
    ]

    yolo = Node(
        package='tb4_maps', executable='yolo_detector', name='yolo_detector', output='screen',
        parameters=[LaunchConfiguration('yolo_params'), {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('yolo')))

    # TurtleBot4Navigator tao node ten 'basic_navigator', name= doi thanh 'goal_navigator'
    navigator = Node(
        package='tb4_maps', executable='goal_navigator', name='goal_navigator', output='screen',
        parameters=[LaunchConfiguration('goals'), {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('navigate')))

    return LaunchDescription(arguments + [yolo, navigator])
