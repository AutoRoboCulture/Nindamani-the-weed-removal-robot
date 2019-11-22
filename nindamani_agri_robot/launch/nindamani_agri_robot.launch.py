import sys
import os
import launch

import pdb
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch import LaunchService
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    
    ld = LaunchDescription([
        Node(package='rpicam_ai_interface', node_executable='rpicam_ai_interface.py', output='screen'),
        Node(package='servo_control', node_executable='servo_ros2_interface.py', output='screen', parameters=["src/servo_control/config/servo_settings.yaml"]),
        Node(package='stepper_control', node_executable='stepper_ros2_interface.py', output='screen', parameters=["/home/arc/nindamani_ws/src/stepper_control/config/stepper_settings.yaml"]),
        Node(package='nindamani_agri_robot', node_executable='nindamani_agri_robot.py', output='screen'),
        ])
    return ld

    

