#!/usr/bin/env python3

import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from tmr4243_utilities.utilities import anon
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    param_file = os.path.join(
        get_package_share_directory('template_joystick_control'),
        'config',
        'param.yaml'
    )

    arg_task = launch.actions.DeclareLaunchArgument(
        'task',
        default_value=launch.substitutions.TextSubstitution(text='simple'),
        description='Joystick control task type. Choose between simple, basin, body and emulated.',
        choices=['simple', 'basin', 'body', 'emulated']
    )

    node_joystick_control = launch_ros.actions.Node(
        package='template_joystick_control',
        executable='control_node.py',
        name=f'{anon()}joystick_control',
        parameters=[
            {'task': launch.substitutions.LaunchConfiguration('task')},
            param_file
        ],
        output='screen'
    )

    node_joy = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
        name=f"{anon()}joy_node",
    )

    return launch.LaunchDescription([
        arg_task,
        node_joystick_control,
        node_joy
    ])
