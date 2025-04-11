#!/usr/bin/env python3

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from tmr4243_utilities.utilities import anon


def generate_launch_description():

    arg_task = launch.actions.DeclareLaunchArgument(
        'task',
        default_value=launch.substitutions.TextSubstitution(
            text='stationkeeping'),
        description='Guidance task type. Choose between stationkeeping, straight_line.',
        choices=['stationkeeping', 'straight_line']
    )

    arg_param_file = launch.actions.DeclareLaunchArgument(
        'param_file',
        default_value=launch.substitutions.PathJoinSubstitution(
            [launch_ros.substitutions.FindPackageShare(
                'template_guidance'), 'config', 'param.yaml']
        )
    )

    node_guidance = launch_ros.actions.Node(
        package='template_guidance',
        executable='guidance_node.py',
        name=f'{anon()}guidance',
        parameters=[
            {'task': launch.substitutions.LaunchConfiguration('task')},
            launch.substitutions.LaunchConfiguration('param_file')
        ],
        output="screen"
    )
    return launch.LaunchDescription([
        arg_task,
        arg_param_file,
        node_guidance
    ])
