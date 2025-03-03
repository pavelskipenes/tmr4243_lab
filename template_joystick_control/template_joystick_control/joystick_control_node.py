#!/usr/bin/env python3
#
# This file is part of CyberShip Enterpries Suite.
#
# CyberShip Enterpries Suite software is free software: you can redistribute it
# and/or modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# CyberShip Enterpries Suite is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# CyberShip Enterpries Suite. If not, see <https://www.gnu.org/licenses/>.
#
# Maintainer: Emir Cem Gezer
# Email: emir.cem.gezer@ntnu.no, emircem.gezer@gmail.com, me@emircem
# Year: 2024
# Copyright (C) 2024 NTNU Marine Cybernetics Laboratory

import numpy as np

import rclpy
import rclpy.node
import std_msgs.msg
import sensor_msgs.msg
# import rcl_interfaces.msg

from template_joystick_control.joystick_mapping import JoystickAxes, JoystickButtons
from template_joystick_control.tasks import Task, get_actuation_and_channel
from template_joystick_control.joystick_emulated import RandomWalk
from template_joystick_control.channel import Channel


class JoystickControl(rclpy.node.Node):
    def __init__(self):
        super().__init__('tmr4243_joystick_control_node')

        self.task_default = Task.SIMPLE
        self.task_last = None
        self.task = None
        self.pubs = {}
        self.subs = {}
        self.random_walk = RandomWalk()
        self.last_eta_msg = std_msgs.msg.Float32MultiArray()
        self.joystick_axes = JoystickAxes()
        self.joystick_buttons = JoystickButtons()

        # Topics
        self.subs["joy"] = self.create_subscription(
            sensor_msgs.msg.Joy, '/joy', self.joy_callback, 10)

        self.subs["eta"] = self.create_subscription(
            std_msgs.msg.Float32MultiArray, '/tmr4243/state/eta', self.eta_callback, 10)

        self.pubs["tau_cmd"] = self.create_publisher(
            std_msgs.msg.Float32MultiArray, '/tmr4243/command/tau', 10)

        self.pubs["u_cmd"] = self.create_publisher(
            std_msgs.msg.Float32MultiArray, '/tmr4243/command/u', 10)

        # dynamic parameters
        # joystick_axes = JoystickAxes.get_names()
        # joystick_buttons = JoystickButtons.get_names()
        #
        # for param in joystick_buttons + joystick_axes:
        #     self.declare_parameter(param, 0)
        #
        # for param in joystick_buttons:
        #     setattr(self.joystick_buttons, param,
        #             self.get_parameter(param).value)
        #
        # for param in joystick_axes:
        #     setattr(self.joystick_axes, param,
        #             self.get_parameter(param).value)
        # self.declare_parameter(
        #     'task',
        #     self.task,
        #     rcl_interfaces.msg.ParameterDescriptor(
        #         description="Task",
        #         type=rcl_interfaces.msg.ParameterType.PARAMETER_STRING,
        #         read_only=False,
        #         additional_constraints=f"Allowed values: {' '.join(TASKS)}"
        #     )
        # )

        # self.timer = self.create_timer(0.1, self.timer_callback)

    # def timer_callback(self):
        # self.task = self.get_parameter(
        #     'task').get_parameter_value().string_value
        #
        # self.get_logger().debug(
        #     f"Parameter task: {self.task}", throttle_duration_sec=1.0)

    def joy_callback(self, msg: sensor_msgs.msg.Joy):

        # TODO: rename channel to topic
        actuation_and_channel_or_none = get_actuation_and_channel(
            msg=msg,
            buttons=self.joystick_buttons,
            axes=self.joystick_axes,
            random_walk=self.random_walk,
            last_eta_msg=self.last_eta_msg,
            task_default=self.task_last or self.task_default)
        if actuation_and_channel_or_none is None:
            # None is returned whenver the input is fucked
            return

        actuation, channel = actuation_and_channel_or_none
        assert isinstance(actuation, np.ndarray)
        assert isinstance(channel, Channel)

        cmd = std_msgs.msg.Float32MultiArray()
        # TODO: check if flatten is needed. Underlying functions "should" not be creating nested arrays anyway
        cmd.data = actuation.flatten().tolist()
        self.pubs[channel].publish(cmd)

    def eta_callback(self, msg):
        self.last_eta_msg = msg


def main(args=None):
    rclpy.init(args=args)

    rclpy.spin(JoystickControl())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
