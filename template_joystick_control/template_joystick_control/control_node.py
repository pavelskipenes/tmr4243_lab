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

from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from template_joystick_control.mapping import JoystickAxes, JoystickButtons
from template_joystick_control.tasks import Task, get_actuation_topic_new_task
import numpy as np
import rcl_interfaces.msg
import rclpy
import rclpy.node
import sensor_msgs.msg
import std_msgs.msg


class JoystickControl(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__('tmr4243_joystick_control_node')

        self.task_default: Task = Task.SIMPLE
        self.task: Task = self.task_default
        self.pubs: dict[str, Publisher] = {}
        self.subs: dict[str, Subscription] = {}
        self.heading: float = 0
        self.joystick_axes: JoystickAxes = JoystickAxes()
        self.joystick_buttons: JoystickButtons = JoystickButtons()

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
        joystick_axes = JoystickAxes.get_names()
        joystick_buttons = JoystickButtons.get_names()

        for param in joystick_buttons + joystick_axes:
            self.declare_parameter(param, 0)

        for param in joystick_buttons:
            setattr(self.joystick_buttons, param,
                    self.get_parameter(param).value)

        for param in joystick_axes:
            setattr(self.joystick_axes, param,
                    self.get_parameter(param).value)
        self.declare_parameter(
            'task',
            self.task,
            rcl_interfaces.msg.ParameterDescriptor(
                description="Task",
                type=rcl_interfaces.msg.ParameterType.PARAMETER_STRING,
                read_only=False,
                additional_constraints=f"Allowed values: {' '.join(list(Task._member_names_))}"
            )
        )

        task_value = self.get_parameter('task').value
        match task_value:
            case "" | None:
                return
            case str():
                try:
                    self.task = Task(task_value)
                except ValueError as v:
                    self.get_logger().fatal(
                        f"Invalid input argument for task {v}")
                    raise Exception("unreachable")
            case _:
                raise Exception("unreachable")

    def joy_callback(self, msg: sensor_msgs.msg.Joy) -> None:

        actuation_topic_and_task = get_actuation_topic_new_task(
            msg=msg,
            buttons=self.joystick_buttons,
            axes=self.joystick_axes,
            heading=self.heading,
            task_default=self.task)

        actuation_topic, new_task = actuation_topic_and_task
        if new_task != self.task:
            self.get_logger().info(
                f"[Joystick] Switching task to {new_task}")
            self.task = new_task

        actuation, topic = actuation_topic
        cmd = std_msgs.msg.Float32MultiArray()
        cmd.data = actuation.flatten().tolist()
        self.pubs[topic].publish(cmd)

    def eta_callback(self, msg: std_msgs.msg.Float32MultiArray) -> None:
        input = msg.data[2]
        match input:
            case float() if input > np.pi:
                raise AssertionError("heading too large")

            case float() if input < - np.pi:
                raise AssertionError("heading is too low")

            case float() if len(msg.data) == 3:
                self.heading = input
            case _:
                raise Exception("someone sent shitty input")


def main(args=None) -> None:
    rclpy.init(args=args)

    rclpy.spin(JoystickControl())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
