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
# Email: emir.cem.gezer@ntnu.no, emircem.gezer@gmail.com, me@emircem.com
# Year: 2022
# Copyright (C) 2024 NTNU Marine Cybernetics Laboratory

from numpy.typing import NDArray
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from template_joystick_control.mapping import JoystickButtons
from template_observer.dead_reconing import DeadReconing
from template_observer.luenberger import Luenberger
from template_observer.raft import Raft
from typing import List
import numpy as np
import rclpy
import rclpy.node
import sensor_msgs.msg
import std_msgs.msg
import tmr4243_interfaces.msg


class Observer(rclpy.node.Node):
    TASK_LUENBERGER = 'luenberger'
    TASK_LIST = [TASK_LUENBERGER]
    joystick_buttons: JoystickButtons

    def __init__(self):
        super().__init__('cse_observer')

        self.subs: dict[str, Subscription] = {}
        self.pubs: dict[str, Publisher] = {}
        self.delta_time: float = 0.1

        self.last_eta: NDArray[np.float64] | None = None
        self.last_tau: NDArray[np.float64] | None = None

        self.subs["tau"] = self.create_subscription(
            std_msgs.msg.Float32MultiArray, '/tmr4243/state/tau', self.tau_callback, 10
        )
        self.subs["eta"] = self.create_subscription(
            std_msgs.msg.Float32MultiArray, '/tmr4243/state/eta', self.eta_callback, 10
        )
        self.pubs['observer'] = self.create_publisher(
            tmr4243_interfaces.msg.Observer, '/tmr4243/observer/eta', 1
        )
        self.subs["joy"] = self.create_subscription(
            sensor_msgs.msg.Joy, '/joy', self.joy_callback, 10)

        raft = Raft()
        self.joystick_buttons: JoystickButtons = JoystickButtons()

        initial_state_estimate = np.zeros((9, 1), dtype=np.float64)
        assert np.shape(initial_state_estimate) == (
            9, 1), np.shape(initial_state_estimate)

        L_1 = np.eye(3) * 2.5
        L_2 = np.eye(3) * 0.1
        L_3 = np.eye(3) * 0.8

        bias_time_constants = np.array([[1.0, 1.0, 0.1]], dtype=np.float64).T

        self.observers: List[Luenberger | DeadReconing] = [observer(
            raft=raft,
            time_step=self.delta_time,
            bias_time_constants=bias_time_constants,
            L_1=L_1,
            L_2=L_2,
            L_3=L_3,
            initial_state_estimate=initial_state_estimate,
        ) for observer in [Luenberger, DeadReconing]]

        # default to Luenberger
        self.task = self.observers[0]

        self.observer_runner = self.create_timer(
            self.delta_time, self.observer_loop)

    def tau_callback(self, msg: std_msgs.msg.Float32MultiArray) -> None:
        assert len(msg.data) == 3, "someone provided fucked input"
        self.last_tau = np.array([msg.data], dtype=float).T

    def eta_callback(self, msg: std_msgs.msg.Float32MultiArray) -> None:
        assert len(msg.data) == 3, "someone provided fucked input"
        self.last_eta = np.array([msg.data], dtype=float).T

    def joy_callback(self, joystick_message: sensor_msgs.msg.Joy) -> None:

        if joystick_message.buttons[self.joystick_buttons.SHOULDER_LEFT] and joystick_message.buttons[self.joystick_buttons.SHOULDER_RIGHT]:
            self.get_logger().info("stop holding both buttons!")
            return

        changed = False
        if joystick_message.buttons[self.joystick_buttons.SHOULDER_LEFT] and not isinstance(self.task, Luenberger):
            self.task = self.observers[0]
            changed = True

        if joystick_message.buttons[self.joystick_buttons.SHOULDER_RIGHT] and not isinstance(self.task, DeadReconing):

            self.task = self.observers[1]
            changed = True

        if changed:
            self.get_logger().info(
                f"new active observer {self.task.get_name()}")

    def observer_loop(self) -> None:
        if self.last_tau is None:
            self.get_logger().debug("observer has not received enough input yet. Cannot estimate")
            return

        assert self.last_tau is not None

        if self.last_eta is None:
            self.get_logger().debug("observer has not received enough input yet. Cannot estimate")
            return

        assert self.last_eta is not None

        observed_state = self.task.observe(
            measurement=self.last_eta,
            actuation_input=self.last_tau,
        )

        assert isinstance(observed_state, list)
        assert len(observed_state) == 9

        observer_message = tmr4243_interfaces.msg.Observer()
        observer_message.eta = observed_state[0:3]
        observer_message.nu = observed_state[4:6]
        observer_message.bias = observed_state[7:9]

        self.pubs['observer'].publish(observer_message)


def main():
    rclpy.init()

    node = Observer()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
