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
from template_observer.luenberger import Luenberger
import numpy as np
import rclpy
import rclpy.node
import std_msgs.msg
import tmr4243_interfaces.msg
from template_observer.raft import Raft


class Observer(rclpy.node.Node):
    TASK_LUENBERGER = 'luenberger'
    TASK_LIST = [TASK_LUENBERGER]

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
        raft = Raft()

        optimal_observer_gains = np.array(
            [[0.01, 0.01, 0.03800937]], dtype=np.float64)

        initial_state_estimate = np.zeros((9, 1), dtype=np.float64)
        assert np.shape(initial_state_estimate) == (
            9, 1), np.shape(initial_state_estimate)

        bias_time_constants = np.array([[1.0, 1.0, 0.1]], dtype=np.float64).T
        self.task = Luenberger(
            damping=raft.damping,
            mass=raft.mass,
            observer_gains=optimal_observer_gains,
            time_step=self.delta_time,
            bias_time_constants=bias_time_constants,
            initial_state_estimate=initial_state_estimate,
        )

        self.observer_runner = self.create_timer(
            self.delta_time, self.observer_loop)

    def tau_callback(self, msg: std_msgs.msg.Float32MultiArray) -> None:
        assert len(msg.data) == 3, "someone provided fucked input"
        self.last_tau = np.array([msg.data], dtype=float).T

    def eta_callback(self, msg: std_msgs.msg.Float32MultiArray) -> None:
        assert len(msg.data) == 3, "someone provided fucked input"
        self.last_eta = np.array([msg.data], dtype=float).T

    def observer_loop(self) -> None:
        if self.last_eta is None or self.last_tau is None:
            self.get_logger().debug("observer has not received enough input yet. Cannot estimate")
            return

        assert self.last_eta is not None
        assert self.last_tau is not None

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
