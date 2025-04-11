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
# Maintainer: Emir Cem Gezer, Petter Hangerhageen
# Email: emir.cem.gezer@ntnu.no, emircem.gezer@gmail.com, me@emircem.com, petthang@stud.ntnu.no
# Year: 2024
# Copyright (C) 2024 NTNU Marine Cybernetics Laboratory

import rclpy
import rclpy.node
import rcl_interfaces.msg
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
import tmr4243_interfaces.msg
import std_msgs.msg
import numpy as np

from template_controller.PID_controller import PID_controller, get_gains
from template_controller.PD_FF_controller import PD_FF_controller
from template_controller.backstepping_controller import backstepping_controller


class Controller(rclpy.node.Node):
    TASK_PD_FF_CONTROLLER = 'PD_FF_controller'
    TASK_PID_CONTROLLER = 'PID_controller'
    TASK_BACKSTEPPING_CONTROLLER = 'backstepping_controller'
    TASKS = [TASK_PD_FF_CONTROLLER, TASK_PID_CONTROLLER,
             TASK_BACKSTEPPING_CONTROLLER]

    def __init__(self):
        super().__init__("tmr4243_controller")

        self.pubs: dict[str, Publisher] = {}
        self.subs: dict[str, Subscription] = {}

        self.subs["reference"] = self.create_subscription(
            tmr4243_interfaces.msg.Reference, '/tmr4243/control/reference', self.received_reference, 10)

        self.subs['observer'] = self.create_subscription(
            tmr4243_interfaces.msg.Observer, '/tmr4243/observer', self.received_observer, 10)

        self.pubs["tau_cmd"] = self.create_publisher(
            std_msgs.msg.Float32MultiArray, '/tmr4243/command/tau', 1)

        self.subs["eta"] = self.create_subscription(
            std_msgs.msg.Float32MultiArray, '/tmr4243/state/eta', self.eta_callback, 10)

        p_gain, i_gain, d_gain = get_gains(0, 0)
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain

        self.k1_gain = [6.8, 8.95, 1.0]
        self.k2_gain = [9.5*2, 9.5*2, 7.0*2]

        self.last_reference = None

        self.last_observation = None
        self.eta = None

        # timer_period = 0.1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        controller_period = 0.01  # seconds
        self.controller_timer = self.create_timer(
            controller_period, self.controller_callback)

        self.task = Controller.TASK_PD_FF_CONTROLLER

    def controller_callback(self):

        if self.last_observation == None:
            self.get_logger().warn("last observation is None",
                                   throttle_duration_sec=1.0)
            return
        if self.last_reference == None:
            self.get_logger().warn("Last reference is None",
                                   throttle_duration_sec=1.0)
            return

        if Controller.TASK_PD_FF_CONTROLLER in self.task:
            tau = PD_FF_controller(
                self.last_observation,
                self.last_reference,
                self.p_gain,
                self.i_gain,
                self.d_gain,
                self.eta,
            )
        elif Controller.TASK_PID_CONTROLLER in self.task:
            tau = PID_controller(
                self.last_observation,
                self.last_reference,
                self.p_gain,
                self.i_gain,
                self.d_gain
            )
        elif Controller.TASK_BACKSTEPPING_CONTROLLER in self.task:
            tau, eta_error = backstepping_controller(
                self.last_observation,
                self.last_reference,
                self.k1_gain,
                self.k2_gain,
                self.eta,
            )
            self.get_logger().info(f"eta_error {eta_error}")
        else:
            tau = np.zeros((3, 1), dtype=np.float64)

        if len(tau) != 3:
            self.get_logger().warn(
                f"tau has length of {len(tau)} but it should be 3: tau := [Fx, Fy, Mz]", throttle_duration_sec=1.0)
            return

        tau = tau.flatten().tolist()
        tau = [min(3, el) for el in tau]
        tau = [max(0, el) for el in tau]

        tau_cmd = std_msgs.msg.Float32MultiArray()
        tau_cmd.data = tau
        self.pubs["tau_cmd"].publish(tau_cmd)

    def eta_callback(self, msg: std_msgs.msg.Float32MultiArray):
        if np.isnan(msg.data).any():
            self.get_logger().warn(
                "disregarding eta message due to NaN elements in the position")
            return

        self.eta = msg.data

    def received_reference(self, msg: tmr4243_interfaces.msg.Reference):
        for eta_d, eta_ds, etd_ds2 in zip(msg.eta_d, msg.eta_ds, msg.eta_ds2):
            if not np.isfinite(eta_d):
                self.get_logger().warn("disregarding fucked reference input")
                return
        self.last_reference = msg

    def received_observer(self, msg: tmr4243_interfaces.msg.Observer):
        for val in msg.eta:
            if not np.isfinite(val):
                self.get_logger().warn("disregarding fucked observer input")
                return
        self.last_observation = msg


def main(args=None):
    rclpy.init(args=args)

    rclpy.spin(Controller())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
