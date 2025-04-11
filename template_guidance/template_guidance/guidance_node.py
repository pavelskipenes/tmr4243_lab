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

from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from template_guidance.straight_line import straight_line
from template_guidance.path import path
from template_guidance.stationkeeping import stationkeeping
from template_guidance.close_enough import close_enough
import numpy as np
import rcl_interfaces.msg
import rclpy
import rclpy.node
import std_msgs.msg
import tmr4243_interfaces.msg


class Guidance(rclpy.node.Node):
    TASK_STATIONKEEPING = 'stationkeeping'
    TASK_STRAIGHT_LINE = 'straight_line'
    TASKS = [TASK_STRAIGHT_LINE, TASK_STATIONKEEPING]

    def __init__(self):
        super().__init__("tmr4243_guidance")

        self.pubs: dict[str, Publisher] = {}
        self.subs: dict[str, Subscription] = {}

        self.pubs["reference"] = self.create_publisher(
            tmr4243_interfaces.msg.Reference, '/tmr4243/control/reference', 1)

        self.subs["observed_eta"] = self.create_subscription(
            tmr4243_interfaces.msg.Observer, '/tmr4243/observer', self.observer_callback, 10)

        self.subs["eta"] = self.create_subscription(
            std_msgs.msg.Float32MultiArray, '/tmr4243/state/eta', self.eta_callback, 10)

        self.last_observation: tmr4243_interfaces.msg.Observer | None = None
        self.eta: std_msgs.msg.Float32MultiArray | None = None
        self.s = np.float64(0.0)

        self.task = Guidance.TASK_STRAIGHT_LINE
        # timer_period = 0.1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # TODO: make into an input
        self.waypoints = np.array(
            [
                [0, 0],

                [2, 0],
                [2, 2],
                [0, 2],

                [0, 0],

                # [-4, 0],
                # [-4, 4],
                # [0, 4],
                #
                # [0, 0],
                #
                # [0, -4],
            ])

        self.guidance_period = np.float64(0.1)  # seconds
        self.guidance_timer = self.create_timer(
            self.guidance_period, self.guidance_callback)

    def timer_callback(self):

        self.task = self.get_parameter(
            'task').get_parameter_value().string_value

        self.get_logger().info(
            f"Parameter task: {self.task}", throttle_duration_sec=1.0)

    def guidance_callback(self):

        msg = tmr4243_interfaces.msg.Reference()
        if Guidance.TASK_STATIONKEEPING in self.task:
            eta_d, eta_ds, eta_ds2 = stationkeeping()

            msg.eta_d = eta_d.flatten().tolist()
            msg.eta_ds = eta_ds.flatten().tolist()
            msg.eta_ds2 = eta_ds2.flatten().tolist()

        elif Guidance.TASK_STRAIGHT_LINE in self.task:
            if self.last_observation is None:
                return
            if self.eta is None:
                return

            # TODO: make into an input
            U_ref = np.float64(0.5)

            if len(self.waypoints) <= 1:
                self.get_logger().info("no more waypoints left")
                # TODO: change to station keeping
                # send the remaining point to station keeping
                return

            eta_d, eta_ds, eta_ds2, norm_p = straight_line(
                self.waypoints, self.s)

            if close_enough(self.s):
                self.get_logger().info(
                    f"reached the waypoint {self.waypoints[0]}, there are {len(self.waypoints) - 1} waypoints left")
                u_ref = np.float64(0.0)
                self.waypoints = self.waypoints[1:]
                self.s = np.float64(0.0)
            else:
                u_ref = U_ref

            eta_hat = np.array(self.last_observation.eta)
            eta_error = eta_hat - eta_d

            # assert self.s >= 0, self.s
            # assert self.s <= 1, self.s
            self.s, s_dot, v_s, v_ss = path(
                eta_ds,
                self.s,
                eta_error,
                u_ref,
                norm_p,
                self.guidance_period,
            )
            # assert self.s >= 0
            # assert self.s <= 1
            # np.clip(self.s, 0, 1)
            # debug_locals = f"""
            #     eta_ds = {eta_ds}
            #     self.s = {self.s}
            #     eta_error = {eta_error}
            #     u_ref = {u_ref}
            #     norm_p = {norm_p}
            #     self.guidance_period = {self.guidance_period}
            # """
            # assert self.s >= 0, debug_locals
            # assert self.s <= 1, debug_locals

            msg.eta_d = eta_d.flatten().tolist()
            msg.eta_ds = eta_ds.flatten().tolist()
            msg.eta_ds2 = eta_ds2.flatten().tolist()
            msg.w = s_dot
            msg.v_s = v_s
            msg.v_ss = np.float64(v_ss)
        self.pubs["reference"].publish(msg)

    def observer_callback(self, msg: tmr4243_interfaces.msg.Observer):
        if np.isnan(msg.eta).any():
            self.get_logger().warn(
                "disregarding observer message due to NaN elements in the observed position")
            return

        if np.isnan(msg.nu).any():
            self.get_logger().warn(
                "disregarding observer message due to NaN elements in the observed velocity")
            return

        if np.isnan(msg.bias).any():
            self.get_logger().warn(
                "disregarding observer message due to NaN elements in the observed bias")
            return

        if self.last_observation is None:
            self.get_logger().info("inserting waypoint")
            self.waypoints = np.insert(self.waypoints, 0, msg.eta[0:1])
        self.last_observation = msg

    def eta_callback(self, msg: std_msgs.msg.Float32MultiArray):
        if np.isnan(msg.data).any():
            self.get_logger().warn(
                "disregarding eta message due to NaN elements in the position")
            return

        if self.eta is None:
            self.waypoints = np.insert(self.waypoints, 0, msg.data[0:1])
            self.get_logger().info("inserting waypoint")

        self.eta = msg.data


def main(args=None):
    np.seterr(all="raise")
    rclpy.init(args=args)

    node = Guidance()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
