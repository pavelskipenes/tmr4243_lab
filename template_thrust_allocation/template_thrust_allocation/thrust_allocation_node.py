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
import std_msgs.msg
import numpy as np

from template_thrust_allocation.thrust_allocation_class import ThrustAllocator, ThrusterPositions


class ThrustAllocation(rclpy.node.Node):
    def __init__(self):
        super().__init__("tmr4243_thrust_allocation_node")      # Super calls for parent class properties

        self.thruster_positions = ThrusterPositions()
        self.thrust_allocator = ThrustAllocator(self.thruster_positions)

        self.pubs = {}
        self.subs = {}

        self.subs["tau_cmd"] = self.create_subscription(
            std_msgs.msg.Float32MultiArray, '/tmr4243/command/tau', self.tau_cmd_callback, 1)

        self.pubs["u_cmd"] = self.create_publisher(
            std_msgs.msg.Float32MultiArray, '/tmr4243/command/u', 1)
        
    def tau_cmd_callback(self, msg):    # Handling of incoming command data, i.e. paste to numpy array
        tau = np.array([msg.data], dtype=float).flatten()
        u_cmd = std_msgs.msg.Float32MultiArray()
        u = self.thrust_allocator.allocate_extended(tau).tolist()
        #u_cmd.data = thruster_allocation(self.tau).flatten().tobytes()  # Convert numpy array to list
        u_cmd.data = u  # Convert numpy array to list
        self.pubs["u_cmd"].publish(u_cmd)

def main(args=None):
    # Initialize the node
    rclpy.init(args=args)   # Initialize communication between node and ROS2

    node = ThrustAllocation()

    rclpy.spin(node)    # Starting the node, Loops
    rclpy.shutdown()    # When node dies, shutoff communication to ROS2


if __name__ == '__main__':
    main()
