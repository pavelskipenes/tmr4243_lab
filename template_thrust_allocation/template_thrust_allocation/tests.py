#!/usr/bin/env python3
from template_thrust_allocation.thrust_allocation_class import ThrustAllocator
from template_thrust_allocation.thruster_positions import ThrusterPositions
from typing import List, Tuple
from numpy.typing import NDArray
import numpy as np

"""
simple input output test for thrust allocator
No idea how to make it run though 🤷
"""


def extended() -> None:
    thruster_positions = ThrusterPositions()
    thrust_allocator = ThrustAllocator(thruster_positions)

    tests: List[Tuple[NDArray, NDArray]] = [
        # (expected, input)
        (np.matrix([[0], [0], [0], [0], [0]]), np.matrix([[0], [0], [0]])),
        # (np.matrix([[0], [0], [0], [0], [0]]), np.matrix([[0], [0], [0]])),
    ]

    perform_test(tests, thrust_allocator.allocate_extended)


def perform_test(tests: list, func, threshold=0.1):
    for test in tests:
        expected, input = test
        result = func(input)
        assert np.linalg.norm(expected - result) < threshold


def main():
    extended()
    # More thruster allocators in the future?


if __name__ == "__main__":
    main()
