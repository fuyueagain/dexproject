#!/usr/bin/env python3

from simple_arm_test_common import run_simple_arm_test
from robot_skills import PORT_LEFT_FOLLOWER


if __name__ == "__main__":
    run_simple_arm_test("左臂", PORT_LEFT_FOLLOWER)

