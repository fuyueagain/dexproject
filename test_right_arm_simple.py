#!/usr/bin/env python3

from simple_arm_test_common import run_simple_arm_test
from robot_skills import PORT_RIGHT_FOLLOWER


if __name__ == "__main__":
    run_simple_arm_test("右臂", PORT_RIGHT_FOLLOWER)

