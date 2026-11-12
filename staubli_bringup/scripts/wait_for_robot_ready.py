#!/usr/bin/env python3
# Copyright 2025 ICube Laboratory
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# From ur_robot_driver/scripts/wait_for_robot_description.py
# Copyright (c) 2024 FZI Forschungszentrum Informatik

import sys

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import String

from rclpy.task import Future
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


class WaitForRoboReady(Node):

    def __init__(self):
        super().__init__("wait_for_robot_ready")

        # Declare parameters
        self.declare_parameter("description_topic", "robot_description")
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("timeout_sec", 30.0)
        self.declare_parameter(
            "expected_joint_names",
            [f"joint_{index}" for index in range(1, 7)],
        )

        # Get parameter values
        self.description_topic = (
            self.get_parameter("description_topic").get_parameter_value().string_value
        )
        self.joint_states_topic = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )
        self.timeout_sec = self.get_parameter("timeout_sec").get_parameter_value().double_value
        self.expected_joint_names = set(
            self.get_parameter("expected_joint_names").get_parameter_value().string_array_value
        )
        if self.timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be greater than zero")

        description_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        # joint_state_broadcaster publishes with volatile durability by default.
        # A transient-local subscriber would be QoS-incompatible and never receive it.
        joint_states_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.robot_description_subscription = self.create_subscription(
            String,
            self.description_topic,
            self.description_callback,
            qos_profile=description_qos,
        )
        self.joint_states_subscription = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_states_callback,
            qos_profile=joint_states_qos,
        )
        self.robot_description_future = Future()
        self.joint_states_future = Future()

        self.get_logger().info(
            f"Waiting for topics '{self.description_topic}' and '{self.joint_states_topic}'..."
        )
        self.robot_is_ready_future = Future()

        # Throttled logging
        self.create_timer(5.0, self.log_waiting_status)

    def log_waiting_status(self):
        """Log waiting status every 5 seconds to avoid spam."""
        if not self.robot_is_ready_future.done():
            desc_status = "✓" if self.robot_description_future.done() else "✗"
            joint_status = "✓" if self.joint_states_future.done() else "✗"
            self.get_logger().info(
                f"""Still waiting... robot_description: {
                    desc_status}, joint_states: {joint_status}"""
            )

    def joint_states_callback(self, msg):
        if not self.joint_states_future.done():
            received_joint_names = set(msg.name)
            missing_joint_names = self.expected_joint_names - received_joint_names
            if missing_joint_names:
                self.get_logger().warning(
                    "Ignoring incomplete joint_states message; missing joints: "
                    + ", ".join(sorted(missing_joint_names))
                )
                return

            self.get_logger().info(
                f"""Received joint state message on {
                    self.resolve_topic_name(self.joint_states_topic)}."""
            )
            self.joint_states_future.set_result(True)

            if self.robot_description_future.done():
                self.emit_robot_ready()

    def description_callback(self, msg):
        if not self.robot_description_future.done():
            if not msg.data.strip():
                self.get_logger().warning("Ignoring empty robot_description message")
                return

            self.get_logger().info(
                f"""Received robot description on {
                    self.resolve_topic_name(self.description_topic)}."""
            )
            self.robot_description_future.set_result(True)

            if self.joint_states_future.done():
                self.emit_robot_ready()

    def emit_robot_ready(self):
        self.get_logger().info("Robot is ready.")
        self.robot_is_ready_future.set_result(True)


def main(args=None):
    rclpy.init(args=args)

    sub = WaitForRoboReady()
    rclpy.spin_until_future_complete(
        sub,
        sub.robot_is_ready_future,
        timeout_sec=sub.timeout_sec,
    )

    exit_code = 0
    if not sub.robot_is_ready_future.done():
        sub.get_logger().error(
            f"Timed out after {sub.timeout_sec:.1f}s waiting for robot_description "
            f"and complete joint_states on '{sub.joint_states_topic}'."
        )
        exit_code = 1

    sub.destroy_node()
    rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
