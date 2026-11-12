# Copyright 2026 ICube Laboratory
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

import sys
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from wait_for_robot_ready import WaitForRoboReady


def test_readiness_accepts_transient_description_and_volatile_joint_states():
    if not rclpy.ok():
        rclpy.init()

    node = WaitForRoboReady()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    description_publisher = node.create_publisher(
        String,
        "robot_description",
        QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
    )
    joint_states_publisher = node.create_publisher(
        JointState,
        "joint_states",
        QoSProfile(depth=10),
    )

    try:
        deadline = time.monotonic() + 5.0
        while not node.robot_is_ready_future.done() and time.monotonic() < deadline:
            description = String()
            description.data = "<robot name='mock'/>"
            description_publisher.publish(description)

            incomplete_joint_states = JointState()
            incomplete_joint_states.name = ["joint_1", "joint_2"]
            joint_states_publisher.publish(incomplete_joint_states)

            complete_joint_states = JointState()
            complete_joint_states.name = [f"joint_{index}" for index in range(1, 7)]
            joint_states_publisher.publish(complete_joint_states)
            executor.spin_once(timeout_sec=0.1)

        assert node.robot_is_ready_future.done()
        assert node.robot_description_future.done()
        assert node.joint_states_future.done()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
