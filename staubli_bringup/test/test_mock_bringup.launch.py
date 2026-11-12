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

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def generate_test_description():
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("staubli_bringup")
            + "/launch/launch_robot_control.launch.py"
        ),
        launch_arguments={
            "robot_model": "tx2_60l",
            "use_mock_hardware": "true",
            "robot_ip": "127.0.0.1",
            "gui": "false",
            "start_controller": "joint_trajectory_controller",
        }.items(),
    )
    return (
        launch.LaunchDescription(
            [bringup_launch, launch_testing.actions.ReadyToTest()]
        ),
        {"bringup_launch": bringup_launch},
    )


class TestMockBringup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_staubli_mock_bringup")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_controller_lifecycle_and_joint_states(self):
        client = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        deadline = time.monotonic() + 90.0
        response = None
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.5):
                future = client.call_async(ListControllers.Request())
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
                if future.done() and future.result() is not None:
                    response = future.result()
                    states = {item.name: item.state for item in response.controller}
                    if (
                        states.get("joint_state_broadcaster") == "active"
                        and states.get("joint_trajectory_controller") == "active"
                    ):
                        break
            time.sleep(0.2)

        self.assertIsNotNone(
            response, "controller_manager list_controllers did not respond"
        )
        states = {item.name: item.state for item in response.controller}
        self.assertEqual(states.get("joint_state_broadcaster"), "active", states)
        self.assertEqual(states.get("joint_trajectory_controller"), "active", states)

        received = []
        subscription = self.node.create_subscription(
            JointState, "/joint_states", lambda msg: received.append(msg), 10
        )
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and not received:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.node.destroy_subscription(subscription)

        self.assertTrue(received, "mock bringup did not publish /joint_states")
        expected = {f"joint_{index}" for index in range(1, 7)}
        self.assertTrue(expected.issubset(set(received[-1].name)), received[-1].name)
