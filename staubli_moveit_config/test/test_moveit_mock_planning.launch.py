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

import unittest

import launch
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


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
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("staubli_bringup")
            + "/launch/launch_moveit.launch.py"
        ),
        launch_arguments={
            "robot_model": "tx2_60l",
            "gui": "false",
            "launch_servo": "false",
            "warehouse_sqlite_path": "/tmp/staubli_moveit_mock_warehouse.sqlite",
        }.items(),
    )
    return (
        launch.LaunchDescription(
            [bringup_launch, moveit_launch, launch_testing.actions.ReadyToTest()]
        ),
        {"bringup_launch": bringup_launch, "moveit_launch": moveit_launch},
    )


class TestMockMoveItPlanning(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_staubli_mock_moveit_planning")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_move_group_plans_joint_goal_without_execution(self):
        client = ActionClient(self.node, MoveGroup, "/move_action")
        self.assertTrue(
            client.wait_for_server(timeout_sec=120.0),
            "MoveGroup action server did not become available",
        )

        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.allowed_planning_time = 8.0
        goal.request.num_planning_attempts = 2
        goal.request.start_state.is_diff = True

        constraints = Constraints()
        target_positions = (0.11, 0.21, 0.31, 0.41, 0.51, 0.61)
        for index, position in enumerate(target_positions, start=1):
            joint = JointConstraint()
            joint.joint_name = f"joint_{index}"
            joint.position = position
            joint.tolerance_above = 0.03
            joint.tolerance_below = 0.03
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        goal.request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = True
        goal.planning_options.replan = False

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=30.0)
        self.assertTrue(send_future.done(), "MoveGroup goal submission timed out")
        goal_handle = send_future.result()
        self.assertIsNotNone(goal_handle)
        self.assertTrue(goal_handle.accepted, "MoveGroup rejected the planning goal")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=30.0)
        self.assertTrue(result_future.done(), "MoveGroup planning result timed out")
        result = result_future.result().result
        self.assertEqual(result.error_code.val, MoveItErrorCodes.SUCCESS)
        self.assertTrue(
            result.planned_trajectory.joint_trajectory.points,
            "MoveIt returned an empty planned trajectory",
        )
        self.assertEqual(
            set(result.planned_trajectory.joint_trajectory.joint_names),
            {f"joint_{index}" for index in range(1, 7)},
        )
