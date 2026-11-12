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

# Author: Thibault Poignonec <thibault.poignonec@gmail.com>

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogError,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Launch arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_model",
            description="Model of the robot (e.g, 'tx2_60l').",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true.",
            choices=["true", "false"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_servo",
            default_value="false",
            description="Whether to launch the Moveit servo node",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Launch Rviz if true.",
            choices=["true", "false"],
        )
    )

    robot_model = LaunchConfiguration("robot_model")
    use_sim_time = LaunchConfiguration("use_sim_time")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    launch_servo = LaunchConfiguration("launch_servo")
    gui = LaunchConfiguration("gui")

    # Include moveit launch file
    moveit_config_package = "staubli_moveit_config"
    moveit_launch_file = "staubli_moveit.launch.py"

    launch_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "launch", moveit_launch_file]
            )
        ),
        launch_arguments={
            "robot_model": robot_model,
            "use_sim_time": use_sim_time,
            "warehouse_sqlite_path": warehouse_sqlite_path,
            "launch_servo": launch_servo,
            "launch_rviz": gui,
        }.items(),
    )

    # Wait for robot description and joint states topics
    wait_for_robot_ready = Node(
        package="staubli_bringup",
        executable="wait_for_robot_ready.py",
        name="wait_for_robot_ready",
        parameters=[
            {
                "timeout_sec": 30.0,
                "expected_joint_names": [f"joint_{index}" for index in range(1, 7)],
            }
        ],
        output="both",
    )

    def on_readiness_check_exit(event, context):
        if event.returncode == 0:
            return [launch_moveit]

        return [
            LogError(
                msg=(
                    "Robot readiness check failed "
                    f"(exit code: {event.returncode}); MoveIt will not be started."
                )
            ),
            EmitEvent(
                event=Shutdown(
                    reason=f"Robot readiness check failed with exit code {event.returncode}"
                )
            ),
        ]

    # Launch MoveIt only after a successful readiness check.
    launch_moveit_when_ready = RegisterEventHandler(
        OnProcessExit(target_action=wait_for_robot_ready, on_exit=on_readiness_check_exit)
    )

    return LaunchDescription(declared_arguments + [wait_for_robot_ready, launch_moveit_when_ready])
