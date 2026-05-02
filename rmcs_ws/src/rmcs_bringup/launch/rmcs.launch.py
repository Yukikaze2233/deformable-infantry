from typing import List, Optional
import os

from launch import (
    LaunchContext,
    LaunchDescription,
    LaunchDescriptionEntity,
)
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class MyLaunchDescriptionEntity(LaunchDescriptionEntity):
    def visit(
        self, context: "LaunchContext"
    ) -> Optional[List["LaunchDescriptionEntity"]]:
        entities = []

        robot_config = LaunchConfiguration("robot").perform(context)
        if robot_config.startswith("auto."):
            is_automatic = True
            robot_name = robot_config[5:]
        else:
            is_automatic = False
            robot_name = robot_config

        entities.append(
            LogInfo(
                msg=f"Starting RMCS on robot '{robot_config}'{'(automatic)' if is_automatic else ''} -> {robot_name}.yaml"
            )
        )

        installed_config_path = os.path.join(
            FindPackageShare("rmcs_bringup").perform(context),
            "config",
            robot_name + ".yaml",
        )
        source_config_path = os.path.join(
            os.environ.get("RMCS_PATH", "/workspaces/RMCS"),
            "rmcs_ws",
            "src",
            "rmcs_bringup",
            "config",
            robot_name + ".yaml",
        )
        config_path = installed_config_path if os.path.isfile(installed_config_path) else source_config_path

        if config_path == source_config_path:
            entities.append(LogInfo(msg=f"Using source config fallback: {source_config_path}"))

        entities.append(
            Node(
                package="rmcs_executor",
                executable="rmcs_executor",
                parameters=[config_path],
                respawn=True,
                respawn_delay=1.0,
                output="log",  # stdout and stderr are logged to launch log file and stderr to the screen.
            )
        )

        if is_automatic:
            pass

        return entities


def generate_launch_description():
    ld = LaunchDescription([MyLaunchDescriptionEntity()])

    return ld
