from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_frontend_dir() -> str:
    env_dir = Path.home() / "colcon_ws" / "src" / "smart_gui" / "frontend"
    candidates = []

    from os import environ
    custom = environ.get("SMART_GUI_FRONTEND_DIR")
    if custom:
        candidates.append(Path(custom).expanduser())

    # Running from source tree.
    candidates.append(Path(__file__).resolve().parents[1] / "frontend")
    # Common workspace convention.
    candidates.append(env_dir)
    # Any *_ws workspace under HOME.
    candidates.extend(Path.home().glob("*_ws/src/smart_gui/frontend"))

    for candidate in candidates:
        if candidate.exists():
            return str(candidate)

    return str(env_dir)


def generate_launch_description():
    host_arg = DeclareLaunchArgument("host", default_value="0.0.0.0")
    port_arg = DeclareLaunchArgument("port", default_value="8000")
    run_frontend_arg = DeclareLaunchArgument("run_frontend", default_value="true")
    frontend_dir_arg = DeclareLaunchArgument(
        "frontend_dir",
        default_value=_default_frontend_dir(),
    )
    frontend_command_arg = DeclareLaunchArgument(
        "frontend_command",
        default_value="flutter run -d linux",
    )

    api_node = Node(
        package="smart_gui",
        executable="smart_gui_api",
        name="smart_gui_api",
        output="screen",
        arguments=[
            "--host",
            LaunchConfiguration("host"),
            "--port",
            LaunchConfiguration("port"),
        ],
    )

    frontend_process = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("run_frontend")),
        cmd=[
            "bash",
            "-lc",
            [
                "cd ",
                LaunchConfiguration("frontend_dir"),
                " && ",
                LaunchConfiguration("frontend_command"),
            ],
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            host_arg,
            port_arg,
            run_frontend_arg,
            frontend_dir_arg,
            frontend_command_arg,
            api_node,
            frontend_process,
        ]
    )
