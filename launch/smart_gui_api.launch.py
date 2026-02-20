"""Launch configuration for Smart GUI backend and optional frontend process.

This launch file starts:
1) A ROS 2 node that exposes the FastAPI backend (`smart_gui_api`).
2) Optionally, a frontend command (usually Flutter web server) as an external process.

Launch arguments documented below:
- `host`: Interface/address used by backend HTTP server.
- `port`: Backend HTTP server port.
- `run_frontend`: Whether to start the frontend process in the same launch.
- `frontend_dir`: Working directory where the frontend command is executed.
- `frontend_command`: Command string used to start the frontend.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_frontend_dir() -> str:
    """Resolve a best-effort default path for the frontend project directory.

    Search order:
    1) `SMART_GUI_FRONTEND_DIR` environment variable.
    2) Source-tree relative path (`../frontend`).
    3) Workspace-relative path inferred from this launch file (`<workspace>/src/smart_gui/frontend`).
    4) Any `<home>/*/src/smart_gui/frontend` match.
    """
    script_path = Path(__file__).resolve()
    package_root = script_path.parents[1]
    candidates = []

    from os import environ
    custom = environ.get("SMART_GUI_FRONTEND_DIR")
    if custom:
        candidates.append(Path(custom).expanduser())

    # Running from source tree.
    candidates.append(package_root / "frontend")

    # If this launch file is executed from an installed package, walk up the tree
    # and try `<ancestor>/src/smart_gui/frontend` as a workspace-style location.
    for ancestor in script_path.parents:
        candidates.append(ancestor / "src" / "smart_gui" / "frontend")

    # Generic home fallback: any workspace-like folder containing src/smart_gui/frontend.
    candidates.extend(Path.home().glob("*/src/smart_gui/frontend"))

    for candidate in candidates:
        if candidate.exists():
            return str(candidate)

    # Last resort keeps command deterministic; caller can always override via launch arg.
    return str(package_root / "frontend")


def generate_launch_description():
    """Create launch description with backend node and optional frontend process."""
    # Backend bind address (default `0.0.0.0` for LAN accessibility).
    host_arg = DeclareLaunchArgument("host", default_value="0.0.0.0")
    # Backend HTTP port.
    port_arg = DeclareLaunchArgument("port", default_value="8000")
    # Toggle to run frontend together with backend in one launch invocation.
    run_frontend_arg = DeclareLaunchArgument("run_frontend", default_value="true")
    # Frontend project directory used as process working directory.
    frontend_dir_arg = DeclareLaunchArgument(
        "frontend_dir",
        default_value=_default_frontend_dir(),
    )
    # Frontend command to execute (e.g., `flutter run -d web-server ...`).
    frontend_command_arg = DeclareLaunchArgument(
        "frontend_command",
        default_value="flutter run -d linux",
    )

    # ROS node wrapping the FastAPI backend executable.
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

    # Optional shell process for frontend execution.
    # Using `bash -lc` allows passing a single command string with shell features.
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

    # Return launch graph with both argument declarations and executable actions.
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
