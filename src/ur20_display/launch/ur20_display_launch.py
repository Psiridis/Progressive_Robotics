"""
UR20 Display Launch File with macOS RViz GUI Forwarding Support

This launch file starts the UR20 robot visualization stack with optional RViz support.

KEY UPDATES FOR macOS GUI FORWARDING:
=====================================
1. Added `use_software_rendering` argument (default: true)
   - Forces Mesa software rendering when forwarding GUI through Docker/XQuartz
   - Disables hardware OpenGL which often fails in virtual environments

2. SetEnvironmentVariable QT_X11_NO_MITSHM=1
   - Fixes X11 shared memory issues when running Qt apps inside Docker
   - Prevents "MIT-SHM" protocol failures over X11 forwarding

3. SetEnvironmentVariable LIBGL_ALWAYS_SOFTWARE
   - Ensures RViz uses CPU-based OpenGL instead of hardware acceleration
   - Critical for macOS + Docker + XQuartz stacks

RUNNING ON macOS:
=================
Required Setup (one-time):
  1. Install XQuartz (if not installed):
     brew install --cask xquartz
  
  2. Configure XQuartz for network access:
     - Open XQuartz Preferences
     - Go to Security tab
     - Enable "Allow connections from network clients"
     - Restart XQuartz
  
  3. Allow localhost connections:
     xhost +localhost

Run Command:
  ros2 launch ur20_display ur20_display_launch.py enable_rviz:=true

If RViz still doesn't appear, use this to test direct X11 forwarding:
  ros2 run rviz2 rviz2  # Should open window within 3-5 seconds
  
If no window, check:
  - Is XQuartz running? (Check Applications > Utilities > XQuartz)
  - Does `xhost` show localhost? (Run: xhost)
  - Are you inside the container? (Check: docker ps)

LAUNCH ARGUMENTS:
=================
- use_sim_time (default: false)
  Use simulated clock instead of wall clock

- enable_rviz (default: false)
  Launch RViz window for visualization (set to true to enable)

- use_software_rendering (default: true)
  Force Mesa software rendering for GUI forwarding scenarios
  Set to false only if running on native Linux with full OpenGL support

- enable_trajectory_plotter (default: false)
  Start the Python trajectory plotter node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_rviz = LaunchConfiguration("enable_rviz")
    use_software_rendering = LaunchConfiguration("use_software_rendering")
    enable_trajectory_plotter = LaunchConfiguration("enable_trajectory_plotter")

    urdf_file = PathJoinSubstitution([
        FindPackageShare("ur20_display"),
        "urdf",
        "ur20_with_gripper.urdf.xacro",
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ur20_display"),
        "rviz",
        "ur20_display.rviz",
    ])

    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"),
            " ",
            urdf_file,
        ])
    }

    launch_actions = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time if true.",
        ),
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="false",
            description="Launch RViz if true.",
        ),
        DeclareLaunchArgument(
            "use_software_rendering",
            default_value="true",
            description=(
                "Force Mesa software rendering for RViz. "
                "Useful when running RViz through Docker/XQuartz on macOS."
            ),
        ),
        DeclareLaunchArgument(
            "enable_trajectory_plotter",
            default_value="false",
            description="Start the Python trajectory plotter node.",
        ),
        # ============================================================================
        # macOS GUI Forwarding Configuration
        # ============================================================================
        # These environment variables are essential for RViz to work through XQuartz
        # on macOS. They disable hardware rendering and fix X11 protocol issues.
        # ============================================================================
        # Fix for X11 shared memory protocol issues in Docker
        # Prevents "MIT-SHM" errors when Qt apps forward via X11 over network
        SetEnvironmentVariable(
            name="QT_X11_NO_MITSHM",
            value="1",
        ),
        # Force software-based OpenGL rendering instead of hardware acceleration
        # This ensures RViz works reliably through XQuartz forwarding on macOS
        # Value is controlled by use_software_rendering argument
        SetEnvironmentVariable(
            name="LIBGL_ALWAYS_SOFTWARE",
            value=use_software_rendering,
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                robot_description,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="ur20_display",
            executable="ur20_display_node",
            name="ur20_display_node",
            output="screen",
            parameters=[
                {
                    "joint_names": [
                        "shoulder_pan_joint",
                        "shoulder_lift_joint",
                        "elbow_joint",
                        "wrist_1_joint",
                        "wrist_2_joint",
                        "wrist_3_joint",
                    ],
                    "joint_positions": [
                        0.0,
                        -1.57,
                        1.57,
                        -1.57,
                        1.57,
                        0.0,
                    ],
                    "world_frame": "world",
                    "elbow_frame": "forearm_link",
                    "gripper_frame": "gripper_link",
                },
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d",
                rviz_config_file,
            ],
            # RViz only launches if enable_rviz:=true is passed to the launch command
            # Example: ros2 launch ur20_display ur20_display_launch.py enable_rviz:=true
            #
            # If RViz window doesn't appear on macOS:
            #   1. Ensure XQuartz is running and configured (see module docstring)
            #   2. Check that xhost +localhost was executed
            #   3. Test with: ros2 run rviz2 rviz2 (direct RViz, no launch file)
            #   4. If still no window, check docker-compose.yml DISPLAY setting
            condition=IfCondition(enable_rviz),
        ),
        Node(
            package="ur20_display",
            executable="trajectory_plotter.py",
            name="trajectory_plotter",
            output="screen",
            condition=IfCondition(enable_trajectory_plotter),
            parameters=[
                {
                    "output_path": "/ros2_ws/src/ur20_display/scripts/ur20_joint_trajectory.png"
                }
            ],
        ),
    ]

    return LaunchDescription(launch_actions)