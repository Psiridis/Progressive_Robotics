from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_rviz = LaunchConfiguration("enable_rviz")

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
            "xacro ",
            urdf_file,
        ])
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time if true",
        ),
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="false",
            description="Launch RViz if true",
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
                    "joint_positions": [0.0, -1.57, 1.57, -1.57, 1.57, 0.0],
                    "world_frame": "world",
                    "elbow_frame": "forearm_link",
                    "gripper_frame": "gripper_link",
                },
            ],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file],
            condition=IfCondition(enable_rviz),
        ),
    ])