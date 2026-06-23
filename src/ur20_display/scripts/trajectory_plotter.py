#!/usr/bin/env python3

from pathlib import Path
import matplotlib
matplotlib.use("Agg")  # Use a non-interactive backend for plotting
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory


class TrajectoryPlotterNode(Node):
    def __init__(self):
        super().__init__("trajectory_plotter_node")

        self.declare_parameter(
            "output_path",
            "/tmp/ur20_joint_trajectory.png",
        )

        trajectory_qos = QoSProfile(depth=1)
        trajectory_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        trajectory_qos.reliability = ReliabilityPolicy.RELIABLE

        self.subscription = self.create_subscription(
            JointTrajectory,
            "/ur20_display/joint_trajectory",
            self.trajectory_callback,
            trajectory_qos,
        )

        self.get_logger().info(
            "Waiting for trajectory on /ur20_display/joint_trajectory..."
        )

    def trajectory_callback(self, message: JointTrajectory):
        if not message.points:
            self.get_logger().warn("Received an empty trajectory message.")
            return

        if not message.joint_names:
            self.get_logger().warn("Received trajectory without joint names.")
            return

        times = [
            point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            for point in message.points
        ]

        positions_by_joint = list(zip(*(point.positions for point in message.points)))

        output_path = Path(
            self.get_parameter("output_path").get_parameter_value().string_value
        )

        output_path.parent.mkdir(parents=True, exist_ok=True)

        plt.figure(figsize=(10, 6))

        for joint_name, positions in zip(message.joint_names, positions_by_joint):
            plt.plot(times, positions, label=joint_name)

        plt.title("UR20 Periodic Joint Trajectory")
        plt.xlabel("Time [s]")
        plt.ylabel("Joint position [rad]")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(output_path, dpi=150)

        self.get_logger().info(
            f"Saved trajectory plot to: {output_path}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryPlotterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()