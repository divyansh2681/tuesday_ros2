import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from datetime import datetime
import math


class ClockPosePublisher(Node):
    """Publishes poses derived from the current time's minute hand position."""

    def __init__(self):
        """Initialize publisher and timer."""
        super().__init__("clock_pose_publisher")
        self.publisher_ = self.create_publisher(PoseStamped, "clock_pose", 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_clock_pose)
        self.get_logger().info("Clock Pose Publisher Started")

    def publish_clock_pose(self):
        """Publish a pose based on the current time (minute hand on a unit circle)."""
        now = datetime.now()
        minute = now.minute + now.second / 60.0
        theta = (minute / 60.0) * 2 * math.pi  # convert to radians

        x = math.cos(theta)
        y = math.sin(theta)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0

        # Fixed orientation (facing outward from the center)
        pose_msg.pose.orientation.z = math.sin(theta / 2)
        pose_msg.pose.orientation.w = math.cos(theta / 2)

        self.publisher_.publish(pose_msg)
        self.get_logger().info(
            f"Published pose: x={x:.2f}, y={y:.2f}, θ={theta:.2f} rad"
        )


def main(args=None):
    """Entry point for the clock pose publisher node."""
    rclpy.init(args=args)
    node = ClockPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
