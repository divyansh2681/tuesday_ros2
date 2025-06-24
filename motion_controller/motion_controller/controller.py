import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math
import time


class MotionController(Node):
    """ROS 2 node that publishes cmd_vel based on clock or GUI pose."""

    def __init__(self):
        """Initialize subscriptions, publisher, timer, and parameters."""
        super().__init__("motion_controller")

        self.declare_parameter("gui_timeout", 30.0)
        self.timeout = self.get_parameter("gui_timeout").value

        self.gui_pose = None
        self.clock_pose = None
        self.last_gui_time = None

        self.sub_gui = self.create_subscription(
            PoseStamped, "gui_pose", self.gui_callback, 10
        )
        self.sub_clock = self.create_subscription(
            PoseStamped, "clock_pose", self.clock_callback, 10
        )
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

        self.timer = self.create_timer(1.0, self.control_loop)
        self.get_logger().info("Motion Controller started.")

    def gui_callback(self, msg):
        """Handle incoming GUI pose and update last received time."""
        self.gui_pose = msg
        self.last_gui_time = time.time()
        self.get_logger().info("Received GUI pose.")

    def clock_callback(self, msg):
        """Update current clock pose."""
        self.clock_pose = msg

    def control_loop(self):
        """Choose pose source and publish a corresponding velocity command."""
        now = time.time()
        use_gui = (
            self.gui_pose is not None
            and self.last_gui_time is not None
            and (now - self.last_gui_time) <= self.timeout
        )

        target_pose = self.gui_pose if use_gui else self.clock_pose

        if target_pose is None:
            self.get_logger().warn("No pose available yet.")
            return

        x = target_pose.pose.position.x
        y = target_pose.pose.position.y
        angle = math.atan2(y, x)
        distance = math.hypot(x, y)

        twist = Twist()
        twist.linear.x = min(distance, 0.5)
        twist.angular.z = angle
        self.publisher_.publish(twist)

        source = "GUI" if use_gui else "Clock"
        self.get_logger().info(
            f"[{source}] --> Pose x={x:.2f}, y={y:.2f}, θ={angle:.2f} --> Published cmd_vel"
        )


def main(args=None):
    """Initialize and run the motion controller node."""
    rclpy.init(args=args)
    node = MotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
