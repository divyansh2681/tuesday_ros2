import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import pygame
import math
import time
from datetime import datetime

RADIUS = 200
WINDOW_SIZE = 500
CENTER = (WINDOW_SIZE // 2, WINDOW_SIZE // 2)


class GuiPosePublisher(Node):
    """Publishes poses based on user clicks on a clock-style GUI."""

    def __init__(self):
        """Initialize publisher, GUI, and timer."""
        super().__init__("gui_pose_publisher")
        self.publisher_ = self.create_publisher(PoseStamped, "gui_pose", 10)
        self.last_click_time = time.time()
        self.pose_valid = False

        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
        pygame.display.set_caption("GUI Pose Issuer")
        self.clock = pygame.time.Clock()

    def publish_pose(self, x, y):
        """Publish a pose based on normalized click coordinates."""
        theta = math.atan2(y, x)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(theta / 2)
        pose.pose.orientation.w = math.cos(theta / 2)

        self.publisher_.publish(pose)
        self.get_logger().info(f"Published GUI pose: x={x:.2f}, y={y:.2f}")
        self.last_click_time = time.time()
        self.pose_valid = True

    def run(self):
        """Main GUI event loop for clock rendering and input handling."""
        font = pygame.font.SysFont("Arial", 24)
        running = True
        gui_override = False
        override_pose = (0.0, 0.0)

        while running:
            self.clock.tick(30)
            self.screen.fill((255, 255, 255))

            # Draw clock circle
            pygame.draw.circle(self.screen, (0, 0, 0), CENTER, RADIUS, 2)

            # Draw numbers 1 to 12
            for h in range(1, 13):
                angle = math.radians((h - 3) * 30)
                num_x = CENTER[0] + (RADIUS - 30) * math.cos(angle)
                num_y = CENTER[1] + (RADIUS - 30) * math.sin(angle)
                text = font.render(str(h), True, (0, 0, 0))
                text_rect = text.get_rect(center=(num_x, num_y))
                self.screen.blit(text, text_rect)

            # Determine mode
            current_time = time.time()
            if gui_override and (current_time - self.last_click_time > 30):
                self.get_logger().info("GUI pose timed out after 30 seconds.")
                gui_override = False

            # Draw hands based on mode
            if gui_override:
                x, y = override_pose
                # Minute hand (blue)
                mx = CENTER[0] + (RADIUS - 20) * x
                my = CENTER[1] + (RADIUS - 20) * y
                pygame.draw.line(self.screen, (0, 0, 255), CENTER, (mx, my), 3)

                # Hour hand (red) – half-length version of same direction
                hx = CENTER[0] + (RADIUS - 60) * x
                hy = CENTER[1] + (RADIUS - 60) * y
                pygame.draw.line(self.screen, (255, 0, 0), CENTER, (hx, hy), 6)
            else:
                now = datetime.now()
                minute_angle = math.radians((now.minute - 15) * 6)
                hour_angle = math.radians(
                    (now.hour % 12 - 3) * 30 + now.minute * 0.5
                )

                # Minute hand
                mx = CENTER[0] + (RADIUS - 20) * math.cos(minute_angle)
                my = CENTER[1] + (RADIUS - 20) * math.sin(minute_angle)
                pygame.draw.line(self.screen, (0, 0, 255), CENTER, (mx, my), 3)

                # Hour hand
                hx = CENTER[0] + (RADIUS - 60) * math.cos(hour_angle)
                hy = CENTER[1] + (RADIUS - 60) * math.sin(hour_angle)
                pygame.draw.line(self.screen, (255, 0, 0), CENTER, (hx, hy), 6)

            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                elif event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = pygame.mouse.get_pos()
                    dx = mx - CENTER[0]
                    dy = my - CENTER[1]
                    dist = math.hypot(dx, dy)
                    if dist <= RADIUS:
                        x = dx / RADIUS
                        y = dy / RADIUS
                        override_pose = (x, y)
                        gui_override = True
                        self.publish_pose(x, y)

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        gui_override = False
                        self.get_logger().info("GUI pose cleared by spacebar.")

            pygame.display.flip()

        pygame.quit()


def main(args=None):
    """Entry point for the GUI pose publisher node."""
    rclpy.init(args=args)
    node = GuiPosePublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
