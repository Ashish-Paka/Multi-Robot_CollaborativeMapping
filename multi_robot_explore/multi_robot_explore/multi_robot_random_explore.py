import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import random

class BallisticMover(Node):
    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_mover')

        self.robot_name = robot_name

        # Publisher for /<robot_name>/cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)

        # Subscriber for /<robot_name>/scan (LiDAR data)
        self.lidar_subscriber = self.create_subscription(LaserScan, f'/{robot_name}/scan', self.lidar_callback, 10)

        # Rotation flags
        self.obstacle_detected = False
        self.is_rotating = False
        self.rotation_time_remaining = 1.0

        # Safe directions
        self.safe_front_distances = []  # Front laser data
        self.front_obstacle_threshold = 1.0  # Threshold for obstacles in front
        self.safe_distance_threshold = 1.0  # Safe distance to consider a direction viable

        # Timer for publishing velocity
        self.timer = self.create_timer(0.1, self.move)

    def lidar_callback(self, msg):
        # Focus on front-facing laser data (e.g., 30 degrees on either side of forward direction)
        total_ranges = len(msg.ranges)
        front_start_index = int(total_ranges * 0)  # Approximate start of front arc
        front_end_index = int(total_ranges * 0.1)  # Approximate end of front arc
        front_ranges = msg.ranges[front_start_index:front_end_index]

        # Filter out invalid ranges (LiDAR might return `inf` or `nan`)
        front_ranges = [dist for dist in front_ranges if not math.isinf(dist) and not math.isnan(dist)]

        # Update safe front distances
        self.safe_front_distances = [dist for dist in front_ranges if dist > self.safe_distance_threshold]

        # Check for obstacles within the front range
        self.obstacle_detected = any(dist < self.front_obstacle_threshold for dist in front_ranges)

    def move(self):
        twist = Twist()

        if self.is_rotating:
            # Handle rotation logic
            if self.rotation_time_remaining > 0:
                twist.angular.z = 0.5  # Rotate at a fixed angular speed
                self.rotation_time_remaining -= 0.1  # Decrease remaining time
                self.cmd_vel_publisher.publish(twist)
                self.get_logger().info(f"{self.robot_name}: Rotating...")
            else:
                # End rotation and reset flags
                self.is_rotating = False
                self.get_logger().info(f"{self.robot_name}: Finished rotating.")
        elif self.obstacle_detected:
            # Trigger rotation when an obstacle is detected in the front arc
            if self.safe_front_distances:
                # Choose a random safe direction (front arc only)
                chosen_angle_index = random.randint(0, len(self.safe_front_distances) - 1)
                angle = math.radians((chosen_angle_index * 30) / len(self.safe_front_distances))  # Map index to angle
                self.rotation_time_remaining = abs(angle) / 0.5  # Duration for rotation
                self.is_rotating = True
                self.get_logger().info(f"{self.robot_name}: Obstacle detected, rotating to safe direction.")
            else:
                self.get_logger().warning(f"{self.robot_name}: No safe direction found!")
        else:
            # Move forward if no obstacles are detected in the front arc
            twist.linear.x = 0.5  # Forward speed
            twist.angular.z = 0.0  # No rotation
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info(f"{self.robot_name}: Moving straight.")

def main(args=None):
    rclpy.init(args=args)

    # Create a BallisticMover for the robot
    robot = BallisticMover('robot1')

    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
