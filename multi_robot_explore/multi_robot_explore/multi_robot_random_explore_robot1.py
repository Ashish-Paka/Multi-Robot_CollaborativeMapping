import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoider_robot1(Node):
    def __init__(self, robot_name):
        super().__init__('obstacle_avoider')

        self.robot_name = robot_name

        # Declare parameters with defaults
        self.declare_parameter(f'{robot_name}.front_min_angle_degs', 60.0)
        self.declare_parameter(f'{robot_name}.front_max_angle_degs', 170.0)
        self.declare_parameter(f'{robot_name}.distance_threshold', 0.5)
        self.declare_parameter(f'{robot_name}.forward_speed', 0.3)
        self.declare_parameter(f'{robot_name}.rotation_speed', 0.3)
        self.declare_parameter(f'{robot_name}.rotation_time', 2.0)

        # Get parameters
        front_min_angle_degs = self.get_parameter(f'{robot_name}.front_min_angle_degs').value
        front_max_angle_degs = self.get_parameter(f'{robot_name}.front_max_angle_degs').value
        self.distance_threshold = self.get_parameter(f'{robot_name}.distance_threshold').value
        self.forward_speed = self.get_parameter(f'{robot_name}.forward_speed').value
        self.rotation_speed = self.get_parameter(f'{robot_name}.rotation_speed').value
        self.rotation_time = self.get_parameter(f'{robot_name}.rotation_time').value


        # Print parameters to console
        self.get_logger().info(f'Front Min Angle Degrees: {front_min_angle_degs}')
        self.get_logger().info(f'Front Max Angle Degrees: {front_max_angle_degs}')
        self.get_logger().info(f'Distance Threshold: {self.distance_threshold}')
        self.get_logger().info(f'Forward Speed: {self.forward_speed}')
        self.get_logger().info(f'Rotation Speed: {self.rotation_speed}')
        self.get_logger().info(f'Rotation Time: {self.rotation_time}')

        # Convert angles to radians
        self.front_min_angle = math.radians(front_min_angle_degs)
        self.front_max_angle = math.radians(front_max_angle_degs)

        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)

        # Timer to publish commands regularly
        self.timer = self.create_timer(0.1, self.timer_callback)

        # State variables
        self.obstacle_detected = False
        self.is_rotating = False
        self.rotation_time_remaining = self.rotation_time

    def angle_in_front_sector(self, angle):
        # Normalize angle into [0, 2π)
        if angle < 0:
            angle += 2*math.pi

        # If no wrap-around needed (front_min < front_max)
        if self.front_min_angle < self.front_max_angle:
            return self.front_min_angle <= angle <= self.front_max_angle
        else:
            # Wrap-around sector: e.g., 330° to 30°
            # Check if angle is above front_min_angle or below front_max_angle
            return (angle >= self.front_min_angle) or (angle <= self.front_max_angle)

    def scan_callback(self, msg: LaserScan):
        self.obstacle_detected = False

        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            if self.angle_in_front_sector(angle):
                if distance < self.distance_threshold:
                    self.get_logger().info(f"{self.robot_name}: Obstacle detected at index {i}, distance {distance:.3f} m")
                    self.obstacle_detected = True
                    break

    def timer_callback(self):
        twist = Twist()

        if self.obstacle_detected:
            # Stop and prepare to rotate
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            self.is_rotating = True
            self.obstacle_detected = False
            self.rotation_time_remaining = self.rotation_time
            self.get_logger().info(f"{self.robot_name}: Stopping and will rotate...")
        elif self.is_rotating:
            # Rotate for the specified time
            if self.rotation_time_remaining > 0:
                twist.angular.z = self.rotation_speed
                self.rotation_time_remaining -= 0.1
                self.publisher.publish(twist)
                self.get_logger().info(f"{self.robot_name}: Rotating... ({self.rotation_time_remaining:.1f}s left)")
            else:
                # Done rotating
                self.is_rotating = False
                self.get_logger().info(f"{self.robot_name}: Finished rotating.")
        else:
            # Move forward if clear
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
            self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider_robot1('robot1')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
