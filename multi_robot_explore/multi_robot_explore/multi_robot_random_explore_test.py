import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoider(Node):
    def __init__(self, robot_name):
        super().__init__('obstacle_avoider')

        self.robot_name = robot_name
        
        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            LaserScan,
            f'/{robot_name}/scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        
        # Timer for regularly publishing velocity commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Store flag whether obstacle is detected
        self.obstacle_detected = False
        self.is_rotating = False
        self.rotation_time_remaining = 2.0
        
        # Threshold distance
        self.distance_threshold = 0.4
        
        # Define the forward sector in radians
        # Angles: 330° to 360° and 0° to 20°
        # Convert degrees to radians
        self.min_angle_1 = math.radians(330.0)  # ~5.7596 rad
        self.max_angle_1 = math.radians(360.0)  # ~6.2832 rad
        self.min_angle_2 = math.radians(0.0)    # 0 rad
        self.max_angle_2 = math.radians(30.0)   # ~0.3491 rad

    def angle_in_front_sector(self, angle):
        # This function checks if the angle is within [330°,360°) or [0°,20°]
        # Given angle is always between angle_min and angle_max of the scan (likely 0 to 2π).
        
        # Check the "upper wrap-around" sector: 335° to 360°
        if angle >= self.min_angle_1 and angle < self.max_angle_1:
            return True
        
        # Check the "lower" sector: 0° to 25°
        if angle >= self.min_angle_2 and angle <= self.max_angle_2:
            return True
        
        return False

    def scan_callback(self, msg: LaserScan):
        self.obstacle_detected = False
        
        # Iterate over all ranges and check those within the forward sector
        for i, distance in enumerate(msg.ranges):
            # Compute the angle for this index
            angle = msg.angle_min + i * msg.angle_increment
            # self.get_logger().info(f"Distance: {distance:.3f} m")
            
            # Normalize angle if needed (usually angle_min=0 for a 360 deg scan, so no normalization needed)
            # If your scan starts at another angle, adjust the logic accordingly.
            
            # Check if angle is in the frontal wedge
            # if self.angle_in_front_sector(angle):
                # Check distance threshold
            if distance < self.distance_threshold:
                # Log info about this measurement
                self.get_logger().info(f"Index {i} has distance {distance:.3f} m (below 0.5 m)")
                self.get_logger().info(f"Angle_rad: {angle:.2f} (Angle_deg: {math.degrees(angle):.2f})")
                    
                    # If any point in the front sector is below threshold, set obstacle_detected = True
                    # self.obstacle_detected = True

        # After checking all points, obstacle_detected indicates whether to stop or not

    def timer_callback(self):
        # Publish cmd_vel based on obstacle detection
        twist = Twist()
        if self.obstacle_detected:
            # Stop the robot
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            self.is_rotating = True
            self.obstacle_detected = False
            self.get_logger().info(f"{self.robot_name}: Stopping...")
        elif self.is_rotating:
            # Handle rotation logic
            if self.rotation_time_remaining > 0:
                twist.angular.z = 0.3  # Rotate at a fixed angular speed
                self.rotation_time_remaining -= 0.1  # Decrease remaining time
                self.publisher.publish(twist)
                self.get_logger().info(f"{self.robot_name}: Rotating...")
            else:
                # End rotation and reset flags
                self.is_rotating = False
                self.obstacle_detected = False
                self.rotation_time_remaining = 2.0
                self.get_logger().info(f"{self.robot_name}: Finished rotating.")
        else:
            # Move forward
            twist.linear.x = 0.0  # adjust speed as desired
            twist.angular.z = 0.0
            self.publisher.publish(twist)

        

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider('robot1')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
