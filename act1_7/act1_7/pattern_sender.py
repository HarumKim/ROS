import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class PatternSender(Node):
    def __init__(self):
        super().__init__('pattern_sender')
        self.cmd_vel_pub = self.create_publisher(Twist, '/snake_robot/cmd_vel', 10)
        self.head_cmd_pub = self.create_publisher(Float64, '/snake_robot/head_cmd', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_commands)
        
        # Command messages
        self.cmd_vel = Twist()
        self.head_angle = Float64()
        
        # Control variables
        self.forward_duration = 6.0  # seconds
        self.forward_speed = 1.5     # m/s
        self.head_straight = 0.0     # Head angle
        
        self.start_time = self.get_clock().now()
        self.pattern_complete = False

    def publish_commands(self):
        if self.pattern_complete:
            return
        
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        
        if elapsed < self.forward_duration:
            # Move forward with head straight
            self.cmd_vel.linear.x = self.forward_speed
            self.cmd_vel.angular.z = 0.
            self.head_angle.data = self.head_straight
        else:
            # Stop the robot
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.head_angle.data = self.head_straight
            self.pattern_complete = True
            self.get_logger().info('Forward motion complete. Robot stopped.')
        
        # Publish commands
        self.cmd_vel_pub.publish(self.cmd_vel)
        self.head_cmd_pub.publish(self.head_angle)

def main(args=None):
    rclpy.init(args=args)
    node = PatternSender()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
