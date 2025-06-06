#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

# Add CoppeliaSim ZMQ API path
sys.path.append('/home/kim/Documents/CoppeliaSim/programming/zmqRemoteApi/clients/python/src')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class SnakeRobotController(Node):
    def __init__(self):
        super().__init__('snake_robot_controller')

        # Snake robot has 8 modules, each with vJoint and hJoint (16 joints total)
        self.num_modules = 8
        self.head_angle = 0.0  # Current head angle command
        self.v_joints = [-1] * self.num_modules  # Vertical joints
        self.h_joints = [-1] * self.num_modules  # Horizontal joints
        
        # Joint names for ROS topics
        self.joint_names = []
        
        # Control parameters (retreived from original Lua script)
        self.max_h_angle = 45 * math.pi / 180  # 45 degrees in radians

        # Connect to CoppeliaSim
        if self.connect_to_sim():
            self.discover_joints()
            self.setup_ros_interface()
    
    def connect_to_sim(self):
        """Establish connection with CoppeliaSim using ZMQ API"""
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.get_logger().info("✅ Connected to CoppeliaSim")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to CoppeliaSim: {e}")
            return False
        
    def discover_joints(self):
        """Find and connect to all snake robot joints"""
        if not self.sim:
            self.get_logger().error("❌ No CoppeliaSim connection")
            return
            
        try:
            # Get vertical and horizontal joints for each module
            for i in range(self.num_modules):
                try:
                    # Vertical joint
                    v_handle = self.sim.getObject(f'./vJoint', {'index': i})
                    self.v_joints[i] = v_handle
                    self.joint_names.append(f'vJoint{i}')
                    self.get_logger().info(f"✅ Connected to vJoint{i} (handle: {v_handle})")
                    
                    # Horizontal joint
                    h_handle = self.sim.getObject(f'./hJoint', {'index': i})
                    self.h_joints[i] = h_handle
                    self.joint_names.append(f'hJoint{i}')
                    self.get_logger().info(f"✅ Connected to hJoint{i} (handle: {h_handle})")
                    
                except Exception as e:
                    self.get_logger().warn(f"⚠️ Could not find joints for module {i}: {e}")
                    
        except Exception as e:
            self.get_logger().error(f"❌ Error discovering joints: {e}")
            
    def setup_ros_interface(self):
        """Setup ROS publishers and subscribers"""
        # Publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscribers for individual joint commands
        self.joint_subs = []
        for name in self.joint_names:
            sub = self.create_subscription(Float64, f'/snake_robot/{name}/command', lambda msg, joint=name: self.joint_command_callback(msg, joint), 10)
            self.joint_subs.append(sub)
            
                # Subscribe to head angle commands
        self.head_cmd_sub = self.create_subscription(Float64, '/snake_robot/head_cmd', self.head_cmd_callback, 10)

        # Receive movement commands via Twist messages
        self.twist_sub = self.create_subscription(Twist, '/snake_robot/cmd_vel',self.twist_callback, 10)
        self.get_logger().info(f"✅ Setup ROS interface for {len(self.joint_names)} joints")
    
    def head_cmd_callback(self, msg):
        """Receive head angle command"""
        self.head_angle = msg.data
        self.get_logger().info(f'🎯 Received head angle command: {self.head_angle:.3f} rad')

    def joint_command_callback(self, msg, joint_name):
        """Handle individual joint commands"""
        self.get_logger().info(f"🎯 Command for {joint_name}: {msg.data:.3f}")
        
        # Find the joint handle and set position
        if joint_name.startswith('vJoint'):
            idx = int(joint_name[6:])  # Extract number from 'vJoint0', 'vJoint1', and so on...
            if 0 <= idx < self.num_modules and self.v_joints[idx] != -1:
                self.sim.setJointTargetPosition(self.v_joints[idx], msg.data)
        elif joint_name.startswith('hJoint'):
            idx = int(joint_name[6:])  # Extract number from 'hJoint0', 'hJoint1', and so on...
            if 0 <= idx < self.num_modules and self.h_joints[idx] != -1:
                self.sim.setJointTargetPosition(self.h_joints[idx], msg.data)
                
    def twist_callback(self, msg):
        """Handle high-level movement commands"""
        # Convert Twist to snake movement 
        linear_x = msg.linear.x  # Forward/backward
        angular_z = msg.angular.z  # Turn left/right
        
        self.generate_snake_movement(linear_x, angular_z)
        
    def generate_snake_movement(self, forward_speed, turn_rate):
        """Generate sinusoidal snake movement based on speed and turn commands"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        base_freq = 3.0 + forward_speed
        
        for i in range(self.num_modules):
            if self.v_joints[i] != -1 and self.h_joints[i] != -1:
                phase_offset = i * 0.8
                v_angle = self.max_h_angle * math.sin(current_time * base_freq + phase_offset)
                h_angle = turn_rate * 0.5 * math.sin(current_time * base_freq * 0.3 + phase_offset)

                self.sim.setJointTargetPosition(self.v_joints[i], v_angle)
                self.sim.setJointTargetPosition(self.h_joints[i], h_angle)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = SnakeRobotController()

        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('🛑 Shutting down snake robot controller')
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()