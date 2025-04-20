import sys
import time
import cv2
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult

# ZMQ API path
sys.path.append('/root/CoppeliaSim/programming/zmqRemoteApi/clients/python/src')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class RobotMovement(Node):
    def __init__(self):
        super().__init__('robot_movement_node')

        # PID parameters
        self.Kp = 0.1	# Stronger Response
        self.Ki = 0.0
        self.Kd = 0.02
        self.get_logger().info(f"PID parameters: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}")

        # Connect to CoppeliaSim
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.sim.setStepping(True)

        # Vision Sensor
        self.cam = self.sim.getObject('/PioneerP3DX/visionSensor')
        self.get_logger().info("📷 Connected to visionSensor")

        # Publisher to velocity topic
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("⏳ Waiting for simulation to start...")
        while self.sim.getSimulationState() == 0:
            time.sleep(0.1)
        self.get_logger().info("▶️ Simulation started")

        # PID Variables
        self.prev_error = 0.0
        self.integral = 0.0
        
        # Create display windows
        cv2.namedWindow("VisionSensor", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("OrangeMask", cv2.WINDOW_AUTOSIZE)

        # Timer to follow the sphere
        self.timer = self.create_timer(0.1, self.track_sphere)

    def track_sphere(self):
        try:
            self.sim.handleVisionSensor(self.cam)
            img, resolution = self.sim.getVisionSensorImg(self.cam, 0)
            
            if not img:
                self.get_logger().warn("No image received from camera")
                return
                
            resX, resY = resolution

            img = np.frombuffer(img, dtype=np.uint8).reshape((resY, resX, 3))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            img = cv2.flip(img, 0)

            # Cnvert to HSV and define orange range
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lower_orange = np.array([10, 150, 150]) 
            upper_orange = np.array([25, 255, 255])
            
            mask = cv2.inRange(hsv, lower_orange, upper_orange)
            
            # Apply morphological operations to clean up the maks
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)
            
            M = cv2.moments(mask)

            cmd = Twist()
            img_display = img.copy()

            if M["m00"] > 100:  # Minimum size to consider a valid detection
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Draw center of the sphere
                cv2.circle(img_display, (cx, cy), 10, (0, 255, 0), -1)
                cv2.line(img_display, (resX//2, resY//2), (cx, cy), (255, 0, 0), 2)
                
                error = (resX // 2) - cx

                # PID control
                self.integral = max(-1000, min(1000, self.integral + error))  
                derivative = error - self.prev_error
                self.prev_error = error

                control = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
                
                control = max(-1.0, min(1.0, control))
                cmd.linear.x = 0.2
                cmd.angular.z = control

                self.get_logger().info(
                    f"🎯 Sphere detected at ({cx}, {cy}) — Error: {error} | Control: {control:.2f}"
                )
            else:
                self.get_logger().info("🧐 Sphere not visible - stopping robot")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.integral = 0.0
                self.prev_error = 0.0
            
            cv2.line(img_display, (resX//2, 0), (resX//2, resY), (0, 0, 255), 1)
            
            self.get_logger().info(f"🚀 Sending cmd_vel — linear.x: {cmd.linear.x:.2f}, angular.z: {cmd.angular.z:.2f}")
            self.cmd_pub.publish(cmd)
            
            cv2.imshow("VisionSensor", img_display)
            cv2.imshow("OrangeMask", mask)
            cv2.waitKey(1)
            
            self.sim.step()

        except Exception as e:
            self.get_logger().error(f"❌ Error in track_sphere: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def param_callback(self, params):
        for param in params:
            if param.name == 'Kp':
                self.Kp = param.value
            elif param.name == 'Ki':
                self.Ki = param.value
            elif param.name == 'Kd':
                self.Kd = param.value
        self.get_logger().info(f"Updated PID parameters: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = RobotMovement()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
