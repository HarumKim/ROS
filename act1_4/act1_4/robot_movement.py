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
        
        self.estado = 'stop'  # Initial state
        self.estado_anterior = 'stop'
        self.timer_pause = None
        self.stop_once = True

        # Create display windows
        cv2.namedWindow("VisionSensor", cv2.WINDOW_AUTOSIZE)

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

            # Convert to HSV
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Define color ranges
            lower_green = np.array([35, 50, 50])
            upper_green = np.array([85, 255, 255])
            
            lower_orange = np.array([10, 100, 100])
            upper_orange = np.array([25, 255, 255])

            lower_blue = np.array([100, 150, 150])
            upper_blue = np.array([140, 255, 255])

            lower_yellow = np.array([25, 150, 150])
            upper_yellow = np.array([35, 255, 255])

            # Masks
            mask_green = cv2.inRange(hsv, lower_green, upper_green)
            mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
            mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # Morphological cleaning
            kernel = np.ones((5, 5), np.uint8)
            mask_green = cv2.erode(mask_green, kernel, iterations=1)
            mask_green = cv2.dilate(mask_green, kernel, iterations=2)
            
            mask_orange = cv2.erode(mask_orange, kernel, iterations=1)
            mask_orange = cv2.dilate(mask_orange, kernel, iterations=2)

            mask_blue = cv2.erode(mask_blue, kernel, iterations=1)
            mask_blue = cv2.dilate(mask_blue, kernel, iterations=2)

            mask_yellow = cv2.erode(mask_yellow, kernel, iterations=1)
            mask_yellow = cv2.dilate(mask_yellow, kernel, iterations=2)

            # Moments
            M_green = cv2.moments(mask_green)
            M_orange = cv2.moments(mask_orange)
            M_blue = cv2.moments(mask_blue)
            M_yellow = cv2.moments(mask_yellow)

            cmd = Twist()
            img_display = img.copy()

            # --- State Machine ---
            if M_yellow["m00"] > 100 and self.stop_once:
                if self.estado != 'stop_temp':
                    self.get_logger().info("💛 Yellow detected - Pausing for 5 seconds")
                    self.estado_anterior = self.estado  # Save current state
                    self.estado = 'stop_temp'
                    self.timer_pausa = time.time()  # Start timer
                    self.stop_once = False

            elif self.estado == 'stop_temp':
                if self.timer_pausa and (time.time() - self.timer_pausa >= 5.0):
                    self.get_logger().info("⏳ Pause complete - Resuming previous state")
                    self.estado = self.estado_anterior
                    self.timer_pausa = None 

            else:
                if M_green["m00"] > 100:  # Green detected
                    if self.estado != 'forward':
                        self.get_logger().info("🟢 Green detected - Moving forward")
                    self.estado = 'forward'

                elif self.estado == 'turn_right':
                    # Stay rotating until green is detected
                    pass

                elif M_orange["m00"] > 100:  # Orange detected
                    if self.estado != 'turn_right':
                        self.get_logger().info("🟠 Orange detected - Starting to rotate right")
                    self.estado = 'turn_right'

                elif self.estado == 'turn_left':
                    # Stay rotating until green is detected
                    pass

                elif M_blue["m00"] > 100:  # Blue detected
                    if self.estado != 'turn_left':
                        self.get_logger().info("🔵 Blue detected - Starting to rotate left")
                    self.estado = 'turn_left'

                else:
                    if self.estado != 'stop':
                        self.get_logger().info("🛑 No sphere detected - Stopping")
                    self.estado = 'stop'


            # --- Act based on state ---
            cmd = Twist()

            if self.estado == 'stop_temp':
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

            elif self.estado == 'forward':
                cx = int(M_green["m10"] / M_green["m00"])
                cy = int(M_green["m01"] / M_green["m00"])

                cv2.circle(img_display, (cx, cy), 10, (0, 255, 0), -1)
                cv2.line(img_display, (resX//2, resY//2), (cx, cy), (255, 0, 0), 2)

                error = (resX // 2) - cx
                self.integral = max(-1000, min(1000, self.integral + error))
                derivative = error - self.prev_error
                self.prev_error = error

                control = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
                control = max(-1.0, min(1.0, control))

                cmd.linear.x = 0.2
                cmd.angular.z = control

            elif self.estado == 'turn_right':
                cmd.linear.x = 0.0
                cmd.angular.z = -0.5

            elif self.estado == 'turn_left':
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5

            elif self.estado == 'stop':
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.integral = 0.0
                self.prev_error = 0.0

            # Display center line
            cv2.line(img_display, (resX//2, 0), (resX//2, resY), (0, 0, 255), 1)

            # Send command
            self.cmd_pub.publish(cmd)

            # Show images
            cv2.imshow("VisionSensor", img_display)
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
