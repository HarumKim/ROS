import math
import time
import rclpy
from rclpy.node import Node

# Conexión ZMQ
import sys
sys.path.append('/root/CoppeliaSim/programming/zmqRemoteApi/clients/python/src')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class SphereMovementNode(Node):
    def __init__(self):
        super().__init__('sphere_movement_node')

        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.sim.setStepping(True)

        self.sphere = self.sim.getObject('/Sphere')
        self.get_logger().info("✅ Connected to Orange Sphere")

        while self.sim.getSimulationState() == 0:
            time.sleep(0.1)

        self.get_logger().info("▶️ Simulation is running")

	# Range of values for sphere's position
        self.t = 0.0
        self.amplitude = 1.0	# Amplitude for X-axis oscillation 
        self.speed_y = 0.1  	# Constant forward speed in Y
        self.height = 0.15      # Constant heiht in Z of the sphere

        self.timer = self.create_timer(0.05, self.update_position)

    def update_position(self):
        x = self.amplitude * math.sin(0.25 * self.t)
        y = self.speed_y * self.t
        z = self.height

        self.sim.setObjectPosition(self.sphere, -1, [x, y, z])
        self.sim.step()

        self.get_logger().info(f"🌐 Sphere Position → x={x:.2f}, y={y:.2f}")
        self.t += 0.05

def main(args=None):
    rclpy.init(args=args)
    node = SphereMovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
