# Generar el movimiento de la esfera

import sys
import rclpy
from rclpy.node import Node
import math
import time

# Añadir ruta al cliente ZMQ de CoppeliaSim
sys.path.append('/root/CoppeliaSim/programming/zmqRemoteApi/clients/python/src')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class SphereMovementNode(Node):
    def __init__(self):
        super().__init__('sphere_movement_node')

        # Conectar con CoppeliaSim
        self.get_logger().info("Conectando con CoppeliaSim...")
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')

        # Obtener el handle de la esfera
        self.sphere_handle = self.sim.getObject('/Sphere')

        self.get_logger().info("Conexión con CoppeliaSim exitosa.")

        # Iniciar simulación si está detenida
        if self.sim.getSimulationState() == self.sim.simulation_stopped:
            self.sim.startSimulation()
            self.get_logger().info("Simulación iniciada.")

        # Tiempo para el movimiento senoidal
        self.t = 0.0

        # Crear un timer (20 Hz)
        self.timer = self.create_timer(0.05, self.move_sphere)

    def move_sphere(self):
        # Calcular nueva posición
        x = math.sin(self.t)
        y = 0.0
        z = 0.0

        # Establecer nueva posición de la esfera
        self.sim.setObjectPosition(self.sphere_handle, -1, [x, y, z])
        self.get_logger().info(f'Moviendo esfera a: x={x:.2f}, y={y}, z={z}')

        # Incrementar tiempo
        self.t += 0.05

    def shutdown(self):
        if self.sim.getSimulationState() != self.sim.simulation_stopped:
            self.sim.stopSimulation()
            self.get_logger().info("Simulación detenida.")

def main(args=None):
    rclpy.init(args=args)
    node = SphereMovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

