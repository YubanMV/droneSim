import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, Vector3
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
import os
from std_msgs.msg import Float32MultiArray


class BearingsControlNode(Node):
    def __init__(self, drone_id):
        super().__init__('bearings_control_node_' + str(drone_id))
        self.drone_id = drone_id
        self.namespace = f'Mavic_2_PRO_{drone_id}'
        self.bearing_error_pub = self.create_publisher(
            Float32MultiArray,
            f'/{self.namespace}/bearing_errors',
            10
        )

        # Cargar configuración del YAML
        package_path = get_package_share_directory('bearings_control')
        yaml_path = os.path.join(package_path, 'config', 'formation.yaml')
        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)

        drone_config = config.get(self.drone_id)
        self.role = drone_config.get('role', 'follower')
        self.neighbors_info = drone_config.get('neighbors', [])

        # Objetivos de posición solo para líderes
        self.leader_targets = {
            1: [6.0, 5.0],  # esquina superior derecha
            2: [3.0, 5.0]   # esquina superior izquierda
        }

        # self.leader_targets = {
        #     1: [5.0, 3.0],
        #     2: [5.0, 6.0]
        # }

        self.is_leader = self.drone_id in self.leader_targets
        self.current_position = None
        self.current_speed = None

        # Ganancias del controlador PD
        self.kp = 1.0
        self.kd = 1.0

        # Subscripciones
        self.create_subscription(
            PointStamped,
            f'/{self.namespace}/gps',
            self.gps_callback,
            10)

        self.create_subscription(
            Vector3,
            f'/{self.namespace}/gps/speed_vector',
            self.speed_callback,
            10)

        # Publicador
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/cmd_vel_{self.namespace}',
            10)

        # Timer solo si es líder
        if self.is_leader:
            self.target_position = self.leader_targets[self.drone_id]
            self.create_timer(0.1, self.control_loop)
            self.get_logger().info(
                f'[{self.namespace}] Nodo líder. Objetivo: {self.target_position}')
        else:
            self.get_logger().info(
                f'[{self.namespace}] Nodo seguidor. Esperando activación posterior.')

        # Si es seguidor, inicializar variables
        self.active = False  # formación activa tras 8s
        if self.role == 'follower':
            self.neighbor_positions = {}  # ID -> Point
            # esperar 8 segundos
            self.create_timer(20.0, self.activate_formation)
            self.get_logger().info(
                f"[{self.namespace}] Soy seguidor. Formación iniciará en 8s.")

            for neighbor in self.neighbors_info:
                neighbor_id = neighbor['neighbor_id']
                self.create_subscription(
                    PointStamped,
                    f'/Mavic_2_PRO_{neighbor_id}/gps',
                    lambda msg, nid=neighbor_id: self.neighbor_position_callback(
                        msg, nid),
                    10)

    def activate_formation(self):
        self.active = True
        self.create_timer(0.1, self.control_loop)  # <-- aquí se activa
        self.get_logger().info(f"[{self.namespace}] Formación activada.")

    def neighbor_position_callback(self, msg, neighbor_id):
        self.neighbor_positions[neighbor_id] = msg.point

    def gps_callback(self, msg):
        self.current_position = msg.point

    def speed_callback(self, msg):
        self.current_speed = msg

    def control_loop(self):
        if self.current_position is None or self.current_speed is None:
            return

        if self.role == 'leader':
            # --- CONTROL PD PARA LÍDERES ---
            ex = self.target_position[0] - self.current_position.x
            ey = self.target_position[1] - self.current_position.y

            dx = -self.current_speed.x
            dy = -self.current_speed.y

            ux = self.kp * ex + self.kd * dx
            uy = self.kp * ey + self.kd * dy

            # Limitar velocidad
            v_max = 1.0
            norm = (ux**2 + uy**2)**0.5
            if norm > v_max:
                scale = v_max / norm
                ux *= scale
                uy *= scale

            self.publish_velocity(ux, uy, 0.0)

        elif self.role == 'follower' and self.active:
            # --- CONTROL GEOMÉTRICO DE BEARINGS ---
            total_ux = 0.0
            total_uy = 0.0
            error_msg = Float32MultiArray()
            error_data = []

            for neighbor in self.neighbors_info:
                nid = neighbor['neighbor_id']
                desired_bearing = neighbor['bearing']
                neighbor_pos = self.neighbor_positions.get(nid)

                if neighbor_pos is None:
                    continue

                dx = neighbor_pos.x - self.current_position.x
                dy = neighbor_pos.y - self.current_position.y
                norm = (dx**2 + dy**2)**0.5
                if norm < 1e-3:
                    continue

                # g_ij (bearing real)
                g_real = [dx / norm, dy / norm]
                g_star = desired_bearing

                # I - g_real g_real^T
                proj_matrix = [
                    [1 - g_real[0] * g_real[0], -g_real[0] * g_real[1]],
                    [-g_real[0] * g_real[1], 1 - g_real[1] * g_real[1]]
                ]

                # u = -(I - g g^T) * g_star
                ux = -(proj_matrix[0][0] * g_star[0] +
                       proj_matrix[0][1] * g_star[1])
                uy = -(proj_matrix[1][0] * g_star[0] +
                       proj_matrix[1][1] * g_star[1])

                total_ux += ux
                total_uy += uy

                # Calcular y acumular error de bearing
                bearing_error = ((g_real[0] - g_star[0])
                                 ** 2 + (g_real[1] - g_star[1])**2)**0.5
                error_data.extend([float(nid), bearing_error])

            # Ganancias
            kp_bearing = 2.5
            kd_bearing = 4.5

            total_ux *= kp_bearing
            total_uy *= kp_bearing

            if self.current_speed is not None:
                total_ux -= kd_bearing * self.current_speed.x
                total_uy -= kd_bearing * self.current_speed.y

            # Limitar velocidad
            v_max = 2.0
            norm = (total_ux**2 + total_uy**2)**0.5
            if norm > v_max:
                scale = v_max / norm
                total_ux *= scale
                total_uy *= scale

            # Publicar velocidad
            self.publish_velocity(total_ux, total_uy, 0.0)

            # Publicar errores
            error_msg.data = error_data
            self.bearing_error_pub.publish(error_msg)

    def publish_velocity(self, vx, vy, vz):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        self.cmd_vel_pub.publish(cmd)
        # self.get_logger().info(
        #     f"[{self.namespace}] Publicado cmd_vel: ({vx:.2f}, {vy:.2f}, {vz:.2f})")


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Uso: ros2 run <paquete> bearings_control_node <drone_id>")
        return

    drone_id = int(sys.argv[1])
    node = BearingsControlNode(drone_id)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
