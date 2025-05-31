import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

class FormationVisualizer3D(Node):
    def __init__(self, drone_ids):
        super().__init__('formation_visualizer_node')
        self.positions = {i: None for i in drone_ids}
        self.history = {i: [] for i in drone_ids}
        self.error_history = {}  # (drone_id, neighbor_id) -> [errores]

        self.drone_ids = drone_ids

        for i in drone_ids:
            self.create_subscription(
                PointStamped,
                f'/Mavic_2_PRO_{i}/gps',
                lambda msg, i=i: self.position_callback(msg, i),
                10)

            self.create_subscription(
                Float32MultiArray,
                f'/Mavic_2_PRO_{i}/bearing_errors',
                lambda msg, i=i: self.bearing_error_callback(msg, i),
                10)

        self.timer = self.create_timer(0.1, self.plot_all)

        plt.ion()
        self.fig = plt.figure(figsize=(12, 6))
        self.ax3d = self.fig.add_subplot(121, projection='3d')
        self.ax_error = self.fig.add_subplot(122)

    def position_callback(self, msg, drone_id):
        pos = (msg.point.x, msg.point.y, msg.point.z)
        self.positions[drone_id] = pos
        self.history[drone_id].append(pos)

        if len(self.history[drone_id]) > 300:
            self.history[drone_id] = self.history[drone_id][-300:]

    def bearing_error_callback(self, msg, drone_id):
        data = msg.data
        for i in range(0, len(data), 2):
            neighbor_id = int(data[i])
            error = float(data[i + 1])
            key = (drone_id, neighbor_id)
            if key not in self.error_history:
                self.error_history[key] = []
            self.error_history[key].append(error)
            if len(self.error_history[key]) > 300:
                self.error_history[key] = self.error_history[key][-300:]

    def plot_all(self):
        self.ax3d.clear()
        self.ax_error.clear()
        ideal_positions = {
            1: [3.0, 5.0, 5.0],          # 3 + 3*0, 5 + 3*0, 2 + 3*1
            5: [6.0, 5.0, 5.0],          # 3 + 3*1, 5 + 3*0, 2 + 3*1
            3: [3.0, 8.0, 5.0],          # 3 + 3*0, 5 + 3*1, 2 + 3*1
            4: [6.0, 8.0, 5.0],          # 3 + 3*1, 5 + 3*1, 2 + 3*1
            7: [3.0, 5.0, 2.0],          # 3 + 3*0, 5 + 3*0, 2 + 3*0
            6: [6.0, 5.0, 2.0],          # 3 + 3*1, 5 + 3*0, 2 + 3*0
            2: [6.0, 8.0, 2.0],          # 3 + 3*1, 5 + 3*1, 2 + 3*0
            8: [3.0, 8.0, 2.0],          # 3 + 3*0, 5 + 3*1, 2 + 3*0
        }


        for pos in ideal_positions.values():
            self.ax3d.scatter(*pos, s=40, marker='x', color='gray', alpha=0.5)

        # Trajectories and positions
        for drone_id in self.drone_ids:
            trajectory = self.history.get(drone_id, [])
            if len(trajectory) > 1:
                xs, ys, zs = zip(*trajectory)
                self.ax3d.plot(xs, ys, zs, label=f'Drone {drone_id}', linewidth=2)

            pos = self.positions.get(drone_id)
            if pos is not None:
                x, y, z = pos
                self.ax3d.scatter(x, y, z, s=50, marker='o', label=f'Drone {drone_id}')


        self.ax3d.set_xlim(0, 10)
        self.ax3d.set_ylim(0, 10)
        self.ax3d.set_zlim(0, 6)
        self.ax3d.set_title('Formación basado en Bearings')
        self.ax3d.set_xlabel('X')
        self.ax3d.set_ylabel('Y')
        self.ax3d.set_zlabel('Z')
        self.ax3d.legend()
        self.ax3d.grid(True)

        # Error plots
        for key, errors in self.error_history.items():
            drone_id, neighbor_id = key
            label = f"{drone_id}→{neighbor_id}"
            self.ax_error.plot(errors, label=label)

        self.ax_error.set_title('Errores de Bearings')
        self.ax_error.set_xlabel('Tiempo')
        self.ax_error.set_ylabel('Error')
        self.ax_error.grid(True)
        self.ax_error.legend()

        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    drone_ids = list(range(1, 9))  # Mostrar drones del 1 al 8
    node = FormationVisualizer3D(drone_ids)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        plt.show()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()