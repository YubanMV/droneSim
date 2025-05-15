#!/usr/bin/env python3

import os
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

qos_transient_local = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)


class PX4InstanceLauncher(Node):
    def __init__(self):
        super().__init__('px4_instance_launcher')

        self.num_drones = 4
        self.delay = 6  # segundos
        self.px4_dir = os.path.expanduser('~/PX4-Autopilot')

        self.pose_publishers = {}

        for i in range(self.num_drones):
            topic_name = f'/drone_{i}/initial_pose'
            self.pose_publishers[i] = self.create_publisher(Point, topic_name, qos_transient_local)

        self.launch_drones()

    def launch_drones(self):
        self.get_logger().info(f"Lanzando {self.num_drones} instancias de PX4")
        for i in range(self.num_drones):
            pose_xyz = [0.0, float(i), 0.0]  # <-- esta es la pose en el mundo
            pose_str = ",".join(str(p) for p in pose_xyz)

            # Lanzar el PX4 con esa pose
            drone_cmd = (
                f'PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="{pose_str}" PX4_GZ_MODEL=x500 '
                f'{self.px4_dir}/build/px4_sitl_default/bin/px4 -i {i}'
            )
            full_cmd = f"cd {self.px4_dir} && {drone_cmd}"
            os.system(f"gnome-terminal --tab -- bash -c \"{full_cmd}\"")

            # Publicar la pose como mensaje Point
            point_msg = Point()
            point_msg.x, point_msg.y, point_msg.z = pose_xyz
            self.pose_publishers[i].publish(point_msg)
            self.get_logger().info(f"Dron {i} lanzado y pose publicada: {pose_xyz}")

            time.sleep(self.delay)


def main(args=None):
    rclpy.init(args=args)
    launcher = PX4InstanceLauncher()
    rclpy.spin(launcher)
    launcher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
