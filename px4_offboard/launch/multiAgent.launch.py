from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    num_drones = 4

    nodes = [
        # Agente XRCE-DDS
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            prefix='gnome-terminal --',
        ),
        # Lanzador de instancias PX4
        Node(
            package='px4_offboard',
            executable='px4_instances',
            name='px4_instance_launcher',
            output='screen'
        ),
        # # Nodo de teclado
        # Node(
        #     package='px4_offboard',
        #     namespace='',
        #     executable='controlMul',  # Nodo del teclado
        #     prefix='gnome-terminal --',
        # )
          Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='control2',
            name='control2',
            prefix='gnome-terminal --',

        ),
           Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='go_to_point',
            name='go_to_point'
        ),
    ]

    # Nodo de control por cada dron
    for i in range(1, num_drones):
        nodes.append(
            Node(
                package='px4_offboard',
                executable='go_to_mul_points',
                namespace=f'px4_{i}',
                name=f'go_to_point_{i}',
                output='screen',
                # prefix='gnome-terminal --',
            )
        )
    

    return LaunchDescription(nodes)
