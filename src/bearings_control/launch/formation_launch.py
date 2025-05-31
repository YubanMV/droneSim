from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = []
    for i in range(1, 9):  # Drones 1 a 5
        nodes.append(
            Node(
                package='bearings_control',
                executable='bearings_3d_control_node',
                name=f'bearings_3d_control_node_{i}',
                output='screen',
                arguments=[str(i)]
            )
        )
    return LaunchDescription(nodes)
