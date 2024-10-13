from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zva_sd0_kisbeadando',
            executable='gen_node',
        ),
        Node(
            package='zva_sd0_kisbeadando',
            executable='sum_node',
        ),
    ])