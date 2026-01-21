from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo twist publisher
        Node(
            package='assignment2_rt_cpp',
            executable='user_twist_publisher',
            name='user_twist_publisher',
            output='screen'
        ),

        # Nodo threshold server
        Node(
            package='assignment2_rt_cpp',
            executable='threshold_server',
            name='threshold_server',
            output='screen'
        ),

        # Nodo distance_check (Python)
        Node(
            package='assignment2_rt_py',
            executable='distance_check',
            name='distance_check',
            output='screen'
        ),
    ])