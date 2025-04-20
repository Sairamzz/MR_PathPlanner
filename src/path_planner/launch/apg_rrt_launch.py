from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_planner',
            executable='apg_rrt_node',
            name='apg_rrt_planner',
            output='screen'
        )
    ])
