from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    planner_params = os.path.join(
    get_package_share_directory('smooth_local_planner'), 'config', 'planner_params.yaml')

    planner_node = Node(
            package='smooth_local_planner',
            executable='smooth_local_planner',
            name='smooth_local_planner',
            parameters=[planner_params],
            output='screen'
        )

    rviz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('smooth_local_planner'), 'config', 'rviz.rviz')]
        )

    return LaunchDescription([
        planner_node,
        rviz
    ])