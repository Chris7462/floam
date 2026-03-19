from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    params = join(
        get_package_share_directory('floam'), 'params',
        'floam_params.yaml'
    )

    lidar_processing_node = Node(
        package='floam',
        executable='lidar_processing_node',
        name='lidar_processing_node',
        output='screen',
        parameters=[
            params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    odom_estimation_node = Node(
        package='floam',
        executable='odom_estimation_node',
        name='odom_estimation_node',
        output='screen',
        parameters=[
            params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    lidar_mapping_node = Node(
        package='floam',
        executable='lidar_mapping_node',
        name='lidar_mapping_node',
        output='screen',
        parameters=[
            params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        lidar_processing_node,
        odom_estimation_node,
        lidar_mapping_node
    ])
