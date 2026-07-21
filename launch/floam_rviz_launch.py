from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('floam')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (bagfile/CARLA) clock if true'
    )

    declare_input_topic = DeclareLaunchArgument(
        'input_topic',
        description='Input lidar point cloud topic name. Required - no '
                     'default. Passed through to floam_launch.py, which '
                     'will refuse to start lidar_processing_node if this '
                     'is not provided.'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        description='Path to the .rviz config file. The camera Image '
                     'display topic is baked into this file, so it must '
                     'match the data source (KITTI vs CARLA vs ...). '
                     'Required - no default, since a mismatched RViz Image '
                     'topic fails silently (blank image) rather than '
                     'erroring.'
    )

    floam_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_share, 'launch', 'floam_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': LaunchConfiguration('input_topic'),
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_input_topic,
        declare_rviz_config,
        floam_mapping_launch,
        rviz_node
    ])
