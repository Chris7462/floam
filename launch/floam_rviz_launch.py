from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('floam')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (bagfile) clock if true'
    )

    bag_exec = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-r', '1.0',
             '/data/kitti/raw/2011_09_30_drive_0018_sync_bag',
             '--clock',
             '--topics', '/kitti/velo', '/kitti/camera/color/left/image_raw',
             '--qos-profile-overrides-path',
             join(pkg_share, 'config', 'qos_override_offline.yaml')]
    )

    floam_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_share, 'launch', 'floam_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', join(pkg_share, 'rviz', 'floam.rviz')]
    )

    return LaunchDescription([
        declare_use_sim_time,
        floam_mapping_launch,
        rviz_node,
        TimerAction(
            period=3.0,
            actions=[
                bag_exec
            ]
        )
    ])
