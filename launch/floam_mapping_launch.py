from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
  params = join(
    get_package_share_directory("floam"), "params", "floam_params.yaml"
  )

  lidar_processing_node = Node(
    package="floam",
    executable="lidar_processing_node",
    name="lidar_processing_node",
    parameters=[params]
  )

  odom_estimation_node = Node(
    package="floam",
    executable="odom_estimation_node",
    name="odom_estimation_node",
    parameters=[params]
  )

  lidar_mapping_node = Node(
    package="floam",
    executable="lidar_mapping_node",
    name="lidar_mapping_node",
    parameters=[params]
  )

  bag_exec = ExecuteProcess(
    cmd=["ros2", "bag", "play", "-r", "0.85", "/data/kitti/raw/2011_09_30_drive_0018_sync_bag" , "--topics", "/kitti/velo", "/kitti/camera/color/left/image_raw", "--clock"]
  )

  rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    arguments=["-d", join(get_package_share_directory("floam"), "rviz/", "floam_mapping.rviz")]
  )

  return LaunchDescription([
    lidar_processing_node,
    odom_estimation_node,
    lidar_mapping_node,
    bag_exec,
    rviz_node
  ])
