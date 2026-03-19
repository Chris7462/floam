# floam

![Version](https://img.shields.io/badge/version-1.0.0-blue)
![License](https://img.shields.io/badge/license-Apache--2.0-green)
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)

This package provides three `rclcpp` nodes that wrap the
[`floam_core`](https://github.com/Chris7462/floam_core) pure C++ library,
handling all ROS2 communication (subscriptions, publishers, TF broadcasting,
parameter loading) while delegating all algorithm logic to `floam_core`.

---

## Nodes

### `lidar_processing_node`

Subscribes to a raw LiDAR point cloud, filters it by distance and scan line,
and extracts edge and surface feature clouds.

#### Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `kitti/velo` *(configurable)* | `sensor_msgs/msg/PointCloud2` | Raw LiDAR point cloud |

#### Published Topics

| Topic | Type | Description |
|---|---|---|
| `lidar_cloud_filtered` | `sensor_msgs/msg/PointCloud2` | Distance-filtered point cloud |
| `lidar_cloud_edge` | `sensor_msgs/msg/PointCloud2` | Edge feature points |
| `lidar_cloud_surf` | `sensor_msgs/msg/PointCloud2` | Surface feature points |

---

### `odom_estimation_node`

Synchronizes edge and surface feature clouds, matches them against a local map,
and estimates the LiDAR odometry using Ceres Solver.

#### Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `lidar_cloud_edge` | `sensor_msgs/msg/PointCloud2` | Edge feature points |
| `lidar_cloud_surf` | `sensor_msgs/msg/PointCloud2` | Surface feature points |

#### Published Topics

| Topic | Type | Description |
|---|---|---|
| `odom` | `nav_msgs/msg/Odometry` | LiDAR odometry estimate |
| `odom_path` | `nav_msgs/msg/Path` | Accumulated odometry path |
| TF `map` → `base_link` | `tf2` | Transform broadcast |


---

### `lidar_mapping_node`

Synchronizes filtered point clouds with odometry estimates and accumulates a
voxel-filtered 3D map.

#### Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `lidar_cloud_filtered` | `sensor_msgs/msg/PointCloud2` | Filtered point cloud |
| `odom` | `nav_msgs/msg/Odometry` | Odometry estimate |

#### Published Topics

| Topic | Type | Description |
|---|---|---|
| `map` | `sensor_msgs/msg/PointCloud2` | Accumulated 3D point cloud map |

---

## Dependencies

| Dependency | Notes |
|---|---|
| ROS2 (Jazzy) | [Installation](https://docs.ros.org/en/jazzy/Installation.html) |
| floam_core | [floam_core](https://github.com/Chris7462/floam_core) |
| PCL | `sudo apt-get install libpcl-all-dev` |
| Eigen3 | `sudo apt-get install libeigen3-dev` |
| Ceres Solver | `sudo apt-get install libceres-dev` |
| pcl_conversions | ROS2 package |
| tf2, tf2_ros, tf2_eigen | ROS2 packages |
| message_filters | ROS2 package |

---

## Building

Clone both `floam_core` and `floam` into your ROS2 workspace and build together:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Chris7462/floam_core.git
git clone https://github.com/Chris7462/floam.git
cd ~/ros2_ws
colcon build --packages-select floam_core floam
source install/setup.bash
```

---

## Usage

All parameters are configured in `params/floam_params.yaml`. Edit this file
to match your LiDAR sensor before launching.

### Launch odometry only

```bash
ros2 launch floam floam_launch.py
```

### Launch with RViz and rosbag playback

```bash
ros2 launch floam floam_rviz_launch.py
```

---

## Related Packages

**[floam_core](https://github.com/Chris7462/floam_core)** — The ROS-agnostic pure
C++ algorithm library that this package wraps.

---

## Reference

**[F-LOAM: Fast LiDAR Odometry and Mapping](https://github.com/wh200720041/floam)** — The original implementation by Wang Han, from which `floam_core` was refactored. Uses `catkin`, C++14, and Ceres Solver.
