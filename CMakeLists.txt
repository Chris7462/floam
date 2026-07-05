cmake_minimum_required(VERSION 3.20)

project(floam
  VERSION 1.0.0
  DESCRIPTION "FLOAM ROS2 node wrapper around floam_core"
  LANGUAGES C CXX)  # C is needed for pcl_conversions

# Default to a Release build if the user doesn't specify one
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type specified, defaulting to 'Release'.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)
find_package(floam_core REQUIRED)

# lidar_processing_node
add_executable(lidar_processing_node
  src/lidar_processing_node.cpp
  src/lidar_processing.cpp)

target_include_directories(lidar_processing_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(lidar_processing_node PRIVATE
  ${rclcpp_TARGETS}
  ${sensor_msgs_TARGETS}
  ${pcl_conversions_TARGETS}
  pcl_common
  ${floam_core_TARGETS})

target_compile_features(lidar_processing_node PRIVATE cxx_std_17)  # Require C++17

target_compile_options(lidar_processing_node PRIVATE
  -Wall
  -Wextra
  -Wpedantic)

# odom_estimation_node
add_executable(odom_estimation_node
  src/odom_estimation_node.cpp
  src/odom_estimation.cpp)

target_include_directories(odom_estimation_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(odom_estimation_node PRIVATE
  ${rclcpp_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
  ${tf2_eigen_TARGETS}
  ${message_filters_TARGETS}
  ${pcl_conversions_TARGETS}
  pcl_common
  Eigen3::Eigen
  ${floam_core_TARGETS})

target_compile_features(odom_estimation_node PRIVATE cxx_std_17)  # Require C++17

target_compile_options(odom_estimation_node PRIVATE
  -Wall
  -Wextra
  -Wpedantic)

# lidar_mapping_node
add_executable(lidar_mapping_node
  src/lidar_mapping_node.cpp
  src/lidar_mapping.cpp)

target_include_directories(lidar_mapping_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(lidar_mapping_node PRIVATE
  ${rclcpp_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${tf2_eigen_TARGETS}
  ${message_filters_TARGETS}
  ${pcl_conversions_TARGETS}
  pcl_common
  Eigen3::Eigen
  ${floam_core_TARGETS})

target_compile_features(lidar_mapping_node PRIVATE cxx_std_17)  # Require C++17

target_compile_options(lidar_mapping_node PRIVATE
  -Wall
  -Wextra
  -Wpedantic)

install(TARGETS lidar_processing_node odom_estimation_node lidar_mapping_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
