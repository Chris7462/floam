cmake_minimum_required(VERSION 3.16)
project(floam)

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
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(g2o REQUIRED)
find_package(floam_core REQUIRED)

# laser_processing_node
add_executable(laser_processing_node
  src/laser_processing_node.cpp
  src/laser_processing.cpp
)

ament_target_dependencies(laser_processing_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_include_directories(laser_processing_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(laser_processing_node
  floam_core::floam_core
  ${PCL_LIBRARIES}
)

target_compile_features(laser_processing_node PUBLIC cxx_std_17)

target_compile_options(laser_processing_node PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:
    -Wall
    -Wextra
    -Wpedantic
    -O3
  >
)

# odom_estimation_node
# add_executable(odom_estimation_node
#   src/odom_estimation_node.cpp
#   src/odom_estimation.cpp
#   src/odom_estimation_class.cpp
#   src/lidar_optimization.cpp
#   src/lidar.cpp
# )
#
# ament_target_dependencies(odom_estimation_node
#   rclcpp
#   sensor_msgs
#   nav_msgs
#   geometry_msgs
#   tf2
#   tf2_ros
#   pcl_conversions
#   Eigen3
# )
#
# target_include_directories(odom_estimation_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${PCL_INCLUDE_DIRS}
# )
#
# target_link_libraries(odom_estimation_node
#   ${PCL_LIBRARIES}
#   Sophus::Sophus
#   g2o::core
#   g2o::solver_dense
# )
#
# target_compile_features(odom_estimation_node PUBLIC cxx_std_17)

# laser_mapping_node
# add_executable(laser_mapping_node
#   src/laser_mapping_node.cpp
#   src/laser_mapping.cpp
#   src/laser_mapping_class.cpp
#   src/lidar.cpp
# )
#
# ament_target_dependencies(laser_mapping_node
#   rclcpp
#   sensor_msgs
#   nav_msgs
#   pcl_conversions
#   pcl_ros
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
#   Eigen3
# )
#
# target_include_directories(laser_mapping_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${PCL_INCLUDE_DIRS}
# )
#
# target_link_libraries(laser_mapping_node
#   ${PCL_LIBRARIES}
# )
#
# target_compile_features(laser_mapping_node PUBLIC cxx_std_17)

install(TARGETS laser_processing_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
