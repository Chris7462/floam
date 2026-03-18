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
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(floam_core REQUIRED)

# lidar_processing_node
add_executable(lidar_processing_node
  src/lidar_processing_node.cpp
  src/lidar_processing.cpp
)

ament_target_dependencies(lidar_processing_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_include_directories(lidar_processing_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(lidar_processing_node
  floam_core::floam_core
  ${PCL_LIBRARIES}
)

target_compile_features(lidar_processing_node PUBLIC cxx_std_17)

target_compile_options(lidar_processing_node PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:
    -Wall
    -Wextra
    -Wpedantic
    -O3
  >
)

# odom_estimation_node
add_executable(odom_estimation_node
  src/odom_estimation_node.cpp
  src/odom_estimation.cpp
)

ament_target_dependencies(odom_estimation_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  message_filters
  pcl_conversions
)

target_include_directories(odom_estimation_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(odom_estimation_node
  floam_core::floam_core
  ${PCL_LIBRARIES}
)

target_compile_features(odom_estimation_node PUBLIC cxx_std_17)

target_compile_options(odom_estimation_node PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:
    -Wall
    -Wextra
    -Wpedantic
    -O3
  >
)

# lidar_mapping_node
add_executable(lidar_mapping_node
  src/lidar_mapping_node.cpp
  src/lidar_mapping.cpp
)

ament_target_dependencies(lidar_mapping_node
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
)

target_include_directories(lidar_mapping_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(lidar_mapping_node
  floam_core::floam_core
  ${PCL_LIBRARIES}
)

target_compile_features(lidar_mapping_node PUBLIC cxx_std_17)

target_compile_options(lidar_mapping_node PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:
    -Wall
    -Wextra
    -Wpedantic
    -O3
  >
)

install(TARGETS lidar_processing_node odom_estimation_node lidar_mapping_node
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
