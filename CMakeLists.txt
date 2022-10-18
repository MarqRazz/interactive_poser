cmake_minimum_required(VERSION 3.8)
project(interactive_poser)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_calibration_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_index_cpp
    rclcpp
    robot_calibration_msgs
    sensor_msgs
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    interactive_markers
    visualization_msgs
    yaml-cpp
)

# ----------------------------------
add_library(interactive_poser_lib SHARED 
  src/interactive_poser.cpp
  src/poser.cpp
  src/camera_poser.cpp
  src/pointcloud_camera_poser.cpp
  # src/joint_pose_from_yaml.cpp
)
target_include_directories(interactive_poser_lib
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>)
ament_target_dependencies(interactive_poser_lib ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(interactive_poser_lib)
target_compile_options(interactive_poser_lib PRIVATE -Wall -Wextra)

# ----------------------------------
add_executable(interactive_poser_node src/interactive_poser_node.cpp)
target_include_directories(interactive_poser_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(interactive_poser_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(interactive_poser_node
  interactive_poser_lib)

# ----------------------------------
# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)
# Install libraries
install(
  TARGETS
    interactive_poser_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
# Install executables
install(
  TARGETS
    interactive_poser_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# Install directories
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}
# )

# ----------------------------------
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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
