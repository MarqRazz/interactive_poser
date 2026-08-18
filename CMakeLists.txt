cmake_minimum_required(VERSION 3.8)
project(interactive_poser)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    sensor_msgs
    geometry_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    interactive_markers
    visualization_msgs
    std_srvs
    std_msgs
    urdf
)

# Parameter declaration, validation and the rqt_reconfigure ranges are all
# generated from src/interactive_poser_parameters.yaml.
generate_parameter_library(interactive_poser_parameters
  src/interactive_poser_parameters.yaml
)

add_library(interactive_poser_lib SHARED src/interactive_poser.cpp)
target_include_directories(interactive_poser_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(interactive_poser_lib ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(interactive_poser_lib
  interactive_poser_parameters
  ${YAML_CPP_LIBRARIES}
)

add_executable(interactive_poser_node src/interactive_poser_node.cpp)
target_link_libraries(interactive_poser_node interactive_poser_lib)
ament_target_dependencies(interactive_poser_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS interactive_poser_lib interactive_poser_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  TARGETS interactive_poser_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} generate_parameter_library yaml-cpp)
ament_package()
