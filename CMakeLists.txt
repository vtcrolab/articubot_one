cmake_minimum_required(VERSION 3.5)
project(articubot_one)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(serial REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Add include directories for dependencies
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${image_geometry_INCLUDE_DIRS}
)

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

add_executable(joy_serial src/joy_serial.cpp)
ament_target_dependencies(joy_serial rclcpp sensor_msgs serial)
install(
  TARGETS joy_serial
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(depth_to_laserscan src/depth_to_laserscan.cpp)
ament_target_dependencies(depth_to_laserscan rclcpp sensor_msgs cv_bridge image_geometry OpenCV)
install(
  TARGETS depth_to_laserscan
  DESTINATION lib/${PROJECT_NAME}
)

# ADDED, one is for color detection and the other is for depth of the camera

add_executable(color_detection_node src/color_detection_node.cpp)
ament_target_dependencies(color_detection_node rclcpp sensor_msgs cv_bridge image_geometry OpenCV)
install(
  TARGETS color_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(depth_processing_node src/depth_processing_node.cpp)
ament_target_dependencies(depth_processing_node rclcpp sensor_msgs cv_bridge image_geometry OpenCV)
install(
  TARGETS depth_processing_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
