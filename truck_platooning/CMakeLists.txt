cmake_minimum_required(VERSION 3.8)
project(truck_platooning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ───── Dependencies ─────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# ───── Include Paths ─────
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# ───── Source Files ─────
set(SOURCES
  src/lane_following_node.cpp
  src/command_publisher.hpp
  src/lane_detect.hpp
  src/pid_controller.hpp
  src/distance_sensor.hpp
  src/platooning_manager.hpp
)

# ───── Executable Node ─────
add_executable(lane_following_node ${SOURCES})

# ───── Dependencies for Executable ─────
ament_target_dependencies(lane_following_node
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  cv_bridge
  image_transport
)

target_link_libraries(lane_following_node ${OpenCV_LIBS})

# ───── Install Targets ─────
install(TARGETS
  lane_following_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/carla_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# ───── Testing (Optional) ─────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
