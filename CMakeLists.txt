cmake_minimum_required(VERSION 3.8)
project(papf_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # <-- ADD THIS LINE
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# Add your core planner logic as a library
add_library(papf_logic src/PAPF_Planner.cpp)
target_include_directories(papf_logic PUBLIC include)

# Add the executable for the ROS node
add_executable(papf_node src/papf_node.cpp)
ament_target_dependencies(papf_node
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "std_msgs"
  "visualization_msgs"
  "sensor_msgs"  # <-- ADD THIS LINE
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

# Link the node executable against your planner library
target_link_libraries(papf_node papf_logic)

install(TARGETS
  papf_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the racelines directory
install(
  DIRECTORY racelines
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()