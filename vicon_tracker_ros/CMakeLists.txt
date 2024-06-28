cmake_minimum_required(VERSION 3.8)
project(vicon_tracker_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Specifica il percorso di installazione di px4_msgs
set(px4_msgs_DIR "ghjko/vicon_tracker_ros/src/px4_msgs")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

# vicon_tracker_client executable
add_executable(vicon_tracker_client src/vicon_tracker_client.cpp)
ament_target_dependencies(vicon_tracker_client rclcpp geometry_msgs tf2_ros)

target_compile_features(vicon_tracker_client PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(vicon_tracker_client ViconDataStreamSDK_CPP ${Boost_LIBRARIES})

install(TARGETS vicon_tracker_client DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}/)

# vicon_to_vehicle_odometry_node executable
add_executable(vicon_to_vehicle_odometry_node src/vicon_to_vehicle_odometry_node
