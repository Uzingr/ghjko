cmake_minimum_required(VERSION 3.8)
project(vicon_tracker_ros)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED PATHS "/ghjko/vicon_tracker_ros/src/px4_msgs/install/share/px4_msgs/cmake")

# vicon_tracker_client executable
add_executable(vicon_tracker_client src/vicon_tracker_client.cpp)
ament_target_dependencies(vicon_tracker_client rclcpp geometry_msgs tf2_ros)

target_compile_features(vicon_tracker_client PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(vicon_tracker_client ViconDataStreamSDK_CPP ${Boost_LIBRARIES})

install(TARGETS vicon_tracker_client DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}/)

# vicon_to_px4 executable
add_executable(vicon_to_odometry_node src/vicon_to_odometry_node.cpp)
ament_target_dependencies(vicon_to_odometry_node rclcpp geometry_msgs px4_msgs)

target_compile_features(vicon_to_odometry_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS vicon_to_odometry_node DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp 
                          px4_msgs geometry_msgs geometry_msgs tf2_ros nav_msgs
                          )

ament_export_include_directories(include)

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

############
# Testing ##
############

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
