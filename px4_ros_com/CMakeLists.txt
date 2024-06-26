cmake_minimum_required(VERSION 3.5)
project(px4_ros_com)

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
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/TargetFRD.msg"
)

set(px4_msgs_DIR /home/ali/ws_px4_msgs/build/px4_msgs/ament_cmake_core)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

#################
# Setup targets #
#################

include_directories(include)

# Add frame_transforms lib
add_library(frame_transforms SHARED src/lib/frame_transforms.cpp)
ament_target_dependencies(frame_transforms Eigen3 geometry_msgs sensor_msgs)
target_include_directories(frame_transforms PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)


# examples/listeners/sensor_combined_listener
##add_executable(sensor_combined_listener src/examples/listeners/sensor_combined_listener.cpp)
#ament_target_dependencies(sensor_combined_listener rclcpp px4_msgs)
#install(TARGETS sensor_combined_listener DESTINATION lib/${PROJECT_NAME})

# examples/listeners/vehicle_gps_position_listener
#add_executable(vehicle_gps_position_listener src/examples/listeners/vehicle_gps_position_listener.cpp)
#ament_target_dependencies(vehicle_gps_position_listener rclcpp px4_msgs)
#install(TARGETS vehicle_gps_position_listener DESTINATION lib/${PROJECT_NAME})

# examples/advertisers/debug_vect_advertiser
#add_executable(debug_vect_advertiser src/examples/advertisers/debug_vect_advertiser.cpp)
#ament_target_dependencies(debug_vect_advertiser rclcpp px4_msgs)
#install(TARGETS debug_vect_advertiser DESTINATION lib/${PROJECT_NAME})


# examples/only_takeoff/only_takeoff
add_executable(only_takeoff src/examples/only_takeoff/only_takeoff.cpp)
ament_target_dependencies(only_takeoff rclcpp px4_msgs)
install(TARGETS only_takeoff DESTINATION lib/${PROJECT_NAME})

# examples/offboard/attitude_command
add_executable(attitude_command src/examples/offboard/attitude_command.cpp)
ament_target_dependencies(attitude_command rclcpp px4_msgs)
install(TARGETS attitude_command DESTINATION lib/${PROJECT_NAME})

# examples/offboard/velocity_command
add_executable(velocity_command src/examples/offboard/velocity_command.cpp)
ament_target_dependencies(velocity_command rclcpp px4_msgs)
install(TARGETS velocity_command DESTINATION lib/${PROJECT_NAME})

# examples/offboard/position_command
add_executable(position_command src/examples/offboard/position_command.cpp)
ament_target_dependencies(position_command rclcpp px4_msgs)
install(TARGETS position_command DESTINATION lib/${PROJECT_NAME})

# examples/listener/relative_position
add_executable(relative_position src/examples/listener/relative_position.cpp)
ament_target_dependencies(relative_position rclcpp px4_msgs)
install(TARGETS relative_position DESTINATION lib/${PROJECT_NAME})

# examples/controller/control
add_executable(control src/examples/controller/control.cpp)
ament_target_dependencies(control rclcpp px4_msgs)
install(TARGETS control DESTINATION lib/${PROJECT_NAME})

rosidl_target_interfaces(relative_position
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
  
rosidl_target_interfaces(control
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs)

ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(frame_transforms)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(TARGETS frame_transforms
	EXPORT export_frame_transforms
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install tests
#install(DIRECTORY test DESTINATION share/${PROJECT_NAME}/)


############
# Testing ##
############

#if(BUILD_TESTING)
#	find_package(ament_lint_auto REQUIRED)
	# the following line skips the linter which checks for copyrights
	# uncomment the line when a copyright and license is not present in all source files
	#set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# uncomment the line when this package is not in a git repo
	#set(ament_cmake_cpplint_FOUND TRUE)
#	ament_lint_auto_find_test_dependencies()
#endif()

ament_package()
