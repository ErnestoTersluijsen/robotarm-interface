cmake_minimum_required(VERSION 3.8)
project(robotarm_hld)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robotarm_lld REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# set message files
set(action_files
  "action/PositionPreset.action"
  "action/ServoPositions.action"
)

# generate message files
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(robotarm_interface
	src/main.cpp
	src/RosInterface.cpp
	src/Position.cpp
	src/RobotarmHLD.cpp
)

target_include_directories(robotarm_interface PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robotarm_interface
	rclcpp
	rclcpp_action
	robotarm_lld
)

# install application
install(TARGETS robotarm_interface
	DESTINATION lib/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(robotarm_interface "${cpp_typesupport_target}")

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
