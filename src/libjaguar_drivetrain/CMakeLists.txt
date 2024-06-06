cmake_minimum_required(VERSION 3.8)
project(libjaguar_drivetrain C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(libjaguar)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

add_executable(jaguar_twist_drivetrain_node src/jaguar_twist_drivetrain.cpp ./libjaguar/src/libjaguar.c ./libjaguar/src/canutil.c)
ament_target_dependencies(jaguar_twist_drivetrain_node rclcpp std_msgs geometry_msgs)
target_include_directories(jaguar_twist_drivetrain_node PUBLIC ./libjaguar/include)

target_compile_definitions(jaguar_twist_drivetrain_node PUBLIC
	-DCANDRIVER_SERIAL=1
)

install(TARGETS
  jaguar_twist_drivetrain_node
  DESTINATION lib/${PROJECT_NAME})
