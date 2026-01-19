cmake_minimum_required(VERSION 3.8)
project(assignment2_rt_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Genera le interfacce dai file .srv
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetThreshold.srv"
)

# -----------------------------
# Nodo threshold server
# -----------------------------
add_executable(threshold_server src/threshold_service.cpp)
ament_target_dependencies(threshold_server rclcpp)
rosidl_target_interfaces(threshold_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
# -----------------------------
# Nodo UI (user input)
# -----------------------------
add_executable(UI src/ui_node.cpp)
ament_target_dependencies(UI rclcpp geometry_msgs)

# -----------------------------
# Nodo Bridge (pubblica cmd_vel)
# -----------------------------
add_executable(user_twist_publisher src/user_twist_publisher.cpp)
ament_target_dependencies(user_twist_publisher rclcpp geometry_msgs)

# Installazione degli eseguibili
install(TARGETS
  UI
  user_twist_publisher
  threshold_server
  DESTINATION lib/${PROJECT_NAME})

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
