cmake_minimum_required(VERSION 3.8)
project(mpc_controller)

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
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(simple_controller REQUIRED)
find_package(tf2 REQUIRED)
find_package(angles REQUIRED)
find_package(ACADO REQUIRED)


include_directories(
  include/mpc_controller
  ${ACADO_INCLUDE_DIR}
  /usr/local/include/acado  # 添加这行，路径根据实际情况调整
)

add_executable(mpc_controller
  src/mpc_controller.cpp
  src/mpc_controller_node.cpp
  src/mpc.cpp
)

# 链接 ACADO 库
target_link_libraries(mpc_controller
  ${ACADO_SHARED_LIBRARIES}
)

ament_target_dependencies(mpc_controller 
  rclcpp 
  nav_msgs 
  sensor_msgs 
  tf2 
  angles 
  simple_controller
)

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

install(TARGETS
  mpc_controller
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
