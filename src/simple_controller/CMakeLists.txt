cmake_minimum_required(VERSION 3.5)
project(simple_controller)

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

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(angles REQUIRED)

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

ament_package()

add_executable(controller_node src/controller.cpp
                                src/controller_node.cpp)

include_directories(
  include/simple_controller
  ${rclcpp_INCLUDE_DIRS}
)
ament_target_dependencies(controller_node 
rclcpp
nav_msgs
tf2 geometry_msgs
std_msgs
angles)

add_executable(velocity_publisher src/velocity_publisher.cpp)
ament_target_dependencies(velocity_publisher rclcpp std_msgs)
install(TARGETS velocity_publisher DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME})

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# install(TARGETS start_rqt
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )
# 安装 start_rqt 脚本
install(PROGRAMS scripts/start_rqt
  DESTINATION lib/${PROJECT_NAME}
)
# 安装 rqt 配置文件
install(DIRECTORY rqt_cfg/
  DESTINATION share/${PROJECT_NAME}/rqt_cfg
)
