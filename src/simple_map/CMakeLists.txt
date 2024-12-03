# CMakeLists.txt 修改如下
cmake_minimum_required(VERSION 3.5)
project(simple_map)

# 设置默认的 C 和 C++ 标准
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找必要的依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) # 确保包含 sensor_msgs
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 包含头文件目录
include_directories(
  include
)

# 添加源文件和生成可执行文件
add_executable(simple_map_node src/simple_map.cpp)

# 链接库
ament_target_dependencies(simple_map_node
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs 
  tf2
  tf2_ros
)

# 安装目标
install(TARGETS simple_map_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装目录（如配置文件或 launch 文件）
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
)
# 声明包
ament_package()



