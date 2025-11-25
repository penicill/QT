cmake_minimum_required(VERSION 3.8)
project(sim_odom_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# 构建库
add_library(${PROJECT_NAME} SHARED
  src/sim_odom_publisher.cpp
)

# 设置库的包含目录
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接依赖库
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  rclcpp_components
)

# 注册组件节点
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "SimOdomPublisher"
  EXECUTABLE ${PROJECT_NAME}_node
)

# 构建可执行文件
add_executable(sim_odom_main src/main.cpp)
target_link_libraries(sim_odom_main ${PROJECT_NAME})

# 安装目标
install(TARGETS
  ${PROJECT_NAME}
  sim_odom_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 测试（可选）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
