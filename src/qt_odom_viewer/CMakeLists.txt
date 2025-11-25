cmake_minimum_required(VERSION 3.10)
project(qt_odom_viewer VERSION 0.1 LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 QT5 依赖（针对Qt 5.15.3优化）
find_package(Qt5 5.15.3 REQUIRED COMPONENTS Core Widgets Gui Concurrent)
set(QT_LIBRARIES Qt5::Core Qt5::Widgets Qt5::Gui Qt5::Concurrent)

# Qt自动处理设置
set(CMAKE_AUTOMOC ON)      # 自动处理QT MOC
set(CMAKE_AUTOUIC ON)      # 自动处理UI文件
set(CMAKE_AUTORCC ON)      # 自动处理资源文件
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 设置UI文件搜索路径
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# 查找 ROS2 依赖
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcutils REQUIRED)

# 设置源文件和头文件
set(PROJECT_SOURCES
  src/main.cpp
  src/odom_viewer_window_ui.cpp
  src/trajectory_frame.cpp
  src/dashboard_frame.cpp
  ui/odom_viewer_window.ui
  include/qt_odom_viewer/odom_viewer_window_ui.h
  include/qt_odom_viewer/odom_subscriber.h
  include/qt_odom_viewer/trajectory_frame.h
  include/qt_odom_viewer/dashboard_frame.h
)

# 创建可执行文件
add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

# 链接依赖库
target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rcutils::rcutils
)

# 设置包含目录
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 安装可执行文件
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# ament 配置
ament_package()
