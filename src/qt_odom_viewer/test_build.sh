#!/bin/bash

echo "开始测试 Qt Odom Viewer 编译..."

# 创建构建目录
mkdir -p build
cd build

# 配置构建
echo "配置 CMake..."
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

if [ $? -ne 0 ]; then
    echo "CMake 配置失败，请检查依赖！"
    exit 1
fi

# 编译项目
echo "开始编译..."
make -j$(nproc)

if [ $? -ne 0 ]; then
    echo "编译失败，请检查错误信息！"
    exit 1
fi

echo "编译成功！"
echo "生成的可执行文件在: build/qt_odom_viewer"