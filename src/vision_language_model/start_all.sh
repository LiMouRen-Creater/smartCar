#!/bin/bash
# vision_language_model 一键启动脚本
# 清理旧 ROS 环境，只使用 ROS2 Humble

# 使用方式：先设置你的 API 密钥，再运行
# export ARK_API_KEY="你的API密钥"
# bash start_all.sh

# 清理旧的 ROS 环境变量，避免 ROS1/ROS2 冲突
unset ROS_DISTRO
unset ROS_PACKAGE_PATH
unset ROS_MASTER_URI

# 只 source ROS2 Humble
source /opt/ros/humble/setup.bash

# source 工作空间
cd /home/ubuntu2204/smartCar
source install/setup.bash

# 检查 API 密钥
if [ -z "$ARK_API_KEY" ]; then
    echo "错误: 请先设置 ARK_API_KEY 环境变量！"
    echo "用法: export ARK_API_KEY='你的API密钥'"
    echo "然后重新运行: bash start_all.sh"
    exit 1
fi

echo "==================================="
echo "  vision_language_model 启动"
echo "==================================="
echo "WSL IP: $(hostname -I)"
echo "API Key: ${ARK_API_KEY:0:8}... (已设置)"
echo ""
echo "终端1 - WebSocket 桥接 (端口 8888)"
echo "终端2 - VLM 模型推理节点"
echo "==================================="
echo ""

# 启动两个节点（后台运行）
echo "启动 WebSocket 图片桥接..."
ros2 run vision_language_model ws_image_bridge &
PID_BRIDGE=$!
sleep 2

echo "启动视觉语言模型节点..."
ros2 run vision_language_model vision_language_model &
PID_VLM=$!

echo ""
echo "两个节点已启动！"
echo "  ws_image_bridge (PID: $PID_BRIDGE) → 等待 Windows 端连接"
echo "  vision_language_model (PID: $PID_VLM) → 处理图片并发布结果"
echo ""
echo "请在 Windows 端运行: python win_cam.py"
echo "  (确保 WS_URL = ws://$(hostname -I | awk '{print $1}'):8888)"
echo ""
echo "按 Ctrl+C 停止所有节点"

# 等待任意一个进程结束
wait