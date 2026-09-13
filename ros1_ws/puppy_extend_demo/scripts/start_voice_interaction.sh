#!/bin/bash
# 开机自启动包装：等 roscore 起来后启动云端 AI 语音交互节点

set -u

# 1. 加载 ROS 与工作空间环境
source /opt/ros/noetic/setup.bash
source /home/pi/puppy_pi/devel/setup.bash

# 2. 等待 roscore 就绪（最多 60 秒）
TIMEOUT=60
ELAPSED=0
while ! rostopic list >/dev/null 2>&1; do
    sleep 1
    ELAPSED=$((ELAPSED + 1))
    if [ "$ELAPSED" -ge "$TIMEOUT" ]; then
        echo "[start_voice_interaction] roscore 未在 ${TIMEOUT}s 内起来，放弃启动" >&2
        exit 1
    fi
done

# 3. 启动语音交互节点（崩溃即退出，不自愈）
exec rosrun puppy_extend_demo voice_interaction_cloud_ai2.py
