#!/bin/bash
# 拉 voice 节点进程的 DASHSCOPE_API_KEY
docker exec -u root puppypi bash <<'EOF'
PID=$(ps -ef | grep voice_interaction_cloud_ai2 | grep -v grep | head -1 | awk '{print $2}')
echo "voice_PID=$PID"
if [ -n "$PID" ]; then
  cat /proc/$PID/environ 2>/dev/null | tr '\0' '\n' | grep -i dash
fi
EOF
