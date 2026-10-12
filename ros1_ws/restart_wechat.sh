#!/bin/bash
# kill 任何残留 wechat 进程
pkill -9 -f wechat_interaction_cloud_ai.py 2>/dev/null
sleep 1
echo "after kill:"
ps -ef | grep wechat | grep -v grep

# 后台启动 wechat 节点
export LD_PRELOAD=/home/ubuntu/.local/lib/python3.8/site-packages/torch.libs/libgomp-804f19d4.so.1.0.0
source /opt/ros/noetic/setup.bash
source /home/ubuntu/puppypi/devel/setup.bash
cd /home/ubuntu/puppypi/src/puppy_extend_demo/scripts

PYTHONUNBUFFERED=1 nohup python3 -u wechat_interaction_cloud_ai.py \
  --config /home/ubuntu/puppypi/src/puppy_extend_demo/config/wechat_config.yaml \
  > /tmp/wecom_node.log 2>&1 &
echo "started PID=$!"
sleep 6
echo "after start:"
ps -ef | grep wechat | grep -v grep
echo "=== log tail ==="
tail -30 /tmp/wecom_node.log
echo "=== 8088 listener ==="
ss -tlnp 2>/dev/null | grep 8088 || echo "no 8088 listener"
