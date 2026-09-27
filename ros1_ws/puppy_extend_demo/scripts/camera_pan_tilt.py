#!/usr/bin/env python3
# coding=utf8
# 云台摄像头控制节点（Pan-Tilt Camera Control）
# Pan: FEETECH FT6335M 360°位置式数字舵机（PWM端口9），磁编码器
# Tilt: 暂未接入

import sys
import time
import rospy
import threading

sys.path.append('/home/ubuntu/software/puppypi_control')
from ros_robot_controller_sdk import Board
from pwm_servo_control import PWMServoControl
from std_msgs.msg import Float32
from std_srvs.srv import Trigger, TriggerResponse

PAN_SERVO_ID = 9     # 水平旋转（FT6335M，PWM端口9）
TILT_SERVO_ID = None # 俯仰暂未接入，后续接入后改为对应PWM端口号

# FT6335M 标定参数
# pulse 500-2500μs 对应 0-360°，中心 180° 对应 pulse 1500
PULSE_MIN = 500
PULSE_MAX = 2500
PULSE_CENTER = 1500
ANGLE_MIN = 0.0      # 0°（顺时针极端，按实测方向定）
ANGLE_MAX = 360.0    # 360°
ANGLE_CENTER = 180.0 # 中心位置
PULSE_PER_DEG = (PULSE_MAX - PULSE_MIN) / (ANGLE_MAX - ANGLE_MIN)  # ≈5.556μs/°

SWEEP_DEFAULT_STEPS = 12
SWEEP_SETTLE_TIME = 0.5  # 每步到位后等待画面稳定

board = Board()
psc = PWMServoControl(board)
lock = threading.RLock()


def angle_to_pulse(angle):
    """角度→脉冲。angle ∈ [0, 360]，超出 clamp"""
    pulse = PULSE_CENTER + (angle - ANGLE_CENTER) * PULSE_PER_DEG
    return int(max(PULSE_MIN, min(PULSE_MAX, pulse)))


def set_pan_angle(angle):
    """设置水平旋转到指定绝对角度
    angle: 0.0 ~ 360.0，中心 180.0，超出 clamp
    """
    with lock:
        angle = max(ANGLE_MIN, min(ANGLE_MAX, angle))
        pulse = angle_to_pulse(angle)
        psc.setPulse(PAN_SERVO_ID, pulse, 50)


def stop_pan():
    """水平舵机回中心 180°"""
    with lock:
        psc.setPulse(PAN_SERVO_ID, PULSE_CENTER, 50)


def set_tilt_angle(angle):
    """设置俯仰角度（俯仰舵机暂未接入）
    angle: 0°(向上最大) ~ 90°(水平) ~ 180°(向下最大)
    """
    if TILT_SERVO_ID is None:
        rospy.logwarn("俯仰舵机未接入，忽略 tilt 指令")
        return
    with lock:
        pulse = int(500 + angle / 180.0 * 2000)
        pulse = max(PULSE_MIN, min(PULSE_MAX, pulse))
        psc.setPulse(TILT_SERVO_ID, pulse, 500)


def sweep_scan(steps=SWEEP_DEFAULT_STEPS):
    """360°全景扫描，分 steps 步定位，每步停留拍摄，最后一步回到起点
    steps: 分多少步（默认12步，每步30°，共13个定位点覆盖完整360°）
    """
    angle_per_step = (ANGLE_MAX - ANGLE_MIN) / steps
    total_positions = steps + 1  # 多一步从最后位置回到 360°(=0°)，闭合扫描
    rospy.loginfo("开始360°扫描，共%d个定位点，每段%.1f°", total_positions, angle_per_step)

    for i in range(total_positions):
        target = min(ANGLE_MAX, i * angle_per_step)
        rospy.loginfo("扫描步骤 %d/%d → %.1f°", i + 1, total_positions, target)
        set_pan_angle(target)
        time.sleep(SWEEP_SETTLE_TIME)

    rospy.loginfo("360°扫描完成")


# ============ ROS1 回调 ============

def pan_angle_cb(msg):
    angle = max(ANGLE_MIN, min(ANGLE_MAX, msg.data))
    set_pan_angle(angle)
    rospy.loginfo("Pan angle set to %.1f°", angle)


def tilt_angle_cb(msg):
    angle = max(0.0, min(180.0, msg.data))
    set_tilt_angle(angle)
    rospy.loginfo("Tilt angle set to %.1f", angle)


def sweep_cb(req):
    sweep_scan(SWEEP_DEFAULT_STEPS)
    return TriggerResponse(success=True, message="360 sweep done")


def stop_cb(req):
    stop_pan()
    rospy.loginfo("Pan stopped at center 180°")
    return TriggerResponse(success=True, message="pan stopped at center 180°")


# ============ 主函数 ============

def init_pan_tilt():
    """初始化云台到默认位置（中心 180°）"""
    rospy.loginfo("初始化云台：Pan 回中心 180°")
    stop_pan()
    time.sleep(0.2)


if __name__ == '__main__':
    rospy.init_node('camera_pan_tilt')

    init_pan_tilt()

    # 订阅话题
    rospy.Subscriber('/camera_pan_tilt/pan_angle', Float32, pan_angle_cb)
    rospy.Subscriber('/camera_pan_tilt/tilt_angle', Float32, tilt_angle_cb)

    # 注册ROS服务
    rospy.Service('/camera_pan_tilt/sweep', Trigger, sweep_cb)
    rospy.Service('/camera_pan_tilt/stop', Trigger, stop_cb)

    rospy.loginfo("camera_pan_tilt 节点已启动")
    rospy.loginfo("话题接口:")
    rospy.loginfo("  /camera_pan_tilt/pan_angle     (Float32: 0~360, 默认 180)")
    rospy.loginfo("  /camera_pan_tilt/tilt_angle    (Float32: 0~180, 暂未接入)")
    rospy.loginfo("服务接口:")
    rospy.loginfo("  /camera_pan_tilt/sweep         (触发360°扫描)")
    rospy.loginfo("  /camera_pan_tilt/stop          (Pan 回中心 180°)")

    rospy.on_shutdown(stop_pan)

    rospy.spin()
