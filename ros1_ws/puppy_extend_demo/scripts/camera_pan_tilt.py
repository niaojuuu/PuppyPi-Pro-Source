#!/usr/bin/env python3
# coding=utf8
# 云台摄像头控制节点（Pan-Tilt Camera Control）
# Pan: FS90R 连续旋转舵机 (PWM端口 9)，控制水平360°旋转
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

PAN_SERVO_ID = 9    # 水平旋转（连续旋转舵机，PWM端口9）
TILT_SERVO_ID = None # 俯仰暂未接入，后续接入后改为对应PWM端口号

# 连续旋转舵机标定参数（需实测校准）
# 标定方法：以0.3速度旋转5秒，测量实际转过的角度，用角度÷5得到此值
PAN_SPEED_DEG_PER_SEC = 30.0  # 舵机在0.3速度下的旋转速率，单位：度/秒（初始估计值，需实测校准）

board = Board()
psc = PWMServoControl(board)
lock = threading.RLock()


def set_pan_speed(speed):
    """设置水平旋转速度
    speed: -1.0(逆时针最大) ~ 0.0(停止) ~ +1.0(顺时针最大)
    连续旋转舵机：脉冲1500=停止，<1500逆时针，>1500顺时针
    """
    with lock:
        pulse = int(1500 + speed * 500)
        pulse = max(500, min(2500, pulse))
        psc.setPulse(PAN_SERVO_ID, pulse, 50)


def stop_pan():
    """停止水平旋转"""
    with lock:
        psc.setPulse(PAN_SERVO_ID, 1500, 50)


def set_tilt_angle(angle):
    """设置俯仰角度（俯仰舵机暂未接入）
    angle: 0°(向上最大) ~ 90°(水平) ~ 180°(向下最大)
    """
    if TILT_SERVO_ID is None:
        rospy.logwarn("俯仰舵机未接入，忽略 tilt 指令")
        return
    with lock:
        pulse = int(500 + angle / 180.0 * 2000)
        pulse = max(500, min(2500, pulse))
        psc.setPulse(TILT_SERVO_ID, pulse, 500)


def sweep_scan(steps=12):
    """360°全景扫描，分steps步旋转，在每个位置暂停拍摄
    steps: 分成多少步（默认12步，每步30°）
    """
    angle_per_step = 360.0 / steps
    rotate_time = angle_per_step / PAN_SPEED_DEG_PER_SEC  # 每步旋转所需时间

    rospy.loginfo("开始360°扫描，共%d步，每步%.1f°，每步转%.2f秒", steps, angle_per_step, rotate_time)

    for i in range(steps):
        rospy.loginfo("扫描步骤 %d/%d", i + 1, steps)
        set_pan_speed(0.3)                # 低速旋转
        time.sleep(rotate_time)           # 旋转一步的角度
        stop_pan()                        # 停止
        time.sleep(0.5)                   # 等待画面稳定

    rospy.loginfo("360°扫描完成")


# ============ ROS1 回调 ============

def pan_speed_cb(msg):
    speed = max(-1.0, min(1.0, msg.data))
    set_pan_speed(speed)
    rospy.loginfo("Pan speed set to %.2f", speed)


def tilt_angle_cb(msg):
    angle = max(0.0, min(180.0, msg.data))
    set_tilt_angle(angle)
    rospy.loginfo("Tilt angle set to %.1f", angle)


def sweep_cb(req):
    sweep_scan(12)
    return TriggerResponse(success=True, message="360 sweep done")


def stop_cb(req):
    stop_pan()
    rospy.loginfo("Pan stopped")
    return TriggerResponse(success=True, message="pan stopped")


# ============ 主函数 ============

def init_pan_tilt():
    """初始化云台到默认位置"""
    rospy.loginfo("初始化云台：Pan停止")
    stop_pan()
    time.sleep(0.2)


if __name__ == '__main__':
    rospy.init_node('camera_pan_tilt')

    init_pan_tilt()

    # 订阅话题
    rospy.Subscriber('/camera_pan_tilt/pan_speed', Float32, pan_speed_cb)
    rospy.Subscriber('/camera_pan_tilt/tilt_angle', Float32, tilt_angle_cb)

    # 注册ROS服务
    rospy.Service('/camera_pan_tilt/sweep', Trigger, sweep_cb)
    rospy.Service('/camera_pan_tilt/stop', Trigger, stop_cb)

    rospy.loginfo("camera_pan_tilt 节点已启动")
    rospy.loginfo("话题接口:")
    rospy.loginfo("  /camera_pan_tilt/pan_speed     (Float32: -1.0~1.0)")
    rospy.loginfo("  /camera_pan_tilt/tilt_angle    (Float32: 0~180, 暂未接入)")
    rospy.loginfo("服务接口:")
    rospy.loginfo("  /camera_pan_tilt/sweep         (触发360°扫描)")
    rospy.loginfo("  /camera_pan_tilt/stop          (停止Pan)")

    rospy.on_shutdown(stop_pan)

    rospy.spin()
