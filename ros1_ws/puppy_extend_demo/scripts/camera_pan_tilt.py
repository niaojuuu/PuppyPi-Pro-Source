#!/usr/bin/env python3
# coding=utf8
# 云台摄像头控制节点（Pan-Tilt Camera Control）
# Pan: FS90R 连续旋转舵机 (ID 12)，控制水平360°旋转
# Tilt: Hiwonder 空心杯舵机 (ID 13)，控制俯仰角度

import sys
import time
import rospy
import threading

sys.path.append('/home/ubuntu/software/puppypi_control')
from ros_robot_controller_sdk import Board
from pwm_servo_control import PWMServoControl
from std_srvs.srv import SetFloat64, SetFloat64Response, Trigger, TriggerResponse

PAN_SERVO_ID = 12   # 水平旋转（连续旋转舵机）
TILT_SERVO_ID = 13  # 俯仰（标准空心杯舵机）

# 连续旋转舵机标定参数（需实测校准）
PAN_SPEED_DEG_PER_SEC = 108.0  # 舵机在0.3速度下的旋转速率，单位：度/秒

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
    """设置俯仰角度
    angle: 0°(向上最大) ~ 90°(水平) ~ 180°(向下最大)
    """
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

    rospy.loginfo("开始360°扫描，共%d步，每步%.1f°", steps, angle_per_step)

    for i in range(steps):
        rospy.loginfo("扫描步骤 %d/%d", i + 1, steps)
        set_pan_speed(0.3)                # 低速旋转
        time.sleep(rotate_time)           # 旋转一步的角度
        stop_pan()                        # 停止
        time.sleep(0.5)                   # 等待画面稳定
        # 此处可扩展：抓取图像并保存/发布

    rospy.loginfo("360°扫描完成")


# ============ ROS1 服务回调 ============

def set_pan_speed_cb(req):
    speed = max(-1.0, min(1.0, req.data))
    set_pan_speed(speed)
    rospy.loginfo("Pan speed set to %.2f", speed)
    return SetFloat64Response(success=True, message="pan speed set to %.2f" % speed)


def set_tilt_angle_cb(req):
    angle = max(0.0, min(180.0, req.data))
    set_tilt_angle(angle)
    rospy.loginfo("Tilt angle set to %.1f", angle)
    return SetFloat64Response(success=True, message="tilt angle set to %.1f" % angle)


def sweep_cb(req):
    sweep_scan(12)
    return TriggerResponse(success=True, message="360 sweep done")


def stop_cb(req):
    stop_pan()
    set_tilt_angle(90)
    rospy.loginfo("Pan-tilt reset to home position")
    return TriggerResponse(success=True, message="pan-tilt stopped and reset")


# ============ 主函数 ============

def init_pan_tilt():
    """初始化云台到默认位置"""
    rospy.loginfo("初始化云台：Pan停止，Tilt水平(90°)")
    stop_pan()
    time.sleep(0.2)
    set_tilt_angle(90)
    time.sleep(0.5)


if __name__ == '__main__':
    rospy.init_node('camera_pan_tilt')

    init_pan_tilt()

    # 注册ROS服务
    rospy.Service('/camera_pan_tilt/set_pan_speed', SetFloat64, set_pan_speed_cb)
    rospy.Service('/camera_pan_tilt/set_tilt_angle', SetFloat64, set_tilt_angle_cb)
    rospy.Service('/camera_pan_tilt/sweep', Trigger, sweep_cb)
    rospy.Service('/camera_pan_tilt/stop', Trigger, stop_cb)

    rospy.loginfo("camera_pan_tilt 节点已启动")
    rospy.loginfo("服务接口:")
    rospy.loginfo("  /camera_pan_tilt/set_pan_speed  (float: -1.0~1.0)")
    rospy.loginfo("  /camera_pan_tilt/set_tilt_angle (float: 0~180)")
    rospy.loginfo("  /camera_pan_tilt/sweep          (触发360°扫描)")
    rospy.loginfo("  /camera_pan_tilt/stop           (停止并复位)")

    rospy.on_shutdown(lambda: (stop_pan(), set_tilt_angle(90)))

    rospy.spin()
