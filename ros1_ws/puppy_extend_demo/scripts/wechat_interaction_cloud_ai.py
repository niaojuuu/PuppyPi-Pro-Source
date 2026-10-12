#!/usr/bin/env python3
# encoding: utf-8
"""云端 AI 微信控制 - 企业微信自建应用 + 文本/语音控制机器狗。

架构：
  企业微信回调 -> 消息队列 -> 阿里云千问 Qwen3.5 理解 (文本)
                              \\-> DashScope Paraformer 语音识别 (语音)
                              -> 动作执行 + 文本回执

与 voice_interaction_cloud_ai2.py 共存运行，共享同一只狗 (用户接受动作冲突)。

代码复用策略：从 voice 脚本完整复制动作/LLM/识别代码，不重构 voice。
原始 tts_and_play 调用全部替换为 user_announce —— voice 脚本里它是 TTS，
本脚本里它发文本回执。
"""
import os
import sys
import json
import math
import time
import argparse
import logging
import queue
import threading
import traceback
import base64
import yaml

# 抑制 ALSA lib 的无关警告（本地人脸识别不依赖音频；保留习惯）
try:
    import ctypes
    _ERROR_HANDLER = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_int,
                                       ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p)
    def _alsa_error_handler(filename, line, function, err, fmt):
        pass
    _alsa_err_cb = _ERROR_HANDLER(_alsa_error_handler)
    ctypes.cdll.LoadLibrary('libasound.so.2').snd_lib_error_set_handler(_alsa_err_cb)
except Exception:
    pass

import rospy
from openai import OpenAI
from std_msgs.msg import *
from std_srvs.srv import Trigger, TriggerResponse
from puppy_control.msg import Velocity, Pose, Gait
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
from puppy_control.srv import SetRunActionName
from ros_robot_controller.msg import RGBState, RGBsState
import sensor.Sonar as Sonar

# 本地人脸识别（与 voice 脚本一致）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from face_recognition_lib import FaceRecognizer
    FACE_RECOGNIZER = FaceRecognizer()
    FACE_LIB_OK = True
except Exception as _e:
    FACE_RECOGNIZER = None
    FACE_LIB_OK = False
    print(f'[人脸识别] 初始化失败：{_e}')

# WeCom 适配层
from wecom_callback_server import start_callback_server, WechatMsg
from audio_decode import decode_amr_to_wav, have_ffmpeg

print('''
**********************************************************
****  功能：企业微信文本/语音控制机器狗              ****
**********************************************************
----------------------------------------------------------
入口：企业微信自建应用 (Flask 回调)
文本：直接走 LLM 理解
语音：ffmpeg AMR->WAV -> 阿里云 Paraformer 语音识别 -> LLM
语言理解：阿里云千问 Qwen3.5 大模型
回复：仅文本消息回执
----------------------------------------------------------
''')


# ==================== 队列 & 全局状态 ====================

INCOMING_QUEUE: queue.Queue = queue.Queue()
REPLY_QUEUE: queue.Queue = queue.Queue()
ACTION_LOCK = threading.Lock()
CURRENT_SENDER: str = ''  # 消息循环设置；user_announce 据此发回执

run_st = True
LOG = None  # 注入


# ==================== 配置 ====================

DASHSCOPE_API_KEY = os.environ.get('DASHSCOPE_API_KEY', 'sk-4fe56ee3d9e44b248516ef93d95190f4')
QWEN_MODEL = 'qwen3.5-plus'
llm_client: OpenAI = None

# 机器狗初始配置（与 voice 脚本一致）
PuppyPose = {'roll': math.radians(0), 'pitch': math.radians(0), 'yaw': 0.000,
             'height': -10, 'x_shift': -0.5, 'stance_x': 0, 'stance_y': 0}
GaitConfig = {'overlap_time': 0.2, 'swing_time': 0.2, 'clearance_time': 0.0, 'z_clearance': 3}
BASE_SPEED = 8
speed_factor = 1.0


# ==================== 用户回执（替代 voice 脚本里的 tts_and_play）====================

def user_announce(text: str):
    """voice 脚本里调用 tts_and_play(tts_engine, text) 走本地 TTS。
    本脚本替换为：日志 + 通过 REPLY_QUEUE 发回执给微信用户。
    """
    if not text:
        return
    LOG.info(f'[回执] {text}')
    if CURRENT_SENDER:
        REPLY_QUEUE.put((CURRENT_SENDER, text))


# ==================== ROS 控制函数 ====================

PuppyPosePub = None
PuppyGaitConfigPub = None
PuppyVelocityPub = None
RGBPub = None
run_action_group_srv = None
PanTiltPanAnglePub = None
PanTiltSweepSrv = None
PanTiltStopSrv = None
find_object_running = False


def set_rgb(r, g, b):
    led1 = RGBState(id=1, r=r, g=g, b=b)
    led2 = RGBState(id=2, r=r, g=g, b=b)
    RGBPub.publish(RGBsState(data=[led1, led2]))


def rgb_off():
    set_rgb(0, 0, 0)


def Stop():
    global run_st
    run_st = False
    rgb_off()
    print('关闭中...')


def pub_pose(roll=0, pitch=0, yaw=0, height=-10):
    PuppyPosePub.publish(stance_x=0, stance_y=0, x_shift=-0.5,
        height=height, roll=math.radians(roll), pitch=math.radians(pitch), yaw=yaw, run_time=500)


def pub_vel(x=0, y=0, yaw_rate=0):
    PuppyVelocityPub.publish(x=x, y=y, yaw_rate=yaw_rate)


def reset_pose():
    pub_pose()


# ==================== 动作函数（从 voice 脚本完整复制）====================

def action_forward():
    speed = BASE_SPEED * speed_factor
    reset_pose()
    rospy.sleep(0.5)
    pub_vel(x=speed)
    rospy.sleep(2)
    pub_vel()


def action_backward():
    speed = BASE_SPEED * speed_factor
    reset_pose()
    rospy.sleep(0.5)
    pub_vel(x=-speed)
    rospy.sleep(2)
    pub_vel()


def action_turn_left():
    reset_pose()
    rospy.sleep(0.5)
    pub_vel(yaw_rate=0.5)
    rospy.sleep(2)
    pub_vel()


def action_turn_right():
    reset_pose()
    rospy.sleep(0.5)
    pub_vel(yaw_rate=-0.5)
    rospy.sleep(2)
    pub_vel()


def action_stop():
    pub_vel()


def action_stand():
    reset_pose()


def action_lie_down():
    pub_pose(height=-6)


def action_sit():
    pub_pose(pitch=15, height=-6)


def action_speed_up():
    global speed_factor
    speed_factor = min(speed_factor + 0.5, 3.0)
    print(f"加速，当前倍率：{speed_factor}")


def action_slow_down():
    global speed_factor
    speed_factor = max(speed_factor - 0.5, 0.5)
    print(f"减速，当前倍率：{speed_factor}")


def action_nod():
    for _ in range(3):
        pub_pose(pitch=15)
        rospy.sleep(0.4)
        reset_pose()
        rospy.sleep(0.4)


def action_sway():
    for _ in range(3):
        pub_pose(roll=15)
        rospy.sleep(0.4)
        pub_pose(roll=-15)
        rospy.sleep(0.4)
    reset_pose()


def action_squat():
    for _ in range(3):
        pub_pose(height=-6)
        rospy.sleep(0.4)
        pub_pose(height=-12)
        rospy.sleep(0.4)
    reset_pose()


def action_shake_head():
    for _ in range(3):
        pub_pose(yaw=0.3)
        rospy.sleep(0.3)
        pub_pose(yaw=-0.3)
        rospy.sleep(0.3)
    reset_pose()


def action_dance():
    for _ in range(4):
        pub_pose(roll=15, height=-8)
        rospy.sleep(0.3)
        pub_pose(roll=-15, height=-12)
        rospy.sleep(0.3)
    reset_pose()


def action_pushup():
    for _ in range(5):
        pub_pose(height=-6)
        rospy.sleep(0.5)
        pub_pose(height=-12)
        rospy.sleep(0.5)
    reset_pose()


def action_kick_left():
    pub_pose(roll=15)
    rospy.sleep(0.5)
    pub_pose(roll=-10, pitch=10)
    rospy.sleep(0.3)
    reset_pose()


def action_kick_right():
    pub_pose(roll=-15)
    rospy.sleep(0.5)
    pub_pose(roll=10, pitch=10)
    rospy.sleep(0.3)
    reset_pose()


def action_twist_waist():
    for _ in range(3):
        pub_pose(roll=10, yaw=0.3)
        rospy.sleep(0.4)
        pub_pose(roll=-10, yaw=-0.3)
        rospy.sleep(0.4)
    reset_pose()


def action_situp():
    for _ in range(4):
        pub_pose(pitch=20, height=-6)
        rospy.sleep(0.5)
        pub_pose(pitch=-10, height=-10)
        rospy.sleep(0.5)
    reset_pose()


def action_bow():
    pub_pose(pitch=25)
    rospy.sleep(1.5)
    reset_pose()


def action_spread_wings():
    pub_pose(height=-12)
    rospy.sleep(0.5)
    pub_pose(pitch=15, roll=15, height=-12)
    rospy.sleep(0.5)
    pub_pose(pitch=15, roll=-15, height=-12)
    rospy.sleep(0.5)
    reset_pose()


def action_wave():
    for _ in range(4):
        pub_pose(pitch=10, yaw=0.2)
        rospy.sleep(0.25)
        pub_pose(pitch=-5, yaw=-0.2)
        rospy.sleep(0.25)
    reset_pose()


def action_march():
    reset_pose()
    rospy.sleep(0.5)
    pub_vel(x=0.1)
    rospy.sleep(3)
    pub_vel()


def action_show_off():
    pub_pose(pitch=20)
    rospy.sleep(0.5)
    for _ in range(2):
        pub_pose(pitch=20, roll=15)
        rospy.sleep(0.3)
        pub_pose(pitch=20, roll=-15)
        rospy.sleep(0.3)
    reset_pose()
    rospy.sleep(0.3)
    pub_pose(height=-6)
    rospy.sleep(0.4)
    reset_pose()


def action_walk_steps():
    reset_pose()
    rospy.sleep(0.5)
    pub_vel(x=8)
    rospy.sleep(2)
    pub_vel()


def action_get_up():
    pub_pose(pitch=25, height=-4)
    rospy.sleep(1)
    reset_pose()


def action_swagger():
    reset_pose()
    rospy.sleep(0.5)
    pub_vel(x=6)
    for _ in range(4):
        pub_pose(roll=12)
        rospy.sleep(0.4)
        pub_pose(roll=-12)
        rospy.sleep(0.4)
    pub_vel()
    reset_pose()


def action_left_hook():
    pub_pose(roll=15, yaw=0.3)
    rospy.sleep(0.3)
    pub_pose(roll=-10, yaw=-0.2)
    rospy.sleep(0.2)
    reset_pose()


def action_right_hook():
    pub_pose(roll=-15, yaw=-0.3)
    rospy.sleep(0.3)
    pub_pose(roll=10, yaw=0.2)
    rospy.sleep(0.2)
    reset_pose()


def action_shake_hands():
    for _ in range(3):
        pub_pose(pitch=15, height=-8)
        rospy.sleep(0.3)
        pub_pose(pitch=5, height=-10)
        rospy.sleep(0.3)
    reset_pose()


def action_lean_right():
    pub_pose(roll=-15)


def action_hello():
    action_nod()


def action_two_leg_stand():
    global run_action_group_srv
    if run_action_group_srv is not None:
        try:
            run_action_group_srv('2_legs_stand.d6ac', True)
            return
        except Exception as e:
            print(f"[两脚站立] 动作组调用失败 ({e})，使用姿态近似")

    PuppyPosePub.publish(stance_x=0, stance_y=3, x_shift=1.0,
        height=-11, roll=math.radians(0), pitch=math.radians(0), yaw=0, run_time=500)
    rospy.sleep(0.6)
    PuppyPosePub.publish(stance_x=0, stance_y=3, x_shift=3.5,
        height=-11, roll=math.radians(0), pitch=math.radians(-12), yaw=0, run_time=500)
    rospy.sleep(0.6)
    PuppyPosePub.publish(stance_x=0, stance_y=3, x_shift=5.5,
        height=-12, roll=math.radians(0), pitch=math.radians(-22), yaw=0, run_time=500)
    rospy.sleep(0.7)
    PuppyPosePub.publish(stance_x=0, stance_y=3, x_shift=7.0,
        height=-13, roll=math.radians(0), pitch=math.radians(-30), yaw=0, run_time=600)
    rospy.sleep(0.8)


# ==================== 云台摄像头控制 ====================
#
# 双摄像头架构：
#   1. 前视固定 /usb_cam (device /dev/video0) — 不可旋转
#   2. 云台可旋转 /usb_cam_surround (device /dev/video1) — 0~360°
# 本节所有 action_camera_* 仅作用于云台摄像头 (2)。
# 视觉算法默认看前视固定，旋转云台不影响它们。

def action_camera_look_left():
    action_camera_look(150.0)


def action_camera_look_right():
    action_camera_look(210.0)


def action_camera_look(angle):
    if PanTiltPanAnglePub is None:
        print('[云台] 未初始化，跳过')
        return
    try:
        angle = float(angle)
    except (TypeError, ValueError):
        print(f'[云台] 角度参数无效：{angle}')
        return
    clamped = max(0.0, min(360.0, angle))
    if clamped != angle:
        print(f'[云台] 角度 {angle} 超出 [0, 360]，clamp 到 {clamped}')
    PanTiltPanAnglePub.publish(clamped)


def action_camera_reset():
    if PanTiltStopSrv is not None:
        try:
            PanTiltStopSrv()
            return
        except Exception as e:
            print(f'[云台] stop 服务失败：{e}，改用话题')
    action_camera_look(180.0)


def action_camera_sweep():
    if PanTiltSweepSrv is None:
        print('[云台] sweep 服务不可用')
        return
    try:
        resp = PanTiltSweepSrv()
        print(f'[云台] 扫描完成：{resp.message}')
    except Exception as e:
        print(f'[云台] sweep 调用失败：{e}')


def action_camera_stop():
    action_camera_reset()


# ==================== 寻物模式 ====================

def action_find_object(target):
    """搜寻指定物品：正常视角 + 抬高视角 + LLM 决策移动 + 超声波避障"""
    import cv2
    import re

    global find_object_running
    find_object_running = True

    bridge = CvBridge()
    latest_image = [None]
    image_lock = threading.Lock()

    def image_callback(ros_image):
        try:
            with image_lock:
                latest_image[0] = bridge.imgmsg_to_cv2(ros_image, 'bgr8')
        except Exception as e:
            print(f'[寻物] 图像错误：{e}')

    image_sub = rospy.Subscriber('/usb_cam/image_raw', RosImage, image_callback,
                                 queue_size=1, buff_size=2**24)

    sonar = None
    try:
        sonar = Sonar.Sonar()
        sonar.setRGBMode(0)
        sonar.setRGB(0, (0, 0, 0))
        sonar.setRGB(1, (0, 0, 0))
    except Exception as e:
        print(f'[寻物] 超声波未连接：{e}')

    def get_dist():
        if sonar:
            try:
                return sonar.getDistance()
            except Exception:
                pass
        return None

    def capture():
        with image_lock:
            latest_image[0] = None
        rospy.sleep(0.15)
        for _ in range(20):
            with image_lock:
                frame = latest_image[0]
                latest_image[0] = None
            if frame is not None:
                _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                return base64.b64encode(buf.tobytes()).decode('utf-8')
            rospy.sleep(0.1)
        return None

    def vision_query(b64, prompt):
        try:
            kwargs = dict(
                model=QWEN_MODEL,
                messages=[{
                    'role': 'user',
                    'content': [
                        {'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{b64}'}},
                        {'type': 'text', 'text': prompt},
                    ]
                }],
                temperature=0.1,
                max_tokens=120,
                extra_body={'enable_thinking': False, 'enable_search': False},
            )
            resp = llm_client.chat.completions.create(**kwargs)
            raw = resp.choices[0].message.content.strip()
            return re.sub(r'<think>.*?</think>', '', raw, flags=re.DOTALL).strip()
        except Exception as e:
            print(f'[寻物] 视觉 LLM 错误：{e}')
            return None

    def check_target(b64):
        prompt = (
            f'这是一只矮小机器狗正前方摄像头拍摄的画面，摄像头位置很低，视角偏低。'
            f'图像中是否有"{target}"？严格只输出 JSON，不要其他内容。'
            f'格式：{{"found":true 或 false,"position":"left 或 center 或 right",'
            f'"hint":"near 或 far 或 blocked"}}'
        )
        raw = vision_query(b64, prompt)
        if not raw:
            return False, 'center', 'unknown'
        s, e = raw.find('{'), raw.rfind('}')
        if s == -1:
            return False, 'center', 'unknown'
        try:
            d = json.loads(raw[s:e + 1])
            return bool(d.get('found')), d.get('position', 'center'), d.get('hint', 'unknown')
        except Exception:
            return False, 'center', 'unknown'

    def ask_decision(b64, search_history):
        if search_history:
            history_text = '搜索历史（从早到晚）：\n' + '\n'.join(
                f'  第{h["round"]}轮：{h["desc"]}' for h in search_history
            )
        else:
            history_text = '搜索历史：暂无（第一轮搜索）。'

        stand_count = sum(1 for h in search_history if 'two_leg_stand' in h.get('desc', ''))
        turn_count = sum(1 for h in search_history if 'turn_' in h.get('desc', ''))
        forward_count = sum(1 for h in search_history if 'forward' in h.get('desc', ''))
        stats_text = f'已执行统计：站立{stand_count}次，转向{turn_count}次，前进{forward_count}次'

        prompt = (
            f'这是一只矮小机器狗正前方摄像头拍摄的画面，摄像头位置很低（离地约 10cm），视角偏低。\n'
            f'机器狗正在寻找"{target}"，当前正常视角未发现目标。\n'
            f'{history_text}\n'
            f'{stats_text}\n\n'
            f'你需要根据目标物品特征、当前画面环境、搜索历史综合判断最佳动作：\n'
            f'可选动作：\n'
            f'  forward   - 向前移动探索新区域（适合：目标可能在前方远处、需要换区域搜索）\n'
            f'  turn_left - 原地左转换方向（适合：需要扫描左侧区域、前方已搜过）\n'
            f'  turn_right- 原地右转换方向（适合：需要扫描右侧区域、前方已搜过）\n'
            f'  two_leg_stand - 仰起身体升高摄像头视角（适合：目标可能在桌面、架子等'
            f'高处；但注意若近几轮已站立搜过仍没找到，应优先移动换位置）\n\n'
            f'搜索策略提示：\n'
            f'- 如果目标是日常小物件（杯子、遥控器等），可能在桌面高处，考虑站立\n'
            f'- 如果目标可能在地面（鞋子、球等），优先转向和前进\n'
            f'- 如果同一位置已经站立过但没找到，应先移动到新位置\n'
            f'- 如果连续多次转向仍没找到，考虑前进换区域\n'
            f'- 避免连续重复同一动作超过 2 次\n\n'
            f'严格只输出 JSON，不要其他内容。'
            f'格式：{{"action":"forward 或 turn_left 或 turn_right 或 two_leg_stand","reason":"简短原因"}}'
        )
        raw = vision_query(b64, prompt)
        if not raw:
            return 'turn_right', '视觉 LLM 无响应'
        s, e = raw.find('{'), raw.rfind('}')
        if s == -1:
            return 'turn_right', '解析失败'
        try:
            d = json.loads(raw[s:e + 1])
            act = d.get('action', 'turn_right')
            if act not in ('forward', 'turn_left', 'turn_right', 'two_leg_stand'):
                act = 'turn_right'
            reason = d.get('reason', '')
            print(f'[寻物] LLM 决策：{act}（{reason}）')
            return act, reason
        except Exception:
            return 'turn_right', '解析异常'

    def move_safe(action, duration=1.5):
        speed = BASE_SPEED * speed_factor
        if action == 'forward':
            dist = get_dist()
            if dist is not None and dist < 300:
                print(f'[寻物] 前方障碍 {dist}mm，改为右转避障')
                pub_vel(yaw_rate=-0.5)
                rospy.sleep(0.8)
                pub_vel()
                rospy.sleep(0.3)
                return
            pub_vel(x=speed)
            elapsed = 0.0
            while elapsed < duration:
                rospy.sleep(0.1)
                elapsed += 0.1
                dist = get_dist()
                if dist is not None and dist < 300:
                    print(f'[寻物] 前方障碍 {dist}mm，停止')
                    break
            pub_vel()
        elif action == 'turn_left':
            pub_vel(yaw_rate=0.5)
            rospy.sleep(duration)
            pub_vel()
        else:
            pub_vel(yaw_rate=-0.5)
            rospy.sleep(duration)
            pub_vel()
        rospy.sleep(0.4)

    def approach_target():
        APPROACH_MAX = 15
        SONAR_COLLISION_MM = 150
        near_count = 0

        print(f'[寻物] 进入靠近阶段，尝试尽可能接近目标...')
        for step in range(APPROACH_MAX):
            if not find_object_running or not run_st or rospy.is_shutdown():
                return False

            dist = get_dist()
            if dist is not None and dist < SONAR_COLLISION_MM:
                print(f'[寻物] 超声波防撞：前方 {dist}mm 有障碍，停止靠近')
                pub_vel()
                return True

            b64 = capture()
            if not b64:
                continue
            found_flag, position, hint = check_target(b64)

            if not found_flag:
                print(f'[寻物] 靠近过程中目标丢失（第{step+1}步）')
                pub_vel()
                return False

            if hint == 'near':
                near_count += 1
                if near_count >= 2:
                    print(f'[寻物] 视觉连续确认已贴近目标')
                    pub_vel()
                    return True
                print(f'[寻物] 视觉判断 near（第{near_count}次），再小步确认...')
                if position == 'left':
                    move_safe('turn_left', 0.3)
                elif position == 'right':
                    move_safe('turn_right', 0.3)
                move_safe('forward', 0.5)
                rospy.sleep(0.2)
                continue
            else:
                near_count = 0

            if position == 'left':
                move_safe('turn_left', 0.4)
            elif position == 'right':
                move_safe('turn_right', 0.4)
            move_safe('forward', 1.0)
            rospy.sleep(0.2)

        print(f'[寻物] 靠近轮次用完，停止')
        pub_vel()
        return True

    reset_pose()
    rospy.sleep(0.5)
    print(f'[寻物] 开始寻找目标：{target}')

    MAX_ITER = 100
    found = False
    search_history = []

    for iteration in range(MAX_ITER):
        if not find_object_running or not run_st or rospy.is_shutdown():
            break

        print(f'[寻物] 第 {iteration + 1}/{MAX_ITER} 轮')

        b64 = capture()
        if b64:
            found_flag, position, hint = check_target(b64)
            if found_flag:
                if hint == 'far':
                    print(f'[寻物] 远处发现目标（{position}），靠近中...')
                    if position == 'left':
                        move_safe('turn_left', 0.6)
                    elif position == 'right':
                        move_safe('turn_right', 0.6)
                    move_safe('forward', 2.0)
                    search_history.append({'round': iteration + 1,
                        'desc': f'正常视角远处发现目标 ({position})，已靠近'})
                    continue
                else:
                    print(f'[寻物] 发现目标（{position}, {hint}），开始靠近...')
                    if position == 'left':
                        move_safe('turn_left', 0.4)
                    elif position == 'right':
                        move_safe('turn_right', 0.4)
                    if approach_target():
                        found = True
                        break
                    else:
                        search_history.append({'round': iteration + 1,
                            'desc': f'正常视角发现目标 ({position}) 但靠近时丢失'})
                        continue

        b64_decision = b64 or capture()
        next_action, reason = ask_decision(b64_decision, search_history) if b64_decision else ('turn_right', '无图像')

        if next_action == 'two_leg_stand':
            print(f'[寻物] LLM 决定站立搜索（{reason}）')
            PuppyPosePub.publish(stance_x=0, stance_y=3, x_shift=3.5,
                height=-11, roll=math.radians(0), pitch=math.radians(-12), yaw=0, run_time=500)
            rospy.sleep(0.6)
            PuppyPosePub.publish(stance_x=0, stance_y=3, x_shift=7.0,
                height=-13, roll=math.radians(0), pitch=math.radians(-30), yaw=0, run_time=600)
            rospy.sleep(0.8)

            b64_high = capture()
            PuppyPosePub.publish(stance_x=0, stance_y=3, x_shift=3.5,
                height=-11, roll=math.radians(0), pitch=math.radians(-12), yaw=0, run_time=500)
            rospy.sleep(0.5)
            reset_pose()
            rospy.sleep(0.5)

            if b64_high:
                found_flag, position, hint = check_target(b64_high)
                if found_flag:
                    print(f'[寻物] 高视角发现目标（{position}），对准并靠近...')
                    if position == 'left':
                        move_safe('turn_left', 0.6)
                    elif position == 'right':
                        move_safe('turn_right', 0.6)
                    move_safe('forward', 2.0)
                    if approach_target():
                        found = True
                        search_history.append({'round': iteration + 1,
                            'desc': f'动作=two_leg_stand，原因={reason}，高视角发现目标 ({position}) 已靠近'})
                        break
                    else:
                        search_history.append({'round': iteration + 1,
                            'desc': f'动作=two_leg_stand，原因={reason}，高视角发现目标 ({position}) 但靠近时丢失'})
                        continue
            search_history.append({'round': iteration + 1,
                'desc': f'动作=two_leg_stand，原因={reason}，高视角未发现'})
        else:
            search_history.append({'round': iteration + 1,
                'desc': f'动作={next_action}，原因={reason}'})
            move_safe(next_action, 1.2)
        rospy.sleep(0.2)

    pub_vel()
    image_sub.unregister()
    if sonar:
        sonar.setRGB(0, (0, 0, 0))
        sonar.setRGB(1, (0, 0, 0))
    find_object_running = False

    if found:
        print(f'[寻物] 成功找到 {target}！')
        user_announce('找到了')
    else:
        print(f'[寻物] 未找到 {target}，搜寻结束')
        user_announce(f'没有找到{target}')


def action_stop_find():
    global find_object_running
    if find_object_running:
        find_object_running = False
        pub_vel()
        print('[寻物] 已停止寻物')


# ==================== 本地人脸识别 ====================

_face_image_sub = None
_face_latest_frame = [None]
_face_frame_lock = threading.Lock()
_pending_face_name = False


def _face_image_cb(ros_image):
    try:
        bgr = CvBridge().imgmsg_to_cv2(ros_image, 'bgr8')
        with _face_frame_lock:
            _face_latest_frame[0] = bgr
    except Exception as e:
        print(f'[人脸] 图像转换错误：{e}')


def _ensure_face_image_source():
    global _face_image_sub
    if _face_image_sub is None:
        _face_image_sub = rospy.Subscriber(
            '/usb_cam/image_raw', RosImage, _face_image_cb,
            queue_size=1, buff_size=2**24)
        rospy.sleep(0.3)


def _get_face_frame():
    with _face_frame_lock:
        return _face_latest_frame[0]


def action_add_face(name=''):
    global FACE_RECOGNIZER, FACE_LIB_OK, _pending_face_name
    if not FACE_LIB_OK or FACE_RECOGNIZER is None:
        msg = '人脸识别功能不可用'
        print(f'[人脸] {msg}')
        user_announce(msg)
        return

    name = (name or '').strip()
    if not name:
        _pending_face_name = True
        print('[人脸] 缺少名字，等待用户下一条消息告知', flush=True)
        user_announce('请告诉我你的名字')
        return

    print(f'[人脸] 开始注册：{name}', flush=True)
    user_announce(f'好的，请面对摄像头，我要记住{name}了')
    rospy.sleep(0.5)

    _ensure_face_image_source()
    rospy.sleep(0.5)

    def cb():
        return _get_face_frame()

    ok, message, dim = FACE_RECOGNIZER.register(name=name, image_callback=cb, timeout=8.0)
    print(f'[人脸] 注册结果：{message} (特征维度={dim})', flush=True)
    user_announce(message)


def try_handle_pending_face_name(text):
    global _pending_face_name
    if not _pending_face_name:
        return False
    _pending_face_name = False
    text = (text or '').strip()
    if not text or len(text) > 12 or any(c in text for c in '？?，,。.!！'):
        print(f'[人脸] 等待名字，但收到"{text}"，忽略', flush=True)
        return False
    print(f'[人脸] 收到名字：{text}', flush=True)
    action_add_face(text)
    return True


def action_who_am_i():
    global FACE_RECOGNIZER, FACE_LIB_OK
    if not FACE_LIB_OK or FACE_RECOGNIZER is None:
        msg = '人脸识别功能不可用'
        print(f'[人脸] {msg}')
        user_announce(msg)
        return

    if FACE_RECOGNIZER.db_size == 0:
        msg = '我还不认识任何人，先添加一张人脸吧'
        print(f'[人脸] {msg}')
        user_announce(msg)
        return

    print('[人脸] 正在识别人脸...', flush=True)
    user_announce('让我看看你是谁')

    _ensure_face_image_source()

    def cb():
        return _get_face_frame()

    name, message, dist = FACE_RECOGNIZER.recognize(image_callback=cb, timeout=8.0)
    print(f'[人脸] 识别结果：name={name}, msg={message}, dist={dist}', flush=True)
    user_announce(message)
    if name is None and '不认识' in message:
        print('[人脸] 未识别：可能是库中无此人或光线不好/侧脸', flush=True)


def action_who_is_he():
    action_who_am_i()


def action_list_faces():
    global FACE_RECOGNIZER, FACE_LIB_OK
    if not FACE_LIB_OK or FACE_RECOGNIZER is None:
        msg = '人脸识别功能不可用'
    else:
        people = FACE_RECOGNIZER.list_people()
        if people:
            msg = '已记住' + '、'.join(people)
        else:
            msg = '人脸库是空的'
    print(f'[人脸] {msg}')
    user_announce(msg)


# ==================== 跟随模式 ====================

follow_thread = None
follow_running = False

FOLLOW_SPEED = 8
FOLLOW_TURN_RATE = 0.8
PERSON_AREA_MIN = 8000
PERSON_AREA_IDEAL = 25000
PERSON_AREA_MAX = 45000
IMG_CENTER_DEADZONE = 30
SEARCH_TURN_RATE = 0.6
SEARCH_MAX_FRAMES = 20
TRACKER_REINIT_INTERVAL = 10
SMOOTH_ALPHA = 0.4

SONAR_TOO_CLOSE = 300
SONAR_CLOSE = 500
SONAR_IDEAL = 800
SONAR_FAR = 1500


def stop_follow():
    global follow_running
    if follow_running:
        follow_running = False
        if follow_thread and follow_thread.is_alive():
            follow_thread.join(timeout=3)
        pub_vel()
        print("[跟随] 已退出跟随模式")


def sonar_speed_from_distance(dist_mm):
    if dist_mm <= 0 or dist_mm >= 5000:
        return None
    if dist_mm <= SONAR_TOO_CLOSE:
        return -FOLLOW_SPEED * 0.5
    elif dist_mm <= SONAR_CLOSE:
        return 0
    elif dist_mm <= SONAR_IDEAL:
        return FOLLOW_SPEED * 0.3
    elif dist_mm <= SONAR_FAR:
        return FOLLOW_SPEED * 0.6
    else:
        return FOLLOW_SPEED


def extract_person_bbox(landmarks, img_w, img_h):
    primary_points = [11, 12, 23, 24]
    secondary_points = [0, 1, 2, 3, 4, 11, 12, 15, 16]
    fallback_points = [0, 1, 2, 3, 4, 5, 6, 7, 8, 11, 12, 13, 14, 15, 16]

    for points in [primary_points, secondary_points, fallback_points]:
        xs = [landmarks[i].x * img_w for i in points if landmarks[i].visibility > 0.3]
        ys = [landmarks[i].y * img_h for i in points if landmarks[i].visibility > 0.3]
        if len(xs) >= 2 and len(ys) >= 2:
            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)
            w = max(xs) - min(xs)
            h = max(ys) - min(ys)
            w = max(w, 20) * 1.3
            h = max(h, 20) * 1.3
            area = w * h
            return (cx, cy, w, h, area)
    return None


def follow_loop():
    global follow_running
    import cv2
    import mediapipe as mp

    bridge = CvBridge()
    mp_pose = mp.solutions.pose
    pose_detector = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=0,
        min_detection_confidence=0.3,
        min_tracking_confidence=0.3,
    )

    sonar = None
    try:
        sonar = Sonar.Sonar()
        sonar.setRGBMode(0)
        sonar.setRGB(0, (0, 0, 0))
        sonar.setRGB(1, (0, 0, 0))
        print("[跟随] 超声波传感器已连接，将融合视觉 + 超声波测距")
    except Exception as e:
        print(f"[跟随] 超声波传感器未连接 ({e})，仅使用视觉跟随")

    tracker = None
    tracker_ok = False

    def create_tracker():
        for factory in [
            lambda: cv2.legacy.TrackerCSRT_create(),
            lambda: cv2.TrackerCSRT_create(),
            lambda: cv2.legacy.TrackerKCF_create(),
            lambda: cv2.TrackerKCF_create(),
        ]:
            try:
                return factory()
            except AttributeError:
                continue
        print("[跟随] 警告：无可用的 OpenCV 跟踪器")
        return None

    latest_image = [None]
    image_lock = threading.Lock()

    def image_callback(ros_image):
        try:
            cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
            with image_lock:
                latest_image[0] = cv_image
        except Exception as e:
            print(f"[跟随] 图像转换错误：{e}")

    image_sub = rospy.Subscriber('/usb_cam/image_raw', RosImage, image_callback,
                                 queue_size=1, buff_size=2**24)

    reset_pose()
    rospy.sleep(0.5)
    print("[跟随] 进入摄像头人体跟随模式（MediaPipe + Tracker 混合）")

    lost_count = 0
    frame_count = 0
    last_direction = 1.0
    prev_speed = 0.0
    prev_turn = 0.0
    mode = "detect"

    while follow_running and run_st and not rospy.is_shutdown():
        try:
            frame_count += 1

            sonar_dist = None
            sonar_spd = None
            if sonar:
                try:
                    sonar_dist = sonar.getDistance()
                    sonar_spd = sonar_speed_from_distance(sonar_dist)
                except Exception:
                    pass

            if sonar and sonar_dist is not None:
                if sonar_dist <= SONAR_TOO_CLOSE:
                    sonar.setRGB(0, (255, 0, 0))
                    sonar.setRGB(1, (255, 0, 0))
                elif sonar_dist <= SONAR_CLOSE:
                    sonar.setRGB(0, (255, 255, 0))
                    sonar.setRGB(1, (255, 255, 0))
                elif sonar_dist <= SONAR_IDEAL:
                    sonar.setRGB(0, (0, 255, 0))
                    sonar.setRGB(1, (0, 255, 0))
                else:
                    sonar.setRGB(0, (0, 0, 255))
                    sonar.setRGB(1, (0, 0, 255))

            if sonar_dist is not None and sonar_dist <= SONAR_TOO_CLOSE:
                pub_vel(x=-FOLLOW_SPEED * 0.5)
                rospy.sleep(0.05)
                continue

            with image_lock:
                frame = latest_image[0]
                latest_image[0] = None

            if frame is None:
                rospy.sleep(0.03)
                continue

            img_h, img_w = frame.shape[:2]
            img_center_x = img_w / 2.0

            person_cx = None
            person_area = None

            run_mediapipe = (mode != "track") or (frame_count % TRACKER_REINIT_INTERVAL == 0)

            if run_mediapipe:
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = pose_detector.process(rgb_frame)

                if results.pose_landmarks:
                    bbox_info = extract_person_bbox(results.pose_landmarks.landmark, img_w, img_h)
                    if bbox_info:
                        person_cx, person_cy, bbox_w, bbox_h, person_area = bbox_info

                        x1 = max(0, int(person_cx - bbox_w / 2))
                        y1 = max(0, int(person_cy - bbox_h / 2))
                        x2 = min(img_w, int(person_cx + bbox_w / 2))
                        y2 = min(img_h, int(person_cy + bbox_h / 2))
                        w = x2 - x1
                        h = y2 - y1
                        if w > 10 and h > 10:
                            new_tracker = create_tracker()
                            if new_tracker is not None:
                                new_tracker.init(frame, (x1, y1, w, h))
                                tracker = new_tracker
                                tracker_ok = True

            if person_cx is None and tracker is not None and tracker_ok:
                try:
                    tracker_ok, bbox = tracker.update(frame)
                    if tracker_ok:
                        x, y, w, h = [int(v) for v in bbox]
                        person_cx = x + w / 2.0
                        person_area = w * h
                except Exception:
                    tracker_ok = False

            if person_cx is not None and person_area is not None:
                if mode == "search":
                    print(f"[跟随] 目标重新捕获")
                    prev_speed = 0.0
                    prev_turn = 0.0
                mode = "track"
                lost_count = 0

                offset_x = person_cx - img_center_x
                if abs(offset_x) > 5:
                    last_direction = 1.0 if offset_x > 0 else -1.0

                if abs(offset_x) > IMG_CENTER_DEADZONE:
                    turn = -offset_x / img_center_x * FOLLOW_TURN_RATE
                else:
                    turn = 0

                if person_area < PERSON_AREA_MIN:
                    vision_speed = FOLLOW_SPEED
                elif person_area > PERSON_AREA_MAX:
                    vision_speed = -FOLLOW_SPEED * 0.5
                elif person_area < PERSON_AREA_IDEAL:
                    vision_speed = FOLLOW_SPEED * 0.5
                else:
                    vision_speed = 0

                if sonar_spd is not None:
                    if vision_speed > 0 and sonar_spd >= 0:
                        speed = min(vision_speed, sonar_spd)
                    elif sonar_spd < 0:
                        speed = sonar_spd
                    else:
                        speed = vision_speed
                else:
                    speed = vision_speed

                speed = SMOOTH_ALPHA * speed + (1 - SMOOTH_ALPHA) * prev_speed
                turn = SMOOTH_ALPHA * turn + (1 - SMOOTH_ALPHA) * prev_turn
                prev_speed = speed
                prev_turn = turn

                pub_vel(x=speed, yaw_rate=turn)
            else:
                lost_count += 1

                if lost_count == 1:
                    mode = "search"
                    print(f"[跟随] 目标丢失，向{'右' if last_direction > 0 else '左'}搜索...")

                if lost_count <= SEARCH_MAX_FRAMES:
                    search_yaw = -last_direction * SEARCH_TURN_RATE
                    pub_vel(x=0, yaw_rate=search_yaw)
                    prev_speed = 0
                    prev_turn = search_yaw
                else:
                    if lost_count == SEARCH_MAX_FRAMES + 1:
                        print("[跟随] 搜索超时，停止等待")
                    pub_vel()
                    prev_speed = 0
                    prev_turn = 0

            rospy.sleep(0.05)
        except Exception as e:
            print(f"[跟随] 错误：{e}")
            rospy.sleep(0.5)

    pub_vel()
    image_sub.unregister()
    pose_detector.close()
    if sonar:
        sonar.setRGB(0, (0, 0, 0))
        sonar.setRGB(1, (0, 0, 0))
    print("[跟随] 循环结束")


def action_follow():
    global follow_running, follow_thread
    stop_follow()
    follow_running = True
    follow_thread = threading.Thread(target=follow_loop, daemon=True)
    follow_thread.start()


def action_stop_follow():
    stop_follow()


# ==================== 动作映射表 ====================

ACTION_MAP = {
    'forward()': action_forward,
    'backward()': action_backward,
    'turn_left()': action_turn_left,
    'turn_right()': action_turn_right,
    'stop()': action_stop,
    'stand()': action_stand,
    'lie_down()': action_lie_down,
    'sit()': action_sit,
    'speed_up()': action_speed_up,
    'slow_down()': action_slow_down,
    'nod()': action_nod,
    'sway()': action_sway,
    'squat()': action_squat,
    'shake_head()': action_shake_head,
    'dance()': action_dance,
    'pushup()': action_pushup,
    'kick_left()': action_kick_left,
    'kick_right()': action_kick_right,
    'twist_waist()': action_twist_waist,
    'situp()': action_situp,
    'bow()': action_bow,
    'spread_wings()': action_spread_wings,
    'wave()': action_wave,
    'march()': action_march,
    'show_off()': action_show_off,
    'walk_steps()': action_walk_steps,
    'get_up()': action_get_up,
    'swagger()': action_swagger,
    'left_hook()': action_left_hook,
    'right_hook()': action_right_hook,
    'shake_hands()': action_shake_hands,
    'lean_right()': action_lean_right,
    'hello()': action_hello,
    'follow()': action_follow,
    'stop_follow()': action_stop_follow,
    'two_leg_stand()': action_two_leg_stand,
    'stop_find()': action_stop_find,
    'camera_look_left()': action_camera_look_left,
    'camera_look_right()': action_camera_look_right,
    'camera_sweep()': action_camera_sweep,
    'camera_stop()': action_camera_stop,
    'camera_reset()': action_camera_reset,
    'who_am_i()': action_who_am_i,
    'who_is_he()': action_who_is_he,
    'list_faces()': action_list_faces,
}


# ==================== 关键词快速匹配 ====================

KEYWORD_RULES = [
    (['前进', '往前走', '向前走', '往前', '向前', '前走'], ['forward()'], '冲冲冲'),
    (['后退', '往后走', '向后走', '往后', '向后', '退后', '后走'], ['backward()'], '好的往后退'),
    (['左转', '向左转', '往左转', '向左'], ['turn_left()'], '向左转'),
    (['右转', '向右转', '往右转', '向右'], ['turn_right()'], '向右转'),
    (['停', '停下', '站住', '别动', '停止', '别走了'], ['stop()'], '好的停下'),
    (['坐下', '坐', '蹲下来'], ['sit()'], '坐好了'),
    (['趴下', '趴着', '卧'], ['lie_down()'], '趴下了'),
    (['站起来', '立正', '站好', '起立', '站起'], ['stand()'], '站好了'),
    (['跳舞', '跳个舞', '舞蹈', '来段舞'], ['dance()'], '看我舞姿'),
    (['点头', '点个头'], ['nod()'], '好的好的'),
    (['摇头', '摇摇头'], ['shake_head()'], '不要不要'),
    (['鞠躬', '鞠个躬'], ['bow()'], '给您鞠躬'),
    (['招手', '挥手', '招招手'], ['wave()'], '你好呀'),
    (['俯卧撑', '做俯卧撑'], ['pushup()'], '看我力量'),
    (['加速', '快点', '快一点', '速度快点'], ['speed_up()'], '加速了'),
    (['减速', '慢点', '慢一点', '速度慢点'], ['slow_down()'], '减速了'),
    (['跟我走', '跟着我', '跟上', '跟我来', '开始跟随'], ['follow()'], '好的我跟着你'),
    (['别跟了', '不要跟了', '停止跟随', '取消跟随'], ['stop_follow()'], '好的不跟了'),
    (['握手', '握个手'], ['shake_hands()'], '你好你好'),
    (['走两步', '走几步'], ['walk_steps()'], '走给你看'),
    (['两脚站立', '后腿站立', '用两脚站', '用两条腿站', '后腿站'], ['two_leg_stand()'], '好的，用两条后腿站起来'),
    (['左看', '看左边', '往左看', '摄像头左看', '向左看', '头往左'], ['camera_look_left()'], '好的，往左看'),
    (['右看', '看右边', '往右看', '摄像头右看', '向右看', '头往右'], ['camera_look_right()'], '好的，往右看'),
    (['环顾四周', '环顾', '扫一圈', '扫视', '看一圈', '四周看看'], ['camera_sweep()'], '我看看四周'),
    (['摄像头停下', '摄像头停', '停止摄像头', '停摄像头'], ['camera_stop()'], '摄像头停了'),
    (['回正', '复位', '摄像头复位', '看前面', '看正前方', '正前方', '回到中间', '回中间', '头回正'], ['camera_reset()'], '好的，摄像头回正'),
    (['我是谁', '你认识我吗', '你认识我'], ['who_am_i()'], '让我看看你是谁'),
    (['他是谁', '她是谁', '这是谁', '你认识他吗', '你认识她吗', '摄像头里是谁'], ['who_is_he()'], '让我看看他是谁'),
    (['你认识哪些人', '你记住了谁', '列出认识的人'], ['list_faces()'], '让我看看'),
    (['添加人脸', '记住我', '记住一张脸', '加张人脸', '加脸'], ['add_face()'], '请告诉我你的名字'),
]


def keyword_match(text):
    for keywords, actions, response in KEYWORD_RULES:
        for kw in keywords:
            if kw in text:
                print(f"[快速匹配] 命中关键词：'{kw}' → {actions}")
                return actions, response
    return None


# ==================== LLM 系统提示词 ====================

SYSTEM_PROMPT = '''你是机器狗，根据用户指令输出 JSON。严格只输出 JSON，不要输出其他内容。
格式：{"action":["函数名"],"response":"简短回复"}
可用函数：forward() backward() turn_left() turn_right() stop() stand() lie_down() sit() speed_up() slow_down() nod() sway() squat() shake_head() dance() pushup() kick_left() kick_right() twist_waist() situp() bow() spread_wings() wave() march() show_off() walk_steps() get_up() swagger() left_hook() right_hook() shake_hands() lean_right() hello() follow() stop_follow() two_leg_stand() find_object('物品名') stop_find() camera_look_left() camera_look_right() camera_look('角度') camera_reset() camera_sweep() camera_stop() who_am_i() who_is_he() list_faces() add_face('人名')
摄像头架构（重要）：
- 机器狗有两个摄像头。①前视固定 /usb_cam（不可旋转）；②云台可旋转 /usb_cam_surround（0~360°）。
- 所有 camera_* 函数（camera_look/reset/sweep/look_left/look_right/stop）都只作用于云台摄像头②，前视固定摄像头①不可控。
- 用户说"摄像头转动/看左/看右/环顾四周/转到X度"等旋转意图时，全部走 camera_*，不要尝试操控前视固定摄像头。
- 人脸识别 (who_am_i/who_is_he/list_faces/add_face) 和寻物 (find_object) 默认看的是前视固定摄像头①的视野，不受云台旋转影响。
注意：
- find_object 的物品名用单引号括起来，如 find_object('水杯')
- add_face 的人名用单引号括起来，如 add_face('张三')
- camera_look 的角度用单引号括起来，范围 0~360，如 camera_look(90)、camera_look(270)；camera_look(180) 等价于 camera_reset()
示例：
用户：往前走两步
{"action":["forward()","forward()"],"response":"冲冲冲"}
用户：跳个舞
{"action":["dance()"],"response":"看我舞姿"}
用户：你叫什么
{"action":[],"response":"我是小幻机器狗"}
用户：跟我走
{"action":["follow()"],"response":"好的，我跟着你"}
用户：别跟了
{"action":["stop_follow()"],"response":"好的，不跟了"}
用户：两脚站立
{"action":["two_leg_stand()"],"response":"好的，用两条后腿站起来"}
用户：帮我找一下水杯
{"action":["find_object('水杯')"],"response":"好的，我去找水杯"}
用户：去找手机
{"action":["find_object('手机')"],"response":"好的，我去找手机"}
用户：不用找了
{"action":["stop_find()"],"response":"好的，停止寻找"}
用户：添加张三到人脸库
{"action":["add_face('张三')"],"response":"好的，我记住张三"}
用户：记住李四的脸
{"action":["add_face('李四')"],"response":"好的，我记住李四"}
用户：我是谁
{"action":["who_am_i()"],"response":"让我看看你是谁"}
用户：摄像头里是谁
{"action":["who_is_he()"],"response":"让我看看他是谁"}
用户：你认识哪些人
{"action":["list_faces()"],"response":"我看一下"}
用户：转到90度
{"action":["camera_look(90)"],"response":"好的，转到90度"}
用户：回正
{"action":["camera_reset()"],"response":"好的，摄像头回正"}
用户：往左看
{"action":["camera_look_left()"],"response":"好的，往左看"}'''


# ==================== 语音识别（DashScope Paraformer）====================

def recognize_speech_with_qwen(wav_path):
    """DashScope Paraformer-v2 语音识别。与 voice 脚本一致：调用方负责 unlink。"""
    print("[千问语音识别] 开始识别...")
    try:
        import dashscope
        from dashscope.audio.asr import Recognition, RecognitionCallback

        dashscope.api_key = DASHSCOPE_API_KEY

        recognition = Recognition(
            model='paraformer-realtime-v2',
            callback=RecognitionCallback(),
            format='wav',
            sample_rate=48000,
        )
        result = recognition.call(file=wav_path)

        sentences = result.get_sentence()
        if sentences:
            text = ''.join(
                s.get('text', '') for s in sentences
                if isinstance(s, dict)
            ).strip()
            print(f"[千问语音识别] 识别结果：{text}")
        else:
            print("[千问语音识别] 未识别到内容")
            text = None

        return text
    except ImportError:
        print("[千问语音识别] dashscope SDK 不可用")
        return None
    except Exception as e:
        print(f"[千问语音识别] 错误：{e}")
        traceback.print_exc()
        return None


# ==================== LLM 调用 ====================

def call_llm(text):
    print(f"[LLM] 发送：{text}")
    try:
        kwargs = dict(
            model=QWEN_MODEL,
            messages=[
                {'role': 'system', 'content': SYSTEM_PROMPT},
                {'role': 'user', 'content': text}
            ],
            temperature=0.3,
            extra_body={'enable_thinking': False, 'enable_search': False},
        )
        resp = llm_client.chat.completions.create(**kwargs)
        content = resp.choices[0].message.content
        print(f"[LLM] 原始返回：{content}")

        import re
        content = re.sub(r'<think>.*?</think>', '', content, flags=re.DOTALL).strip()

        start = content.find('{')
        end = content.rfind('}')
        if start != -1 and end != -1:
            json_str = content[start:end + 1]
            result = json.loads(json_str)
            action_list = result.get('action', [])
            response = result.get('response', '收到')
            print(f"[LLM] 解析结果 - 动作：{action_list}, 回复：{response}")
            return action_list, response
        else:
            print("[LLM] 未找到 JSON, 作为纯文本回复")
            return [], content if content else '我没听懂，再说一次？'
    except Exception as e:
        err_str = str(e)
        print(f"[LLM] 错误：{err_str}")
        if 'AuthenticationError' in err_str or 'api_key' in err_str.lower():
            return [], 'API 密钥不对，请检查'
        elif 'ConnectionError' in err_str or 'connect' in err_str.lower():
            return [], '网络连不上，请检查联网'
        return [], '我想了想，没想明白'


# ==================== 动作执行 ====================

def execute_actions(action_list):
    import re
    non_bg = not any(a in ('follow()', 'stop_follow()', 'stop_find()') or
                     a.startswith('find_object(') for a in action_list)
    if follow_running and non_bg:
        stop_follow()
    if find_object_running and non_bg:
        action_stop_find()

    for action_str in action_list:
        if action_str.startswith('find_object('):
            m = re.search(r"find_object\(['\"](.+?)['\"]\)", action_str)
            if m:
                target = m.group(1)
                print(f'[动作] 执行：find_object({target})')
                try:
                    action_find_object(target)
                except Exception as e:
                    print(f'[动作] 执行出错：{e}')
            else:
                print(f'[动作] 无法解析寻物目标：{action_str}')
            continue

        if action_str.startswith('add_face('):
            m = re.search(r"add_face\(['\"](.+?)['\"]\)", action_str)
            if m:
                name = m.group(1)
                print(f'[动作] 执行：add_face({name})')
                try:
                    action_add_face(name)
                except Exception as e:
                    print(f'[动作] 执行出错：{e}')
            else:
                print('[动作] add_face 缺少名字，提示用户')
                try:
                    action_add_face('')
                except Exception as e:
                    print(f'[动作] 执行出错：{e}')
            continue

        if action_str.startswith('camera_look('):
            m = re.search(r"camera_look\(\s*['\"]?([0-9.]+)\s*['\"]?\)", action_str)
            if m:
                angle = float(m.group(1))
                print(f'[动作] 执行：camera_look({angle})')
                try:
                    action_camera_look(angle)
                except Exception as e:
                    print(f'[动作] 执行出错：{e}')
            else:
                print(f'[动作] camera_look 缺少角度：{action_str}')
            continue

        func = ACTION_MAP.get(action_str)
        if func:
            print(f"[动作] 执行：{action_str}")
            try:
                func()
            except Exception as e:
                print(f"[动作] 执行出错：{e}")
        else:
            print(f"[动作] 未知动作：{action_str}")


# ==================== 消息循环 ====================

def message_loop():
    global CURRENT_SENDER
    while run_st and not rospy.is_shutdown():
        try:
            msg = INCOMING_QUEUE.get(timeout=0.1)
        except queue.Empty:
            continue

        with ACTION_LOCK:
            CURRENT_SENDER = msg.from_user
            try:
                _handle_message(msg)
            except Exception as e:
                LOG.error(f'[主循环] 处理消息异常：{e}')
                traceback.print_exc()
            finally:
                CURRENT_SENDER = ''


def _handle_message(msg):
    if msg.msg_type == 'voice':
        if not have_ffmpeg():
            LOG.error('[主循环] 容器未装 ffmpeg，无法处理语音')
            user_announce('听不清，请发文字')
            return
        LOG.info(f'[主循环] 语音消息：{msg.media_path}')
        try:
            wav_path = decode_amr_to_wav(msg.media_path, target_sr=48000)
        except Exception as e:
            LOG.error(f'[主循环] AMR->WAV 失败：{e}')
            user_announce('听不清，请发文字')
            try:
                if msg.media_path:
                    os.unlink(msg.media_path)
            except OSError:
                pass
            return
        try:
            text = recognize_speech_with_qwen(wav_path)
        finally:
            try:
                os.unlink(wav_path)
            except OSError:
                pass
        try:
            if msg.media_path:
                os.unlink(msg.media_path)
        except OSError:
            pass
        if not text:
            user_announce('没听清，请再说一次或发文字')
            return
        LOG.info(f'[主循环] ASR 结果：{text}')
    elif msg.msg_type == 'text':
        text = msg.text
    else:
        return

    if try_handle_pending_face_name(text):
        return

    match = keyword_match(text)
    if match:
        action_list, response = match
    else:
        action_list, response = call_llm(text)

    REPLY_QUEUE.put((msg.from_user, response or '收到'))
    execute_actions(action_list)


# ==================== 主入口 ====================

def load_config(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f'配置文件不存在：{path}')
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def setup_ros():
    global PuppyPosePub, PuppyGaitConfigPub, PuppyVelocityPub, RGBPub
    global run_action_group_srv, PanTiltPanAnglePub, PanTiltSweepSrv, PanTiltStopSrv

    rospy.init_node('wechat_interaction_cloud_ai')
    rospy.on_shutdown(Stop)

    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    RGBPub = rospy.Publisher('/ros_robot_controller/set_rgb', RGBsState, queue_size=1)
    PanTiltPanAnglePub = rospy.Publisher('/camera_pan_tilt/pan_angle', Float32, queue_size=1)

    rospy.sleep(0.5)

    try:
        rospy.wait_for_service('/camera_pan_tilt/sweep', timeout=3.0)
        PanTiltSweepSrv = rospy.ServiceProxy('/camera_pan_tilt/sweep', Trigger)
        rospy.wait_for_service('/camera_pan_tilt/stop', timeout=3.0)
        PanTiltStopSrv = rospy.ServiceProxy('/camera_pan_tilt/stop', Trigger)
        LOG.info('[初始化] 云台服务已连接')
    except Exception as e:
        LOG.warning(f'[初始化] 云台服务不可用 ({e})，将仅使用 pan_angle 话题')

    try:
        rospy.wait_for_service('/puppy_control/runActionGroup', timeout=5.0)
        run_action_group_srv = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)
        LOG.info('[初始化] runActionGroup 服务已连接')
    except Exception as e:
        LOG.warning(f'[初始化] runActionGroup 服务不可用 ({e})，两脚站立将使用姿态近似')

    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'],
        x_shift=PuppyPose['x_shift'], height=PuppyPose['height'],
        roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=500)
    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(overlap_time=GaitConfig['overlap_time'], swing_time=GaitConfig['swing_time'],
        clearance_time=GaitConfig['clearance_time'], z_clearance=GaitConfig['z_clearance'])


def main():
    global DASHSCOPE_API_KEY, llm_client, LOG

    parser = argparse.ArgumentParser(description='企业微信文本/语音控制机器狗')
    parser.add_argument('--config', required=True, help='YAML 配置文件路径')
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()

    config = load_config(args.config)
    DASHSCOPE_API_KEY = config.get('dashscope_api_key') or os.environ.get('DASHSCOPE_API_KEY', '')
    if not DASHSCOPE_API_KEY:
        print('[致命] 未配置 dashscope_api_key，无法调用 LLM/ASR', file=sys.stderr)
        sys.exit(1)

    wework_cfg = config.get('wework', {})
    if os.environ.get('WECOM_CORP_SECRET'):
        wework_cfg['corp_secret'] = os.environ['WECOM_CORP_SECRET']
        LOG.info('[WeCom] corp_secret 由环境变量 WECOM_CORP_SECRET 覆盖')
    config['wework'] = wework_cfg

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s [%(levelname)s] %(message)s',
        datefmt='%H:%M:%S',
    )
    LOG = logging.getLogger('wechat')
    LOG.info('启动 wechat_interaction_cloud_ai')

    llm_client = OpenAI(
        api_key=DASHSCOPE_API_KEY,
        base_url='https://dashscope.aliyuncs.com/compatible-mode/v1',
    )
    LOG.info(f'[LLM] 使用模型：{QWEN_MODEL}')

    setup_ros()

    t_msg = threading.Thread(target=message_loop, name='msg-loop', daemon=True)
    t_msg.start()

    LOG.info('[初始化] 进入 Flask 回调（阻塞主线程）')
    try:
        start_callback_server(config, INCOMING_QUEUE, REPLY_QUEUE, LOG)
    except KeyboardInterrupt:
        LOG.info('KeyboardInterrupt')
    except Exception as e:
        LOG.error(f'[Flask] 致命错误：{e}')
        traceback.print_exc()
    finally:
        Stop()
        stop_follow()
        action_stop_find()
        LOG.info('程序退出')


if __name__ == '__main__':
    main()
