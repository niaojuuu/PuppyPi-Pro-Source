#!/usr/bin/env python3
# encoding: utf-8
"""
云端 AI 语音交互 - 阿里云千问大模型语音识别方案
架构：WonderEchoPro 唤醒 → 麦克风录音 → 阿里云千问 Qwen-Audio 语音识别 + LLM 理解 → 动作执行 + sherpa-onnx TTS
运行环境：树莓派 5 (需联网)
"""
import os
import sys
import json
import math
import time
import wave
import struct
import threading
import serial
import numpy as np
import ctypes
import base64
import tempfile
import requests

# 抑制 ALSA lib 的大量无关警告（Unknown PCM surround50/51/71 等）
try:
    _ERROR_HANDLER = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_int,
                                       ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p)
    def _alsa_error_handler(filename, line, function, err, fmt):
        pass  # 静默所有 ALSA 错误消息
    _alsa_err_cb = _ERROR_HANDLER(_alsa_error_handler)
    ctypes.cdll.LoadLibrary('libasound.so.2').snd_lib_error_set_handler(_alsa_err_cb)
except Exception:
    pass
import pyaudio
import sherpa_onnx
import rospy
from openai import OpenAI
from std_msgs.msg import *
from puppy_control.msg import Velocity, Pose, Gait
from puppy_control.srv import SetRunActionName
from ros_robot_controller.msg import RGBState, RGBsState
import sensor.Sonar as Sonar

print('''
**********************************************************
****  功能：云端 AI 语音交互 - 自然语言控制机器狗  ****
**********************************************************
----------------------------------------------------------
唤醒词："小幻小幻" (通过 WonderEchoPro 语音模块)
语音识别：阿里云千问 Qwen-Audio 语音识别 (云端)
语言理解：阿里云千问 Qwen3.5 大模型
语音合成：sherpa-onnx 本地 TTS (中文 VITS)
----------------------------------------------------------
Tips:
 * 按下 Ctrl+C 可关闭程序
 * 需设置环境变量：export DASHSCOPE_API_KEY="sk-xxxx"
 * 确保树莓派已联网
 * 确保 sherpa-onnx 模型文件存在于 ~/models/sherpa-onnx/
----------------------------------------------------------
''')

# ==================== 配置 ====================

# 模型路径
HOME = os.path.expanduser('~')
TTS_MODEL_DIR = os.path.join(HOME, 'models/sherpa-onnx/vits-zh-hf-fanchen-C')

# 音频路径
WAKEUP_WAV = os.path.join(os.path.dirname(os.path.abspath(__file__)),
    '../../large_models/resources/audio/wakeup.wav')

# 串口配置
SERIAL_BAUD = 115200

def find_serial_port():
    """自动检测 WonderEchoPro 语音模块串口。

    在 /dev/ttyUSB* 中查找 CH340 芯片 (VID:1A86 PID:7523) 的设备。
    如果有多个 CH340 设备，通过 USB 物理端口位置 (LOCATION) 区分。
    首次运行时会在脚本同目录生成 .voice_port_location 文件记录物理端口，
    之后即使设备编号变化也能精准匹配。
    """
    import serial.tools.list_ports
    VOICE_VID = 0x1A86
    VOICE_PID = 0x7523
    LOCATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.voice_port_location')

    # 筛选 ttyUSB 设备中的 CH340
    matches = [p for p in serial.tools.list_ports.comports()
               if p.device.startswith('/dev/ttyUSB') and p.vid == VOICE_VID and p.pid == VOICE_PID]

    if not matches:
        raise RuntimeError("未在 /dev/ttyUSB* 中找到 CH340 设备，请检查语音模块连接")

    if len(matches) == 1:
        port = matches[0]
        # 记录物理端口位置，方便后续多设备时区分
        if port.location:
            with open(LOCATION_FILE, 'w') as f:
                f.write(port.location)
        print(f"[串口] 已识别语音模块：{port.device} (LOCATION={port.location})")
        return port.device

    # 多个 CH340 设备，通过已记录的物理端口位置匹配
    if os.path.exists(LOCATION_FILE):
        with open(LOCATION_FILE, 'r') as f:
            saved_location = f.read().strip()
        for p in matches:
            if p.location == saved_location:
                print(f"[串口] 通过物理端口匹配语音模块：{port.device} (LOCATION={port.location})")
                return p.device

    # 无法自动区分，列出候选让用户确认
    devices_info = ', '.join(f"{p.device}(LOCATION={p.location})" for p in matches)
    raise RuntimeError(
        f"发现多个 CH340 设备：{devices_info}\n"
        f"请手动确认语音模块端口后，将其 LOCATION 写入：{LOCATION_FILE}"
    )

# 录音配置
MIC_SAMPLE_RATE = 48000  # USB 麦克风硬件采样率（48k 是标准 USB 音频采样率）
CHANNELS = 1
MIC_CHUNK = 4800  # 100ms per chunk (48000 * 0.1)
SILENCE_THRESHOLD = 3000  # 能量阈值（根据实际底噪调整）
SILENCE_DURATION = 1.0   # 静音持续秒数触发结束
MAX_RECORD_SECONDS = 15  # 云端识别支持更长录音

# LLM 配置 - 阿里云 DashScope
DASHSCOPE_API_KEY = os.environ.get('DASHSCOPE_API_KEY', 'sk-4fe56ee3d9e44b248516ef93d95190f4')
QWEN_MODEL = 'qwen3.5-plus'  # 语音识别和语言理解都使用 Qwen3.5

if not DASHSCOPE_API_KEY:
    print("[警告] 未设置 DASHSCOPE_API_KEY 环境变量！")
    print("  请执行：export DASHSCOPE_API_KEY=\"sk-xxxx\"")
    print("  获取地址：https://bailian.console.aliyun.com/")

print(f"[LLM] 使用模型：{QWEN_MODEL} (语音识别 + 语言理解)")

# 初始化 OpenAI 兼容客户端（阿里云 DashScope）
llm_client = OpenAI(
    api_key=DASHSCOPE_API_KEY,
    base_url='https://dashscope.aliyuncs.com/compatible-mode/v1',
)

# 机器狗初始配置
PuppyPose = {'roll': math.radians(0), 'pitch': math.radians(0), 'yaw': 0.000,
             'height': -10, 'x_shift': -0.5, 'stance_x': 0, 'stance_y': 0}
GaitConfig = {'overlap_time': 0.2, 'swing_time': 0.2, 'clearance_time': 0.0, 'z_clearance': 3}
BASE_SPEED = 8
speed_factor = 1.0

# ==================== ROS 控制函数 ====================

PuppyPosePub = None
PuppyGaitConfigPub = None
PuppyVelocityPub = None
RGBPub = None
run_action_group_srv = None
tts_engine = None
find_object_running = False

run_st = True

def set_rgb(r, g, b):
    """设置两个板载 RGB 灯的颜色"""
    led1 = RGBState(id=1, r=r, g=g, b=b)
    led2 = RGBState(id=2, r=r, g=g, b=b)
    RGBPub.publish(RGBsState(data=[led1, led2]))

def rgb_off():
    """关闭 RGB 灯"""
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

# ==================== 动作函数 ====================

def action_forward():
    """前进"""
    speed = BASE_SPEED * speed_factor
    reset_pose()
    rospy.sleep(0.5)
    pub_vel(x=speed)
    rospy.sleep(2)
    pub_vel()

def action_backward():
    """后退"""
    speed = BASE_SPEED * speed_factor
    reset_pose()
    rospy.sleep(0.5)
    pub_vel(x=-speed)
    rospy.sleep(2)
    pub_vel()

def action_turn_left():
    """左转"""
    reset_pose()
    rospy.sleep(0.5)
    pub_vel(yaw_rate=0.5)
    rospy.sleep(2)
    pub_vel()

def action_turn_right():
    """右转"""
    reset_pose()
    rospy.sleep(0.5)
    pub_vel(yaw_rate=-0.5)
    rospy.sleep(2)
    pub_vel()

def action_stop():
    """停止"""
    pub_vel()

def action_stand():
    """立正"""
    reset_pose()

def action_lie_down():
    """趴下"""
    pub_pose(height=-6)

def action_sit():
    """坐下"""
    pub_pose(pitch=15, height=-6)

def action_speed_up():
    """加速"""
    global speed_factor
    speed_factor = min(speed_factor + 0.5, 3.0)
    print(f"加速，当前倍率：{speed_factor}")

def action_slow_down():
    """减速"""
    global speed_factor
    speed_factor = max(speed_factor - 0.5, 0.5)
    print(f"减速，当前倍率：{speed_factor}")

def action_nod():
    """点头"""
    for _ in range(3):
        pub_pose(pitch=15)
        rospy.sleep(0.4)
        reset_pose()
        rospy.sleep(0.4)

def action_sway():
    """左右摇摆"""
    for _ in range(3):
        pub_pose(roll=15)
        rospy.sleep(0.4)
        pub_pose(roll=-15)
        rospy.sleep(0.4)
    reset_pose()

def action_squat():
    """蹲起"""
    for _ in range(3):
        pub_pose(height=-6)
        rospy.sleep(0.4)
        pub_pose(height=-12)
        rospy.sleep(0.4)
    reset_pose()

def action_shake_head():
    """摇头"""
    for _ in range(3):
        pub_pose(yaw=0.3)
        rospy.sleep(0.3)
        pub_pose(yaw=-0.3)
        rospy.sleep(0.3)
    reset_pose()

def action_dance():
    """跳舞"""
    for _ in range(4):
        pub_pose(roll=15, height=-8)
        rospy.sleep(0.3)
        pub_pose(roll=-15, height=-12)
        rospy.sleep(0.3)
    reset_pose()

def action_pushup():
    """俯卧撑"""
    for _ in range(5):
        pub_pose(height=-6)
        rospy.sleep(0.5)
        pub_pose(height=-12)
        rospy.sleep(0.5)
    reset_pose()

def action_kick_left():
    """左脚射门"""
    pub_pose(roll=15)
    rospy.sleep(0.5)
    pub_pose(roll=-10, pitch=10)
    rospy.sleep(0.3)
    reset_pose()

def action_kick_right():
    """右脚射门"""
    pub_pose(roll=-15)
    rospy.sleep(0.5)
    pub_pose(roll=10, pitch=10)
    rospy.sleep(0.3)
    reset_pose()

def action_twist_waist():
    """扭腰"""
    for _ in range(3):
        pub_pose(roll=10, yaw=0.3)
        rospy.sleep(0.4)
        pub_pose(roll=-10, yaw=-0.3)
        rospy.sleep(0.4)
    reset_pose()

def action_situp():
    """仰卧起坐"""
    for _ in range(4):
        pub_pose(pitch=20, height=-6)
        rospy.sleep(0.5)
        pub_pose(pitch=-10, height=-10)
        rospy.sleep(0.5)
    reset_pose()

def action_bow():
    """鞠躬"""
    pub_pose(pitch=25)
    rospy.sleep(1.5)
    reset_pose()

def action_spread_wings():
    """大鹏展翅"""
    pub_pose(height=-12)
    rospy.sleep(0.5)
    pub_pose(pitch=15, roll=15, height=-12)
    rospy.sleep(0.5)
    pub_pose(pitch=15, roll=-15, height=-12)
    rospy.sleep(0.5)
    reset_pose()

def action_wave():
    """招手"""
    for _ in range(4):
        pub_pose(pitch=10, yaw=0.2)
        rospy.sleep(0.25)
        pub_pose(pitch=-5, yaw=-0.2)
        rospy.sleep(0.25)
    reset_pose()

def action_march():
    """原地踏步"""
    reset_pose()
    rospy.sleep(0.5)
    pub_vel(x=0.1)
    rospy.sleep(3)
    pub_vel()

def action_show_off():
    """露一手"""
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
    """走两步"""
    reset_pose()
    rospy.sleep(0.5)
    pub_vel(x=8)
    rospy.sleep(2)
    pub_vel()

def action_get_up():
    """跌倒起立"""
    pub_pose(pitch=25, height=-4)
    rospy.sleep(1)
    reset_pose()

def action_swagger():
    """大摇大摆"""
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
    """左勾拳"""
    pub_pose(roll=15, yaw=0.3)
    rospy.sleep(0.3)
    pub_pose(roll=-10, yaw=-0.2)
    rospy.sleep(0.2)
    reset_pose()

def action_right_hook():
    """右勾拳"""
    pub_pose(roll=-15, yaw=-0.3)
    rospy.sleep(0.3)
    pub_pose(roll=10, yaw=0.2)
    rospy.sleep(0.2)
    reset_pose()

def action_shake_hands():
    """握手"""
    for _ in range(3):
        pub_pose(pitch=15, height=-8)
        rospy.sleep(0.3)
        pub_pose(pitch=5, height=-10)
        rospy.sleep(0.3)
    reset_pose()

def action_lean_right():
    """右倾"""
    pub_pose(roll=-15)

def action_hello():
    """打招呼"""
    action_nod()

def action_two_leg_stand():
    """两脚站立（后腿直立）- 通过代码编排尽可能站高站稳，获取更多摄像头视野"""
    global run_action_group_srv
    # 优先调用预制动作组文件
    if run_action_group_srv is not None:
        try:
            run_action_group_srv('2_legs_stand.d6ac', True)
            return
        except Exception as e:
            print(f"[两脚站立] 动作组调用失败 ({e})，使用姿态近似")

    # puppy.py PoseFun 限制：|pitch|<=31°, -15<=height<=-5,
    #   |stance_x|<=5, |stance_y|<=5, |x_shift|<=10
    # 策略：分步渐进——先降重心、展开后腿增稳，再逐步后移重心 + 仰角抬升前身

    # 第 1 步：降低重心，后腿横向展开增加稳定性
    PuppyPosePub.publish(stance_x=0, stance_y=3, x_shift=1.0,
        height=-11, roll=math.radians(0), pitch=math.radians(0), yaw=0, run_time=500)
    rospy.sleep(0.6)

    # 第 2 步：重心开始后移，小幅仰角
    PuppyPosePub.publish(stance_x=0, stance_y=3, x_shift=3.5,
        height=-11, roll=math.radians(0), pitch=math.radians(-12), yaw=0, run_time=500)
    rospy.sleep(0.6)

    # 第 3 步：继续后移重心，加大仰角，前腿开始离地
    PuppyPosePub.publish(stance_x=0, stance_y=3, x_shift=5.5,
        height=-12, roll=math.radians(0), pitch=math.radians(-22), yaw=0, run_time=500)
    rospy.sleep(0.7)

    # 第 4 步：最大站立姿态——大幅后移重心 + 接近仰角上限 + 后腿充分伸展
    PuppyPosePub.publish(stance_x=0, stance_y=3, x_shift=7.0,
        height=-13, roll=math.radians(0), pitch=math.radians(-30), yaw=0, run_time=600)
    rospy.sleep(0.8)

# ==================== 跟随模式（摄像头人体检测） ====================
follow_thread = None
follow_running = False

# ==================== 寻物模式 ====================

def action_find_object(target):
    """搜寻指定物品：正常视角 + 抬高视角 + LLM 决策移动 + 超声波避障"""
    import cv2
    import base64
    import re
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image as RosImage

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

    # 初始化超声波
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
        """停稳后获取清晰帧，编码为 base64 JPEG。"""
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
        """调用视觉大模型，返回去除思考标签后的原始文本"""
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
        """检测图像中是否有目标，返回 (found, position, hint)"""
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
        """根据当前图像和完整搜索历史，让 LLM 决定下一步搜索策略"""
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
        """带超声波实时避障的移动，结束后等待机身稳定"""
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
        """发现目标后持续靠近，以视觉判断为主、超声波仅作防撞保护。"""
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

    # ======== 主搜寻循环 ========
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

    # ======== 清理 ========
    pub_vel()
    image_sub.unregister()
    if sonar:
        sonar.setRGB(0, (0, 0, 0))
        sonar.setRGB(1, (0, 0, 0))
    find_object_running = False

    if found:
        print(f'[寻物] 成功找到 {target}！')
        if tts_engine:
            tts_and_play(tts_engine, '找到了')
    else:
        print(f'[寻物] 未找到 {target}，搜寻结束')
        if tts_engine:
            tts_and_play(tts_engine, f'没有找到{target}')


def action_stop_find():
    """停止寻物模式"""
    global find_object_running
    if find_object_running:
        find_object_running = False
        pub_vel()
        print('[寻物] 已停止寻物')


# ==================== 跟随模式（摄像头人体检测） 续 ====================

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
    """停止跟随模式"""
    global follow_running
    if follow_running:
        follow_running = False
        if follow_thread and follow_thread.is_alive():
            follow_thread.join(timeout=3)
        pub_vel()
        print("[跟随] 已退出跟随模式")

def sonar_speed_from_distance(dist_mm):
    """根据超声波距离返回建议速度"""
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
    """从 MediaPipe 关键点提取人体 bbox 和中心"""
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
    """摄像头人体跟随主循环"""
    global follow_running
    import cv2
    import mediapipe as mp
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image as RosImage

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
    """开始跟随模式"""
    global follow_running, follow_thread
    stop_follow()
    follow_running = True
    follow_thread = threading.Thread(target=follow_loop, daemon=True)
    follow_thread.start()

def action_stop_follow():
    """停止跟随模式"""
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
}

# ==================== 关键词快速匹配（短路 LLM） ====================

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
]

def keyword_match(text):
    """关键词快速匹配"""
    for keywords, actions, response in KEYWORD_RULES:
        for kw in keywords:
            if kw in text:
                print(f"[快速匹配] 命中关键词：'{kw}' → {actions}")
                return actions, response
    return None

# ==================== LLM 系统提示词 ====================

SYSTEM_PROMPT = '''你是机器狗，根据用户指令输出 JSON。严格只输出 JSON，不要输出其他内容。
格式：{"action":["函数名"],"response":"简短回复"}
可用函数：forward() backward() turn_left() turn_right() stop() stand() lie_down() sit() speed_up() slow_down() nod() sway() squat() shake_head() dance() pushup() kick_left() kick_right() twist_waist() situp() bow() spread_wings() wave() march() show_off() walk_steps() get_up() swagger() left_hook() right_hook() shake_hands() lean_right() hello() follow() stop_follow() two_leg_stand() find_object('物品名') stop_find()
注意：find_object 的物品名用单引号括起来，如 find_object('水杯')
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
{"action":["stop_find()"],"response":"好的，停止寻找"}'''

# ==================== TTS 初始化 ====================

def init_tts():
    """初始化 sherpa-onnx TTS"""
    tts_config = sherpa_onnx.OfflineTtsConfig(
        model=sherpa_onnx.OfflineTtsModelConfig(
            vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                model=os.path.join(TTS_MODEL_DIR, 'vits-zh-hf-fanchen-C.onnx'),
                lexicon=os.path.join(TTS_MODEL_DIR, 'lexicon.txt'),
                tokens=os.path.join(TTS_MODEL_DIR, 'tokens.txt'),
            ),
            num_threads=2,
        ),
        rule_fsts=os.path.join(TTS_MODEL_DIR, 'rule.fst') if os.path.exists(os.path.join(TTS_MODEL_DIR, 'rule.fst')) else '',
    )
    tts = sherpa_onnx.OfflineTts(tts_config)
    print("[TTS] 模型加载完成")
    return tts

# ==================== 录音 ====================

def find_usb_mic(pa):
    """查找 USB 麦克风设备索引"""
    for i in range(pa.get_device_count()):
        info = pa.get_device_info_by_index(i)
        if 'USB' in info['name'] and info['maxInputChannels'] > 0:
            print(f"[麦克风] 使用设备：Index {i} - {info['name']}")
            return i
    # 未找到 USB 设备，使用默认输入设备
    default_device = pa.get_default_input_device_info()
    print(f"[麦克风] 使用默认设备：Index {default_device['index']} - {default_device['name']}")
    return default_device['index']

# ==================== 录音 + 保存为 WAV ====================

def record_audio_wav():
    """录音并保存为 WAV 文件，返回文件路径"""
    pa = pyaudio.PyAudio()
    dev_index = find_usb_mic(pa)

    try:
        mic_stream = pa.open(format=pyaudio.paInt16, channels=CHANNELS,
                             rate=MIC_SAMPLE_RATE, input=True,
                             input_device_index=dev_index, frames_per_buffer=MIC_CHUNK)
    except Exception as e:
        print(f"[麦克风] 打开失败：{e}")
        # 列出所有可用输入设备
        print("[麦克风] 可用输入设备:")
        for i in range(pa.get_device_count()):
            info = pa.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                print(f"  Index {i}: {info['name']} (channels={info['maxInputChannels']})")
        pa.terminate()
        raise
    print("[录音] 开始录音...")

    silence_chunks = 0
    speech_detected = False
    max_chunks = int(MAX_RECORD_SECONDS * MIC_SAMPLE_RATE / MIC_CHUNK)
    silence_limit = int(SILENCE_DURATION * MIC_SAMPLE_RATE / MIC_CHUNK)
    total_samples = 0
    audio_frames = []

    for i in range(max_chunks):
        data = mic_stream.read(MIC_CHUNK, exception_on_overflow=False)
        total_samples += MIC_CHUNK
        audio_frames.append(data)

        # 计算能量
        audio_data = np.frombuffer(data, dtype=np.int16)
        energy = np.abs(audio_data).mean()
        if energy >= SILENCE_THRESHOLD:
            speech_detected = True
            silence_chunks = 0
        elif speech_detected:
            silence_chunks += 1

        # 语音出现后，连续静音超过阈值则停止录音
        if speech_detected and silence_chunks >= silence_limit:
            print("[录音] 检测到静音，停止录音")
            break

    mic_stream.stop_stream()
    mic_stream.close()
    pa.terminate()

    duration = total_samples / MIC_SAMPLE_RATE
    print(f"[录音] 完成，时长：{duration:.1f}s")

    # 保存为临时 WAV 文件
    tmp_wav = tempfile.NamedTemporaryFile(suffix='.wav', delete=False)
    tmp_wav_path = tmp_wav.name
    tmp_wav.close()

    with wave.open(tmp_wav_path, 'w') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(2)
        wf.setframerate(MIC_SAMPLE_RATE)
        wf.writeframes(b''.join(audio_frames))

    print(f"[录音] 保存至：{tmp_wav_path}")
    return tmp_wav_path, duration

# ==================== 音频编码为 base64 ====================

def encode_audio_to_base64(wav_path):
    """将 WAV 文件编码为 base64 字符串"""
    with open(wav_path, 'rb') as f:
        audio_data = f.read()
    return base64.b64encode(audio_data).decode('utf-8')

# ==================== 千问语音识别（通过阿里云 FunASR） ====================

def recognize_speech_with_qwen(wav_path):
    """使用阿里云 FunASR 进行语音识别"""
    print("[千问语音识别] 开始识别...")

    try:
        # 使用阿里云 DashScope 的语音识别 API
        import requests

        # 读取音频文件
        with open(wav_path, 'rb') as f:
            audio_bytes = f.read()

        # 方式 1: 使用 DashScope 语音识别 API v2
        url = 'https://dashscope.aliyuncs.com/api/v1/services/audio/asr/transcription'

        # 先上传文件
        upload_url = 'https://dashscope.aliyuncs.com/api/v1/uploads'
        headers = {
            'Authorization': f'Bearer {DASHSCOPE_API_KEY}',
        }

        # 简化方式：直接调用 dashscope 库
        try:
            import dashscope
            from dashscope import AudioTranscription

            # 使用 Paraformer 模型进行语音识别
            result = AudioTranscription.call(
                model='paraformer-turbo-v2',
                format='wav',
                file=wav_path
            )

            if result.status_code == 200:
                text = result.output.text.strip()
                print(f"[千问语音识别] 识别结果：{text}")
            else:
                print(f"[千问语音识别] 错误：{result.message}")
                text = None

        except ImportError:
            # 如果没有安装 dashscope 库，使用 requests 调用 API
            print("[千问语音识别] 使用 HTTP API 调用...")

            # 上传文件获取 URL
            files = {'file': audio_bytes}
            upload_resp = requests.post(upload_url, files=files, headers=headers)
            if upload_resp.status_code != 200:
                print(f"[千问语音识别] 上传失败：{upload_resp.text}")
                return None
            file_url = upload_resp.json()['data']['url']

            # 创建识别任务
            task_url = 'https://dashscope.aliyuncs.com/api/v1/services/audio/asr/transcription/tasks'
            task_data = {
                'model': 'paraformer-turbo-v2',
                'input': {'file_url': file_url},
                'parameters': {'format': 'wav'}
            }
            task_resp = requests.post(task_url, json=task_data, headers=headers)
            if task_resp.status_code != 200:
                print(f"[千问语音识别] 创建任务失败：{task_resp.text}")
                return None
            task_id = task_resp.json()['data']['task_id']

            # 轮询任务状态
            import time
            while True:
                time.sleep(1)
                status_url = f'{task_url}/{task_id}'
                status_resp = requests.get(status_url, headers=headers)
                status = status_resp.json()['data']['task_status']
                if status == 'SUCCEEDED':
                    text = status_resp.json()['data']['output']['text'].strip()
                    print(f"[千问语音识别] 识别结果：{text}")
                    break
                elif status in ('FAILED', 'CANCELED'):
                    print(f"[千问语音识别] 任务失败：{status}")
                    text = None
                    break

        # 清理临时文件
        try:
            os.unlink(wav_path)
        except Exception:
            pass

        return text

    except Exception as e:
        print(f"[千问语音识别] 错误：{e}")
        try:
            os.unlink(wav_path)
        except Exception:
            pass
        return None

# ==================== LLM 调用 ====================

def call_llm(text):
    """调用大模型理解意图"""
    import re
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

# ==================== TTS 合成 + 播放 ====================

def tts_and_play(tts_engine, text):
    """TTS 合成并播放"""
    if not text:
        return
    print(f"[TTS] 合成：{text}")
    try:
        audio = tts_engine.generate(text, sid=0, speed=1.0)
        tmp_wav = '/tmp/tts_output.wav'
        with wave.open(tmp_wav, 'w') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(audio.sample_rate)
            samples = np.array(audio.samples)
            samples = (samples * 32767).astype(np.int16)
            wf.writeframes(samples.tobytes())
        os.system(f'aplay -q {tmp_wav}')
        print("[TTS] 播放完成")
    except Exception as e:
        print(f"[TTS] 错误：{e}")

# ==================== 动作执行 ====================

def execute_actions(action_list):
    """执行动作列表"""
    non_bg = not any(a in ('follow()', 'stop_follow()', 'stop_find()') or
                     a.startswith('find_object(') for a in action_list)
    if follow_running and non_bg:
        stop_follow()
    if find_object_running and non_bg:
        action_stop_find()

    for action_str in action_list:
        if action_str.startswith('find_object('):
            import re
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

        func = ACTION_MAP.get(action_str)
        if func:
            print(f"[动作] 执行：{action_str}")
            try:
                func()
            except Exception as e:
                print(f"[动作] 执行出错：{e}")
        else:
            print(f"[动作] 未知动作：{action_str}")

# ==================== 串口唤醒检测 ====================

def init_serial():
    """初始化串口（自动检测端口）"""
    port = find_serial_port()
    ser = serial.Serial(
        port=port,
        baudrate=SERIAL_BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.02
    )
    ser.rts = False
    ser.dtr = False

    time.sleep(0.1)
    while ser.in_waiting > 0:
        ser.read(ser.in_waiting)

    ser.write(b'\xAA\x55\x09\x00\xFB')
    time.sleep(0.1)
    ser.write(b'\xAA\x55\x06\x00\xFB')
    time.sleep(0.1)
    while ser.in_waiting > 0:
        ser.read(ser.in_waiting)

    print(f"[串口] 已连接 {port}，等待唤醒...")

    return ser

def check_wakeup(ser):
    """检测唤醒词，返回 True 表示被唤醒"""
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        if b'\xAA\x55\x03\x00\xFB' in data:
            print(">>> 唤醒成功")
            return True
    return False

def play_wakeup_reply(tts_engine):
    """用 TTS 合成并播放唤醒回复'我在'"""
    tts_and_play(tts_engine, '我在')

# ==================== 主循环 ====================

def main():
    global PuppyPosePub, PuppyGaitConfigPub, PuppyVelocityPub, RGBPub, run_action_group_srv

    rospy.init_node('voice_interaction_cloud_ai2')
    rospy.on_shutdown(Stop)

    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    RGBPub = rospy.Publisher('/ros_robot_controller/set_rgb', RGBsState, queue_size=1)

    rospy.sleep(0.5)

    try:
        rospy.wait_for_service('/puppy_control/runActionGroup', timeout=5.0)
        run_action_group_srv = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)
        print("[初始化] runActionGroup 服务已连接")
    except Exception as e:
        print(f"[初始化] runActionGroup 服务不可用 ({e})，两脚站立将使用姿态近似")

    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'],
        x_shift=PuppyPose['x_shift'], height=PuppyPose['height'],
        roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=500)
    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(overlap_time=GaitConfig['overlap_time'], swing_time=GaitConfig['swing_time'],
        clearance_time=GaitConfig['clearance_time'], z_clearance=GaitConfig['z_clearance'])

    ser = init_serial()

    print("[初始化] 加载 TTS 模型...")
    global tts_engine
    tts_engine = init_tts()

    play_wakeup_reply(tts_engine)
    print("\n===== 系统就绪，请说「小幻小幻」唤醒 =====\n")

    while run_st and not rospy.is_shutdown():
        try:
            if check_wakeup(ser):
                play_wakeup_reply(tts_engine)
                last_interact_time = time.time()

                while time.time() - last_interact_time < 20 and run_st and not rospy.is_shutdown():
                    set_rgb(255, 0, 0)

                    # 录音并保存为 WAV
                    wav_path, duration = record_audio_wav()

                    rgb_off()
                    if duration < 0.3:
                        print("[跳过] 录音太短")
                        continue

                    last_interact_time = time.time()

                    # 使用千问进行语音识别
                    set_rgb(0, 255, 0)
                    text = recognize_speech_with_qwen(wav_path)
                    rgb_off()

                    if not text:
                        print("[跳过] 未识别到语音")
                        continue

                    # 先尝试关键词快速匹配
                    match_result = keyword_match(text)
                    if match_result:
                        action_list, response = match_result
                    else:
                        tts_and_play(tts_engine, '好')
                        set_rgb(0, 255, 0)
                        action_list, response = call_llm(text)
                        rgb_off()

                    tts_thread = threading.Thread(target=tts_and_play, args=(tts_engine, response))
                    tts_thread.start()
                    time.sleep(0.3)
                    execute_actions(action_list)
                    tts_thread.join()

                    last_interact_time = time.time()
                    print(f"[连续对话] 等待下一条指令（{20}秒超时）...")

                print("[连续对话] 对话结束，等待重新唤醒")

            rospy.sleep(0.05)

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"[错误] {e}")
            import traceback
            traceback.print_exc()

    print("\n程序已退出")

if __name__ == '__main__':
    main()
