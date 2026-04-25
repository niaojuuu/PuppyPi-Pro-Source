#!/usr/bin/env python3
# encoding: utf-8
import os

# 百度相关的密钥
llm_app_id = ""
llm_conversation_id = ""
llm_token = ""

asr_api_key = ''
asr_secret_key = ''
asr_cuid = ''

tts_api_key = ''
tts_secret_key = ''
tts_cuid = ""

# 零一万物相关key
lingyi_api_key=''
lingyi_base_url=''
# 获取程序所在路径
code_path = os.path.abspath(os.path.split(os.path.realpath(__file__))[0])

# 唤醒模型的路径
wakeup_model_path = [os.path.join(code_path, 'resources/models', 'xiaohuan.pmdl')]

# 录音音频的路径
recording_audio_path = os.path.join(code_path, 'resources/audio', 'recording.wav')

# 语音合成音频的路径
tts_audio_path = os.path.join(code_path, 'resources/audio', "tts_audio.wav")

# 启动音频的路径
start_audio_path = os.path.join(code_path, 'resources/audio', "start_audio.wav")

# 唤醒回答音频的路径
wakeup_audio_path = os.path.join(code_path, 'resources/audio', "wakeup.wav")

# 出错音频的路径
error_audio_path = os.path.join(code_path, 'resources/audio', "error.wav")

# 没有检测到声音时音频的路径
no_voice_audio_path = os.path.join(code_path, 'resources/audio', "no_voice.wav")

# 录音完成时音频的路径
dong_audio_path = os.path.join(code_path, 'resources/audio', "dong.wav")

# 声纹相关
voiceprint_dir = os.path.join(code_path, 'resources', 'voiceprint')
owner_embedding_path = os.path.join(voiceprint_dir, 'owner_embedding.npy')
voiceprint_threshold = 0.7  # 声纹匹配阈值（0~1），越高越严格
not_owner_audio_path = os.path.join(code_path, 'resources', 'audio', 'not_owner.wav')
