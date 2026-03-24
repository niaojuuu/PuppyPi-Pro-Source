#!/usr/bin/env python3
# encoding: utf-8
# 生成声纹验证失败的提示音频 not_owner.wav
# 用法: python generate_not_owner_audio.py

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import *
from speech import speech


def main():
    text = "你不是我的主人，我不听你的命令"
    output_path = not_owner_audio_path

    if not all([tts_api_key, tts_secret_key, tts_cuid]):
        print("错误: config.py 中百度 TTS 密钥未配置")
        print("请先填写 tts_api_key, tts_secret_key, tts_cuid")
        sys.exit(1)

    print(f"正在生成提示音频: {text}")
    tts = speech.TTS(tts_api_key, tts_secret_key, tts_cuid)
    tts.tts(text, file_path=output_path)
    print(f"已保存到: {output_path}")


if __name__ == '__main__':
    main()
