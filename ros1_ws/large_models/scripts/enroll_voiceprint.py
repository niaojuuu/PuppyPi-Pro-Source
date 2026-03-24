#!/usr/bin/env python3
# encoding: utf-8
# 声纹录入脚本 - 录制主人语音样本并保存声纹特征
# 用法: python enroll_voiceprint.py [--samples 3] [--duration 5]

import os
import sys
import argparse
import tempfile
import wave

# 将当前目录加入 path，以便导入同级模块
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import owner_embedding_path, voiceprint_dir
from voiceprint import VoicePrint


def record_audio(duration=5, sample_rate=16000):
    """
    使用 pyaudio 录制音频
    :param duration: 录音时长（秒）
    :param sample_rate: 采样率
    :return: 临时音频文件路径
    """
    try:
        import pyaudio
    except ImportError:
        print("错误: 需要安装 pyaudio 库")
        print("请运行: pip install pyaudio")
        sys.exit(1)

    chunk = 1024
    format = pyaudio.paInt16
    channels = 1

    p = pyaudio.PyAudio()
    stream = p.open(
        format=format,
        channels=channels,
        rate=sample_rate,
        input=True,
        frames_per_buffer=chunk
    )

    print(f"  正在录音 {duration} 秒，请说话...")
    frames = []
    for _ in range(0, int(sample_rate / chunk * duration)):
        data = stream.read(chunk, exception_on_overflow=False)
        frames.append(data)

    print("  录音结束")
    stream.stop_stream()
    stream.close()
    p.terminate()

    # 保存到临时文件
    tmp_file = tempfile.NamedTemporaryFile(suffix='.wav', delete=False)
    tmp_path = tmp_file.name
    tmp_file.close()

    wf = wave.open(tmp_path, 'wb')
    wf.setnchannels(channels)
    wf.setsampwidth(p.get_sample_size(format))
    wf.setframerate(sample_rate)
    wf.writeframes(b''.join(frames))
    wf.close()

    return tmp_path


def main():
    parser = argparse.ArgumentParser(description='主人声纹录入工具')
    parser.add_argument('--samples', type=int, default=3, help='录制样本数量 (默认: 3)')
    parser.add_argument('--duration', type=int, default=5, help='每段录音时长/秒 (默认: 5)')
    parser.add_argument('--threshold', type=float, default=None, help='声纹匹配阈值 (默认使用config中的值)')
    args = parser.parse_args()

    print("=" * 50)
    print("  PuppyPi Pro 声纹录入工具")
    print("=" * 50)

    # 确保目录存在
    os.makedirs(voiceprint_dir, exist_ok=True)

    voiceprint = VoicePrint(owner_embedding_path, threshold=args.threshold or 0.7)

    if voiceprint.is_enrolled():
        confirm = input("检测到已有声纹记录，是否重新录入？(y/n): ").strip().lower()
        if confirm != 'y':
            print("已取消")
            return

    print(f"\n将录制 {args.samples} 段语音，每段 {args.duration} 秒")
    print("请在每段录音开始后，用正常说话的音量和语速说话\n")

    audio_paths = []
    for i in range(args.samples):
        input(f"按 Enter 开始第 {i + 1}/{args.samples} 段录音...")
        path = record_audio(duration=args.duration)
        audio_paths.append(path)
        print()

    print("正在处理声纹...")
    success = voiceprint.enroll_from_samples(audio_paths)

    # 清理临时文件
    for path in audio_paths:
        try:
            os.unlink(path)
        except OSError:
            pass

    if success:
        print(f"\n声纹录入成功！保存路径: {owner_embedding_path}")
        print("现在只有主人的语音命令会被执行。")
    else:
        print("\n声纹录入失败，请重试。")


if __name__ == '__main__':
    main()
