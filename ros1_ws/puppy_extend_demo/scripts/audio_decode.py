#!/usr/bin/env python3
# encoding: utf-8
"""企业微信语音消息 AMR -> WAV。

WeCom 语音文件固定 AMR 编码 (8 kHz mono)，需要转成与现有
recognize_speech_with_qwen 一致的 48 kHz mono WAV。
"""
import os
import shutil
import subprocess
import tempfile


def have_ffmpeg() -> bool:
    return shutil.which('ffmpeg') is not None


def decode_amr_to_wav(amr_path: str, target_sr: int = 48000) -> str:
    """AMR 文件 -> 临时 WAV (48kHz mono by default)。

    Returns: WAV 文件绝对路径。调用方负责 unlink。
    Raises: RuntimeError 如果 ffmpeg 缺失或转码失败。
    """
    if not have_ffmpeg():
        raise RuntimeError(
            '[致命] 容器未安装 ffmpeg，无法处理语音消息。'
            '请在 Dockerfile 中 apt-get install -y ffmpeg'
        )
    if not os.path.exists(amr_path):
        raise FileNotFoundError(amr_path)

    fd, wav_path = tempfile.mkstemp(suffix='.wav')
    os.close(fd)

    cmd = [
        'ffmpeg', '-y',
        '-i', amr_path,
        '-ar', str(target_sr),
        '-ac', '1',
        wav_path,
    ]
    try:
        subprocess.run(
            cmd,
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=10,
        )
    except subprocess.TimeoutExpired:
        _safe_unlink(wav_path)
        raise RuntimeError(f'[音频] ffmpeg 转码超时: {amr_path}')
    except subprocess.CalledProcessError as e:
        _safe_unlink(wav_path)
        raise RuntimeError(f'[音频] ffmpeg 转码失败 rc={e.returncode}: {amr_path}')
    return wav_path


def _safe_unlink(p):
    try:
        os.unlink(p)
    except OSError:
        pass
