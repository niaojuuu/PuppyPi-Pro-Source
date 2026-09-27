#!/usr/bin/env python3
# encoding: utf-8
# 声纹录入 / 管理脚本（带语音提示）
# 用法:
#   python enroll_voiceprint.py                       # 默认模式：注册"主人"声纹（旧行为）
#   python enroll_voiceprint.py --name 张三             # 注册张三的声纹（写入 voiceprint_db.yaml）
#   python enroll_voiceprint.py --list                 # 列出已录入的人
#   python enroll_voiceprint.py --remove 张三           # 删除张三的声纹
# 公共参数:
#   --samples 3      录制样本数量
#   --duration 5     每段录音时长/秒
#   --threshold 0.7  声纹匹配阈值（仅在使用多用户库时生效）
#   --silent         关闭语音播报，仅打印文字（自动化/测试用）
#
# 语音提示:
#   - 默认会通过 sherpa-onnx TTS 播报提示音（与 voice_interaction_cloud_ai2.py 同源）
#   - 需要 ~/models/sherpa-onnx/vits-zh-hf-fanchen-C/ 模型目录存在
#   - 模型缺失或 sherpa-onnx 未装时自动降级为纯文字，不影响功能
#   - 播放依赖系统 aplay (ALSA)。无 aplay 时静默跳过

import os
import sys
import time
import argparse
import tempfile
import wave

# 将当前目录加入 path，以便导入同级模块
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import owner_embedding_path, voiceprint_dir, voiceprint_db_path
from voiceprint import VoicePrint, VoicePrintDB


# ==================== TTS 语音播报（可选） ====================

# 优先用 VOICEPRINT_TTS_MODEL_DIR 环境变量;其次 /home/ubuntu(ros 容器内统一位置);
# 再次回退到 ~/models(开发机上能命中)。这样不管 SSH/roslaunch 怎么传 HOME 都不会炸。
_TTS_CANDIDATES = [
    os.environ.get('VOICEPRINT_TTS_MODEL_DIR', '').strip(),
    '/home/ubuntu/models/sherpa-onnx/vits-zh-hf-fanchen-C',
    os.path.join(os.path.expanduser('~'), 'models/sherpa-onnx/vits-zh-hf-fanchen-C'),
]
_TTS_MODEL_DIR = next((p for p in _TTS_CANDIDATES if p and os.path.isdir(p)), '')
_TTS_WAV_TMP = '/tmp/tts_output.wav'

_tts_engine = None
_tts_tried = False
TTS_OK = False  # 是否真的能播报


def _init_tts():
    """懒加载 TTS 引擎。失败不抛异常，返回 None。"""
    global _tts_engine, _tts_tried, TTS_OK
    if _tts_tried:
        return _tts_engine
    _tts_tried = True
    model_path = os.path.join(_TTS_MODEL_DIR, 'vits-zh-hf-fanchen-C.onnx')
    if not os.path.exists(model_path):
        print(f'[TTS] 模型目录不存在：{_TTS_MODEL_DIR}（仅文字提示）')
        return None
    try:
        import sherpa_onnx
    except Exception as e:
        print(f'[TTS] sherpa_onnx 未安装 ({e})，仅文字提示')
        return None
    try:
        tts_config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                    model=model_path,
                    lexicon=os.path.join(_TTS_MODEL_DIR, 'lexicon.txt'),
                    tokens=os.path.join(_TTS_MODEL_DIR, 'tokens.txt'),
                ),
                num_threads=2,
            ),
            rule_fsts=os.path.join(_TTS_MODEL_DIR, 'rule.fst')
                if os.path.exists(os.path.join(_TTS_MODEL_DIR, 'rule.fst')) else '',
        )
        _tts_engine = sherpa_onnx.OfflineTts(tts_config)
        TTS_OK = True
        print('[TTS] TTS 引擎加载成功，将语音播报提示')
        return _tts_engine
    except Exception as e:
        print(f'[TTS] 初始化失败 ({e})，仅文字提示')
        return None


def _play_wav(path):
    """尝试用系统命令播放 wav；都失败就静默跳过。
    播完后稍等 2s 让 ALSA / PulseAudio 真正释放音频设备，避免下次 pyaudio 录音时撞车。"""
    # 优先 aplay (ALSA)，回退 paplay (PulseAudio)，再回退 sox
    for cmd in (f'aplay -q {path}', f'paplay {path}', f'play -q {path}'):
        try:
            ret = os.system(f'{cmd} >/dev/null 2>&1')
            if ret == 0:
                time.sleep(2.0)
                return True
        except Exception:
            pass
    return False


def _say(text, silent=False):
    """打印文字 + 语音播报（若 TTS 可用）"""
    text = (text or '').strip()
    if not text:
        return
    print(text)
    if silent:
        return
    engine = _init_tts()
    if engine is None:
        return
    try:
        import numpy as np
        audio = engine.generate(text, sid=0, speed=1.0)
        with wave.open(_TTS_WAV_TMP, 'w') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(audio.sample_rate)
            samples = np.array(audio.samples)
            samples = (samples * 32767).astype(np.int16)
            wf.writeframes(samples.tobytes())
        _play_wav(_TTS_WAV_TMP)
    except Exception as e:
        print(f'[TTS] 播报失败 ({e})')


def _ask(prompt, silent=False):
    """TTS 提示并等待用户在终端输入"""
    _say(prompt, silent=silent)
    try:
        return input('').strip()
    except EOFError:
        return ''


# ==================== 录音 ====================

def record_audio(duration=5, sample_rate=48000, silent=False):
    """
    使用 pyaudio 录制固定时长音频
    默认 48000 Hz 匹配 USB PnP 麦(其它采样率返回 Invalid sample rate);
    Resemblyzer 内部会自动重采样到 16000,不影响声纹特征。
    :return: 临时音频文件路径
    """
    try:
        import pyaudio
    except ImportError:
        _say('错误：需要安装 pyaudio 库，请运行 pip install pyaudio', silent=silent)
        sys.exit(1)

    _say(f'正在录音 {duration} 秒，请说话。', silent=silent)

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

    frames = []
    for _ in range(0, int(sample_rate / chunk * duration)):
        data = stream.read(chunk, exception_on_overflow=False)
        frames.append(data)

    stream.stop_stream()
    stream.close()
    p.terminate()
    _say('本段录音结束', silent=silent)

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


def _record_samples(samples, duration, silent=False):
    """录制多段样本并返回路径列表。每次录音前 TTS 播报提示。"""
    _say(f'接下来要录 {samples} 段声音，每段 {duration} 秒。请用正常说话的音量。', silent=silent)
    audio_paths = []
    for i in range(samples):
        input(f'  [按 Enter 开始第 {i + 1}/{samples} 段录音...]')
        path = record_audio(duration=duration, silent=silent)
        audio_paths.append(path)
        print()
    return audio_paths


def _cleanup(paths):
    for path in paths:
        try:
            os.unlink(path)
        except OSError:
            pass


# ==================== 命令实现 ====================

def cmd_enroll_owner(args):
    """旧的主人声纹录入（保持向后兼容）"""
    _say('=' * 30 + ' 主人声纹录入 ' + '=' * 30, silent=args.silent)
    _say('PuppyPi Pro 主人声纹录入工具', silent=args.silent)

    os.makedirs(voiceprint_dir, exist_ok=True)
    voiceprint = VoicePrint(owner_embedding_path, threshold=args.threshold or 0.7)

    if voiceprint.is_enrolled():
        ans = _ask('检测到已有主人声纹，是否重新录入？说 y 确认', silent=args.silent)
        if ans.lower() != 'y':
            _say('已取消', silent=args.silent)
            return

    audio_paths = _record_samples(args.samples, args.duration, silent=args.silent)
    success = voiceprint.enroll_from_samples(audio_paths)
    _cleanup(audio_paths)

    if success:
        _say(f'声纹录入成功。保存路径：{owner_embedding_path}。现在只有主人的语音命令会被执行。', silent=args.silent)
    else:
        _say('声纹录入失败，请重试。', silent=args.silent)


def cmd_register(args):
    """注册指定名字的声纹（多用户库）"""
    _say(f'PuppyPi Pro 声纹注册：{args.name}', silent=args.silent)

    os.makedirs(voiceprint_dir, exist_ok=True)
    db = VoicePrintDB(voiceprint_db_path, threshold=args.threshold or 0.7)

    if not args.force and args.name in db.list_people():
        ans = _ask(f'声纹库中已有 {args.name}，是否覆盖？说 y 确认', silent=args.silent)
        if ans.lower() != 'y':
            _say('已取消', silent=args.silent)
            return

    audio_paths = _record_samples(args.samples, args.duration, silent=args.silent)
    ok, msg = db.register(args.name, audio_paths, overwrite=args.force)
    _cleanup(audio_paths)

    _say(msg, silent=args.silent)
    if ok:
        people = db.list_people()
        names = '、'.join(people) if people else '空'
        _say(f'当前声纹库共 {len(people)} 人：{names}', silent=args.silent)


def cmd_list(args):
    _say('PuppyPi Pro 声纹库', silent=args.silent)
    db = VoicePrintDB(voiceprint_db_path, threshold=args.threshold or 0.7)
    people = db.list_people()
    if people:
        _say(f'已注册 {len(people)} 人：{"、".join(people)}', silent=args.silent)
    else:
        _say('声纹库是空的', silent=args.silent)
        _say('可以这样注册：python enroll_voiceprint.py --name 张三', silent=args.silent)


def cmd_remove(args):
    _say('PuppyPi Pro 删除声纹', silent=args.silent)
    db = VoicePrintDB(voiceprint_db_path, threshold=args.threshold or 0.7)

    if not args.yes:
        ans = _ask(f'确认删除 {args.name} 的声纹？说 y 确认', silent=args.silent)
        if ans.lower() != 'y':
            _say('已取消', silent=args.silent)
            return

    ok, msg = db.remove(args.name)
    _say(msg, silent=args.silent)
    if ok:
        people = db.list_people()
        names = '、'.join(people) if people else '空'
        _say(f'剩余 {len(people)} 人：{names}', silent=args.silent)


# ==================== 入口 ====================

def main():
    parser = argparse.ArgumentParser(
        description='PuppyPi Pro 声纹录入 / 管理工具（带语音提示）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--samples', type=int, default=3, help='录制样本数量 (默认: 3)')
    parser.add_argument('--duration', type=int, default=5, help='每段录音时长/秒 (默认: 5)')
    parser.add_argument('--threshold', type=float, default=None, help='声纹匹配阈值 (默认使用config中的值)')
    parser.add_argument('--silent', action='store_true', help='关闭语音播报，仅打印文字')

    group = parser.add_mutually_exclusive_group()
    group.add_argument('--name', type=str, help='注册/录入指定名字的声纹（写入 voiceprint_db.yaml）')
    group.add_argument('--list', action='store_true', help='列出已注册的声纹')
    group.add_argument('--remove', type=str, metavar='NAME', help='删除指定名字的声纹')

    parser.add_argument('--force', action='store_true', help='覆盖已有同名记录（不询问）')
    parser.add_argument('--yes', '-y', action='store_true', help='删除时跳过确认')

    args = parser.parse_args()

    if args.list:
        cmd_list(args)
    elif args.remove is not None:
        cmd_remove(_RemNamespace(args, args.remove))
    elif args.name is not None:
        cmd_register(args)
    else:
        cmd_enroll_owner(args)


class _RemNamespace:
    """把 --remove 的值塞进 namespace 让 cmd_remove 能读到 name"""
    def __init__(self, original, name):
        self.__dict__.update(original.__dict__)
        self.name = name


if __name__ == '__main__':
    main()
