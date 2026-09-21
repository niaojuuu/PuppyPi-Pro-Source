#!/usr/bin/env python3
# coding=utf8
"""
本地人脸识别库
- 优先使用 face_recognition (dlib, 128维特征)
- 缺失时回退到 MediaPipe FaceMesh (468个3D关键点, 几何特征)

配置文件: puppy_extend_demo/config/face_db.yaml
格式:
    名字:
      encoding: [float, float, ...]
      created_at: "2026-09-21 10:00:00"
      sample_count: 1
"""
import os
import time
import threading
import numpy as np
import cv2

# ---------- 配置 ----------
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(THIS_DIR, '..', 'config')
FACE_DB_PATH = os.path.join(CONFIG_DIR, 'face_db.yaml')

# 匹配阈值 (face_recognition 推荐 0.6，越小越严格；0.45 较严)
FACE_MATCH_TOLERANCE = 0.45

# 取帧质量评估：最多连拍 N 帧，挑人脸最大那张
CAPTURE_MAX_FRAMES = 15
CAPTURE_INTERVAL = 0.20  # 秒

# MediaPipe 检测置信度（树莓派摄像头+室内可能有噪点，0.3 容易漏；降到 0.2 更稳）
MEDIAPIPE_DETECT_CONF = 0.2


def _enhance_for_face_detect(bgr):
    """
    预处理：CLAHE 直方图均衡化（处理逆光/欠曝），增强人脸区域对比度。
    仅用于检测阶段，不影响最终编码质量（编码仍用原图）。
    """
    try:
        # BGR -> LAB，对 L 通道做 CLAHE，保留色彩
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        merged = cv2.merge([l, a, b])
        return cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)
    except Exception:
        return bgr

# ---------- 依赖探测 ----------
FACE_REC_OK = False
MEDIAPIPE_OK = False
face_recognition = None
mp = None

try:
    import face_recognition as _fr
    face_recognition = _fr
    FACE_REC_OK = True
    print('[FaceLib] 使用 face_recognition (dlib, 128维)')
except Exception:
    FACE_REC_OK = False
    print('[FaceLib] face_recognition 不可用，将尝试 MediaPipe 兜底')

if not FACE_REC_OK:
    try:
        import mediapipe as mp  # type: ignore
        MEDIAPIPE_OK = True
        print('[FaceLib] 回退到 MediaPipe FaceMesh (468关键点)')
    except Exception:
        MEDIAPIPE_OK = False
        print('[FaceLib] MediaPipe 也不可用，人脸识别功能不可用')


if not FACE_REC_OK and not MEDIAPIPE_OK:
    raise ImportError('人脸识别功能依赖全部缺失：请安装 face_recognition 或 mediapipe')


# ---------- 配置加载/保存 (纯 Python，避免 yaml 依赖) ----------
def _ensure_db_file():
    """确保配置文件存在"""
    os.makedirs(CONFIG_DIR, exist_ok=True)
    if not os.path.exists(FACE_DB_PATH):
        with open(FACE_DB_PATH, 'w', encoding='utf-8') as f:
            f.write('# 人脸库配置：{名字: [128维特征编码]}\n')
            f.write('# 自动生成，请勿手动删除\n')


def load_face_db():
    """
    加载人脸库
    返回: dict[str, np.ndarray]
    """
    _ensure_db_file()
    db = {}
    try:
        import yaml  # type: ignore
        with open(FACE_DB_PATH, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        for name, info in data.items():
            if isinstance(info, dict) and 'encoding' in info:
                db[name] = np.array(info['encoding'], dtype=np.float32)
            elif isinstance(info, list):
                # 兼容纯列表格式
                db[name] = np.array(info, dtype=np.float32)
    except ImportError:
        # 没装 yaml 时，用简单手写解析
        db = _parse_simple_yaml()
    except Exception as e:
        print(f'[FaceLib] 加载人脸库出错：{e}')
    return db


def _parse_simple_yaml():
    """
    简易 yaml 解析（仅支持本项目生成的格式），避免强制依赖 PyYAML
    """
    db = {}
    if not os.path.exists(FACE_DB_PATH):
        return db
    current_name = None
    current_vec = []
    with open(FACE_DB_PATH, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.rstrip()
            if not line or line.startswith('#'):
                continue
            # 名字行:  "张三:"
            if not line.startswith(' ') and line.endswith(':'):
                # 保存上一个
                if current_name and current_vec:
                    db[current_name] = np.array(current_vec, dtype=np.float32)
                current_name = line[:-1].strip()
                current_vec = []
                continue
            # encoding 行: "  encoding: [..]"
            stripped = line.strip()
            if stripped.startswith('encoding:'):
                vec_str = stripped[len('encoding:'):].strip()
                if vec_str.startswith('[') and vec_str.endswith(']'):
                    vec_str = vec_str[1:-1]
                    try:
                        current_vec = [float(x.strip()) for x in vec_str.split(',') if x.strip()]
                    except Exception:
                        current_vec = []
            # 其他字段(创建时间、计数)跳过
    if current_name and current_vec:
        db[current_name] = np.array(current_vec, dtype=np.float32)
    return db


def save_face_db(db):
    """
    保存人脸库
    db: dict[str, np.ndarray]
    """
    _ensure_db_file()
    try:
        import yaml  # type: ignore
        out = {}
        for name, vec in db.items():
            out[name] = {
                'encoding': [float(x) for x in vec.tolist()],
                'created_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                'sample_count': 1,
            }
        with open(FACE_DB_PATH, 'w', encoding='utf-8') as f:
            yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False)
    except ImportError:
        # 手写格式
        with open(FACE_DB_PATH, 'w', encoding='utf-8') as f:
            f.write('# 人脸库配置：{名字: 128维特征编码}\n')
            f.write(f'# 更新时间: {time.strftime("%Y-%m-%d %H:%M:%S")}\n')
            for name, vec in db.items():
                arr = [float(x) for x in vec.tolist()]
                # 写多行避免单行过长
                f.write(f'{name}:\n')
                f.write(f'  encoding: [{", ".join(f"{x:.6f}" for x in arr)}]\n')
                f.write(f'  created_at: "{time.strftime("%Y-%m-%d %H:%M:%S")}"\n')
                f.write(f'  sample_count: 1\n')
    print(f'[FaceLib] 人脸库已保存：{FACE_DB_PATH} (共 {len(db)} 人)')


# ---------- 检测 + 特征提取 ----------
def _diagnose_no_face(stats):
    """
    根据捕获统计推断"为什么没看到人脸"，返回 TTS 友好的中文提示。
    stats: dict，含 frames_seen / mean_brightness / attempts
    """
    frames = stats.get('frames_seen', 0)
    if frames == 0:
        return '摄像头没拿到画面，请检查一下摄像头'
    mean = stats.get('mean_brightness', 128)
    if mean < 40:
        return '光线太暗了，换个亮一点的地方再试'
    if mean > 220:
        return '光线太强逆光了，换个角度再试'
    return '没看到你，请面对摄像头站好'


def _capture_best_frame(image_callback, timeout=5.0):
    """
    连续取若干帧，返回人脸面积最大的一帧（OpenCV BGR 格式）
    image_callback: callable() -> BGR ndarray or None
    返回: (best_frame, best_area, stats) ; (None, 0, stats) 表示未取到帧或未检测到人脸
    stats: dict {frames_seen, attempts, mean_brightness, best_area}
        - frames_seen: 实际拿到帧的数量
        - attempts: 回调尝试次数（含拿到 None 的次数）
        - mean_brightness: 检测阶段最佳帧的灰度均值（无检测命中时取所有帧中最亮的一帧）
        - best_area: 检测阶段命中的最大面积

    策略：每帧同时用原图和 CLAHE 增强图各跑一次检测（双保险：逆光下增强图更易出结果）。
    """
    deadline = time.time() + timeout
    best_frame = None
    best_area = 0
    best_brightness = 0.0  # 最大亮度（用于诊断）
    attempts = 0
    frames_seen = 0

    while time.time() < deadline and attempts < CAPTURE_MAX_FRAMES:
        frame = image_callback()
        if frame is None:
            time.sleep(CAPTURE_INTERVAL)
            attempts += 1
            continue
        frames_seen += 1

        # 记录本帧亮度（灰度均值）
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cur_brightness = float(gray.mean())
        except Exception:
            cur_brightness = 0.0
        if cur_brightness > best_brightness:
            best_brightness = cur_brightness

        # 原图 + 增强图各跑一次
        candidates = [(frame, '原图')]
        enhanced = _enhance_for_face_detect(frame)
        if enhanced is not frame:
            candidates.append((enhanced, '增强图'))

        for img, label in candidates:
            area, ok = _detect_face_area(img)
            if ok and area > best_area:
                best_area = area
                # 最终保存原图（用于编码，保持色彩真实）
                best_frame = frame
                if best_area > 0:
                    print(f'[FaceLib] 命中 ({label}) 面积={int(best_area)}', flush=True)

        time.sleep(CAPTURE_INTERVAL)
        attempts += 1

    stats = {
        'frames_seen': frames_seen,
        'attempts': attempts,
        'mean_brightness': best_brightness,
        'best_area': best_area,
    }
    print(f'[FaceLib] 连拍结束：attempts={attempts}, 拿到帧={frames_seen}, '
          f'最佳面积={int(best_area)}, 亮度={best_brightness:.1f}', flush=True)
    return best_frame, best_area, stats


def _detect_face_area(bgr):
    """
    检测单帧的人脸面积。
    返回: (area, ok) ; ok=True 表示检测到
    """
    try:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    except Exception:
        return 0, False

    if FACE_REC_OK:
        try:
            locs = face_recognition.face_locations(rgb, model='hog')
            if locs:
                top, right, bottom, left = locs[0]
                area = (right - left) * (bottom - top)
                return area, True
        except Exception as e:
            print(f'[FaceLib] dlib 检测出错：{e}', flush=True)
        return 0, False

    if MEDIAPIPE_OK:
        try:
            with mp.solutions.face_mesh.FaceMesh(
                static_image_mode=True, max_num_faces=1,
                refine_landmarks=False, min_detection_confidence=MEDIAPIPE_DETECT_CONF) as fm:
                res = fm.process(rgb)
                if res.multi_face_landmarks:
                    ih, iw = bgr.shape[:2]
                    xs = [lm.x * iw for lm in res.multi_face_landmarks[0].landmark]
                    ys = [lm.y * ih for lm in res.multi_face_landmarks[0].landmark]
                    area = (max(xs) - min(xs)) * (max(ys) - min(ys))
                    return float(area), True
        except Exception as e:
            print(f'[FaceLib] MediaPipe 检测出错：{e}', flush=True)
        return 0, False

    return 0, False


def extract_face_encoding(frame_bgr):
    """
    从单帧 BGR 图像中提取人脸特征编码。
    返回 np.ndarray or None (检测不到人脸或检测到多张脸时返回 None)
    """
    if frame_bgr is None:
        return None

    if FACE_REC_OK:
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        locs = face_recognition.face_locations(rgb, model='hog')
        if not locs:
            return None
        if len(locs) > 1:
            print(f'[FaceLib] 检测到 {len(locs)} 张人脸，请保持单人', flush=True)
            return None
        encs = face_recognition.face_encodings(rgb, known_face_locations=locs)
        if not encs:
            return None
        return np.array(encs[0], dtype=np.float32)

    if MEDIAPIPE_OK:
        # 先用增强图检测，再用原图提取（编码用原图保持色彩真实）
        enhanced = _enhance_for_face_detect(frame_bgr)
        rgb_enh = cv2.cvtColor(enhanced, cv2.COLOR_BGR2RGB)
        with mp.solutions.face_mesh.FaceMesh(
            static_image_mode=True, max_num_faces=1,
            refine_landmarks=False, min_detection_confidence=MEDIAPIPE_DETECT_CONF) as fm:
            res = fm.process(rgb_enh)
            if not res.multi_face_landmarks:
                # 退化：再试原图
                rgb_orig = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                res = fm.process(rgb_orig)
                if not res.multi_face_landmarks:
                    return None
                lms = res.multi_face_landmarks[0].landmark
            else:
                lms = res.multi_face_landmarks[0].landmark
        # 468 关键点 × 3 维 = 1404 维特征（归一化坐标）
        vec = []
        for lm in lms:
            vec.extend([lm.x, lm.y, lm.z])
        return np.array(vec, dtype=np.float32)

    return None


# ---------- 高层接口 ----------
class FaceRecognizer:
    """
    人脸识别器：负责注册和识别
    使用方式：
        fr = FaceRecognizer()
        # 注册（image_callback: callable -> BGR ndarray or None）
        fr.register(name='张三', image_callback=lambda: latest_frame[0])
        # 识别
        name, ok = fr.recognize(image_callback=lambda: latest_frame[0])
    """
    def __init__(self):
        self._db = load_face_db()
        self._lock = threading.Lock()
        print(f'[FaceLib] 已加载人脸库：{len(self._db)} 人')

    @property
    def db_size(self):
        with self._lock:
            return len(self._db)

    def reload(self):
        with self._lock:
            self._db = load_face_db()
        return len(self._db)

    def register(self, name, image_callback, timeout=8.0):
        """
        注册新的人脸
        name: 名字（字符串）
        image_callback: callable() -> BGR ndarray or None，从摄像头取帧
        timeout: 取帧最大等待秒数（默认 8 秒，给用户站位留时间）
        返回: (success: bool, message: str, encoding_len: int)
            - message 既用于 print 日志，也直接喂给 TTS，所以面向用户友好
        """
        name = (name or '').strip()
        if not name:
            return False, '名字不能为空', 0

        frame, area, stats = _capture_best_frame(image_callback, timeout=timeout)
        if frame is None or area <= 0:
            msg = _diagnose_no_face(stats)
            print(f'[FaceLib] 注册失败诊断：{msg} (stats={stats})', flush=True)
            return False, msg, 0

        encoding = extract_face_encoding(frame)
        if encoding is None:
            return False, '脸不够清晰，请正对摄像头再试一次', 0

        with self._lock:
            self._db[name] = encoding
            save_face_db(self._db)

        return True, f'已记住{name}', len(encoding)

    def recognize(self, image_callback, timeout=8.0):
        """
        识别当前摄像头前的人脸
        返回: (name or None, message: str, distance: float or None)
        """
        frame, area, stats = _capture_best_frame(image_callback, timeout=timeout)
        if frame is None or area <= 0:
            msg = _diagnose_no_face(stats)
            print(f'[FaceLib] 识别失败诊断：{msg} (stats={stats})', flush=True)
            return None, msg, None

        encoding = extract_face_encoding(frame)
        if encoding is None:
            return None, '脸不够清晰，请正对摄像头再试一次', None

        with self._lock:
            db_snapshot = dict(self._db)

        if not db_snapshot:
            return None, '人脸库是空的，请先添加', None

        if FACE_REC_OK:
            # 统一编码维度（防止库是其它来源 128 维混合）
            known_encodings = list(db_snapshot.values())
            known_names = list(db_snapshot.keys())
            results = face_recognition.compare_faces(
                known_encodings, encoding, tolerance=FACE_MATCH_TOLERANCE)
            distances = face_recognition.face_distance(known_encodings, encoding)
            best_idx = int(np.argmin(distances)) if len(distances) else -1
            if best_idx >= 0 and results[best_idx]:
                return known_names[best_idx], f'你是{known_names[best_idx]}', float(distances[best_idx])
            return None, '不认识', float(distances[best_idx]) if best_idx >= 0 else None
        else:
            # MediaPipe 几何特征：用归一化余弦相似度
            best_name = None
            best_sim = -1.0
            for n, vec in db_snapshot.items():
                # 维度必须一致才比对
                if vec.shape != encoding.shape:
                    continue
                a = encoding / (np.linalg.norm(encoding) + 1e-9)
                b = vec / (np.linalg.norm(vec) + 1e-9)
                sim = float(np.dot(a, b))
                if sim > best_sim:
                    best_sim = sim
                    best_name = n
            # 阈值 0.85 较严
            if best_name and best_sim >= 0.85:
                return best_name, f'你是{best_name}', 1.0 - best_sim
            return None, '不认识', 1.0 - best_sim if best_sim > 0 else None

    def list_people(self):
        """返回已注册的人名列表"""
        with self._lock:
            return list(self._db.keys())

    def remove(self, name):
        """从库中删除某个人"""
        with self._lock:
            if name in self._db:
                del self._db[name]
                save_face_db(self._db)
                return True
            return False
