#!/usr/bin/env python3
# encoding: utf-8
# 声纹识别模块
# - VoicePrint : 单主人验证（保留旧 API）
# - VoicePrintDB : 多用户声纹库，支持注册 / 识别 / 列出 / 删除
#
# 配置文件: voiceprint_db.yaml (与 face_db.yaml 同风格)
#    名字:
#      embedding: [256维声纹向量]
#      created_at: "2026-09-21 10:30:00"
#      sample_count: 3

import os
import time
import threading
import numpy as np
from resemblyzer import VoiceEncoder, preprocess_wav


# ==================== 向后兼容：单主人验证 ====================
class VoicePrint:
    def __init__(self, embedding_path, threshold=0.7):
        """
        初始化声纹验证模块
        :param embedding_path: 主人声纹嵌入向量文件路径 (.npy)
        :param threshold: 声纹匹配阈值，0~1，越高越严格
        """
        self.embedding_path = embedding_path
        self.threshold = threshold
        self.encoder = VoiceEncoder()
        self.owner_embedding = None
        self.load_owner_embedding()

    def load_owner_embedding(self):
        """加载主人的声纹嵌入向量"""
        if os.path.exists(self.embedding_path):
            self.owner_embedding = np.load(self.embedding_path)
            return True
        return False

    def extract_embedding(self, audio_path):
        """
        从音频文件提取说话人嵌入向量
        :param audio_path: 音频文件路径
        :return: 嵌入向量 (numpy array) 或 None
        """
        try:
            wav = preprocess_wav(audio_path)
            embedding = self.encoder.embed_utterance(wav)
            return embedding
        except Exception as e:
            print(f"[VoicePrint] 提取嵌入向量失败: {e}")
            return None

    def enroll(self, audio_path):
        """
        录入主人声纹（单段音频）
        :param audio_path: 音频文件路径
        :return: 是否成功
        """
        embedding = self.extract_embedding(audio_path)
        if embedding is not None:
            os.makedirs(os.path.dirname(self.embedding_path), exist_ok=True)
            np.save(self.embedding_path, embedding)
            self.owner_embedding = embedding
            print(f"[VoicePrint] 声纹已保存到: {self.embedding_path}")
            return True
        return False

    def enroll_from_samples(self, audio_paths):
        """
        从多段音频样本录入主人声纹（取平均嵌入向量）
        :param audio_paths: 音频文件路径列表
        :return: 是否成功
        """
        embeddings = []
        for path in audio_paths:
            emb = self.extract_embedding(path)
            if emb is not None:
                embeddings.append(emb)
            else:
                print(f"[VoicePrint] 跳过无效样本: {path}")

        if len(embeddings) == 0:
            print("[VoicePrint] 没有有效的声纹样本")
            return False

        avg_embedding = np.mean(embeddings, axis=0)
        avg_embedding = avg_embedding / np.linalg.norm(avg_embedding)

        os.makedirs(os.path.dirname(self.embedding_path), exist_ok=True)
        np.save(self.embedding_path, avg_embedding)
        self.owner_embedding = avg_embedding
        print(f"[VoicePrint] 已从 {len(embeddings)} 个样本生成声纹，保存到: {self.embedding_path}")
        return True

    def verify(self, audio_path):
        """
        验证音频是否匹配主人声纹
        :param audio_path: 待验证的音频文件路径
        :return: (is_match: bool, confidence: float)
        """
        if self.owner_embedding is None:
            print("[VoicePrint] 未找到主人声纹，跳过验证")
            return True, 0.0

        embedding = self.extract_embedding(audio_path)
        if embedding is None:
            print("[VoicePrint] 无法提取声纹特征")
            return False, 0.0

        similarity = np.dot(self.owner_embedding, embedding) / (
            np.linalg.norm(self.owner_embedding) * np.linalg.norm(embedding)
        )
        confidence = float(similarity)
        is_match = confidence >= self.threshold

        print(f"[VoicePrint] 声纹相似度: {confidence:.4f}, 阈值: {self.threshold}, 匹配: {is_match}")
        return is_match, confidence

    def is_enrolled(self):
        """检查是否已录入主人声纹"""
        return self.owner_embedding is not None


# ==================== 多用户声纹库（新增） ====================
def _parse_simple_yaml(path):
    """
    简易 YAML 解析（兼容无 PyYAML 的环境）。
    解析如下格式：
        名字:
          embedding: [0.1, 0.2, ...]
          created_at: "..."
          sample_count: 3
    """
    db = {}
    if not os.path.exists(path):
        return db
    current_name = None
    current_emb = []
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.rstrip()
            if not line or line.startswith('#'):
                continue
            if not line.startswith(' ') and line.endswith(':'):
                if current_name and current_emb:
                    db[current_name] = np.array(current_emb, dtype=np.float32)
                current_name = line[:-1].strip()
                current_emb = []
                continue
            stripped = line.strip()
            if stripped.startswith('embedding:'):
                vec_str = stripped[len('embedding:'):].strip()
                if vec_str.startswith('[') and vec_str.endswith(']'):
                    vec_str = vec_str[1:-1]
                    try:
                        current_emb = [float(x.strip()) for x in vec_str.split(',') if x.strip()]
                    except Exception:
                        current_emb = []
            # 其他字段(创建时间、计数)跳过
    if current_name and current_emb:
        db[current_name] = np.array(current_emb, dtype=np.float32)
    return db


class VoicePrintDB:
    """
    多用户声纹库。提供增删查能力。

    使用方式：
        db = VoicePrintDB(yaml_path, threshold=0.7)
        ok, msg = db.register(name='张三', audio_paths=[wav1, wav2])
        name, conf = db.identify(audio_path)               # 返回最相似的人及置信度
        is_match, conf = db.verify_owner(audio_path)       # 任一注册人即视为"主人"
        db.list_people()
        db.remove('张三')
    """
    def __init__(self, db_path, threshold=0.7):
        self.db_path = db_path
        self.threshold = threshold
        # 共享 encoder 实例以节省加载时间
        self.encoder = VoiceEncoder()
        self._db = {}
        self._lock = threading.Lock()
        self.reload()
        print(f'[VoicePrintDB] 声纹库已加载：{len(self._db)} 人（路径={db_path}）')

    # ---------- 底层 IO ----------
    def _ensure_db_file(self):
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        if not os.path.exists(self.db_path):
            with open(self.db_path, 'w', encoding='utf-8') as f:
                f.write('# 声纹库配置：{名字: 256维声纹嵌入向量}\n')
                f.write('# 自动生成，请勿手动删除\n')

    def reload(self):
        """从 yaml 重新加载库"""
        self._ensure_db_file()
        loaded = {}
        try:
            import yaml  # type: ignore
            with open(self.db_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            for name, info in data.items():
                if isinstance(info, dict) and 'embedding' in info:
                    loaded[name] = np.array(info['embedding'], dtype=np.float32)
                elif isinstance(info, list):
                    loaded[name] = np.array(info, dtype=np.float32)
        except ImportError:
            loaded = _parse_simple_yaml(self.db_path)
        except Exception as e:
            print(f'[VoicePrintDB] 加载声纹库出错：{e}')

        with self._lock:
            self._db = loaded
        return len(self._db)

    def _save(self):
        """把当前 db 写回 yaml。调用前请先持锁。"""
        self._ensure_db_file()
        try:
            import yaml  # type: ignore
            out = {}
            for name, vec in self._db.items():
                # 取出存入时的元数据（若有），否则填默认
                out[name] = {
                    'embedding': [float(x) for x in vec.tolist()],
                    'created_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                    'sample_count': 1,
                }
            with open(self.db_path, 'w', encoding='utf-8') as f:
                yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False)
        except ImportError:
            with open(self.db_path, 'w', encoding='utf-8') as f:
                f.write('# 声纹库配置：{名字: 256维声纹嵌入向量}\n')
                f.write(f'# 更新时间: {time.strftime("%Y-%m-%d %H:%M:%S")}\n')
                for name, vec in self._db.items():
                    arr = [float(x) for x in vec.tolist()]
                    f.write(f'{name}:\n')
                    f.write(f'  embedding: [{", ".join(f"{x:.6f}" for x in arr)}]\n')
                    f.write(f'  created_at: "{time.strftime("%Y-%m-%d %H:%M:%S")}"\n')
                    f.write(f'  sample_count: 1\n')

    # ---------- 特征提取 ----------
    def extract_embedding(self, audio_path):
        """从音频文件提取 256 维声纹嵌入"""
        try:
            wav = preprocess_wav(audio_path)
            return self.encoder.embed_utterance(wav)
        except Exception as e:
            print(f'[VoicePrintDB] 提取声纹失败：{e}')
            return None

    @staticmethod
    def _average_embeddings(embeddings):
        """对多段样本取平均并归一化"""
        if not embeddings:
            return None
        avg = np.mean(embeddings, axis=0)
        norm = np.linalg.norm(avg)
        if norm > 0:
            avg = avg / norm
        return avg.astype(np.float32)

    # ---------- 高层接口 ----------
    @property
    def db_size(self):
        with self._lock:
            return len(self._db)

    def is_enrolled(self):
        """是否至少录入了一个人的声纹"""
        with self._lock:
            return len(self._db) > 0

    def list_people(self):
        """返回已注册的人名列表"""
        with self._lock:
            return list(self._db.keys())

    def register(self, name, audio_paths, overwrite=False):
        """
        注册/更新一个人的声纹。
        :param name: 名字
        :param audio_paths: 音频文件路径列表（会取平均）
        :param overwrite: 若已存在，是否覆盖
        :return: (success: bool, message: str)
        """
        name = (name or '').strip()
        if not name:
            return False, '名字不能为空'
        if not audio_paths:
            return False, '没有提供音频样本'

        embeddings = []
        for p in audio_paths:
            emb = self.extract_embedding(p)
            if emb is not None:
                embeddings.append(emb)
            else:
                print(f'[VoicePrintDB] 跳过无效样本：{p}')

        if not embeddings:
            return False, '所有音频样本都提取不出声纹，请重试'

        avg_emb = self._average_embeddings(embeddings)
        if avg_emb is None:
            return False, '声纹特征为空'

        with self._lock:
            existed = name in self._db
            if existed and not overwrite:
                return False, f'{name} 已在声纹库中，如需覆盖请使用 overwrite=True'
            self._db[name] = avg_emb
            self._save()

        action = '更新' if existed else '注册'
        return True, f'已{action}{name}的声纹（{len(embeddings)} 段样本）'

    def remove(self, name):
        """删除某人的声纹。返回是否存在并被删除。"""
        name = (name or '').strip()
        with self._lock:
            if name in self._db:
                del self._db[name]
                self._save()
                return True, f'已删除{name}的声纹'
            return False, f'声纹库里没有{name}'

    def identify(self, audio_path):
        """
        从音频中找出最像的那个人。
        返回: (name or None, confidence, is_match)
            - name: 最佳匹配的人名；若没人超过阈值则为 None
            - confidence: 与最佳匹配人的余弦相似度
            - is_match: 是否超过阈值
        """
        emb = self.extract_embedding(audio_path)
        if emb is None:
            return None, 0.0, False

        with self._lock:
            db_snapshot = dict(self._db)

        if not db_snapshot:
            return None, 0.0, False

        best_name = None
        best_sim = -1.0
        for n, ref in db_snapshot.items():
            if ref.shape != emb.shape:
                continue
            a = emb / (np.linalg.norm(emb) + 1e-9)
            b = ref / (np.linalg.norm(ref) + 1e-9)
            sim = float(np.dot(a, b))
            if sim > best_sim:
                best_sim = sim
                best_name = n

        if best_name is None:
            return None, 0.0, False

        is_match = best_sim >= self.threshold
        name_to_return = best_name if is_match else None
        return name_to_return, best_sim, is_match

    def verify_owner(self, audio_path):
        """
        验证音频是否匹配库中任一注册人。
        返回: (is_match: bool, best_name: str or None, confidence: float)
        """
        name, conf, ok = self.identify(audio_path)
        return ok, name, conf
