#!/usr/bin/env python3
# encoding: utf-8
# 声纹验证模块 - 使用 Resemblyzer 提取说话人嵌入向量进行声纹匹配

import os
import numpy as np
from resemblyzer import VoiceEncoder, preprocess_wav


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
            # 确保目录存在
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

        # 取平均嵌入向量
        avg_embedding = np.mean(embeddings, axis=0)
        # 归一化
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

        # 计算余弦相似度
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
