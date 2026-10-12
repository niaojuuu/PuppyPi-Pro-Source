#!/usr/bin/env python3
# encoding: utf-8
"""企业微信自建应用回调加解密工具。

签名规则：
  msg_signature = sha1(''.join(sorted([token, timestamp, nonce, encrypt])))

加/解密：
  AES-CBC，key = base64decode(aes_key)[:32]，iv = key[:16]
  PKCS#7 padding

明文结构：
  random(16B) | msg_len(4B, big-endian) | msg | receive_id
  加密时 receive_id = corpid；解密时也应等于 corpid 否则报错。

参考：https://developer.work.weixin.qq.com/document/path/90268
"""
import os
import base64
import hashlib
import struct
import socket
import string
import random
import xml.etree.ElementTree as ET

from Crypto.Cipher import AES


class WeComCryptoError(Exception):
    pass


def _sha1_sign(token: str, timestamp: str, nonce: str, encrypt: str) -> str:
    items = sorted([token, timestamp, nonce, encrypt])
    return hashlib.sha1(''.join(items).encode('utf-8')).hexdigest()


def _pkcs7_pad(data: bytes, block_size: int = 32) -> bytes:
    pad_len = block_size - (len(data) % block_size)
    if pad_len == 0:
        pad_len = block_size
    return data + bytes([pad_len]) * pad_len


def _pkcs7_unpad(data: bytes, block_size: int = 32) -> bytes:
    if not data:
        raise WeComCryptoError('解密结果为空')
    pad_len = data[-1]
    if pad_len < 1 or pad_len > block_size:
        raise WeComCryptoError(f'PKCS#7 padding 非法：{pad_len}')
    return data[:-pad_len]


class WeComCrypto:
    def __init__(self, token: str, encoding_aes_key: str, corpid: str):
        if not token:
            raise ValueError('token 不能为空')
        if len(encoding_aes_key) != 43:
            raise ValueError(f'encoding_aes_key 应为 43 字符，得到 {len(encoding_aes_key)}')
        if not corpid:
            raise ValueError('corpid 不能为空')

        self.token = token
        self.corpid = corpid
        self.aes_key = base64.b64decode(encoding_aes_key + '=')
        assert len(self.aes_key) == 32, f'aes_key 长度应为 32，得到 {len(self.aes_key)}'
        self.iv = self.aes_key[:16]

    # ---------- 签名 ----------
    def verify_signature(self, msg_signature: str, timestamp: str,
                         nonce: str, encrypt: str) -> bool:
        expected = _sha1_sign(self.token, timestamp, nonce, encrypt)
        return expected == msg_signature

    def sign(self, timestamp: str, nonce: str, encrypt: str) -> str:
        return _sha1_sign(self.token, timestamp, nonce, encrypt)

    # ---------- 加/解密 ----------
    def decrypt(self, encrypt_b64: str) -> ET.Element:
        try:
            cipher_bytes = base64.b64decode(encrypt_b64)
            cipher = AES.new(self.aes_key, AES.MODE_CBC, self.iv)
            plain_padded = cipher.decrypt(cipher_bytes)
        except Exception as e:
            raise WeComCryptoError(f'AES 解密失败：{e}')

        plain = _pkcs7_unpad(plain_padded)
        if len(plain) < 20:
            raise WeComCryptoError('解密结果过短')
        content = plain[16:]
        msg_len = socket.ntohl(struct.unpack('I', plain[16:20])[0])
        if len(content) < msg_len + 4:
            raise WeComCryptoError(
                f'消息长度字段 ({msg_len}) 与实际不符 (剩余 {len(content)-4})'
            )
        msg = content[4:4 + msg_len].decode('utf-8')
        receive_id = content[4 + msg_len:].decode('utf-8')
        if receive_id != self.corpid:
            raise WeComCryptoError(
                f'receive_id 不匹配：得到 {receive_id!r}，期望 {self.corpid!r}'
            )
        try:
            return ET.fromstring(msg)
        except ET.ParseError as e:
            raise WeComCryptoError(f'XML 解析失败：{e}')

    def encrypt(self, reply_xml: str, nonce: str = '') -> str:
        rand_bytes = os.urandom(16)
        msg_bytes = reply_xml.encode('utf-8')
        length_bytes = struct.pack('I', socket.htonl(len(msg_bytes)))
        plain = rand_bytes + length_bytes + msg_bytes + self.corpid.encode('utf-8')
        padded = _pkcs7_pad(plain, 32)
        cipher = AES.new(self.aes_key, AES.MODE_CBC, self.iv)
        encrypted = cipher.encrypt(padded)
        return base64.b64encode(encrypted).decode('utf-8')

    def decrypt_echostr(self, echostr_b64: str) -> str:
        try:
            cipher_bytes = base64.b64decode(echostr_b64)
            cipher = AES.new(self.aes_key, AES.MODE_CBC, self.iv)
            plain_padded = cipher.decrypt(cipher_bytes)
        except Exception as e:
            raise WeComCryptoError(f'echostr 解密失败：{e}')
        plain = _pkcs7_unpad(plain_padded)
        if len(plain) < 20:
            raise WeComCryptoError('echostr 解密结果过短')
        # 跟 decrypt 一样按 msg_len 提取 msg 字段（不是 plain[16:]）
        msg_len = socket.ntohl(struct.unpack('I', plain[16:20])[0])
        if len(plain) < 20 + msg_len:
            raise WeComCryptoError(f'echostr msg 长度 {msg_len} 越界')
        return plain[20:20 + msg_len].decode('utf-8')

    def check_signature_and_decrypt(self, msg_signature: str, timestamp: str,
                                    nonce: str, body_xml: str) -> ET.Element:
        try:
            root = ET.fromstring(body_xml)
        except ET.ParseError as e:
            raise WeComCryptoError(f'XML 解析失败：{e}')
        encrypt_b64 = root.findtext('Encrypt', '')
        if not encrypt_b64:
            raise WeComCryptoError('请求体不含 <Encrypt> 字段')
        if not self.verify_signature(msg_signature, timestamp, nonce, encrypt_b64):
            raise WeComCryptoError('签名校验失败')
        return self.decrypt(encrypt_b64)
