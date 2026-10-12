"""真实模拟企业微信 GET 验证：生成合法签名+密文，通过公网发到 Flask"""
import base64
import hashlib
import os
import random
import socket
import struct
import time
import requests
from Crypto.Cipher import AES

token = 'puppyctrl2026dog'
aes_key_b64 = '16ybcsW8OZMsEHDWPJciT7fuofCdPpezZ0GhsZUu5I8'
corpid = 'ww592b0eb99f890bc4'

key = base64.b64decode(aes_key_b64 + '=')
iv = key[:16]

# 明文结构: random(16B) + msg_len(4B) + msg_xml + receive_id
# URL 验证时 msg 是空字符串, msg_len=0
msg_xml = ''
rand = os.urandom(16)
msg_len_bytes = struct.pack('I', socket.htonl(len(msg_xml)))
plain = rand + msg_len_bytes + msg_xml.encode('utf-8') + corpid.encode('utf-8')

# PKCS#7
pad_len = 32 - (len(plain) % 32)
plain_padded = plain + bytes([pad_len]) * pad_len

cipher = AES.new(key, AES.MODE_CBC, iv)
encrypted = cipher.encrypt(plain_padded)
echostr = base64.b64encode(encrypted).decode()

timestamp = str(int(time.time()))
nonce = ''.join(random.choices('0123456789', k=10))
sig_input = ''.join(sorted([token, timestamp, nonce, echostr]))
msg_signature = hashlib.sha1(sig_input.encode('utf-8')).hexdigest()

url = f'https://xbcx.com.cn/zpjwecom/callback?msg_signature={msg_signature}&timestamp={timestamp}&nonce={nonce}&echostr={echostr}'
print('=== URL ===')
print(url[:200], '...')
print()
print('=== GET 响应 ===')
r = requests.get(url, timeout=10, verify=True)
print(f'Status: {r.status_code}')
print(f'Body  : {r.text!r}')
print()
print('=== 期望 ===')
print('Status: 200')
print('Body  : (空明文，因为 msg_xml=""，只返回空字符串)')
print('如果验签/AES/corpid 任一步失败，会返回 200 "verify fail" 或 raise WeComCryptoError')
