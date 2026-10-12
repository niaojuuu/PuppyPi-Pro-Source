"""模拟企业微信 GET 验证请求，定位 Flask 验签/AES 解密失败原因"""
import base64
import hashlib
import os
import random
import time
from urllib.parse import quote
import requests
from Crypto.Cipher import AES

token = 'puppyctrl2026dog'
aes_key_b64 = '16ybcsW8OZMsEHDWPJciT7fuofCdPpezZ0GhsZUu5I8'
corpid = 'ww592b0eb99f890bc4'

key = base64.b64decode(aes_key_b64)[:32]
iv = key[:16]

random_bytes = os.urandom(16)
msg = random_bytes + len(corpid).to_bytes(4, 'big') + corpid.encode('utf-8')
pad_len = 32 - (len(msg) % 32)
msg_padded = msg + bytes([pad_len] * pad_len)
cipher = AES.new(key, AES.MODE_CBC, iv)
encrypted = cipher.encrypt(msg_padded)
echostr = base64.b64encode(encrypted).decode()

timestamp = str(int(time.time()))
nonce = ''.join(random.choices('0123456789', k=10))
sig_list = sorted([token, timestamp, nonce, echostr])
msg_signature = hashlib.sha1(''.join(sig_list).encode()).hexdigest()

url = f'https://xbcx.com.cn/zpjwecom/callback?msg_signature={msg_signature}&timestamp={timestamp}&nonce={nonce}&echostr={echostr}'
print('GET URL:', url[:120], '...')
print()
print('=== 响应 ===')
r = requests.get(url, timeout=10, verify=True)
print(f'Status: {r.status_code}')
print(f'Body: {r.text!r}')
print()
print('=== 期望 ===')
print('Status: 200')
print('Body: 明文 echostr (16 字节随机 + 4 字节长度 + corpid)')
