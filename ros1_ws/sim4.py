"""正确 URL encode 的模拟请求"""
import base64, hashlib, os, struct, socket, time, random
import requests
from Crypto.Cipher import AES

token = 'puppyctrl2026dog'
aes_key_b64 = '16ybcsW8OZMsEHDWPJciT7fuofCdPpezZ0GhsZUu5I8'
corpid = 'ww592b0eb99f890bc4'

key = base64.b64decode(aes_key_b64 + '=')
iv = key[:16]
msg_xml = ''
rand = os.urandom(16)
msg_len_bytes = struct.pack('I', socket.htonl(len(msg_xml)))
plain = rand + msg_len_bytes + msg_xml.encode('utf-8') + corpid.encode('utf-8')
pad_len = 32 - (len(plain) % 32)
plain_padded = plain + bytes([pad_len]) * pad_len
cipher = AES.new(key, AES.MODE_CBC, iv)
encrypted = cipher.encrypt(plain_padded)
echostr = base64.b64encode(encrypted).decode()

timestamp = str(int(time.time()))
nonce = ''.join(random.choices('0123456789', k=10))
sig_input = ''.join(sorted([token, timestamp, nonce, echostr]))
msg_signature = hashlib.sha1(sig_input.encode('utf-8')).hexdigest()

print('=== 关键: 模拟企业微信请求 ===')
print(f'echostr (原始 base64): {echostr}')
print(f'  + 出现: {echostr.count("+")} 次, / 出现: {echostr.count("/")} 次')
print()

# 用 params dict, requests 自动 URL encode
url = 'https://xbcx.com.cn/zpjwecom/callback'
params = {
    'msg_signature': msg_signature,
    'timestamp': timestamp,
    'nonce': nonce,
    'echostr': echostr,
}
print('=== 用 params=dict 发请求 (自动 URL encode) ===')
r = requests.get(url, params=params, timeout=10, verify=True)
print(f'Status: {r.status_code}')
print(f'Body  : {r.text!r}')
