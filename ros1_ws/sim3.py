"""直接 import Flask 用的 wecom_crypto，验证算法一致性"""
import sys
sys.path.insert(0, '/home/ubuntu/puppypi/src/puppy_extend_demo/scripts')
from wecom_crypto import WeComCrypto

token = 'puppyctrl2026dog'
aes_key = '16ybcsW8OZMsEHDWPJciT7fuofCdPpezZ0GhsZUu5I8'
corpid = 'ww592b0eb99f890bc4'

crypto = WeComCrypto(token, aes_key, corpid)
print(f'aes_key bytes len = {len(crypto.aes_key)} (expect 32)')
print(f'iv bytes len     = {len(crypto.iv)} (expect 16)')

# 模拟 Flask 收到的请求（按官方算法）
import base64, hashlib, os, struct, socket, time, random
from Crypto.Cipher import AES

key = base64.b64decode(aes_key + '=')
iv = key[:16]
print(f'\nkey == crypto.aes_key: {key == crypto.aes_key}')
print(f'iv == crypto.iv: {iv == crypto.iv}')

# 同样的明文
msg_xml = ''
rand = os.urandom(16)
msg_len_bytes = struct.pack('I', socket.htonl(len(msg_xml)))
plain = rand + msg_len_bytes + msg_xml.encode('utf-8') + corpid.encode('utf-8')
pad_len = 32 - (len(plain) % 32)
plain_padded = plain + bytes([pad_len]) * pad_len
cipher = AES.new(key, AES.MODE_CBC, iv)
encrypted = cipher.encrypt(plain_padded)
echostr = base64.b64encode(encrypted).decode()

timestamp = '1700000000'
nonce = '1234567890'
expected_sig = ''.join(sorted([token, timestamp, nonce, echostr]))
my_sig = hashlib.sha1(expected_sig.encode('utf-8')).hexdigest()
print(f'\nmy msg_signature: {my_sig}')

# 调用 Flask 的 verify_signature
result = crypto.verify_signature(my_sig, timestamp, nonce, echostr)
print(f'crypto.verify_signature: {result}')

# 调用 decrypt_echostr
try:
    plain = crypto.decrypt_echostr(echostr)
    print(f'decrypt_echostr OK: {plain!r}')
except Exception as e:
    print(f'decrypt_echostr FAIL: {e}')
