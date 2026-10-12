#!/usr/bin/env python3
# encoding: utf-8
"""企业微信自建应用回调服务器。

职责：
  - Flask 监听 /wecom/callback
    * GET  URL 验证 (echostr)
    * POST 解密 -> 分发到 incoming_queue (text/voice/event)
  - Sender worker 线程消费 reply_queue，调用 /cgi-bin/message/send
  - access_token 缓存 + 自动续期（线程安全）

队列约定（与主 ROS 节点通信）：
  incoming_queue: get() -> WechatMsg
  reply_queue   : put((from_user, text)) -> 异步被 sender worker 消费
"""
import os
import sys
import time
import shutil
import tempfile
import threading
import queue
import logging
from dataclasses import dataclass
from typing import Optional, List, Callable

import requests

try:
    from flask import Flask, request
except ImportError as e:
    print(f'[WeCom] 缺少 flask: pip install flask ({e})', file=sys.stderr)
    raise

from wecom_crypto import WeComCrypto, WeComCryptoError


@dataclass
class WechatMsg:
    msg_type: str
    from_user: str
    text: str = ''
    media_path: Optional[str] = None


class WeComClient:
    BASE = 'https://qyapi.weixin.qq.com/cgi-bin'

    def __init__(self, corp_id: str, corp_secret: str, agent_id: str,
                 logger: logging.Logger):
        self.corp_id = corp_id
        self.corp_secret = corp_secret
        self.agent_id = str(agent_id)
        self.logger = logger
        self._token: Optional[str] = None
        self._token_expire_at: float = 0.0
        self._lock = threading.Lock()

    def get_token(self, force: bool = False) -> str:
        with self._lock:
            now = time.time()
            if not force and self._token and now < self._token_expire_at - 300:
                return self._token
            url = f'{self.BASE}/gettoken'
            try:
                r = requests.get(url, params={'corpid': self.corp_id,
                                              'corpsecret': self.corp_secret},
                                 timeout=5)
                data = r.json()
            except Exception as e:
                self.logger.error(f'[WeCom] gettoken 网络错误：{e}')
                raise
            errcode = data.get('errcode', -1)
            if errcode != 0:
                raise RuntimeError(f'[WeCom] gettoken 失败：{data}')
            self._token = data['access_token']
            self._token_expire_at = now + int(data.get('expires_in', 7200))
            self.logger.info(f'[WeCom] access_token 已刷新 (expires_in={data.get("expires_in")}s)')
            return self._token

    def send_text(self, touser: str, content: str) -> bool:
        if not content:
            content = '收到'
        try:
            token = self.get_token()
        except Exception as e:
            self.logger.error(f'[WeCom] 取 token 失败：{e}')
            return False
        url = f'{self.BASE}/message/send'
        payload = {
            'touser': touser,
            'msgtype': 'text',
            'agentid': int(self.agent_id),
            'text': {'content': content},
        }
        try:
            r = requests.post(url, params={'access_token': token}, json=payload, timeout=5)
            data = r.json()
        except Exception as e:
            self.logger.error(f'[WeCom] send 网络错误：{e}')
            return False
        if data.get('errcode') != 0:
            self.logger.error(f'[WeCom] send 失败：{data}')
            return False
        self.logger.info(f'[WeCom] → {touser}: {content!r}')
        return True

    def download_media(self, media_id: str) -> Optional[str]:
        try:
            token = self.get_token()
        except Exception as e:
            self.logger.error(f'[WeCom] 下载 media 取 token 失败：{e}')
            return None
        url = f'{self.BASE}/media/get'
        try:
            r = requests.get(url, params={'access_token': token,
                                          'media_id': media_id},
                             timeout=10, stream=True)
        except Exception as e:
            self.logger.error(f'[WeCom] 下载 media 网络错误：{e}')
            return None
        ctype = r.headers.get('Content-Type', '')
        if 'json' in ctype or 'text' in ctype:
            self.logger.error(f'[WeCom] 下载 media 失败（非二进制）：{r.text[:200]}')
            return None
        fd, path = tempfile.mkstemp(suffix='.amr')
        os.close(fd)
        try:
            with open(path, 'wb') as f:
                for chunk in r.iter_content(chunk_size=8192):
                    if chunk:
                        f.write(chunk)
        except Exception as e:
            self.logger.error(f'[WeCom] 写 media 文件失败：{e}')
            try:
                os.unlink(path)
            except OSError:
                pass
            return None
        return path


class CallbackServer:
    def __init__(self, crypto: WeComCrypto, client: WeComClient,
                 incoming_queue: queue.Queue, allowed_users: List[str],
                 public_base_url: str, logger: logging.Logger):
        self.crypto = crypto
        self.client = client
        self.incoming_queue = incoming_queue
        self.allowed_users = set(allowed_users or [])
        self.public_base_url = public_base_url
        self.logger = logger
        self.app = Flask('wecom_callback')
        self.app.add_url_rule('/zpjwecom/callback',
                              view_func=self._dispatch,
                              methods=['GET', 'POST'])

    def _dispatch(self):
        if request.method == 'GET':
            return self._handle_verify()
        return self._handle_message()

    def _handle_verify(self):
        msg_signature = request.args.get('msg_signature', '')
        timestamp = request.args.get('timestamp', '')
        nonce = request.args.get('nonce', '')
        echostr = request.args.get('echostr', '')
        self.logger.info(f'[WeCom] GET /zpjwecom/callback 验证 ts={timestamp} nonce={nonce}')
        if not self.crypto.verify_signature(msg_signature, timestamp, nonce, echostr):
            self.logger.error('[WeCom] 验签失败')
            return 'verify fail', 403
        try:
            plain = self.crypto.decrypt_echostr(echostr)
        except WeComCryptoError as e:
            self.logger.error(f'[WeCom] echostr 解密失败：{e}')
            return 'decrypt fail', 400
        self.logger.info(f'[WeCom] 验签成功，echostr={plain!r}')
        return plain, 200

    def _handle_message(self):
        msg_signature = request.args.get('msg_signature', '')
        timestamp = request.args.get('timestamp', '')
        nonce = request.args.get('nonce', '')
        body = request.get_data(as_text=True)
        try:
            root = self.crypto.check_signature_and_decrypt(
                msg_signature, timestamp, nonce, body)
        except WeComCryptoError as e:
            self.logger.error(f'[WeCom] 入站解密失败：{e}')
            return 'decrypt fail', 400

        msg_type = root.findtext('MsgType', '')
        from_user = root.findtext('FromUserName', '')
        if self.allowed_users and from_user not in self.allowed_users:
            self.logger.info(f'[WeCom] 用户 {from_user} 不在 allowlist，丢弃')
            return 'success', 200

        if msg_type == 'text':
            text = root.findtext('Content', '')
            self.logger.info(f'[WeCom] 收到文本 from={from_user} text={text!r}')
            self.incoming_queue.put(WechatMsg('text', from_user, text=text))
        elif msg_type == 'voice':
            media_id = root.findtext('MediaId', '')
            self.logger.info(f'[WeCom] 收到语音 from={from_user} media_id={media_id}')
            media_path = self.client.download_media(media_id)
            if not media_path:
                self.logger.error(f'[WeCom] 语音下载失败，丢弃')
                return 'success', 200
            self.incoming_queue.put(WechatMsg('voice', from_user,
                                              text='', media_path=media_path))
        elif msg_type == 'event':
            event = root.findtext('Event', '')
            self.logger.info(f'[WeCom] 收到 event={event} from={from_user}')
        else:
            self.logger.info(f'[WeCom] 忽略 MsgType={msg_type!r}')

        return 'success', 200

    def start(self, host: str, port: int):
        self.logger.info(f'[WeCom] Flask 已绑定 http://{host}:{port}/zpjwecom/callback')
        self.logger.info(f'[WeCom] 公网回调 URL：{self.public_base_url}')
        self.app.run(host=host, port=port, threaded=True,
                     use_reloader=False, debug=False)


def sender_worker(reply_queue: queue.Queue, client: WeComClient,
                  logger: logging.Logger):
    while True:
        try:
            item = reply_queue.get()
        except Exception as e:
            logger.error(f'[WeCom sender] 队列异常：{e}')
            continue
        if item is None:
            return
        try:
            from_user, text = item
        except Exception:
            logger.error(f'[WeCom sender] 队列元素格式错误：{item!r}')
            continue
        try:
            client.send_text(from_user, text)
        except Exception as e:
            logger.error(f'[WeCom sender] send 异常：{e}')


def start_callback_server(config: dict, incoming_queue: queue.Queue,
                          reply_queue: queue.Queue, logger: logging.Logger):
    wework = config['wework']
    crypto = WeComCrypto(
        token=wework['token'],
        encoding_aes_key=wework['aes_key'],
        corpid=wework['corp_id'],
    )
    client = WeComClient(
        corp_id=wework['corp_id'],
        corp_secret=wework['corp_secret'],
        agent_id=wework['agent_id'],
        logger=logger,
    )
    server = CallbackServer(
        crypto=crypto,
        client=client,
        incoming_queue=incoming_queue,
        allowed_users=config.get('allowed_users', []),
        public_base_url=wework.get('public_base_url', ''),
        logger=logger,
    )

    t_sender = threading.Thread(target=sender_worker, args=(reply_queue, client, logger),
                                name='wecom-sender', daemon=True)
    t_sender.start()

    flask_kwargs = dict(
        host=wework.get('callback_host', '0.0.0.0'),
        port=int(wework.get('callback_port', 8088)),
    )
    # Flask 必须起，否则企业微信后台「保存」时 GET 验证无法响应。
    # 即便 corp_id/token 是占位、get_token 失败，也先让 Flask 监听端口。
    try:
        client.get_token()
        logger.info('[WeCom] access_token 预热成功')
    except Exception as e:
        logger.warning(f'[WeCom] access_token 预热失败（配置未就绪？）：{e}；Flask 仍会启动，URL 验证可响应')
    try:
        server.start(**flask_kwargs)
    except Exception as e:
        logger.error(f'[WeCom] Flask 启动失败：{e}')
        raise
