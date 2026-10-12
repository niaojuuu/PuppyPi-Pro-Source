# 企业微信回调公网隧道部署

## 为什么需要

企业微信服务器在公网，机器狗在 `192.168.x.x` 私网。Flask 服务 (`http://0.0.0.0:8088`) 无法被企业微信直连。需要反向隧道把狗端 `8088` 暴露到公网 HTTPS。

## 推荐：Tailscale Funnel（默认）

- 免费
- URL 稳定（`*.ts.net`），重启不换地址
- 自带 HTTPS
- 国内偶发不稳，但作为默认配置足够

### 一次性安装

```bash
# 狗端 host 上执行
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
# 按提示打开浏览器登录 Tailscale 账号
```

### 启动反向隧道

```bash
sudo tailscale funnel 8088 on
```

输出形如：
```
Available on:
    https://<machine-name>.<tailnet>.ts.net/
```

把 `https://<machine-name>.<tailnet>.ts.net/wecom/callback` 填到企业微信后台"接收消息 → API 接收"。

### 开机自启

```bash
# /etc/rc.local 或开机 cron：
sudo tailscale funnel 8088 on
```

### 检查状态

```bash
sudo tailscale status
sudo tailscale funnel status
```

## 备选 1：Cloudflare 命名隧道（生产级稳定）

需要：
- 一个域名（NS 指向 Cloudflare）
- Cloudflare 账号

```bash
# 安装 cloudflared（狗端 host）
curl -fsSL https://pkg.cloudflare.com/cloudflare-main.gpg \
  | sudo tee /usr/share/keyrings/cloudflare-main.gpg >/dev/null
echo 'deb [signed-by=/usr/share/keyrings/cloudflare-main.gpg] https://pkg.cloudflare.com/cloudflared focal main' \
  | sudo tee /etc/apt/sources.list.d/cloudflared.list
sudo apt update && sudo apt install -y cloudflared

# 登录、创建命名隧道
cloudflared tunnel login
cloudflared tunnel create puppy-wecom
cloudflared tunnel route dns puppy-wecom wecom.example.com

# 配置 ~/.cloudflared/config.yml
cat > ~/.cloudflared/config.yml <<EOF
tunnel: <TUNNEL_ID>
credentials-file: /home/ubuntu/.cloudflared/<TUNNEL_ID>.json

ingress:
  - hostname: wecom.example.com
    service: http://localhost:8088
  - service: http_status:404
EOF

# 启动
cloudflared tunnel run puppy-wecom
```

把 `https://wecom.example.com/wecom/callback` 填到企业微信后台。

## 备选 2：frp（已有国内 VPS 时）

```bash
# VPS 上：frps.ini
[common]
bind_port = 7000
vhost_https_port = 443

# 狗端：frpc.ini
[common]
server_addr = <VPS_IP>
server_port = 7000

[wecom]
type = https
local_port = 8088
custom_domains = wecom.example.com
```

## 临时测试：Cloudflare 快速隧道

无需账号，每次重启 URL 变。

```bash
cloudflared tunnel --url http://localhost:8088
# 输出 https://<random>.trycloudflare.com
```

适合本地调试 5 分钟上线，不要用于生产。
