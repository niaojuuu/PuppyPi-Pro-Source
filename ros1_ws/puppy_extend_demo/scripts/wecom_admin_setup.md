# 企业微信后台配置 Step-by-Step

> 配套：`wecom_tunnel_setup.md`（Tailscale Funnel 部署公网回调）
> 配置文件：`puppy_extend_demo/config/wechat_config.yaml`

## 流程图

```
[1 注册企业 / 拿 CorpID]
        ↓
[2 创建自建应用]
        ↓
[3 拿 AgentId + Secret]
        ↓
[4 配「接收消息」API 接收 (URL/Token/EncodingAESKey)]
        ↓
[5 让用户能看到应用（微信插件 + 可见范围）]
        ↓
[6 填 yaml]
        ↓
[7 验证]
```

## 1. 登录管理后台

- https://work.weixin.qq.com/wework_admin/
- 用企业管理员账号登录（一般是公司 IT / 老板）
- 没注册过企业：先在「我的企业」按提示创建，拿到 **CorpID**（形如 `ww1234567890abcdef`）

## 2. 创建自建应用

- 左菜单 → 「应用管理」 → 顶部 Tab 选「自建」 → 右上角「+ 创建应用」
- 填：
  - 应用 Logo：随便一张
  - **应用名称**：例如 "机器狗控制"（用户微信里会看到这个名）
  - 应用介绍：随便写
  - **可见范围**：必须选部门或成员；不选 = 没人能用

## 3. 拿 AgentId + Secret

- 应用详情页顶部：
  - **AgentId**：数字，如 `1000002`
  - **Secret**：点「查看」→ 扫码验证 → 拿到一串字符（yaml 里叫 `corp_secret`）

## 4. 配「接收消息」（最关键）

- 应用详情页往下翻 → 「**接收消息**」→「设置 API 接收」
- 勾选「**API 接收消息**」
- 填 3 个字段：
  - **URL**：`https://<你的-tailscale-节点名>.ts.net/wecom/callback`
    - 例：`https://puppy.tail-name.ts.net/wecom/callback`
    - ⚠️ 必须是 HTTPS + 公网可达
    - 狗端先 `sudo tailscale funnel 8088 on` 拿到这个 URL
  - **Token**：自己编一串字符（20~30 位，字母数字），保存好
  - **EncodingAESKey**：点「**随机生成**」按钮 → 43 字符 base64 字符串
- 点「**保存**」
- 保存会触发 GET 验证
  - 后台显示「保存成功」 = URL 验证通过，狗端 Flask 必须已起
  - 后台显示「保存失败」= 检查 Tailscale/Flask/URL

## 5. 让用户能用应用（重要！）

### 方式 A：微信插件（用户用普通微信，推荐）

- 左菜单 → 「我的企业」 → 「**微信插件**」 → 拉到下面「邀请关注」→ 生成二维码
- 用户用**普通微信**扫码
- 用户使用流程：微信 → 通讯录 → 你们公司名 → 「机器狗控制」→ 发消息

### 方式 B：企业微信 App

- 用户装「企业微信」App
- 通讯录里能看到应用

⚠️ **不做这一步，前面配的等于白搭**，用户根本看不到这个应用。

## 6. 填 yaml

`puppy_extend_demo/config/wechat_config.yaml`：

```yaml
wework:
  corp_id: "ww1234567890abcdef"            # 第 1 步
  corp_secret: "KFH...你的Secret..."       # 第 3 步
  agent_id: "1000002"                       # 第 3 步
  token: "your_token_12345"                 # 第 4 步自己编的
  aes_key: "43_char_base64_xxxxx..."        # 第 4 步生成的
  callback_host: "0.0.0.0"
  callback_port: 8088
  public_base_url: "https://puppy.tail-name.ts.net"  # 你的 Tailscale URL
```

## 7. 验证

- 微信 → 应用 → 发「前进」
- 狗端日志应出现：`[WeCom] 收到文本 from=<userid> text='前进'`
- 狗前进 + 微信收到「冲冲冲」 = 通了

## 常见坑

| 现象 | 原因 | 解决 |
|---|---|---|
| 后台保存 URL 失败 | Tailscale Funnel 没开 / URL 写错 / 用了 HTTP | 检查 `tailscale funnel status` |
| 狗端日志 `签名校验失败` | Token 或 aes_key 写错一位 | 重新复制后台和 yaml 两边对照 |
| 用户看不到应用 | 没设可见范围 / 没做微信插件 | 回第 5 步 |
| 消息发出去但用户收不到 | AgentId 写错 / 应用被禁用 | 检查 yaml 里的 `agent_id` 是不是数字字符串 |
| 后台成功保存但狗没动作 | Flask 没起 / Tailscale 掉线 | `curl https://<your>.ts.net/wecom/callback?msg_signature=x&timestamp=x&nonce=x&echostr=test` 看是否 200 |

## 重置流程

如果想换 corp / token：
1. 后台改 Token / EncodingAESKey → 「保存」会重新触发 GET 验证
2. 同步更新 yaml
3. 重启微信节点（kill -KILL + nohup 起来）

如果换 corp_id / agent_id：
1. 后台另建自建应用
2. 重新走第 4~6 步
3. 旧应用可保留或删除
