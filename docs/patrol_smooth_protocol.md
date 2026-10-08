# 平滑巡逻服务协议（patrol_smooth.py）

## 服务信息

| 项目 | 值 |
|------|-----|
| 默认端口 | `5101` |
| 启动命令 | `python3 patrol_smooth.py` |

---

## 接口

### POST /start
开始巡逻。首次从索引 0 出发；停止后重启从最近 POI 的下一个续行，不倒退。

**请求体**：无

**响应**
```json
{ "ok": true, "message": "平滑巡逻已开始，起点索引=0" }
```
```json
{ "ok": false, "message": "已在巡逻中" }
```

**curl**
```bash
curl -X POST http://localhost:5101/start
```

---

### POST /stop
立即停止：同时发送底盘停止指令 + 设置退出标志，无轮询延迟。

**请求体**：无

**响应**
```json
{ "ok": true, "message": "停止指令已发送" }
```

**curl**
```bash
curl -X POST http://localhost:5101/stop
```

---

### GET /status
查询当前巡逻状态。

**响应**
```json
{
  "running": true,
  "current_poi": "3",
  "message": "→ 索引 2，队列 12 点"
}
```

**curl**
```bash
curl http://localhost:5101/status
```

| 字段 | 说明 |
|------|------|
| `running` | 是否正在巡逻 |
| `current_poi` | 当前批次起点 POI ID |
| `message` | 状态描述，含队列点数 |

---

## 配置（patrol_smooth.py 顶部宏定义）

| 常量 | 默认值 | 说明 |
|------|--------|------|
| `ROBOT_IP` | `192.168.123.163` | 底盘 IP |
| `ROBOT_PORT` | `1448` | 底盘 API 端口 |
| `SERVICE_PORT` | `5101` | 本服务监听端口 |
| `POI_START_ID` | `1` | 巡逻起始 POI ID（整数，含） |
| `POI_END_ID` | `10` | 巡逻结束 POI ID（整数，含），超出实际数量自动截断 |
| `DIRECTION` | `"asc"` | 巡逻方向：`asc` 升序 / `desc` 降序 |
| `QUEUE_LOOPS` | `3` | 每批次圈数，越大重发间隔越长、停顿越稀少 |
| `SPEED` | `0.3` | 巡逻速度 m/s，启动时写入底盘，重启后恢复默认 |
| `POLL_INTERVAL` | `0.5` | 轮询 action 状态间隔（秒） |

---

## 行为说明

- **批量发点**：每次发 `QUEUE_LOOPS × POI数` 个点给 `SeriesMoveToAction`，走完一批才有短暂停顿
- **停止立即生效**：`/stop` 和 Ctrl+C / kill 均直接发底盘停止指令，无延迟
- **续行不倒退**：停止后取最近 POI 的下一个为起点，避免机器人回头
- **导航失败**：自动定位当前位置，从最近的下一个 POI 继续
- **速度设置**：仅本次运行有效，底盘重启恢复原值
