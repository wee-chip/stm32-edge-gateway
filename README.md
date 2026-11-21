# 工业边缘网关（STM32F411 + FreeRTOS）

基于 STM32F411 的边缘数据采集网关：通过 RS485 采集温湿度与光照，经 ESP-01S 接入 MQTT 上云，本地用 ILI9341 + LVGL 做实时显示与日志查看，掉线期间数据落盘、恢复后补传，并支持云端按需查询历史数据。

载板为自绘 PCB（嘉立创 EDA），经两轮打样、手工焊接后在本板上完成全部软硬件联调。

## 硬件构成

| 模块 | 型号 | 接口 |
|---|---|---|
| 主控 | STM32F411CEU6（WeAct 核心板） | — |
| 温湿度 | XY-MD02 | RS485 / Modbus-RTU，9600 8N1 |
| 光照 | B-RS-L30 | RS485 / Modbus-RTU，9600 8N1 |
| 无线 | ESP-01S（ESP-AT 固件） | USART1，115200，AT 指令 |
| 存储 | W25Q64 | SPI2，分日志 / 离线缓存 / 历史记录三区 |
| 显示 | ILI9341 2.8" + 电阻触摸 | SPI1，LVGL |

## 软件架构

五个 FreeRTOS 任务，优先级由高到低：

| 任务 | 优先级 | 职责 |
|---|---|---|
| Dog | AboveNormal+2 | 喂狗、系统心跳灯 |
| UI | AboveNormal | LVGL 刷新、页面切换 |
| Modbus | Normal+1 | 轮询从站、组包入队 |
| AT（网络） | Normal | 连接管理、心跳、重连状态机 |
| MQTT | Normal | 实时上报、离线缓存、补传、历史回传 |

共享资源：`uart1_mutex` 保护 ESP8266 串口，`flash_mutex` 保护 SPI Flash，`data_queue` / `lvgl_data_queue` 传递采集结果。

## 目录结构

```
Core/          CubeMX 生成的外设初始化
Drivers/       ST HAL 库
Middlewares/   FreeRTOS
Lvgl/          LVGL 源码 + LVGL_myGUI/ 界面实现
ESP8266/       AT 指令层（command.c 串口收发、ESP_at.c 指令解析、pal.c 连接管理）
model/         Modbus 主站与采集调度
SPI_Flash/     日志环形缓冲、离线缓存、历史读取
code/          任务实现（Taskhandel.c）、LCD / 触摸驱动
MDK-ARM/       Keil 工程
```

## 通信协议

主题统一为 `STM32F411`（设备订阅与发布同一主题）。

**实时数据**（每 6 秒一条）

```json
{"temp":25.2,"humi":27.0,"light":251,"alm":0,"th":"OK","rx":9,"rxl":9,
 "ok":244,"age":0,"inv":0,"txf":0,"lp":0,"lr":250791,
 "ts":"2026-11-21 09:56:41"}
```

- `ts`：ESP8266 SNTP 校时后写入 RTC，未校时成功时为空字符串
- `alm`：告警位，bit0 温度过高 / bit1 温度过低 / bit2 湿度过高 / bit3 湿度过低

**自动补传**（重连后把离线缓存推上去，与实时数据交替发送）

```json
{"off":1,"batch":[{"temperature":25.0,"shidu":27.6,"light":149,"off":1,"ts":"..."}]}
```

**云端按需查询历史**

下发：`{"cmd":"history","n":10}`（n 上限 30）
回传：`{"off":1,"h":[[1795330000,25.2,27.0,251],[...]]}`

数组元素为 `[Unix时间戳, 温度, 湿度, 光照]`，每页 10 条，按时间从旧到新。
数据源是独立的历史记录区（每 10 秒一条，与在线状态无关）。

**清空离线缓存**

下发：`{"cmd":"clearcache"}`

## 已实现

- Modbus-RTU 主站：自实现 CRC16、设备表轮询、逐设备连续失败计数
- MQTT 接入与断线自愈：三级重连（仅重连 MQTT / 重新配网 / 硬复位兜底），DHCP 后校验 IP
- 掉线缓存与补传：离线 10 秒一条落盘，恢复后与实时数据交替补发，发布成功才标记已发
- 历史记录：每 10 秒一条独立落盘（与在线状态无关），云端指令触发分页回传，失败不推进游标
- 阈值告警：温度 / 湿度越界时在实时报文的 `alm` 字段置位
- 时间同步：ESP8266 SNTP 校时写入 RTC，避免无电池 RTC 输出错误时间
- 事件日志：SPI Flash 环形缓冲，支持屏幕查看，正常上电清空
- 本地 UI：实时数值、在线状态、日志列表

## 已知限制

- 历史记录区 256KB，约 45 小时容量，写满后环形覆盖最旧数据
- 升级到本分区布局后，旧的日志 / 缓存数据不再可达（首次启动会自动重建）
- 巴法云免费版对 QoS1 支持异常（重复投递、不回 PUBACK），因此固定使用 QoS0
- 设备订阅与发布同一主题，会收到自己消息的回灌，靠 payload 中是否含 `cmd` 字段区分
- 单条上报超过 200 字节只能走 `AT+MQTTPUBRAW`，字符串兜底路径有 256 字节命令长度限制

## 模块来源

| 模块 | 来源 |
|---|---|
| `Core/`、`Drivers/`、`Middlewares/` | STM32CubeMX 生成 / ST HAL |
| `Lvgl/`（除 `LVGL_myGUI/`） | LVGL 官方仓库 |
| `ESP8266/command.c` | 基于串口 DMA + 环形缓冲的公开示例改写 |
| `ESP8266/ESP_at.c`、`pal.c` | 自研：AT 逐行解析、连接状态机、SNTP 校时、三级重连 |
| `SPI_Flash/` | 自研：三区存储布局、日志环形缓冲、离线缓存、批量补传、历史记录与分页 |
| `model/modbus.c` | 在公开 Modbus 示例框架上重构：设备表驱动 + 诊断计数 |
| `code/Taskhandel.c` | 自研：任务模型、上报节流、补传调度、云端指令处理 |

## 编译

Keil MDK 5 + ARMCC 5.06，打开 `MDK-ARM/F4project.uvprojx` 直接 Build。

## 致谢

串口收发与 Modbus 帧格式的初始实现参考了公开示例工程，通信层与存储层已按本项目需求重写。
