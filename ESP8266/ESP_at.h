#ifndef __ESP_AT_H__
#define __ESP_AT_H__

#include <stdint.h>

typedef enum {
    ESP_OK      = 0,
    ESP_ERROR   = 1,
    ESP_TIMEOUT = 2
} ESP_Status_t;

/* 最近一次 AT+MQTTCONN? 查询解析出的 MQTT 状态（ESP-AT v2.2.0 官方定义）：
     -1 = 尚未查询/解析失败
     0  = 未初始化   1 = 已设 MQTTUSERCFG   2 = 已设 MQTTCONNCFG
     3  = 连接已断开（WiFi 一断固件立即置这个值）
     4  = 已建立连接 5 = 已连接未订阅      6 = 已连接且已订阅
   心跳任务用它做权威断线判定：>=4 才算在线。 */
extern volatile int g_mqtt_conn_state;

/* 最近一次 AT+CWJAP? 解析出的 WiFi 关联状态：
     -1 = 尚未查询/解析失败
      0 = 未连接任何 AP（回 "No AP"）—— 热点一关，这个值几秒内就变 0
      1 = 已关联到 AP（回包带 +CWJAP: 行）
   这是比 MQTT 状态更早、更硬的断线信号：TCP 半开时 MQTT 还报"已连接"，
   但 WiFi 层已经知道链路没了。 */
extern volatile int g_wifi_joined;

/* 最近一次 AT+CIFSR 解析出的 STA IP 状态：
     -1 = 未查询     0 = 0.0.0.0 / 没有 IP（关联上了但 DHCP 没过）
      1 = 拿到了有效 IP
   关键区别：AT+CWJAP? 只能证明"关联到 AP"，不能证明"能上网"；
   只有 CIFSR 有真实 IP，MQTT 才可能连出去。 */
extern volatile int g_sta_ip_ok;

/* 最近一次 AT+CIFSR 解析到的 IP 字符串原文（如 "192.168.43.5"）。
   g_sta_ip_ok 只回答"有没有 IP"，这个字符串回答"IP 是多少" ——
   排查时经常需要确认板子是不是拿到了和手机同网段的地址。 */
extern char g_sta_ip_str[20];

/* ===== 发布失败取证（2026-11-19 晚，只写诊断，不改任何流程）=====
   现象：巴法云显示在线几秒 → 掉线 → 板子重连 → 循环，且 probe FAIL。
   缺的唯一信息是"AT+MQTTPUB 到底死在哪一步"。下面两个量用来把它钉死：

   g_pub_stage  最近一次发布失败发生在哪一阶段
       0 = 无失败记录
       1 = 没等到 '>' 提示符（固件不支持 MQTTPUBRAW，或命令被拒）
       2 = 收到 +MQTTPUB:FAIL（broker 明确拒绝这条消息）
       3 = 收到 ERROR / ERR CODE / busy p（命令没被执行）
       4 = 送完数据后无任何回执（超时，链路可能已断）
       5 = 收到 +MQTTDISCONNECTED（模块自己报链路断了）

   g_pub_errtxt  触发失败的那一行 AT 原文（截 40 字符，非打印字符替换成 '.'）
                 —— 这比任何推断都硬：是 busy p、是 FAIL、还是 ERROR，一看便知。

   g_disc_cnt    上电以来收到 +MQTTDISCONNECTED 的累计次数。
                 这个数在涨 = 是"链路被断开"（broker 踢 / WiFi 掉），
                 这个数不涨 = 链路还在，只是发不出去。两种情况修法完全不同。 */
extern volatile uint8_t  g_pub_stage;
extern char              g_pub_errtxt[48];
extern volatile uint16_t g_disc_cnt;

/* 仅取证：把环形缓冲里积压的行扫一遍，统计 +MQTTDISCONNECTED 的真实次数。
   ⚠ 它**不修改** g_mqtt_connected，只做计数 —— 目的是"看见"，不是"动作"，
   所以对现有行为零影响。
   为什么需要它：URC 只有在 ESP_SendCmd / ESP_MQTTPubRaw 正在读的那一刻才会
   被解析；其余时间它就躺在环形缓冲里，被下一条命令开头的 ESP_ClearRxBuf()
   或 AT 任务里的 RingBuf_Clear() 直接丢掉。于是 d= 会低估断连次数，
   而"d 到底是不是 0"恰恰是区分"链路被断"和"链路还在但发不出去"的关键。
   调用位置：AT 任务心跳前、RingBuf_Clear() 之前（此时已持 uart1_mutex）。 */
void ESP_UrcSniff(void);

/* ===== 云端指令接收（2026-11-21，第三层：按需拉取历史）=====
   板子订阅了 BEMFA_TOPIC，云端往这个主题发消息时模块会吐 URC：
     +MQTTSUBRECV:<LinkID>,<"topic">,<len>,<payload>
   ESP_UrcPoll() 把这类 URC 收割出来，只有 payload 里带 "cmd" 的才当成指令，
   存进 g_cmd_payload 并置 g_cmd_pending；其余（包括板子自己发布回灌的数据）
   直接丢弃。
   为什么必须"先收割再清缓冲"：AT 命令开头原本是 ESP_ClearRxBuf()，
   指令会连同其它 URC 一起被扔掉，云端就永远等不到回应。 */
extern volatile uint8_t g_cmd_pending;
extern char             g_cmd_payload[96];

/* 把环形缓冲里积压的字节全部按行解析一遍。只在"没有 AT 命令在途"时调用
   （持锁或空闲），不能在 ESP_SendCmd 的读取循环中间插。 */
void ESP_UrcPoll(void);

// 通用 AT 指令发送（带超时和期望应答）
ESP_Status_t ESP_SendCmd(const char *cmd, const char *expected_ack, uint32_t timeout_ms);

// 简化版：发指令等 "OK"
ESP_Status_t ESP_SendCmd_OK(const char *cmd, uint32_t timeout_ms);

/* MQTT 发布专用：AT+MQTTPUB 的回包是两段式
     第一段 OK            —— 命令被固件接受（不代表发出去了）
     第二段 +MQTTPUB:OK   —— 真正投递给 broker 成功
            +MQTTPUB:FAIL —— 投递失败
   旧代码只等第一段的 OK 就返回成功，于是 +MQTTPUB:FAIL 被当普通文本丢掉，
   软件侧永远"发布成功"，云端却一条数据也没有 —— 这就是一直查不出问题的根源。
   本函数两段都等；若固件在 QoS0 下不发 URC，超时后按已接受(OK)处理。 */
ESP_Status_t ESP_MQTTPub(const char *cmd, uint32_t timeout_ms);

/* MQTT 原始字节发布：AT+MQTTPUBRAW=<LinkID>,"topic",<len>,<qos>,<retain>
   固件回：OK + ">" 提示符 → 送入 len 个原始字节 → +MQTTPUB:OK / +MQTTPUB:FAIL
   相比 AT+MQTTPUB 的三个好处（依据 ESP-AT 官方 MQTT AT 指令文档）：
     1) payload 原样发送，JSON 里的双引号不需要任何 \" 转义 —— 去掉一整类解析歧义；
     2) 不受"单条 AT 命令总长 < 256 字节"的限制；
     3) +MQTTPUB:OK 是 broker 收没收到的确证（AT+MQTTPUB 只回一个 OK，只管"命令被接受"）。
   返回：ESP_OK=broker 已确认；ESP_ERROR=被拒/不支持（连 '>' 都没等到）；ESP_TIMEOUT=送完后无回执 */
ESP_Status_t ESP_MQTTPubRaw(const char *topic, const uint8_t *data, uint16_t len, uint32_t timeout_ms);


#endif
