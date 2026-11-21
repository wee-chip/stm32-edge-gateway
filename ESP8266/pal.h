#ifndef PAL_H
#define PAL_H

#include "main.h"
#include <string.h>
#include "usart.h"
#include "stm32f4xx.h"
#include "stdio.h"




// °Í·¨ÔÆ²ÎÊýÅäÖÃ
#define WIFI_SSID   "xiaom999"
#define WIFI_PASS   "421299472"
#define BEMFA_BROKER "mqtt.bemfa.com"
#define BEMFA_PORT   9501
#define BEMFA_TOPIC  "STM32F411"
#define BEMFA_UID    "49ae8934e289d1f790ba23d2b41da061"  // °Í·¨ÔÆ¿ØÖÆÌ¨ÓÒÉÏ½ÇµÄË½Ô¿
#define TOPIC_PUB    "stm32"   // Äã´´½¨µÄÖ÷ÌâÃû

/* ===== 发布用主题 =====
   巴法云官方文档《MQTT协议接入》「set 指令」原文：
     "推送消息时：主题名后加 /set 推送消息，表示向所有订阅这个主题的设备们推送
      消息，假如推送者自己也订阅了这个主题，消息不会被推送给它自己，
      以防止自己推送的消息被自己接收。"

   本板子既订阅 STM32F411 又往 STM32F411 发布（为了收云端下发的指令），
   不加 /set 的话每发一条就会收到一条自己的 +MQTTSUBRECV 回灌。
   2026-11-17 实测：正因为这个回灌，每条发布后面都跟着一串 URC，
   AT 解析被搅得一团糟。

   BEMFA_PUB_USE_SET = 1 时发布到 "STM32F411/set"，broker 不再推回给自己；
   = 0 时直接发布到 "STM32F411"（原始行为）。

   2026-11-17 12:42 手发实测（ESP-01S + USB-TTL）：
     AT+MQTTPUB=0,"STM32F411","q0",0,0       → OK，MQTTX 收到 1 条，模块自己回灌 1 条 SUBRECV
     AT+MQTTPUB=0,"STM32F411/set","set0",0,0 → OK，MQTTX 收到 1 条，模块自己 0 条回灌
   结论：/set 有效，订阅者照收，发布者不再自收。
   ⚠ 但 2026-11-17 13:5x 实测：改成 /set 后板子重新上电，MQTT 在线却一条数据都到不了
     云端（MQTTX 0/0）。为把变量减到最少，先回到**板子自己唯一成功过的配置**——
     纯主题 STM32F411。等通了再单独验证 /set。 */
#define BEMFA_PUB_USE_SET  0

#if BEMFA_PUB_USE_SET
#define BEMFA_PUB_TOPIC   BEMFA_TOPIC "/set"
#else
#define BEMFA_PUB_TOPIC   BEMFA_TOPIC
#endif

/* ===== 发布 QoS =====
   必须用 0。2026-11-17 手发实测（D:\Desktop\20.txt + 全部数据.json）：
     AT+MQTTPUB=...,1,0  → 一条消息被投递 25 次（12:29:33~12:30:02，间隔约 1.2s），
                            且固件在等不到 PUBACK 时**连 OK 都不回**，
                            导致 ESP_SendCmd(pub,"OK",5000) 每次都超时 ——
                            这就是"消息发出去了但软件认为失败"的根源。
     QoS 0 没有 PUBACK、没有重传、立即回 OK。
   ESP-AT 的 QoS1 在巴法云这个 broker 上不能用。 */
#define BEMFA_PUB_QOS     0

/* 把宏的值变成字符串，供 AT 命令拼接用 */
#define PAL_STR_(x) #x
#define PAL_STR(x)  PAL_STR_(x)

/* ===== 上报间隔（毫秒）=====
   全工程只此一处控制"多久往云端发一条"，想改就只动这个数。
   背景：巴法云免费版对推送频率有限制，公开资料普遍写 5s/条。
   实测：6 秒一条可长时间稳定；压到 2 秒左右会出现
        "板子每一条都拿到 +MQTTPUB:OK，云端却明显少收" 的现象。
     6000 = 10 条/分钟（默认，稳）
     4000 = 15 条/分钟（想更跟手可以试，但留意云端是否开始漏收）
     2000 = 30 条/分钟（不建议，实测会漏）
   注意：采集是独立的，一直按 1 秒多一轮在跑，改这个只影响"多久往云端报一次"。 */
#define PUB_INTERVAL_MS   6000

/* ===== 鉴权方式（依据巴法云官方文档《MQTT协议接入》步骤二）=====
   方式一（BEMFA_AUTH_MODE2 = 0，当前使用）：
       ClientID = 私钥，用户名/密码留空。
       ⚠ 致命缺点：巴法云网页控制台、MQTTX、以及板子三方面如果都用私钥当
       ClientID，MQTT 3.1.1 规定同一 ClientID 同时只能有一个连接，
       后连的会把先连的顶下线 —— 板子发出去的数据会直接丢在半路。

   方式二（BEMFA_AUTH_MODE2 = 1）：
       ClientID 自己起一个唯一名字（不占私钥），
       用户名 = appID，密码 = secretKey。
       这样板子再也不会和网页/MQTTX 抢同一个 ClientID。
       官方文档原文："如果客户端ID设置错误或不匹配，系统会使用用户名和密码
       进行身份验证，其中用户名是appID，密码是secretKey"。

   appID / secretKey 获取：登录后打开
   https://cloud.bemfa.com/web/v2/index.html#/connection-secret/index */
#define BEMFA_AUTH_MODE2   0

#define BEMFA_CLIENT_ID    "stm32gw01"          /* 自定义，全局唯一即可，不用私钥 */
#define BEMFA_APPID        "在此填appID"
#define BEMFA_SECRETKEY    "在此填secretKey"

/* ===== MQTT 遗嘱（Last Will）=====
   板子非正常掉线（拔电/断网/死机）时，broker 会在 keepalive 超时后
   替它把这条消息发到遗嘱主题，订阅者据此能明确知道"设备掉线了"，
   而不是靠"数据不刷新"去猜。

   ⚠ 默认关闭（0）。开启前必须在巴法云控制台先建好同名主题，
      否则 broker 可能在 CONNACK 阶段拒绝连接，导致板子完全连不上。
      关闭时整个遗嘱分支不执行，行为与加遗嘱之前完全一致。 */
#define BEMFA_WILL_ENABLE 0

#define BEMFA_WILL_TOPIC  "STM32F411STATE"
#define BEMFA_WILL_MSG    "offline"
#define BEMFA_KEEPALIVE   120     /* 秒。broker 在 1.5 倍即 180 秒后判定掉线 */



typedef struct {
    uint8_t client_id[50];
    uint8_t username[24];
    uint8_t password[24];
    int socket;
    uint32_t keepalive_interval;
    uint8_t clean_session;
    
    // 关键：函数指针成员，用于发送和接收数据
    int (*send)(void* sock, const void *buf, unsigned int len);
    int (*recv)(int sock, uint8_t *buf, int buf_len, int timeout);
    
    void *socket_info; 
} mqtt_broker_handle_t;



void ESP_HardReset(void);



void MQTT_Init(void);
void RE_MQTT_Init(void);

/* 诊断遥测：开机/重连各发一次（带 busy 保护），运行期由 AT 任务周期调用 */
ESP_Status_t MQTT_SendProbe(void);   /* 返回真实投递结果，供重连后判定会话是否真的可用 */
void MQTT_SendTelemetry(void);

/* ===== 全工程唯一的发布入口 =====
   传入**纯 JSON**（不要自己转义引号、不要拼 AT 命令），内部：
     ① 先走 AT+MQTTPUBRAW（原样字节，回 +MQTTPUB:OK/FAIL）
     ② 固件不支持时自动退回 AT+MQTTPUB 字符串形式（内部负责转义）
   返回值：ESP_OK = broker 已确认收到（PUBRAW 路径）或命令被接受（兜底路径）。
   调用方必须已经持有 uart1_mutex。 */
ESP_Status_t MQTT_Publish(const char *json, uint32_t timeout_ms);

int MQTT_SendTempHumi(float temp, float humi);
ESP_Status_t MQTT_SendPing(void);

void MQTT_ReConnect(void);
#endif
