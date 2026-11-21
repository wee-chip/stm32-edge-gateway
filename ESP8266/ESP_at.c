#include "esp_at.h"
#include "command.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>     /* atoi：解析 +MQTTCONN? 的 state 字段 */

/* 最近一次 +MQTTCONN:<...>,<state> 解析结果，语义见 ESP_at.h 注释 */
volatile int g_mqtt_conn_state = -1;
/* 最近一次 AT+CWJAP? 解析结果：0=No AP（未连），1=已连，-1=未知 */
volatile int g_wifi_joined = -1;
/* 最近一次 AT+CIFSR 解析结果：0=无 IP，1=有 IP，-1=未查询 */
volatile int g_sta_ip_ok = -1;
/* 最近一次 AT+CIFSR 解析到的 IP 原文 */
char g_sta_ip_str[20] = {0};

/* 发布失败取证（详见 ESP_at.h）：阶段 + 触发失败的那一行原文 + 断连次数 */
volatile uint8_t  g_pub_stage = 0;
char              g_pub_errtxt[48] = {0};
volatile uint16_t g_disc_cnt = 0;

/* 云端指令（详见 ESP_at.h）。payload 超过 95 字节的一律不收：
   一方面我们自己回灌的数据都有 160+ 字节，另一方面指令本来就很短。 */
volatile uint8_t g_cmd_pending = 0;
char             g_cmd_payload[96] = {0};

/* 记录"发布失败发生在哪一阶段"，并保存那一行 AT 原文。
   只保留可打印 ASCII（0x20~0x7E），其余替换成 '.' ——
   AT 回包里可能夹着 0x00/0x0D，直接塞进日志会把整条日志截断。 */
static void PubMark(uint8_t stage, const char *line)
{
    uint8_t i = 0;

    g_pub_stage = stage;

    if (line != NULL)
    {
        while (line[i] != '\0' && i < 40 && i < (sizeof(g_pub_errtxt) - 1))
        {
            char c = line[i];
            g_pub_errtxt[i] = (c >= 0x20 && c < 0x7F) ? c : '.';
            i++;
        }
    }
    g_pub_errtxt[i] = '\0';
}

// 引用阶段一创建的全局环形缓冲区
extern uint8_t buffer[BUFFER_SIZE];

/* 把环形缓冲里积压的字节按行解析一遍。
   为什么要它取代原来的"直接丢弃"：云端下发的指令是以 URC（+MQTTSUBRECV）
   形式到达的，如果每条 AT 命令开头都无脑清缓冲，指令就会在还没被看见时被扔掉。
   这里改成"先收割、再发命令"，URC 不会丢，脏数据也照样被消费掉。
   ⚠ 只允许在"没有 AT 命令在途"时调用（持锁或空闲），不能插进读取循环中间。 */
void ESP_UrcPoll(void)
{
    char line[160];
    uint8_t n = 0;
    uint8_t ch;

    while (RingBuf_Read(&ch))
    {
        if (ch == '\n')
        {
            line[n] = '\0';
            if (n >= 2)
            {
                extern volatile uint8_t g_mqtt_connected;
                extern volatile uint8_t g_pub_fail_cnt;

                if (strstr(line, "+MQTTDISCONNECTED") != NULL)
                {
                    g_mqtt_connected = 0;
                    g_pub_fail_cnt   = 0;
                    g_disc_cnt++;
                }
                else if (strstr(line, "+MQTTSUBRECV") != NULL)
                {
                    /* 只收带 "cmd" 的；板子自己发布到同一主题会回灌自己的数据，
                       那些没有 cmd 字段，直接丢掉，不当指令。 */
                    const char *q = strchr(line, ':');
                    uint8_t commas = 0;

                    if (q != NULL)
                    {
                        q++;
                        while (*q != '\0' && commas < 3)
                        {
                            if (*q == ',') commas++;
                            q++;
                        }
                        if (commas == 3 && *q != '\0')
                        {
                            size_t len = strlen(q);
                            if (len > 0 && len < sizeof(g_cmd_payload) &&
                                strstr(q, "\"cmd\"") != NULL)
                            {
                                memcpy(g_cmd_payload, q, len + 1);
                                g_cmd_pending = 1;
                            }
                        }
                    }
                }
                else if (strstr(line, "+MQTTCONN:") != NULL)
                {
                    char *c = strrchr(line, ',');
                    if (c != NULL) g_mqtt_conn_state = atoi(c + 1);
                }
                else if (strstr(line, "+CWJAP:") != NULL)
                {
                    g_wifi_joined = 1;
                }
                else if (strncmp(line, "No AP", 5) == 0)
                {
                    g_wifi_joined = 0;
                }
                else if (strstr(line, "+CIFSR:STAIP") != NULL)
                {
                    char *c = strchr(line, '"');
                    if (c != NULL)
                        g_sta_ip_ok = (strncmp(c + 1, "0.0.0.0", 7) == 0) ? 0 : 1;
                }
            }
            n = 0;
        }
        else if (ch != '\r')
        {
            if (n < sizeof(line) - 1) line[n++] = ch;
        }
    }
}


/*
 * 通用 AT 指令发送并逐行解析应答       
 * cmd           : 要发送的 AT 指令（必须包含 \r\n）
 * expected_ack  : 期望出现的应答关键词，如 "OK", "CONNECT", "WIFI GOT IP"
 * timeout_ms    : 超时时间（毫秒）
 * 返回         : ESP_OK(成功), ESP_TIMEOUT(超时), ESP_ERROR(未使用但预留)
 */
//发送 AT 指令 →ESP8266返回数据-> 从环形缓冲区逐行读取应答 → 匹配期望关键词 → 返回成功或超时。
ESP_Status_t ESP_SendCmd(const char *cmd, const char *expected_ack, uint32_t timeout_ms)
{
    // 1. 清空环形缓冲区，丢弃旧数据 逻辑清空 移动读位 逻辑清空
    ESP_UrcPoll();

    // 2. 发送指令 ESP8266 uart1波特率115200
    HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), 1000);

    // 3. 在超时时间内逐字节读取，按行解析
    char line[512];          // 存放一行数据
    uint16_t line_pos = 0;   // 当前行写入位置
    uint32_t start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        uint8_t ch;
			while (RingBuf_Read(&ch)) {									//读缓冲区
            /* 内层循环必须有退出条件：ESP 持续刷 URC/订阅消息时，
               若只有外层判断，内层会一直被新字节喂饱导致永不返回。 */
            if ((HAL_GetTick() - start_tick) >= timeout_ms) {
                return ESP_TIMEOUT;
            }

            if (ch == '\n') {
                // 一行结束
                line[line_pos] = '\0';				//手动赋值结束标志

                /* 跳过 AT 回显行：固件默认开回显，回显内容就是刚发出去的命令本身。
                   payload 里可能含 "OK"（如 "th":"OK"、probe 的 "01:OK"），
                   若不跳过会在这里误判成功，把真正的 ERROR 掩盖掉。 */
                if (line_pos >= 2 && line[0] == 'A' && line[1] == 'T') {
                    line_pos = 0;
                    continue;
                }

                /* 先处理主动上报：MQTT 断线时固件吐 +MQTTDISCONNECTED:<LinkID>。
                   这是唯一能及时发现"被同 ClientID 的客户端顶下线"的信号 ——
                   AT 指令本身照样回 OK，光靠 AT 心跳永远发现不了。 */
                if (strstr(line, "+MQTTDISCONNECTED") != NULL) {
                    extern volatile uint8_t g_mqtt_connected;
                    extern volatile uint8_t g_pub_fail_cnt;
                    g_mqtt_connected = 0;
                    g_pub_fail_cnt   = 0;
                    g_disc_cnt++;               /* 取证：链路是被"断开"的，不是发不出去 */
                    PubMark(5, line);
                    line_pos = 0;
                    continue;
                }

                /* AT+MQTTCONN? 的查询回包：+MQTTCONN:0,1,"host",port,"path",reconnect,<state>
                   state 是最后一个逗号后的数字（官方表：4/5/6=已连接，3=已断开）。
                   注意：热点关闭时 TCP 可能半开，这个 state 不一定立刻变 3，
                   所以心跳还要配合 AT+CWJAP? 一起判（见 pal.c MQTT_SendPing）。 */
                if (strstr(line, "+MQTTCONN:") != NULL) {
                    char *p = strrchr(line, ',');
                    if (p != NULL)
                        g_mqtt_conn_state = atoi(p + 1);
                }

                /* AT+CWJAP? 回包：+CWJAP 表示已关联，No AP 表示未连接（须从行首匹配，
   SSID 里可能含同样字样）。 */
                if (strstr(line, "+CWJAP:") != NULL) {
                    g_wifi_joined = 1;
                } else if (strncmp(line, "No AP", 5) == 0) {
                    /* 必须从行首匹配：SSID 里也可能出现 "No AP" 字样，
                       用 strstr 会把"已连接"误判成"未连接"，进而误触发硬复位。 */
                    g_wifi_joined = 0;
                }

                /* AT+CIFSR 的回包：+CIFSR:STAIP,"192.168.43.5"
                   用于区分"关联上热点"和"真的拿到 IP"。0.0.0.0 = DHCP 没过，
                   此时 MQTT 一定连不出去。 */
                if (strstr(line, "+CIFSR:STAIP") != NULL) {
                    char *q = strchr(line, '"');
                    if (q != NULL) {
                        g_sta_ip_ok = (strncmp(q + 1, "0.0.0.0", 7) == 0) ? 0 : 1;
                        /* 顺便把 IP 原文留下来，日志里能直接看到是不是 0.0.0.0 */
                        uint8_t k = 0;
                        const char *s = q + 1;
                        while (*s != '\0' && *s != '"' && k < sizeof(g_sta_ip_str) - 1)
                            g_sta_ip_str[k++] = *s++;
                        g_sta_ip_str[k] = '\0';
                    }
                }

                /* 先判错误再判成功：同一行同时含两者时不能误返回成功。 */
                if (strstr(line, "ERROR") != NULL ||
                    strstr(line, "ERR CODE:") != NULL ||
                    strstr(line, "+MQTTPUB:FAIL") != NULL ||
                    strstr(line, "busy p") != NULL) {
                    return ESP_ERROR;
                }

                if (expected_ack != NULL) {
                    /* "OK" 要求整行精确匹配；其它关键字（WIFI GOT IP、+MQTTCONNECTED）用子串 */
                    if (strcmp(expected_ack, "OK") == 0) {
                        if (strcmp(line, "OK") == 0) {
                            return ESP_OK;
                        }
                    } else if (strstr(line, expected_ack) != NULL) {
                        return ESP_OK;
                    }
                }

                // 清空 line，准备读下一行
                line_pos = 0;
            } 
            else if (ch != '\r') {
                // 非回车字符，追加到 line
                if (line_pos < sizeof(line) - 1) {
                    line[line_pos++] = ch;
                }
            }
        }
    }
    
    return ESP_TIMEOUT;
}

// 简化版：发指令，只等待 "OK"
ESP_Status_t ESP_SendCmd_OK(const char *cmd, uint32_t timeout_ms)
{
    return ESP_SendCmd(cmd, "OK", timeout_ms);
}

/* 仅取证：见 ESP_at.h 的说明。
   逐行扫一遍环形缓冲里已经积压的字节，只统计 +MQTTDISCONNECTED 次数，
   不碰 g_mqtt_connected、不碰任何业务标志。
   这样即使断连 URC 落在"没人读串口"的空档里，d= 也不会漏计。 */
void ESP_UrcSniff(void)
{
    /* 兼容旧调用名：现在整套 URC 解析都在 ESP_UrcPoll 里，
       这样指令接收和断连计数共用同一份逻辑，不会出现两个解析器互相打架。 */
    ESP_UrcPoll();
}


/* MQTT 发布：AT+MQTTPUB 的应答分两段（OK 表示命令被接受，+MQTTPUB:OK 才是投递成功）。
   只等前一段会把失败当成成功，故两段都判。 */
ESP_Status_t ESP_MQTTPub(const char *cmd, uint32_t timeout_ms)
{
    g_pub_stage = 0;                 /* 取证：每轮发布重新记录 */
    ESP_UrcPoll();
    HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), 1000);

    char line[256];
    uint16_t line_pos = 0;
    uint32_t start_tick = HAL_GetTick();
    uint8_t accepted = 0;          /* 已收到第一段 OK */

    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        uint8_t ch;
        while (RingBuf_Read(&ch)) {
            if ((HAL_GetTick() - start_tick) >= timeout_ms) goto pub_done;

            if (ch == '\n') {
                line[line_pos] = '\0';

                /* 跳过命令回显行（回显内容就是刚发出去的命令本身） */
                if (line_pos >= 2 && line[0] == 'A' && line[1] == 'T') {
                    line_pos = 0;
                    continue;
                }

                /* 第二段：真正的结果 */
                if (strstr(line, "+MQTTPUB:OK")   != NULL) return ESP_OK;
                if (strstr(line, "+MQTTPUB:FAIL") != NULL) { PubMark(2, line); return ESP_ERROR; }

                /* 其它失败形式 */
                if (strstr(line, "+MQTTDISCONNECTED") != NULL) {
                    extern volatile uint8_t g_mqtt_connected;
                    g_mqtt_connected = 0;
                    g_disc_cnt++;
                    PubMark(5, line);
                    return ESP_ERROR;
                }
                if (strstr(line, "ERROR")    != NULL ||
                    strstr(line, "ERR CODE:") != NULL ||
                    strstr(line, "busy p")   != NULL) {
                    PubMark(3, line);
                    return ESP_ERROR;
                }

                /* 第一段：命令被接受，再给 300ms 看有没有 +MQTTPUB:FAIL。
                   QoS0 下固件通常只有 OK 没有 URC，不能在这儿死等。 */
                if (strcmp(line, "OK") == 0) {
                    accepted = 1;
                    start_tick = HAL_GetTick() - (timeout_ms > 300 ? (timeout_ms - 300) : timeout_ms);
                }

                line_pos = 0;
            }
            else if (ch != '\r') {
                if (line_pos < sizeof(line) - 1) line[line_pos++] = ch;
            }
        }
    }

pub_done:
    if (!accepted) PubMark(4, NULL);       /* 连第一段 OK 都没等到 = 无任何回执 */
    return accepted ? ESP_OK : ESP_TIMEOUT;
}


/* 原始字节发布：AT+MQTTPUBRAW → OK → '>' → 数据 → +MQTTPUB:OK/FAIL。
   payload 原样发送，无需转义，也不受单条命令 256 字节限制。 */
ESP_Status_t ESP_MQTTPubRaw(const char *topic, const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    char cmd[96];
    char line[192];
    uint16_t line_pos = 0;
    uint32_t start_tick;
    uint8_t got_prompt = 0;

    if (topic == NULL || data == NULL || len == 0)
        return ESP_ERROR;

    g_pub_stage = 0;                 /* 取证：每轮发布重新记录 */

    snprintf(cmd, sizeof(cmd), "AT+MQTTPUBRAW=0,\"%s\",%u,0,0\r\n", topic, (unsigned)len);

    ESP_UrcPoll();
    HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), 1000);

    /* ---- 阶段一：等 '>' 提示符。3 秒等不到 = 固件不支持/参数被拒，交回上层兜底 ----
       ⚠ 取证：这一阶段原来只认 '>'，模块回 ERROR 也照样干等满 3 秒。
       现在把读到的每一行都过一遍失败关键字，命中就立刻记下原文返回，
       不再白等 —— 既省 3 秒，也把"是不是固件根本不支持 MQTTPUBRAW"变成可读的证据。 */
    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < 3000) {
        uint8_t ch;
        while (RingBuf_Read(&ch)) {
            if (ch == '>') { got_prompt = 1; break; }
            /* 顺手做行级失败识别 */
            if (ch == '\n') {
                line[line_pos] = '\0';
                if (!(line_pos >= 2 && line[0] == 'A' && line[1] == 'T')) {
                    if (strstr(line, "+MQTTPUB:FAIL") != NULL) { PubMark(2, line); return ESP_ERROR; }
                    if (strstr(line, "+MQTTDISCONNECTED") != NULL) { g_disc_cnt++; PubMark(5, line); return ESP_ERROR; }
                    if (strstr(line, "ERROR")     != NULL ||
                        strstr(line, "ERR CODE:") != NULL ||
                        strstr(line, "busy p")    != NULL) { PubMark(3, line); return ESP_ERROR; }
                }
                line_pos = 0;
            } else if (ch != '\r') {
                if (line_pos < sizeof(line) - 1) line[line_pos++] = ch;
            }
        }
        if (got_prompt) break;
    }
    if (!got_prompt)
    {
        PubMark(1, NULL);            /* 3 秒内没出现 '>' —— 固件不支持或参数被拒 */
        return ESP_ERROR;
    }

    line_pos = 0;                    /* '>' 之前可能留了半行，清掉，别污染阶段三的解析 */

    /* ---- 阶段二：原样送 len 个字节 ----
       不能多送 \r\n：多出来的字节会被算进 payload 或当成下一条 AT 命令的开头。 */
    HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 2000);

    /* ---- 阶段三：等 broker 的真实回执 ----
       QoS0 下 +MQTTPUB:OK 通常在送出后 100ms 内就回来了。
       这里把等待上限压到 4 秒：调用方持有 uart1_mutex，等太久会把
       AT 任务的心跳/重连一起堵住（心跳任务取锁超时 3 秒就跳过本轮）。 */
    if (timeout_ms > 4000) timeout_ms = 4000;

    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        uint8_t ch;
        while (RingBuf_Read(&ch)) {
            if ((HAL_GetTick() - start_tick) >= timeout_ms) { PubMark(4, NULL); return ESP_TIMEOUT; }

            if (ch == '\n') {
                line[line_pos] = '\0';
                if (line_pos >= 2 && line[0] == 'A' && line[1] == 'T') { line_pos = 0; continue; }

                if (strstr(line, "+MQTTPUB:OK")   != NULL) return ESP_OK;
                if (strstr(line, "+MQTTPUB:FAIL") != NULL) { PubMark(2, line); return ESP_ERROR; }

                if (strstr(line, "+MQTTDISCONNECTED") != NULL) {
                    extern volatile uint8_t g_mqtt_connected;
                    g_mqtt_connected = 0;
                    g_disc_cnt++;
                    PubMark(5, line);
                    return ESP_ERROR;
                }
                if (strstr(line, "ERROR")     != NULL ||
                    strstr(line, "ERR CODE:") != NULL ||
                    strstr(line, "busy p")    != NULL ||
                    (strstr(line, "+MQTTPUBRAW") != NULL && strstr(line, "FAIL") != NULL)) {
                    PubMark(3, line);
                    return ESP_ERROR;
                }
                line_pos = 0;
            }
            else if (ch != '\r') {
                if (line_pos < sizeof(line) - 1) line[line_pos++] = ch;
            }
        }
    }
    PubMark(4, NULL);                /* 数据送完了，但一条回执都没有 */
    return ESP_TIMEOUT;
}



/**
 * @brief  等待环形缓冲区中出现指定关键字（超时返回）
 * @param  keyword   要匹配的关键字（字符串）
 * @param  timeout_ms 超时时间（毫秒）
 * @return 1-匹配成功，0-超时未匹配
 */
//int ESP_WaitFor(const char *keyword, uint32_t timeout_ms)
//{
//    int kw_len = strlen(keyword);
//    if (kw_len == 0) return 1;  // 空字符串视为立即成功

//    char match_buf[kw_len];     // 滑动窗口，保存最近 kw_len 个字节
//    memset(match_buf, 0, kw_len);
//    int pos = 0;                // 累计已读取字节数
//    uint32_t start = HAL_GetTick();

//    while ((HAL_GetTick() - start) < timeout_ms)
//    {
//        uint8_t ch;
//        if (RingBuf_Read(&ch))
//        {
//            // 将新字节写入环形窗口
//            match_buf[pos % kw_len] = ch;
//            pos++;

//            // 只有填满窗口后才开始匹配
//            if (pos >= kw_len)
//            {
//                // 窗口中最旧字节的索引（即最近 kw_len 个字节的起始位置）
//                int start_idx = (pos - kw_len) % kw_len;

//                // 数据在 match_buf 中可能分成两段：
//                // 第一段：从 start_idx 到数组末尾
//                // 第二段：从 0 到 start_idx-1
//                int first_len = kw_len - start_idx;       // 第一段长度
//                int second_len = kw_len - first_len;      // 第二段长度，即 start_idx

//                // 比较第一段（如果 first_len > 0）
//                if (first_len > 0 &&
//                    memcmp(&match_buf[start_idx], keyword, first_len) != 0)
//                {
//                    continue;   // 第一段不匹配，直接继续
//                }

//                // 比较第二段（如果存在）
//                if (second_len > 0 &&
//                    memcmp(&match_buf[0], keyword + first_len, second_len) != 0)
//                {
//                    continue;
//                }

//                // 两段都匹配成功
//                return 1;
//            }
//        }
//        // 可以适当加一个短延时避免忙等待，但视系统而定，此处省略
//    }
//    return 0;   // 超时
//}
