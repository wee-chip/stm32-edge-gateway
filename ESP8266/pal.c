#include "esp_at.h"
#include "command.h"

/* ===== 临时诊断：MQTT 握手结果，问题解决后删掉 ===== */
volatile int8_t   g_conn_ret   = -9;   // 收到的字节数：4=收到CONNACK，-1=超时，-9=未执行
volatile uint8_t  g_connack0   = 0;    // 应为 0x20
volatile uint8_t  g_connack3   = 0;    // 0=成功 1=协议版本 2=ID被拒 4=账号密码 5=未授权
volatile uint8_t  g_wifi_ok    = 0;    // CWJAP 是否返回 0（0=成功）

extern uint8_t Command_GetLength(void);   /* command.c 里定义，头文件未导出 */

/* 诊断：LED2（PB3，低电平点亮）闪 n 次，停 2 秒后循环 */
static void DiagBlink(uint8_t n)
{
    if(n == 0) n = 10;
    for(uint8_t i = 0; i < n; i++){
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_Delay(250);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
        HAL_Delay(400);
    }
    HAL_Delay(2000);
}


#include "usart.h"
#include <string.h>
#include <stdio.h>
#include "stdlib.h"
#include "libemqtt.h"
#include "pal.h"
#include <string.h> 
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "flash.h"
#include "Taskhandel.h"
#include "rtc.h"        /* RTC_SetDateTime：SNTP 校时写进 MCU 的 RTC */

extern uint8_t a[256];
extern DMA_HandleTypeDef hdma_usart1_rx;
/* 串口互斥锁（定义在 freertos.c）。文件级声明，方便各处统一取锁。 */
extern SemaphoreHandle_t uart1_mutex;

/* SNTP 校时（实现在文件后半部分）。MQTT_Init 在它之前就要用，先声明。 */
static uint8_t ESP_SyncTime(void);

/* 所有网络参数只从 pal.h 读取。手机热点分配网段会变化，不能在此写死 IP/网关。 */
static ESP_Status_t ESP_JoinWiFi(void)
{
    char cmd[128];

    /* 由热点 DHCP 分配 IP、网关和 DNS；忽略不支持该命令的旧 AT 固件返回值。 */
    ESP_SendCmd_OK("AT+CWDHCP=1,1\r\n", 2000);

    /* 先查是不是已经连着这个热点。
       模块 flash 里存过 AP 时可能已自连完成，这时再发 AT+CWJAP 不会再打印
       WIFI CONNECTED / WIFI GOT IP，只回一个 OK —— 而下面只等 "WIFI GOT IP"
       会白等 15 秒判失败。现象就是：手机热点上明明显示 ESP 已连接，
       固件却认为 WiFi 没连上、根本不去连 MQTT。 */
    if (ESP_SendCmd("AT+CWJAP?\r\n", WIFI_SSID, 2000) == ESP_OK)
        return ESP_OK;

    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PASS);
    /* 手机热点 DHCP 偶尔要 15 秒以上，给到 25 秒 */
    if (ESP_SendCmd(cmd, "WIFI GOT IP", 25000) == ESP_OK)
        return ESP_OK;

    /* 已经连上时上面可能只回 OK，再查一次兜底 */
    if (ESP_SendCmd("AT+CWJAP?\r\n", WIFI_SSID, 2000) == ESP_OK)
        return ESP_OK;

    return ESP_ERROR;
}




/* ESP8266 uart1 波特率115200 DMA空闲中断*/
//+++正确退出透传后 下一条语句就不会输出error
/* ===== PB8 hard reset for ESP8266 ===== */
void ESP_HardReset(void)
{
	HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_RESET);
	vTaskDelay(pdMS_TO_TICKS(20));
	HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_SET);
	vTaskDelay(pdMS_TO_TICKS(500));
}

/* ================= MQTT 连接参数与遗嘱 =================
   ESP-AT 的 AT+MQTTCONNCFG 必须在 AT+MQTTUSERCFG 之后、AT+MQTTCONN 之前下发。
   官方语法：
     AT+MQTTCONNCFG=<LinkID>,<keepalive>,<disable_clean_session>,
                    <lwt_topic>,<lwt_msg>,<lwt_qos>,<lwt_retain>
   注意 disable_clean_session 是反的：0 = 使能清理会话，1 = 禁用。
   这里传 0，与改动前的默认行为一致（每次重连都重新订阅）。 */

#if BEMFA_WILL_ENABLE
static ESP_Status_t ESP_MQTT_SetWill(uint8_t enable)
{
    char cmd[160];

    if (!enable) return ESP_OK;   /* 降级路径：什么都不下发 = 恢复改动前的默认参数 */

    snprintf(cmd, sizeof(cmd),
        "AT+MQTTCONNCFG=0,%u,0,\"%s\",\"%s\",0,0\r\n",
        (unsigned)BEMFA_KEEPALIVE, BEMFA_WILL_TOPIC, BEMFA_WILL_MSG);

    return ESP_SendCmd(cmd, "OK", 5000);
}
#endif

static ESP_Status_t ESP_MQTT_ConnRaw(void)
{
    char cmd[128];
    ESP_Status_t st;

    snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=0,\"%s\",%d,1\r\n", BEMFA_BROKER, BEMFA_PORT);

    st = ESP_SendCmd(cmd, "+MQTTCONNECTED", 15000);
    if (st == ESP_OK)
        return ESP_OK;

    /* ⚠ 只有"被明确判错"才重试 —— busy p... / ERROR / ERR CODE 表示命令根本没被执行。
       超时（ESP_TIMEOUT）绝不能重试：那说明命令可能已经执行成功、只是 +MQTTCONNECTED
       被别的 URC 冲掉了。再发一次会收到 ALREADY CONNECTED 的 ERROR，
       于是"其实已经连上"被判成"连不上"，g_mqtt_connected 一直是 0 ——
       现象就是巴法云显示在线、板子却一条数据都不发。 */
    if (st != ESP_ERROR)
        return st;

    vTaskDelay(pdMS_TO_TICKS(500));
    return ESP_SendCmd(cmd, "+MQTTCONNECTED", 15000);
}

/* 下发 MQTT 用户配置。全工程只有这一处拼 AT+MQTTUSERCFG，
   这样切换鉴权方式只需要改 pal.h 里的宏。 */
static ESP_Status_t ESP_MQTT_UserCfg(void)
{
    char cmd[160];

#if BEMFA_AUTH_MODE2
    /* 方式二（巴法云官方文档「MQTT协议接入」步骤二）：
       ClientID 用自定义的唯一名，用户名 = appID，密码 = secretKey。
       好处：不占用私钥当 ClientID，永远不会被网页控制台 / MQTTX 顶下线。 */
    snprintf(cmd, sizeof(cmd),
        "AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"\r\n",
        BEMFA_CLIENT_ID, BEMFA_APPID, BEMFA_SECRETKEY);
#else
    /* 方式一：私钥直接当 ClientID，用户名密码留空 */
    snprintf(cmd, sizeof(cmd), "AT+MQTTUSERCFG=0,1,\"%s\",\"\",\"\",0,0,\"\"\r\n", BEMFA_UID);
#endif

    if (ESP_SendCmd(cmd, "OK", 5000) == ESP_OK)
        return ESP_OK;

    /* 同上：busy p... 会把 USERCFG 丢掉，补一次重试，否则后面 MQTTCONN 必然失败 */
    vTaskDelay(pdMS_TO_TICKS(300));
    return ESP_SendCmd(cmd, "OK", 5000);
}

/* 连接 broker：先带遗嘱尝试；连不上就去掉遗嘱重来一次。
   这样即使巴法云不认这个遗嘱主题，板子也只是开机慢一点，不会永久连不上。 */
static ESP_Status_t ESP_MQTT_Connect(void)
{
#if !BEMFA_WILL_ENABLE
    /* 遗嘱关闭：一条 CONNCFG 都不发，也不走降级路径，
       行为与"加遗嘱之前"逐字节一致 —— 出问题时可以立刻回到这个已知可用状态。 */
    return ESP_MQTT_ConnRaw();
#else
    ESP_Status_t will_st;

    will_st = ESP_MQTT_SetWill(1);
    if (will_st != ESP_OK)
        Log_Write(LOG_WARN, "will cfg rejected");   /* 固件不支持/参数非法，继续按默认参数连 */

    if (ESP_MQTT_ConnRaw() == ESP_OK)
        return ESP_OK;

    if (will_st != ESP_OK)
        return ESP_ERROR;      /* 遗嘱本来就没配上，说明失败与遗嘱无关，不再重试 */

    /* 带遗嘱连不上 → 多半是 broker 拒绝了这个遗嘱主题，去掉遗嘱重连 */
    Log_Write(LOG_WARN, "conn w/will fail, retry");
    ESP_SendCmd_OK("AT+MQTTCLEAN=0\r\n", 3000);
    vTaskDelay(pdMS_TO_TICKS(500));

    /* AT+MQTTCLEAN 会清掉用户配置，必须重新下发 USERCFG 才能再连 */
    if (ESP_MQTT_UserCfg() != ESP_OK)
        return ESP_ERROR;

    return ESP_MQTT_ConnRaw();   /* 不发 CONNCFG，保持改动前的默认参数 */
#endif
}

/* ================= 全工程唯一的 MQTT 发布入口 =================
   之前 modbus.c / flash.c / pal.c 各自拼 AT+MQTTPUB，三处都要手工把 JSON 里的
   双引号转义成 \" 再塞进 AT 命令。只要有一处转义写错，整条命令就会被 AT 解析器
   截断 —— 固件回 ERROR 或干脆不回，而软件侧看起来"发了"，云端一条没有。
   现在统一走这里：

   ① 首选 AT+MQTTPUBRAW：payload 原样送字节，一个引号都不用转义，
      并且固件会回 +MQTTPUB:OK/FAIL —— 这是"broker 到底收没收到"的唯一确证。
   ② 若固件连 '>' 提示符都不给（不支持 PUBRAW），自动退回老的字符串形式，
      与改动前的行为完全一致，不会比原来更差。
   s_raw_miss 记录"连 '>' 都没等到"的次数，连续 2 次就永久改走字符串形式，
   避免在不支持的固件上每条消息白等 3 秒。 */
static uint8_t s_raw_miss = 0;

/* 本次发布若在 MQTTPUBRAW 阶段失败，记下那时的阶段码。
   字符串兜底会重置 g_pub_stage，不单独存一份就看不到原始原因。
   （原 PubFailLog 的逐条失败日志已于 2026-11-19 22:35 撤掉：
     故障已定位，逐条打印只会把日志页塞满，日志留给"发生了什么"，
     不再记录"每一帧死在哪"。需要细查时把 g_pub_stage 打进 probe JSON 即可。） */
static uint8_t s_raw_stage = 0;

ESP_Status_t MQTT_Publish(const char *json, uint32_t timeout_ms)
{
    char esc[300];
    char pub[384];
    size_t i, e = 0;
    ESP_Status_t st;

    if (json == NULL || json[0] == '\0')
        return ESP_ERROR;

    s_raw_stage = 0;

    if (s_raw_miss < 2)
    {
        st = ESP_MQTTPubRaw(BEMFA_PUB_TOPIC, (const uint8_t *)json,
                            (uint16_t)strlen(json), timeout_ms);
        if (st == ESP_OK)
        {
            s_raw_miss = 0;
            return ESP_OK;
        }
        /* 取证：记下 PUBRAW 死在哪一步（字符串兜底会把 g_pub_stage 覆盖掉） */
        s_raw_stage = g_pub_stage;

        /* 只有"连 '>' 都没出现"才判定固件不支持 PUBRAW；
           其它失败（+MQTTPUB:FAIL / ERROR / 无回执）说明 PUBRAW 是可用的，
           不改 s_raw_miss，仍按改动前的行为落到字符串兜底再试一次。 */
        if (g_pub_stage == 1)
            s_raw_miss++;
    }

    /* ---- 兜底：AT+MQTTPUB 字符串形式 ----
       必须把 " 和 \ 都转义，因为 AT 解析器靠引号切分参数。

       ⚠ 2026-11-21：这条路有两条硬限制 —— esc[300] 缓冲区，以及
       ESP-AT 单条命令 < 256 字节。批量补传的数组可能有 300~400 字节，
       硬塞进来会被**静默截断**，发上去就是半截 JSON，比不发更糟。
       所以超过 200 字节直接判失败，绝不硬塞。 */
    if (strlen(json) > 200)
        return ESP_ERROR;

    for (i = 0; json[i] != '\0' && e < sizeof(esc) - 3; i++)
    {
        if (json[i] == '"' || json[i] == '\\')
            esc[e++] = '\\';
        esc[e++] = json[i];
    }
    esc[e] = '\0';

    snprintf(pub, sizeof(pub),
        "AT+MQTTPUB=0,\"" BEMFA_PUB_TOPIC "\",\"%s\"," PAL_STR(BEMFA_PUB_QOS) ",0\r\n", esc);

    return ESP_MQTTPub(pub, timeout_ms);
}

/* 上报一次诊断快照（云端可见，不写本地 Flash）。
   首次连接成功后发一次；每次重连成功后再发一次 —— 这样即使实时数据发不出去，
   云端也能从这条 probe 看出板子卡在哪一步：
     tx 不涨        → MQTT 任务根本没进发送路径（g_mqtt_connected=0，或 data_queue 一直空）
     tx 涨、pf 也涨 → 进了发送路径但 broker 不收
     lk 涨          → MQTT 任务卡在串口锁上取不到
     up 变小        → 板子重启过（看是不是看门狗复位循环） */
/* 把 probe 命令拼出来。长度预算：ESP8266 AT 固件单条命令上限 256 字节
   （官方 ESP-AT 文档），这里压在 140 字节以内。
   字段名的含义：
     th  = 温湿度最近一次读取结果 OK/FAIL，两个原值（÷10 得实际值）
     rx  = 最后一帧 Modbus 收到几个字节（0 = 485 上一个字节都没回来）
     ok/er = 累计成功/失败帧数
     t   = 开机秒数
     p   = 连续发布失败计数（>0 说明 AT+MQTTPUB 被拒）
     l   = MQTT 任务取串口锁失败次数
     u   = 进入实时发送分支的次数（不涨 = 任务根本没走到发布）
     tf  = 任务创建失败位图（0 = 五个任务都建成功） */
static void MQTT_ProbeBuild(char *pmsg, size_t n)
{
    extern volatile uint8_t  g_th_ok;
    extern uint16_t g_th_raw[2];
    extern volatile uint8_t  g_rx_count;      /* 最后一次 Modbus 收到的字节数 */
    extern volatile uint32_t g_ok_count;      /* 累计成功帧 */
    extern volatile uint32_t g_fail_count;    /* 累计失败帧 */
    extern volatile uint8_t  g_pub_fail_cnt;  /* 连续发布失败次数 */
    extern volatile uint32_t g_tx_cnt;        /* 进入实时发送分支的次数 */
    extern volatile uint32_t g_lock_fail;     /* 取串口锁失败次数 */
    extern volatile uint8_t  g_task_fail;     /* 任务创建失败位图（bit3=MQTT任务） */

    /* 这里只拼纯 JSON，不含任何 AT 语法 —— 转义交给 MQTT_Publish() 统一处理。
       probe 与业务数据走同一条发布链路：probe 能到云端，就证明发布链路是通的。 */
    snprintf(pmsg, n,
        "{\"probe\":\"th:%s/%u/%u rx:%u ok:%u er:%u\","
        "\"t\":%u,\"p\":%u,\"l\":%u,\"u\":%u,\"tf\":%u}",
        g_th_ok ? "OK" : "FAIL",
        (unsigned)g_th_raw[0],
        (unsigned)g_th_raw[1],
        (unsigned)g_rx_count,
        (unsigned)g_ok_count,
        (unsigned)g_fail_count,
        (unsigned)(xTaskGetTickCount() / 1000u),
        (unsigned)g_pub_fail_cnt,
        (unsigned)g_lock_fail,
        (unsigned)g_tx_cnt,
        (unsigned)g_task_fail);
}

/* 快闪：发布失败时用它一眼就能看出来，不阻塞太久 */
static void QuickBlink(uint8_t n)
{
    for (uint8_t i = 0; i < n; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_Delay(120);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
        HAL_Delay(150);
    }
}

/* 开机 / 重连时用：带等待 + 重试，成功不闪，最终失败快闪 3 下。
   必须自己取串口锁：MQTT_Init 跑在 AT 任务里，而 g_mqtt_connected 已经置 1，
   MQTT 发布任务此刻可能正在发实时数据。两个任务都不持锁去操作 UART1，
   ESP_SendCmd 开头的清缓冲区会把对方的应答清掉，双方一起超时。
   ⚠ 闪码约定：这里闪 3 下 = 「MQTT 已连接，但 AT+MQTTPUB 发不出去」。
      和 11/21/9/22 这些"连不上"的码区分开，方便一眼定位。 */
ESP_Status_t MQTT_SendProbe(void)
{
    char json[256];
    extern SemaphoreHandle_t uart1_mutex;
    uint8_t try;

    MQTT_ProbeBuild(json, sizeof(json));

    /* 置 g_mqtt_connected=1 之后 MQTT 任务会立刻插进来发实时数据，
       两条 AT+MQTTPUB 挨太近会撞固件的 "busy p..."（实测存在的字符串），
       命令被直接丢掉。先让开一段时间。 */
    vTaskDelay(pdMS_TO_TICKS(2000));

    for (try = 0; try < 3; try++)
    {
        if (xSemaphoreTake(uart1_mutex, pdMS_TO_TICKS(5000)) != pdTRUE)
            continue;

        if (MQTT_Publish(json, 5000) == ESP_OK)
        {
            xSemaphoreGive(uart1_mutex);
            return ESP_OK;                /* 成功，不闪灯 */
        }
        xSemaphoreGive(uart1_mutex);

        vTaskDelay(pdMS_TO_TICKS(600));
    }

    QuickBlink(3);                        /* 三次都失败：闪 3 下 */
    return ESP_ERROR;
}

/* 运行期遥测：由 AT 任务每 30 秒调一次。
   屏幕已经坏了，这是唯一能在不改硬件的前提下"看见"板子内部状态的通道 ——
   probe 这条链路已被实测证明能到达巴法云（开机那条就是它）。
   有了 u/p/l 三个计数，就能区分到底卡在哪一环：
     u 不涨        → MQTT 任务没走到发布分支（g_mqtt_connected=0 或队列空）
     u 涨、p 涨    → 走到了发布，但 AT+MQTTPUB 被固件拒（busy p / ERR CODE）
     u 涨、p 不涨  → 发布成功但云端看不到 → 问题在 broker 侧（主题/互踢/限流）
     l 涨          → 卡在串口锁上，AT 任务与 MQTT 任务在互相抢 */
void MQTT_SendTelemetry(void)
{
    char json[256];
    MQTT_ProbeBuild(json, sizeof(json));
    MQTT_Publish(json, 5000);
}

void MQTT_Init(void)
{
	/* 任务创建失败位图（freertos.c 里置位）。
	   堆不够时 xTaskCreate 返回 pdFAIL，任务就这么"消失"了，完全没有报错 ——
	   表现恰好是「MQTT 显示在线，但一条数据都发不出来」。
	   闪的次数 = 位图值：1=UI 2=Modbus 4=AT 8=MQTT 16=Dog。
	   放在最前面，一眼就能排除"任务根本没跑起来"这种可能。 */
	{
		extern volatile uint8_t g_task_fail;
		if (g_task_fail != 0)
			DiagBlink(g_task_fail);
	}

	ESP_HardReset();          /* hard reset first: clean state */
	vTaskDelay(1500);         /* 等启动日志打完再清，避免日志混进 AT 响应 */
	RingBuf_Clear();          /* drop boot log (74880bps) before AT handshake */

	/* 诊断：重新确认 DMA 接收在跑。返回 HAL_ERROR 说明启动失败 */
	if(HAL_UARTEx_ReceiveToIdle_DMA(&huart1, a, sizeof(a)) == HAL_ERROR)
	{
		DiagBlink(18);        /* DMA 接收启动失败 */
		return;
	}
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

	/* 诊断：先测 AT 有没有回应。闪 15 次 = 一个字节都没收到 */
	HAL_UART_Transmit(&huart1, (uint8_t *)"AT\r\n", 4, 1000);
	vTaskDelay(500);
	{
		uint8_t rx_n = Command_GetLength();
		extern volatile uint32_t g_rx_cb_count;
		if(g_rx_cb_count == 0)      DiagBlink(17);   /* 回调从未触发 = DMA/中断没工作 */
		else if(rx_n == 0)          DiagBlink(15);   /* 回调跑了但缓冲空 */
		else                        DiagBlink(rx_n); /* 收到 N 字节 */
	}

	
	  HAL_UART_Transmit(&huart1, (uint8_t *)"+++\r\n", 3, 100);
    vTaskDelay(1000);
	

    // 任意合法AT指令，完成模式切换（不操作TCP，安全无断连）
    ESP_SendCmd_OK("AT\r\n", 2000);
    vTaskDelay(200);
	
    ESP_SendCmd_OK("AT+CWMODE=1\r\n", 2000);

    /* 关掉自动重连，避免 DHCP 超时后无限重连 */
    ESP_SendCmd_OK("AT+CWAUTOCONN=0\r\n", 2000);

    if(ESP_JoinWiFi() != ESP_OK){
        Log_Write(LOG_ERROR, "WiFi join failed");
        DiagBlink(11);   /* WiFi 没连上 */
        return;
    }
		vTaskDelay(500);

    /* 联网了，先对时（ESP8266 SNTP）。放在这里而不是连上 MQTT 之后：
       此刻没有任何数据在发，不会干扰收发包。失败不影响联网。 */
    if (!g_time_valid)
        ESP_SyncTime();
	
    /* ===== MQTT 连接：1471 固件内置 MQTT 指令，无需手写报文/透传 ===== */
    char cmd[128];

    /* 保险：先清掉 LinkID 0 上可能残留的旧连接。
       上一次运行如果非正常结束（看门狗复位 / 直接断电），固件里可能还留着 link 状态，
       此时直接 USERCFG 会回 ERROR，后面的 MQTTCONN 就永远连不上。
       清不掉也无所谓（返回什么都能继续），只是多花几十毫秒。 */
    ESP_SendCmd_OK("AT+MQTTCLEAN=0\r\n", 3000);
    vTaskDelay(pdMS_TO_TICKS(200));

    if(ESP_MQTT_UserCfg() != ESP_OK){
        Log_Write(LOG_ERROR, "MQTTUSERCFG failed");
        DiagBlink(21);   /* MQTT 用户配置失败 */
        return;
    }

    /* 下发连接参数（含遗嘱）并连接；内部自带"去掉遗嘱重试"的降级 */
    if(ESP_MQTT_Connect() != ESP_OK){
        Log_Write(LOG_ERROR, "MQTTCONN failed");
        DiagBlink(9);    /* MQTT 服务器连接失败 */
        g_mqtt_connected = 0;
        return;
    }

    snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=0,\"%s\",1\r\n", BEMFA_TOPIC);
    if(ESP_SendCmd(cmd, "OK", 5000) != ESP_OK){
        Log_Write(LOG_ERROR, "MQTTSUB failed");
        DiagBlink(22);   /* 订阅失败 */
        g_mqtt_connected = 0;
        return;
    }

    /* ===== 顺序回归原作者写法 =====
       原版是「置在线标志 → Log_Write → 发 probe」，11:48 我把 probe 挪到了
       Log_Write 之前，结果从那一刻起一条消息都发不出去。原因是：
       AT+MQTTSUB 回的 OK 只代表"订阅请求已提交"，broker 的 SUBACK 还在路上，
       紧接着 PUB 会撞上固件的 "busy p..."，命令被直接丢弃且不返回 ERROR。
       原版 Log_Write 写 SPI Flash 花掉的几十毫秒，恰好躲过了这个窗口。
       现在恢复原顺序，probe 内部再保留 800ms 等待当双保险。 */
    g_mqtt_connected = 1;
    connect = 1;
    /* 新连接的失败计数必须从 0 开始：否则复用上一轮的 >=5，
       下一次心跳（10s）会立刻又判离线，形成"连上→秒断"的新循环。 */
    {
        extern volatile uint8_t g_pub_fail_cnt;
        g_pub_fail_cnt = 0;
    }
    Log_Write(LOG_INFO, "MQTT online");

    MQTT_SendProbe();
}

/* ================= 连接后取证 =================
   重连/开机成功后调一次，检查第一次投递到底通不通。
   这条日志是唯一能区分"链路被断"和"发不出去"的东西。 */
static void MQTT_DiagAfterConn(void)
{
    /* 通了就什么都不写 —— 日志只记异常，不记"一切正常"。 */
    if (MQTT_SendProbe() == ESP_OK)
        return;

    /* 只留一行：原始阶段/兜底阶段 + 断连次数。
       阶段码含义见 ESP_at.h 的 g_pub_stage；
       d = 上电以来 +MQTTDISCONNECTED 的次数，涨说明链路是被断掉的。
       原来还有第二次 AT+CIFSR / AT+MQTTCONN? 查询和单独一行 ip/wifi/mq 日志，
       故障已定位，逐字段记录只会把日志页塞满，已撤掉。 */
    char d[64];
    snprintf(d, sizeof(d), "probe FAIL st=%u/%u d=%u",
             (unsigned)s_raw_stage, (unsigned)g_pub_stage, (unsigned)g_disc_cnt);
    Log_Write(LOG_ERROR, d);
}

/* ================= 时间同步（ESP8266 SNTP）=================
   为什么必须从 ESP 取时间，而不是直接用 MCU 的 RTC：
     本板 RTC 没有备份电池，而且 MX_RTC_Init() 每次上电都把时间强制写成
     2000-01-01 00:00:00。直接拿 RTC 当时间戳，云端看到的永远是 2000 年，
     比没有时间戳更糟（数据看着"有"，其实全错）。

   ESP-AT 自带 SNTP，不需要 MCU 自己实现协议：
     AT+CIPSNTPCFG=1,8,"ntp.aliyun.com"    打开 SNTP，时区 +8
     AT+CIPSNTPTIME?                       读回 "+CIPSNTPTIME:Sat Nov 21 08:37:00 2026"

   调用时机：MQTT_SoftConn() 开头 —— 此时 WiFi 已连、MQTT 还没连、没有任何
   数据在发。绝不插进收发包中间（这类干扰之前把好好的连接弄断过）。
   拿不到时间不影响联网：失败就跳过，下一轮重连再试。 */

static const char *kMonName[12] = {
    "Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"
};

static uint8_t SntpMonth(const char *m)
{
    uint8_t i;
    for (i = 0; i < 12; i++)
    {
        if (m[0] == kMonName[i][0] && m[1] == kMonName[i][1] && m[2] == kMonName[i][2])
            return (uint8_t)(i + 1);
    }
    return 0;
}

/* STM32 的 RTC_WEEKDAY：Monday=1 ... Sunday=7 */
static uint8_t SntpWeekday(const char *w)
{
    static const char *kDay[7] = {"Mon","Tue","Wed","Thu","Fri","Sat","Sun"};
    uint8_t i;
    for (i = 0; i < 7; i++)
    {
        if (w[0] == kDay[i][0] && w[1] == kDay[i][1] && w[2] == kDay[i][2])
            return (uint8_t)(i + 1);
    }
    return 0;
}

/* 把 "+CIPSNTPTIME:Sat Nov 21 08:37:00 2026" 解析后写进 RTC */
static uint8_t SntpApply(const char *line)
{
    const char *p = strchr(line, ':');
    char wd[4] = {0}, mo[4] = {0};
    unsigned day, hh, mm, ss, yr;
    uint8_t mon, week;

    if (p == NULL)
        return 0;
    p++;   /* 跳过 "+CIPSNTPTIME:" */

    if (sscanf(p, " %3s %3s %u %u:%u:%u %u", wd, mo, &day, &hh, &mm, &ss, &yr) != 7)
        return 0;

    mon  = SntpMonth(mo);
    week = SntpWeekday(wd);

    /* 明显不合理的值一律丢掉（SNTP 还没同步时可能读到 1970 或全 0） */
    if (mon == 0 || week == 0 || yr < 2020 || yr > 2099 ||
        day < 1 || day > 31 || hh > 23 || mm > 59 || ss > 59)
        return 0;

    RTC_SetDateTime((uint16_t)yr, mon, (uint8_t)day,
                    (uint8_t)hh, (uint8_t)mm, (uint8_t)ss, week);
    return 1;
}

/* 返回 1 = 校时成功。最多试 3 轮：SNTP 刚打开时第一次查询往往是空的，
   要等 1~2 秒模块跟服务器对上时间。 */
static uint8_t ESP_SyncTime(void)
{
    char line[96];
    uint16_t n;
    uint8_t ch, tries;

    ESP_SendCmd_OK("AT+CIPSNTPCFG=1,8,\"ntp.aliyun.com\"\r\n", 2000);

    for (tries = 0; tries < 3; tries++)
    {
        uint32_t start;

        RingBuf_Clear();
        HAL_UART_Transmit(&huart1, (uint8_t *)"AT+CIPSNTPTIME?\r\n", 17, 1000);

        n = 0;
        start = HAL_GetTick();
        while ((HAL_GetTick() - start) < 2000)
        {
            while (RingBuf_Read(&ch))
            {
                if (ch == '\n')
                {
                    line[n] = '\0';
                    if (strstr(line, "+CIPSNTPTIME:") != NULL && SntpApply(line))
                        return 1;
                    n = 0;
                }
                else if (ch != '\r')
                {
                    if (n < sizeof(line) - 1) line[n++] = ch;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(400));
    }
    return 0;
}

/* ================= 软重连（不复位模块）=================
   只做 MQTT 层：清会话 → 重下 USERCFG → 连接 → 订阅。
   返回 1 = 成功（g_mqtt_connected 已置 1），0 = 失败。 */
static uint8_t MQTT_SoftConn(void)
{
    char cmd[128];

    /* 先对时：此刻 WiFi 已连、还没连 MQTT、没有数据在发，是最干净的空窗期。
       失败不阻塞联网，下一轮再试。 */
    if (!g_time_valid)
        ESP_SyncTime();

    /* MQTTCLEAN 会关闭 TCP 连接并清掉用户配置：
       不清就发 USERCFG 会回 ERROR；清了不重下 USERCFG，CONN 必然失败。
       这一步同时让模块给 broker 发出正常的 FIN —— 会话立刻释放，
       不会像硬复位那样留下僵尸会话。 */
    ESP_SendCmd_OK("AT+MQTTCLEAN=0\r\n", 3000);
    vTaskDelay(pdMS_TO_TICKS(200));

    if (ESP_MQTT_UserCfg() != ESP_OK)
    {
        Log_Write(LOG_ERROR, "USERCFG reconn fail");
        return 0;
    }
    if (ESP_MQTT_Connect() != ESP_OK)
    {
        Log_Write(LOG_ERROR, "CONN reconn fail");
        return 0;
    }

    snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=0,\"%s\",1\r\n", BEMFA_TOPIC);
    if (ESP_SendCmd(cmd, "OK", 5000) != ESP_OK)
    {
        Log_Write(LOG_ERROR, "SUB reconn fail");
        return 0;
    }

    g_mqtt_connected = 1;
    connect = 1;
    {
        extern volatile uint8_t g_pub_fail_cnt;
        g_pub_fail_cnt = 0;      /* 新连接从 0 开始，否则复用上一轮的 >=5 会立刻又判离线 */
    }
    return 1;
}

/* ================= 重连（分级）=================
   ⚠ 2026-11-19 根因修复：这里原来是无条件 ESP_HardReset()。

   为什么这是根因：
     模块被硬复位时，它的 TCP 不会发 FIN。broker 那侧的会话就成了"僵尸"，
     要等 keepalive 超时（我们设的 120s，broker 按 1.5 倍算 180s）才回收。
     而硬复位后 2 秒我们就用**同一个 ClientID**（方式一：私钥即 ClientID）重新
     CONNECT —— 几秒内撞上自己的僵尸会话。巴法云的表现就是：
     先把新连接显示为在线，随后把这条连接关掉；板子收到 +MQTTDISCONNECTED，
     判离线，又硬复位，又连…… 于是"巴法云只在线几秒、不断重连、probe 永远 FAIL"。

   修法：能软重连就绝不复位
     0 级  AT 探活 → WiFi 还在 → 只重连 MQTT（约 2 秒，不复位）
     1 级  模块不回 AT / WiFi 已掉 / 软重连失败 → 硬复位 + 重新配网
   软重连让模块自己正常关闭 TCP，broker 立刻释放会话，僵尸不会产生。 */
/* ================= WiFi 层两个工具函数 =================
   为什么必须单独抽出来：AT+CWJAP? 回 +CWJAP: 只能证明 802.11 关联成立，
   证明不了 DHCP 成功。关联上了但没有 IP，MQTT 一样连不出去，
   而且 AT+MQTTCONN 要白等 15 秒才超时。只有 AT+CIFSR 拿到非 0.0.0.0
   的 STAIP，才算"真的能上网"。 */

/* 最多重试 3 次（每次间隔 500ms）确认拿到真实 IP。
   手机热点 DHCP 偶尔要几秒才下来，一次查不到就判失败会误伤。 */
static uint8_t WiFi_HasIp(void)
{
    uint8_t k;

    for (k = 0; k < 3; k++)
    {
        g_sta_ip_ok = -1;
        g_sta_ip_str[0] = '\0';
        if (ESP_SendCmd_OK("AT+CIFSR\r\n", 2000) == ESP_OK && g_sta_ip_ok == 1)
            return 1;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    return 0;
}

/* 重新配网：保证走一次完整的"关联 + DHCP"。
   ⚠ 不能直接复用 ESP_JoinWiFi()：它第一步就查 AT+CWJAP?，只要还关联着
   就直接返回成功 —— 而"关联着但没有 IP"正是需要重新 DHCP 的情形，
   会被它误判成"已经好了"。所以这里先 CWQAP 断开、再 CWJAP 重建。
   反正此刻本来就没有可用链路，断开没有任何损失。 */
static uint8_t WiFi_Rejoin(void)
{
    char cmd[128];

    ESP_SendCmd_OK("AT+CWQAP\r\n", 3000);
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_SendCmd_OK("AT+CWDHCP=1,1\r\n", 2000);

    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PASS);
    if (ESP_SendCmd(cmd, "WIFI GOT IP", 25000) != ESP_OK)
    {
        /* 已经连着同一个热点时，固件可能只回 OK、不再打印 WIFI GOT IP，再查一次兜底 */
        if (ESP_SendCmd("AT+CWJAP?\r\n", WIFI_SSID, 2000) != ESP_OK)
            return 0;
    }

    return WiFi_HasIp();     /* 关联成功还不够，必须真的有 IP */
}

/* ================= 重连（分级）=================
   ⚠ 2026-11-19 根因修复：这里原来是无条件 ESP_HardReset()。

   为什么这是根因：
     模块被硬复位时，它的 TCP 不会发 FIN。broker 那侧的会话就成了"僵尸"，
     要等 keepalive 超时（我们设的 120s，broker 按 1.5 倍算 180s）才回收。
     而硬复位后 2 秒我们就用**同一个 ClientID**（方式一：私钥即 ClientID）重新
     CONNECT —— 几秒内撞上自己的僵尸会话。巴法云的表现就是：
     先把新连接显示为在线，随后把这条连接关掉；板子收到 +MQTTDISCONNECTED，
     判离线，又硬复位，又连…… 于是"巴法云只在线几秒、不断重连、probe 永远 FAIL"。

   修法：能不复位就绝不复位。模块还在，就让它自己正常关闭旧 TCP（发 FIN），
   broker 立刻释放会话，僵尸不会产生。

   0   级  AT 活着 + WiFi 关联着 + 真有 IP  → 只重连 MQTT（约 2 秒）
   0.5 级  AT 活着 + WiFi 掉了 / 没 IP     → 重新配网（CWQAP→CWJAP），仍不复位
   1   级  AT 不回应 / 连续 2 次软路径失败 → 硬复位 + 重新配网（最后手段）

   为什么要"连续 2 次失败就升级"：
     模块"半死"时（状态机残留、卡在旧连接上）软路径会永远打转，2 秒一次、
     永远恢复不了。连续失败说明残留状态已经不可信，必须硬清一次。 */

#define SOFT_FAIL_TO_HARD   2
static uint8_t s_soft_fail = 0;      /* 软路径连续失败次数 */

void RE_MQTT_Init(void)
{
    uint8_t at_alive = 0;
    uint8_t i;

    /* ---- 先探活（最多约 2.1 秒）。模块连 AT 都不回，只有硬复位能救 ---- */
    for (i = 0; i < 3; i++)
    {
        if (ESP_SendCmd_OK("AT\r\n", 500) == ESP_OK) { at_alive = 1; break; }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    if (at_alive && s_soft_fail < SOFT_FAIL_TO_HARD)
    {
        RingBuf_Clear();

        /* 关联状态：1 关联着 / 0 No AP / -1 查不到（老固件，按"可能掉了"处理） */
        g_wifi_joined = -1;
        ESP_SendCmd_OK("AT+CWJAP?\r\n", 2000);

        if (g_wifi_joined == 1 && WiFi_HasIp())
        {
            /* ---- 0 级：WiFi 是好的，只是 MQTT 那条链路断了 ---- */
            if (MQTT_SoftConn())
            {
                s_soft_fail = 0;
                Log_Write(LOG_INFO, "reconnect ok(soft)");
                MQTT_DiagAfterConn();
                return;
            }
            Log_Write(LOG_WARN, "soft reconn fail");
            s_soft_fail++;
        }
        else
        {
            /* ---- 0.5 级：模块活着，但 WiFi 掉了或没拿到 IP ----
               关热点再开热点走的就是这一条。不硬复位，只重新配网。 */
            if (WiFi_Rejoin() && MQTT_SoftConn())
            {
                s_soft_fail = 0;
                Log_Write(LOG_INFO, "reconnect ok(wifi)");
                MQTT_DiagAfterConn();
                return;
            }
            Log_Write(LOG_WARN, "wifi reconn fail");
            s_soft_fail++;
        }
    }
    else if (at_alive)
    {
        /* 软路径连续失败已到上限 —— 直接硬清，不再在软路径上打转 */
        Log_Write(LOG_WARN, "soft x2 -> hard reset");
        s_soft_fail = 0;
    }

    /* ---- 1 级：硬复位 + 重新配网（最后手段）---- */
    s_soft_fail = 0;          /* 硬清之后残留状态归零，下一轮重新从 0 级开始试 */
    ESP_HardReset();
    vTaskDelay(1500);         /* 先等启动日志（74880bps）打完，避免混进 AT 应答 */
    RingBuf_Clear();

    /* 复位期间 DMA 接收可能已被启动日志的帧错误打断，这里显式重新武装一次。 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, a, sizeof(a));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    /* 轮询到模块真正回 OK 为止（最多约 5 秒），确认它活了再配置。 */
    {
        uint8_t at_ready = 0;
        for (i = 0; i < 10; i++)
        {
            if (ESP_SendCmd_OK("AT\r\n", 500) == ESP_OK) { at_ready = 1; break; }
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        if (!at_ready)
        {
            Log_Write(LOG_ERROR, "ESP no response after reset");
            DiagBlink(11);
            return;                      /* 模块没起来，10 秒后再试 */
        }
        RingBuf_Clear();
    }

    ESP_SendCmd_OK("AT+CWMODE=1\r\n", 2000);
    ESP_SendCmd_OK("AT+CWAUTOCONN=0\r\n", 2000);

    if (ESP_JoinWiFi() != ESP_OK || !WiFi_HasIp())
    {
        Log_Write(LOG_ERROR, "WiFi rejoin failed");
        DiagBlink(11);
        return;
    }

    if (!MQTT_SoftConn())
    {
        Log_Write(LOG_WARN, "reconnect fail");
        g_mqtt_connected = 0;
        connect = 0;
        return;
    }

    Log_Write(LOG_INFO, "reconnect success");
    MQTT_DiagAfterConn();
}

// ==================== MQTT 心跳包 ====================
/* 2026-11-19 修复"关掉热点仍显示在线"：
   原来发的是裸 "AT\r\n" —— 只测得到"串口活着"，ESP 模块 WiFi/MQTT 全断了
   它照样回 OK，于是心跳永远成功、g_mqtt_connected 永远不清零：
   PA15 常亮、屏幕显示在线，板子根本不知道自己掉线（TCP 半开）。

   现在两级判据，任一为负即判离线：
     ① AT+MQTTCONN? —— 固件内部 MQTT 状态机（官方定义）
          0 未初始化 / 1 已设 USERCFG / 2 已设 CONNCFG / 3 已断开
          4 已建立 / 5 已连未订阅 / 6 已连已订阅（我们订阅过，在线为 6）
     ② AT+CWJAP?   —— WiFi 关联状态
          回 "No AP" = 热点已经没了。
   为什么要 ②：热点一关，TCP 常进入半开状态，ESP 的 MQTT 状态机要等
   keepalive 超时（可达一到两分钟）才转 3，这段时间 ① 仍报"已连接"；
   而 WiFi 层几乎立刻就知道链路断了 —— ② 才是最快、最硬的信号。

   兼容性兜底：若查询命令本身不被老固件支持（返回 ERROR/超时），
   退回裸 AT，保持原来"6 次连续失败才断线"的保守语义，绝不因查不到就误判。 */
ESP_Status_t MQTT_SendPing(void)
{
    /* 2026-11-19 二次回退：心跳只做"模块是否还活着"这一件事。
       中间试过每 10 秒发 AT+CWJAP? / AT+MQTTCONN? 主动查状态，
       实测刚连上几秒就掉线、且再也回不来 —— 这些查询会插进模块正在
       处理的 MQTT 收发包中间，把链路自己弄断，得不偿失。

       断线检测改为纯被动、不碰数据通道的两条路：
         ① +MQTTDISCONNECTED 主动上报（ESP_SendCmd 里已解析，立刻置 0）
         ② 业务发布连续失败次数（AT 任务里判，约 30 秒确认）
       热点关闭时，发布必然连续失败，30 秒内能识别；
       而正常的 MQTT 流量不会被任何查询命令打扰。 */
    return ESP_SendCmd_OK("AT\r\n", 1000);
}

/**
 * @brief  透传模式下从 ESP8266 接收原始 TCP 数据
 * @param  sock       未使用（保留参数，兼容之前的接口设计）
 * @param  buf        接收缓冲区指针
 * @param  len        期望接收的字节数
 * @param  timeout_ms 超时时间（毫秒）
 * @return 实际接收到的字节数，超时或失败返回 -1
 */


/* MQTT 数据流中可能夹着 SUBACK 或平台下发消息；不能假定前两个字节就是 PINGRESP。 */

// 在 mqtt.c 中新增
//void MQTT_SendTempHumi_NoWait(float temp, float humi,float light)
//{
//    char payload[128];
//    sprintf(payload, "{\"temperature\":%.1f,\"shidu\":%.1f,\"light\":%.0f}", temp, humi,light);

//    uint8_t buf[128];
//    int pos = 0;
//    buf[pos++] = 0x32;		//固定报头 报文类型

//		/*可变报头 主题长度：2 主题名称：5 报文标识符：2 有效载荷：*/
//    uint16_t topic_len = 5;
//    uint16_t msg_len = strlen(payload);
//    uint16_t rem = 2 + topic_len + 2 + msg_len;

//		/*变长编码计算剩余长度 小端排序（低位数据放在低地址） 低位在前  (一般低地址在前)*/
//    do {
//        uint8_t b = rem % 128;
//        rem /= 128;
//        if (rem > 0) b |= 0x80;
//        buf[pos++] = b;
//    } while (rem > 0);

//		/*可变报头  主题长度 0x00 0x05两个字节 高位在前*/
//    buf[pos++] = 0;
//    buf[pos++] = topic_len;
//    memcpy(&buf[pos], "stm32", 5);
//    pos += 5;

//		/*报文标识符 0x00 0x01*/
//    buf[pos++] = 0;
//    buf[pos++] = 1;

//		/*有效载荷*/
//    memcpy(&buf[pos], payload, msg_len);
//    pos += msg_len;

//    HAL_UART_Transmit(&huart1, buf, pos, 5000);
//    vTaskDelay(100);
//    // 不等待 PUBACK，直接返回
//}


