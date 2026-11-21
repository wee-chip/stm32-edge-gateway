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

    /* 先查是否已连上：模块存过 AP 时会自连，此时再发 CWJAP 只回 OK、不打印
   WIFI GOT IP，等它就白等 15 秒。 */
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

/* 连接参数：AT+MQTTCONNCFG 须在 USERCFG 之后、MQTTCONN 之前下发；
   disable_clean_session 语义相反，0 = 使能清理会话。 */

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

    /* 只有被明确判错（busy/ERROR）才重试。超时不能重试：命令可能已成功，
   再发会收到 ALREADY CONNECTED 的 ERROR。 */
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

/* PUBRAW 阶段的失败码（字符串兜底会覆盖 g_pub_stage，故单独保留一份）。 */
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

    /* 兜底：AT+MQTTPUB 字符串形式，需转义引号。该路径受 esc[300] 与 AT 单命令
   256 字节限制，超长直接判失败，避免被静默截断。 */
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

/* 诊断快照：开机与每次重连后各发一次，云端可见，不写本地 Flash。 */
/* 任务创建失败位图（freertos.c 置位）：1 UI / 2 Modbus / 4 AT / 8 MQTT / 16 Dog。 */
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

/* 开机/重连时发送 probe，带重试与失败闪灯；调用方须持有 uart1_mutex。 */
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

/* 运行期遥测：由 AT 任务周期调用，上报发送侧计数。 */
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

    /* probe 失败只记一行：阶段码 + 断连次数。 */
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

/* 重连分级：
     0   级  WiFi 在且有 IP        → 只重连 MQTT
     0.5 级  WiFi 掉或无 IP        → 重新配网，不复位
     1   级  AT 无响应或软路径连续失败 → 硬复位兜底
   原则：能不复位就不复位。硬复位不发 FIN，broker 侧会话要等 keepalive 超时才回收，
   新连接用同一 ClientID 会撞上自己的残留会话而被踢下线。 */


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
/* 心跳：只探测模块是否存活（裸 AT）。主动状态查询（CWJAP? / MQTTCONN?）会插进
   模块的收发包之间打断链路，禁止在心跳里使用；断线由 +MQTTDISCONNECTED
   和发布失败计数判定。 */

ESP_Status_t MQTT_SendPing(void)
{
    
    return ESP_SendCmd_OK("AT\r\n", 1000);
}
