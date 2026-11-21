#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "lcd.h"
#include "touch.h"
#include "stdio.h"
#include <stdlib.h>       /* atoi：解析云端指令里的 n */
#include "string.h"
#include "lvgl.h"               
#include "lv_port_disp.h"       
#include "lv_port_indev.h"      
#include "page_main.h"
#include "modbus.h"
#include "command.h"
#include "ESP_at.h"
#include "libemqtt.h"
#include "pal.h"
#include "flash.h"
#include "rtc.h"          /* 离线缓存要带时间戳 */
#include "Taskhandel.h"
#include "bsp_flash.h"
#include "flash.h"


// 空闲熄屏相关
/* 屏幕行为开关（跟屏幕好坏无关，屏一修好就生效）：
     LCD_IDLE_OFF_MS  无操作多久自动熄屏（毫秒）。设为 0 = 永不熄屏。
     LCD_WAKE_ON_DATA 收到新数据时是否自动点亮背光。
                      1 = 有数据就一直亮（演示/调试推荐，默认）
                      0 = 只有触摸才点亮（省电，但看着像"界面不动了"） */
#define LCD_IDLE_OFF_MS   10000u
#define LCD_WAKE_ON_DATA  1

TickType_t idle_tick;
uint8_t lcd_bl_en = 1; //1亮屏 0熄屏
uint8_t power_on_lock = 1;
TickType_t power_tick;
lv_obj_t *main_scr;

 uint8_t fail_count = 0; // 记录连续失败次数
 uint8_t count;
QueueHandle_t     data_queue;
SemaphoreHandle_t uart1_mutex;

QueueHandle_t lvgl_data_queue;   
uint32_t last_ping = 0;

extern IWDG_HandleTypeDef hiwdg;
 volatile uint8_t g_mqtt_connected;
volatile uint8_t connect;

/* MQTT 连续发布失败计数（modbus.c / flash.c 里维护）。
   用途：AT 命令只能证明 ESP8266 模块活着，证明不了 MQTT 链路活着 ——
   被同 ClientID 的网页控制台/MQTTX 踢下线后，AT 照样回 OK。
   唯一可靠的在线判据是"发布能不能成功"。 */
volatile uint8_t g_pub_fail_cnt = 0;

/* 诊断计数：随 probe 一起上报，用来区分"任务没走到发送分支"和"发不出去"。
   tx 不涨 → MQTT 任务根本没进发送路径（g_mqtt_connected=0 或队列一直空）
   tx 涨而 pf 也涨 → 进了发送路径但 broker 不收
   lk 涨 → 卡在串口锁上 */
volatile uint32_t g_tx_cnt    = 0;   /* 进入实时发送分支的次数 */
volatile uint32_t g_lock_fail = 0;   /* MQTT 任务取串口锁失败次数 */

/* 任务创建失败位图（freertos.c 里置位，随 probe 上报）。
   xTaskCreate 在堆不足时返回 pdFAIL，原代码不检查 —— 任务就这么"消失"了，
   表现为某个功能整体不工作却完全没有报错。
   bit0=UI bit1=Modbus bit2=AT bit3=MQTT bit4=Dog */
volatile uint8_t g_task_fail = 0;

// ==============================================
// Modbus 采集
// ==============================================
//void Modbus(void)
//{
//    Data_t data;				//结构体变量

//    while(1)
//    {
//        Modbus_Get_Temp(&data.temp, &data.shi);		//采集温湿度            
//				xQueueOverwrite(data_queue, &data);					// 永远只保留最新一次采集结果，旧值被直接覆盖 该队列长度为1
//        vTaskDelay(500);
//    }
//}

void Modbus(void *argument)
{
    (void)argument;	/* FreeRTOS 传入参数本任务不用 */

    /* 上电等系统稳定后再探测光照（结果存全局变量，不写 Flash） */
    vTaskDelay(pdMS_TO_TICKS(5000));
    Modbus_ProbeLight();

    while(1)
    {
			
        Modbus_Poll_AllDev();					//读到的数据赋值给结构体数组
        
        Modbus_Upload_Json();					//将采集到的数据赋值给结构体 然后写入队列
				
        vTaskDelay(pdMS_TO_TICKS(800)); //秒一轮轮询所有设备（时间短点测数据就更灵敏）
    }
}

/*补传绿灯亮 正常红灯亮 离线存储黄灯亮*/
/*
正常在线：先补传历史数据，再发送实时数据，通知 UI 更新。

断网期间：把数据存入 SPI Flash，等待恢复。

重连后：自动补传缓存数据（每次最多 5 条，分批补传）。

从数组里获取数据保证数据是最新的 
*/
/* ================= 云端按需拉取历史（2026-11-21，第三层）=================
   流程：MQTTX 往订阅主题发 {"cmd":"history","n":20}
        → ESP8266 吐 +MQTTSUBRECV → ESP_UrcPoll() 收进 g_cmd_payload
        → 这里解析，按 CACHE_HIST_PAGE 条一页回传。
   为什么放在"发布窗口"里发：实时数据永远先发、按 6 秒节奏不变；
   历史页只吃这个窗口剩余的空隙，不占用实时的发送时隙。 */
static uint8_t  s_hist_active = 0;
static uint16_t s_hist_skip   = 0;    /* 已跳过最新多少条（翻页用） */
static uint16_t s_hist_total  = 0;    /* 本次要发多少条 */
static uint16_t s_hist_sent   = 0;

static void CloudCmd_Handle(void)
{
    const char *p;
    int n;

    if (!g_cmd_pending)
        return;
    g_cmd_pending = 0;                 /* 只有一条槽，取走就清，避免重复触发 */

    /* 宽松匹配：允许 {"cmd": "history"} 这种带空格的写法。
       先确认是"指令"（有 cmd 字段），再看是哪一条 —— 板子自己的数据回灌
       没有 cmd 字段，会在第一步就被挡掉。 */
    if (strstr(g_cmd_payload, "\"cmd\"") == NULL)
        return;

    /* 清空缓存区：把旧固件写的、没有时间戳的历史条目清掉。
       只擦首扇区（约 400ms），不会卡住网络。 */
    if (strstr(g_cmd_payload, "clearcache") != NULL)
    {
        FlashCache_Reset();
        Log_Write(LOG_INFO, "cache cleared");
        return;
    }

    if (strstr(g_cmd_payload, "history") == NULL)
        return;

    n = 30;
    p = strstr(g_cmd_payload, "\"n\":");
    if (p != NULL) n = atoi(p + 4);
    if (n < 1)  n = 1;
    if (n > 30) n = 30;                /* 上限 30 条，5 页，别把配额一次烧光 */

    s_hist_total  = (uint16_t)n;
    s_hist_sent   = 0;
    s_hist_skip   = 0;
    s_hist_active = 1;
    Log_Write(LOG_INFO, "hist query");
}

static void CloudCmd_SendPage(void)
{
    static char buf[1000];             /* 放静态区：MQTT 任务栈只有 4KB */
    static uint32_t last_tick = 0;
    static uint8_t  fail_cnt  = 0;
    uint8_t  got  = 0;
    uint16_t left;

    if (!s_hist_active)
        return;

    /* 自己管节拍：不管是"有采集数据"的正常路径，还是"传感器故障、没有新数据"
       的路径调进来，都保证最多 6 秒一页，不会把配额打爆。 */
    {
        uint32_t now = HAL_GetTick();
        if ((now - last_tick) < (uint32_t)PUB_INTERVAL_MS)
            return;
        last_tick = now;
    }

    left = (uint16_t)(s_hist_total - s_hist_sent);
    if (left == 0) { s_hist_active = 0; return; }
    if (left > CACHE_HIST_PAGE) left = CACHE_HIST_PAGE;

    if (FlashCache_PeekHistory(buf, sizeof(buf), s_hist_skip, (uint8_t)left, &got) != 0 || got == 0)
    {
        s_hist_active = 0;
        Log_Write(LOG_INFO, "hist none");     /* 没有更多历史了 */
        return;
    }

    if (xSemaphoreTake(uart1_mutex, pdMS_TO_TICKS(3000)) != pdTRUE)
        return;                        /* 拿不到锁：这一页留到下个窗口，状态不变 */

    if (MQTT_Publish(buf, 8000) == ESP_OK)
    {
        fail_cnt = 0;
        s_hist_skip += got;
        s_hist_sent += got;
        if (s_hist_sent >= s_hist_total)
        {
            s_hist_active = 0;
            Log_Write(LOG_INFO, "hist done");
        }
    }
    else
    {
        /* 失败：skip/sent 都不动，下个窗口重发同一页。
           但连续失败 3 次就放弃本次查询 —— 否则一旦 PUBRAW 不可用，
           会每个窗口白占 3 秒串口直到天荒地老。 */
        if (++fail_cnt >= 3)
        {
            s_hist_active = 0;
            fail_cnt = 0;
            Log_Write(LOG_WARN, "hist fail");
        }
    }
    xSemaphoreGive(uart1_mutex);
}

void MQTT(void *argument)
{
    (void)argument;	/* FreeRTOS 传入参数本任务不用 */
    vTaskDelay(5000);
    Data_t data_q;
    while (1)
    {
				 
		//	xSemaphoreTake(uart1_mutex, portMAX_DELAY);				//获取锁
			
			/* 原来这里是 portMAX_DELAY：队列里没有新数据就永远醒不来，
			   PA15 也就永远不更新 —— 一旦 Modbus 那边没喂队列，在线灯就一动不动，
			   看着像"没连上"，其实可能连接好好的。这是上一轮 PA15 误判的同一类问题。
			   现在改成 1 秒超时：有数据就发，没数据也要每秒刷新一次在线灯。 */
			BaseType_t got_data = xQueueReceive(data_queue, &data_q, pdMS_TO_TICKS(1000));

            /* PA15 = 「MQTT 在线」指示灯（SET=亮，RESET=灭）。
               放在取队列之后、且不受 got_data 影响：只要本任务还活着，
               它每秒都会跟着 g_mqtt_connected 走，反映的是"连接状态"而不是"有没有新数据"。 */
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,
                              g_mqtt_connected ? GPIO_PIN_SET : GPIO_PIN_RESET);

            /* URC 收割（含云端指令）：1 秒一次，取不到锁就直接跳过 ——
               绝不在这里阻塞，发布节奏优先。 */
            if (xSemaphoreTake(uart1_mutex, 0) == pdTRUE)
            {
                ESP_UrcSniff();
                /* 解析也放在锁内：g_cmd_payload 由 ESP_UrcPoll 写，
                   AT 任务同样会调它；在锁内解析才不会读到被覆盖一半的内容。 */
                CloudCmd_Handle();
                xSemaphoreGive(uart1_mutex);
            }

            if (got_data != pdPASS)
            {
                /* 这一秒没有新采集数据：不发布实时，但如果云端正在查历史，
                   仍然要回一页 —— 否则传感器故障时查询永远没响应。
                   节拍由 CloudCmd_SendPage 自己把关（6 秒一页）。 */
                if (g_mqtt_connected && s_hist_active)
                    CloudCmd_SendPage();
                continue;
            }

            if (g_mqtt_connected)			//在线
            {
/* ===== 断网补传（2026-11-21 重新启用）=====
   原实现有三个问题，所以一直关着：
     ① 每轮采集（约 1.6 秒）就补 5 条，实际约 190 条/分钟，
        直接撑爆巴法云免费版的推送频率，云端会静默丢包；
     ② 发布失败也照样 FlashCache_MarkSent()，数据被标成"已发送"永久丢失；
     ③ 补完调 my_clear() 把读指针拉回起点，绕过了擦除逻辑。
   现在改为：补传与实时【交替】，每个 6 秒发布窗口只用一次 ——
   要么补一条旧数据，要么发一条实时数据。总速率始终 10 条/分钟，不会超限；
   失败不标记，下一轮重试。代价是补传速度约 6 秒/条（≈500 条/小时），
   覆盖普通断网时长足够。 */
static uint32_t last_pub_tick = 0;
{
    uint32_t now_tick = HAL_GetTick();
    if ((now_tick - last_pub_tick) >= (uint32_t)PUB_INTERVAL_MS)
    {
        last_pub_tick = now_tick;

        uint8_t served = 0;          /* 本窗口是否已经发过东西 */

#if CACHE_REPLAY_ENABLE
        /* replay_turn 每次窗口翻转：1 = 这轮优先补传 */
        static uint8_t replay_turn = 1;
        if (replay_turn && FlashCache_HasData())
        {
            char batch[448];
            uint8_t got = 0;

            /* 先打包（只读不消耗）。这样即使下一句发布失败，也一条数据都不会丢 ——
               读指针和条目魔数都没动过，下一轮原样重试。 */
            if (FlashCache_PeekBatch(batch, sizeof(batch), CACHE_REPLAY_BATCH, &got) == 0 && got > 0)
            {
                if (xSemaphoreTake(uart1_mutex, pdMS_TO_TICKS(3000)) == pdTRUE)
                {
                    /* broker 确认收到（+MQTTPUB:OK）之后，才标记已发送并推进读指针 */
                    if (MQTT_Publish(batch, 8000) == ESP_OK)
                    {
                        FlashCache_CommitBatch(got);
                        if (!FlashCache_HasData())
                            Log_Write(LOG_INFO, "replay done");
                    }
                    xSemaphoreGive(uart1_mutex);
                }
                else
                {
                    g_lock_fail++;
                }
                served = 1;
            }
            /* 打包不出东西（没有有效条目）→ served 保持 0，这个窗口让给实时数据 */
            replay_turn = 0;
        }
        else
        {
            replay_turn = 1;
        }
#endif
        if (!served)
        {
            if (xSemaphoreTake(uart1_mutex, pdMS_TO_TICKS(3000)) == pdTRUE)
            {
                g_tx_cnt++;                 /* 诊断：确实进入了发送路径 */
                MQTT_SendAllDev_NoWait();
                xSemaphoreGive(uart1_mutex);					//释放锁
            }
            else
            {
                g_lock_fail++;              /* 诊断：实时发送取锁失败 */
            }
        }

        /* 实时数据发完，同一个窗口再补一页历史（云端查询的结果）。
           只在"这个窗口确实发了实时数据"时补，避免和补传抢同一个时隙。 */
        if (!served && s_hist_active)
            CloudCmd_SendPage();
    }
    else
    {
        xQueueOverwrite(lvgl_data_queue, &data_q);  /* 只刷屏，不占云端配额 */
    }
}
                /* 注意：PB4 现在归 Dog 任务做"系统心跳"（每秒翻转），
                   这里不能再写它，否则心跳图案会被打乱。 */

                /* PB3 语义（低电平点亮，必须自洽，不能再出现"灯灭=没连上"那种误判）：
                     成功发出一条 → 亮 60ms 后灭（看到它在闪 = 数据在流）
                     连续发布失败 → 常亮（有在发但云端收不到）
                   再配合 PA15 常亮 = MQTT 在线，两个灯就能把故障定到"链路"还是"发布"。 */
                if (g_pub_fail_cnt == 0)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
                    vTaskDelay(pdMS_TO_TICKS(60));
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
                }
                else
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
                }
//                xQueueOverwrite(lvgl_data_queue, &data_q);			// 写队列 永远只保留最新一次采集结果，旧值被直接覆盖 该队列长度为1
            }
            else	//断线中
            {
                /* 离线缓存按 10 秒一条写：Modbus 每 800ms 采一次，全写进去就是
                   1.25 条/秒，4MB 缓存约 7 小时写满，而且补传根本追不上。
                   每条带上采集时刻，恢复后云端能看出这是补的历史数据。 */
                static uint32_t last_cache_tick = 0;
                uint32_t now_cache = HAL_GetTick();
                if ((now_cache - last_cache_tick) >= 10000u)
                {
                    char json[192];
                    char ts[24];
                    int n;

                    last_cache_tick = now_cache;
                    if (RTC_GetDateTimeStr(ts, sizeof(ts)) == 0)
                        ts[0] = '\0';      /* 未校时就不写，不谎报 2000 年 */

                    n = snprintf(json, sizeof(json),
                                 "{\"temperature\":%.1f,\"shidu\":%.1f,\"light\":%.0f,\"off\":1,\"ts\":\"%s\"}",
                                 data_q.temp, data_q.shi, data_q.light, ts);
                    if (n > 0 && n < (int)sizeof(json))
                        FlashCache_Write(json, (uint16_t)n);
                }
                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);

                /* 断网时也要刷新屏幕，否则 UI 停在最后一次在线值 */
                xQueueOverwrite(lvgl_data_queue, &data_q);
            }
						
           
						
        }
        /* 原 vTaskDelay(2000) 写在了 for 之外（上一行 //} 把循环右括号注释掉了），
           是死代码；本任务节奏由开头的 xQueueReceive(portMAX_DELAY) 控制，无需再延时 */
    //}
}


/*
每10秒发心跳检测 然后读心跳应答 看是否正常
连续3次心跳应答错误或失败则判断离线
离线后 每10秒执行重连函数直到连接成功
*/
void AT(void *argument)
{
    (void)argument;	/* FreeRTOS 传入参数本任务不用 */
	MQTT_Init();							//连接成功后 g_mqtt_connected为1 否则为0

    uint8_t heart_fail = 0;  // 心跳连续失败次数
    uint8_t tele_div   = 0;  // 遥测节拍：每 3 次心跳（30 秒）上报一次
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));  // 每 10 秒检测一次心跳

        /* 注意：这里不再根据 g_pub_fail_cnt 判定掉线。
           发布失败≠链路断开（固件忙时会回 busy p... 直接拒掉命令，连接本身是好的），
           而重连的第一步是 ESP_HardReset()，一复位 broker 那侧就真掉线了 ——
           用不可靠的信号触发最暴力的动作，结果就是板子自己把自己踢下线
           （巴法云订阅者数从 2 变 1）。真正的断线交给 +MQTTDISCONNECTED 上报。 */

        /* 带超时取锁：拿不到就跳过本轮。
           原 portMAX_DELAY 会让本任务无限期卡在锁上，与 MQTT 发布任务互相拖死。 */
        if (xSemaphoreTake(uart1_mutex, pdMS_TO_TICKS(3000)) != pdTRUE)
            continue;

				/* 取证（不改行为）：趁还没清缓冲区，先把积压的 URC 扫一遍。
				   +MQTTDISCONNECTED 只有恰好落在某次 AT 命令读取期间才会被解析，
				   其余时间会被下面这行 RingBuf_Clear() 丢掉 —— 那 d= 就会低估，
				   而"d 是不是 0"正是判断"链路被断"还是"发不出去"的关键。
				   本函数只累加 g_disc_cnt，不动 g_mqtt_connected。 */
				ESP_UrcSniff();

				RingBuf_Clear();														//关键点 先清空缓冲区 否则有脏数据影响判断
        if (g_mqtt_connected)
        {
            // 链路存活探测（AT 固件内置心跳，用 AT 命令的应答结果判定）
            if (MQTT_SendPing() == ESP_OK)
            {
                // 心跳正常
                heart_fail = 0;
							connect=0;
						
            }
            else							// 心跳超时或数据错误
            {
                /* 修复：原来是 if(connect==0) 才计数，但重连成功会把 connect 置 1，
                   之后没有任何地方再把它清 0 —— 一次重连之后，后续心跳失败永远不计数，
                   AT 任务再也触发不了第二次重连。
                   现在：connect==1 表示"刚重连完，给一次宽限"，
                   这一次失败只清标志不计数；从下一次起正常累加。既保住宽限，又不会卡死。 */
                if (connect == 0)
                {
                    heart_fail++;
                }
                else
                {
                    connect = 0;      /* 宽限用掉 */
                }
            }

            /* ===== 断线判定（2026-11-19 最终版，纯被动、不干扰数据通道）=====
               心跳发的是裸 AT，只证明串口活着——模块断网后它照样回 OK，
               所以"心跳成功"不等于"链路通"。真正能反映链路的是业务发布：
                 连续 5 次投递失败（6 秒/次 ≈ 30 秒）→ 云端一条都收不到 → 判离线。
               为什么不主动查 AT+CWJAP?/AT+MQTTCONN?：
               实测这类查询会插进模块正在处理的 MQTT 收发包中间，把本来好的
               连接自己弄断（连上几秒就掉、还回不来）。宁可慢 30 秒，也不能
               为了检测断线而把在线弄断。
               另外 +MQTTDISCONNECTED 主动上报仍会在 ESP_at.c 里立刻置 0。 */
            if (g_pub_fail_cnt >= 5)
            {
                heart_fail = 6;
            }

            if (heart_fail >= 6)  // 心跳连续失败 或 发布连续失败 → 断线
            {
                g_mqtt_connected = 0;					//断网
									Log_Write(LOG_WARN, "MQTT disconnect");
									HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
									HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
                heart_fail = 0;
            }
        }
				else
				{				//重连
                /* 必须先放锁再重连：RE_MQTT_Init() 内部会做硬复位 + 重新配网 +
                   重连 + 发 probe，而 MQTT_SendProbe() 自己会取 uart1_mutex。
                   持着锁调进去 = 同一个任务二次取同一把非递归锁 = 死锁，
                   AT 任务永久卡死，而 MQTT 任务还在等这把锁 → 整个网络层停摆。 */
                xSemaphoreGive(uart1_mutex);
                RE_MQTT_Init();
                continue;                   /* 锁已经交出去了，跳过下面的释放 */
				}
        xSemaphoreGive(uart1_mutex);					//释放锁

        /* ===== 运行期遥测：每 60 秒上报一次内部状态 =====
           （原来 30 秒一条；巴法云免费版有推送频率限制，把配额尽量让给业务数据，
             遥测够用就行——它的作用是"板子还活着、卡在哪一步"，不需要高频。）
           屏幕已经坏了，这条 probe 是唯一能"看见"板子内部状态的通道
           —— 开机那条 probe 走的就是同一条链路，已被实测证明能到达巴法云。
           有了 u/p/l 三个计数就能定位卡点：
             u 不涨        → MQTT 任务没走到发布分支（标志为 0 或队列空）
             u 涨、p 涨    → 走到了发布，但 AT+MQTTPUB 被固件拒（busy p / ERR CODE）
             u 涨、p 不涨  → 发布成功却看不到 → 问题在 broker 侧（主题/互踢/限流）
             l 涨          → 卡在串口锁上，两个任务在互相抢
           单独开一段持锁窗口，避免把心跳的持锁时间从 2 秒拉长到 7 秒。 */
        if (g_mqtt_connected && ++tele_div >= 6)
        {
            tele_div = 0;
            if (xSemaphoreTake(uart1_mutex, pdMS_TO_TICKS(1000)) == pdTRUE)
            {
                RingBuf_Clear();
                MQTT_SendTelemetry();
                xSemaphoreGive(uart1_mutex);
            }
        }
    }
}


// ==============================================
// UI 显示
// ==============================================
void UI(void *argument)
{
    (void)argument;	/* FreeRTOS 传入参数本任务不用 */
    Data_t disp_data;

    /* 2026-11-19 停用：开机擦日志会连续占用 CPU 数秒（最坏 12.8 秒），
       UI 任务优先级高于 AT/MQTT，擦除期间网络任务被饿死，
       表现为"刚连上几秒就掉线 + WiFi rejoin failed"。
       历史日志暂不清理（只影响日志页观感，不影响功能）；
       要清日志请改在日志页手动触发，不要在开机路径里做。
       函数本身保留：Log_EraseRecent()。
    Log_EraseRecent();
    */

    LCD_Init();
    TP_Init();
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();

    main_scr = page_main_create();		//屏幕界面
    lv_scr_load(main_scr);				//加载界面

    uint32_t start_tick = xTaskGetTickCount();
    
    uint8_t  last_status = 0xFF;      // 记录上一次的 MQTT 状态
    uint32_t last_uptime  = 0xFFFFFFFF; // 记录上一次的运行时间    魔法数
	
	while (1)				//任务函数 不会退出该函数 故局部变量不会被清空
    {
		
				// 空闲10s无动作 → 息屏
        if(LCD_IDLE_OFF_MS > 0 && lcd_bl_en == 1 && (xTaskGetTickCount() - idle_tick >= pdMS_TO_TICKS(LCD_IDLE_OFF_MS)))
        {
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
            lcd_bl_en = 0;
        }

        /* 数据变化 → 更新画面（并且按 LCD_WAKE_ON_DATA 决定要不要保持亮屏）。
           原代码这里只有一句 page_main_update_data()，注释却写着"亮屏 + 重置计时"，
           那两行其实从来没写 —— 结果是开机亮 10 秒后自动熄屏，之后除非去摸触摸屏，
           否则屏幕一直黑着，看起来就像"界面死了、数据不刷新"。
           LCD_WAKE_ON_DATA = 1：只要有新数据就保持亮（演示/调试推荐，默认）
                            = 0：只有触摸才点亮，数据照样在后台刷新（省电） */
        if (xQueueReceive(lvgl_data_queue, &disp_data, 0) == pdPASS)
        {
            page_main_update_data(disp_data.temp, disp_data.shi,disp_data.light);

#if LCD_WAKE_ON_DATA
            if (lcd_bl_en == 0)
            {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   /* 背光开 */
                lcd_bl_en = 1;
            }
            idle_tick = xTaskGetTickCount();                          /* 重置息屏计时 */
#endif
        }

        // MQTT连接状态变更也刷新亮屏
        if (g_mqtt_connected != last_status)
        {
            last_status = g_mqtt_connected;
						page_main_update_status(g_mqtt_connected);				//更新ui状态显示
					
            idle_tick = xTaskGetTickCount();
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
            lcd_bl_en = 1;
        }

				
				//更新运行时间
        uint32_t uptime = (xTaskGetTickCount() - start_tick) / configTICK_RATE_HZ;			//秒数
        if (uptime != last_uptime)
        {
            last_uptime = uptime;
            page_main_update_uptime(uptime);					//更新ui时间显示
        }
				
        lv_timer_handler();			//5毫秒刷新函数
        vTaskDelay(5);
    }
}


/*看门狗任务 3秒喂一次狗 溢出时间是20秒*/
/* ⚠ PB4 = 系统心跳灯：每秒翻转一次。
   它只依赖"本任务还在跑"，跟 MQTT、485、屏幕全都没关系 —— 屏坏了、网络断了，
   它照跳。所以它回答的是最基本的问题：MCU 到底还活着吗？
     PB4 每秒翻一次  → MCU 活着，问题在网络/串口层
     PB4 完全不动    → MCU 卡死、或正在看门狗复位循环（这时其它灯也都是死的）
   排查"发着发着就没消息了"这类问题时，先看这个灯能省掉一半瞎猜。 */
void Dog_task(void *argument)
{
    (void)argument;	/* FreeRTOS 传入参数本任务不用 */

	while(1)
	{
		HAL_IWDG_Refresh(&hiwdg);			//3秒喂一次狗 溢出时间是20秒     俗称喂狗：将倒计时计数器重置为初始值，阻止看门狗复位。
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);   /* 心跳灯 */
		vTaskDelay(1000);
	}
}
