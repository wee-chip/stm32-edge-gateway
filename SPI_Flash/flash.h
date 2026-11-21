#ifndef FLASH_H
#define FLASH_H


#include <stdint.h>
#include "ESP_at.h"   /* ESP_Status_t：MQTT_PublishJson 返回真实投递结果 */
#include "FreeRTOS.h"
#include "semphr.h"


//分区封装Flash_Part_Write→BufferWrite→PageWrite，分层调用。

// ===================== 1. W25Q64 分区定义（固定不变）=====================
//设计优势：物理分区隔离，一个分区损坏不影响其他功能，符合工业设备设计规范。
/*前4M日志区 后4M补传缓存区*/

/* ===== W25Q64 分区（8MB）=====
   0x000000 ~ 0x0FFFFF   1MB    事件日志（环形，约 8192 条）
   0x100000 ~ 0x7BFFFF   6.75MB 离线缓存（掉线期间每 10 秒一条）
   0x7C0000 ~ 0x7FFFFF   256KB  历史记录（每 10 秒一条，与在线状态无关，约 1.9 天）
   历史记录与离线缓存分开：前者是完整的时间序列，后者只服务断线补传。 */
#define LOG_PART_ADDR       0x000000U
#define LOG_PART_SIZE       (0x100000U-1)								//日志 1MB


#define CACHE_START_ADDR    (1 * 1024 * 1024)					//补传缓冲区
#define CACHE_END_ADDR      (0x7C0000U)
#define CACHE_SECTOR_SIZE   4096
#define CACHE_ENTRY_SIZE    128

/* 一次补传打包几条。
   取 4 而不是 5：单条缓存 JSON 约 85 字节，4 条 + 外壳 ≈ 370 字节，
   留足余量给 ESP-AT 的 MQTTPUBRAW（这条路径没有 256 字节命令长度限制，
   但没必要贴着上限跑）。吞吐 = 4 条 / 12 秒 ≈ 20 条/分钟，
   远高于离线产生速率 6 条/分钟，所以积压能被清空。 */
#define CACHE_REPLAY_BATCH  4

/* 历史查询每页回传几条。6 条 × 约 85 字节 + 外壳 ≈ 530 字节，
   远低于 ESP-AT MQTTPUBRAW 的限制，也留足 UART 传输时间。 */
#define CACHE_HIST_PAGE     10

/* ===== 历史记录区 =====
   每条 16 字节：魔数 + Unix 时间戳 + 温度×10 + 湿度×10 + 光照 + 保留
   256KB / 16 = 16384 条，10 秒一条 ≈ 45 小时 */
#define HIST_START_ADDR     (0x7C0000U)
#define HIST_END_ADDR       (0x800000U)
#define HIST_REC_SIZE       16U

void Hist_Init(void);
void Hist_Write(uint32_t epoch, float temp, float humi, float light);
void Hist_Reset(void);
/* 分页读取：skip 跳过最新几条，max_cnt 最多几条。
   输出 {"off":1,"h":[[epoch,temp,humi,light],...]}，按时间从旧到新。 */
int  Hist_PeekPage(char *out, uint16_t out_size, uint16_t skip, uint8_t max_cnt, uint8_t *got);

#define MAGIC_VALID         0xDEADBEEF
#define MAGIC_EMPTY         0xFFFFFFFF
/* 已发送标记：NOR Flash 只能 1→0，0xFFFFFFFF 写不进去（等于没改），
   必须用一个"从 0xDEADBEEF 单向编程可达"的值，全 0 即可 */
#define MAGIC_SENT          0x00000000

/* 单条日志内容长度：取 116 → Log_Obj = 4+4+116+2 = 126，4 字节对齐后 128，
   正好整除 4096 扇区（每扇区 32 条）。旧值 64 得 76 字节，不整除扇区，
   跨扇区写入不擦除 → 新旧数据按位 AND → 日志乱码。 */
#define LOG_CONTENT_LEN     116U    // 单条日志内容最大长度
#define LOG_MAGIC           0x55AAU // 有效日志魔数

// ===================== 2. 日志等级 & 日志结构体 =====================
typedef enum
{
    LOG_INFO  = 0,   // 正常信息：联网、补传、启动
    LOG_WARN  = 1,   // 警告：断网、重连
    LOG_ERROR = 2    // 错误：Modbus超时、SPI故障
} Log_Level;






typedef struct
{
    uint32_t    time_stamp;    // 时间戳：系统运行滴答数 HAL_GetTick()
    Log_Level   level;         // 日志等级
    uint8_t     content[LOG_CONTENT_LEN]; // 日志文本内容
    uint16_t    magic;         // 魔数：判断这条日志是否有效
} Log_Obj;

#define LOG_OBJ_SIZE        sizeof(Log_Obj)					//76

// ===================== 3. 全局互斥锁（Flash 全局保护）=====================
extern SemaphoreHandle_t flash_mutex;

// ===================== 4. 分区通用读写接口（新增）=====================
uint8_t Flash_Part_Read(uint32_t part_base, uint32_t offset, uint8_t *buf, uint16_t len);
uint8_t Flash_Part_Write(uint32_t part_base, uint32_t offset, uint8_t *buf, uint16_t len);

// ===================== 5. 日志接口（新增）=====================
void Log_Write(Log_Level level, const char *str);								//全工程通用日志写入入口
uint8_t Log_ReadOne(uint32_t log_offset, Log_Obj *log_obj);			//单条日志读取，供 LVGL 界面使用。
void Log_EraseRecent(void);                                        //临时：清日志区近期数据（调试用）

// ===================== 6. 原有环形缓存接口（全部保留）=====================
extern volatile uint8_t g_mqtt_connected;

/* ===== 断网补传总开关 =====
   0 = 关闭：只发实时数据，不回头翻旧账。
   1 = 打开（2026-11-21 起启用）：上线后补传 Flash 里积压的离线数据。

   实现要点见 code/Taskhandel.c 里 MQTT() 的注释：
   补传与实时【交替】，每个 6 秒窗口只用一次，总速率不变（10 条/分钟），
   所以不会像旧实现那样把巴法云的频率限制撑爆；发布失败不标记"已发送"，
   下一轮重试，数据不丢。离线写入也节流到 10 秒一条，避免缓存被瞬时写满。 */
#define CACHE_REPLAY_ENABLE 1

void FlashCache_Init(void);
int  FlashCache_Write(const char *json, uint16_t len);
/* 批量补传：只读不消耗地打包最多 max_cnt 条成 JSON 数组；成功发布后再 Commit。
   两步分离是为了"发布失败也绝不丢数据"。 */
int  FlashCache_PeekBatch(char *out, uint16_t out_size, uint8_t max_cnt, uint8_t *got);
void FlashCache_CommitBatch(uint8_t cnt);
int  FlashCache_HasData(void);


/* 彻底重置缓存区（2026-11-21）：擦除首扇区 + 读写指针归零。
   用途：清掉旧固件写入的、没有时间戳的历史条目。
   只擦一个扇区（约 400ms），后续写入会逐扇区擦除，不需要全擦 4MB。 */
void FlashCache_Reset(void);
#endif

