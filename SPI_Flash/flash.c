#include "ESP_at.h"
#include "flash.h"
#include "bsp_flash.h"
#include "pal.h"          /* 发布主题 / QoS 宏 */
#include <string.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// ===================== 新增：Flash 全局互斥锁 =====================
SemaphoreHandle_t flash_mutex = NULL;

// ===================== 新增：日志区写入偏移（循环写标记）=====================
 uint32_t log_write_offset = 0U;						//  下一条要写入的空白地址，最后一条有效日志在它前面(日志写指针 为空白地址)  日志偏移地址



extern UART_HandleTypeDef huart1;

//缓存 结构体对齐大小后为128字节
typedef struct {
    uint32_t magic;          // 魔术字：标记条目是否有效
    uint32_t timestamp;     // 时间戳
    uint16_t data_len;      // 有效数据长度
    uint8_t  data[CACHE_ENTRY_SIZE - 12]; // 数据区
} CacheEntry_t;

static uint32_t write_addr = CACHE_START_ADDR;
static uint32_t read_addr  = CACHE_START_ADDR;
static uint8_t  initialized = 0;


/**
 * @brief  分区读数据
 * @param  part_base 分区基地址
 * @param  offset    分区内偏移
 * @param  buf       接收缓冲区
 * @param  len       读取长度
 * @retval 0成功 1失败
 */
uint8_t Flash_Part_Read(uint32_t part_base, uint32_t offset, uint8_t *buf, uint16_t len)
{
    if(buf == NULL) return 1;
    uint32_t real_addr = part_base + offset;			//分区基地址 + 分区内偏移 = Flash 真实物理地址

    // 加锁保护
    //xSemaphoreTake(flash_mutex, portMAX_DELAY);
    SPI_FLASH_BufferRead(buf, real_addr, len);				// 调用底层硬件读
    //xSemaphoreGive(flash_mutex);

    return 0;
}

/**
 * @brief  分区写数据
 * @param  part_base 分区基地址
 * @param  offset    分区内偏移
 * @param  buf       待写数据
 * @param  len       写入长度
 * @retval 0成功 1失败
 */
uint8_t Flash_Part_Write(uint32_t part_base, uint32_t offset, uint8_t *buf, uint16_t len)
{
    if(buf == NULL) return 1;
    uint32_t real_addr = part_base + offset;

  // xSemaphoreTake(flash_mutex, portMAX_DELAY);
    SPI_FLASH_BufferWrite(buf, real_addr, len);
  //  xSemaphoreGive(flash_mutex);

    return 0;
}



//业务点位Log_Write() → W25Q日志分区存储 → 打开日志页触发page_log_refresh() → 倒序读Flash → LVGL列表(新日志置顶)
/**
入参校验：空字符串 / 锁未创建，直接退出，防止崩溃。
结构体赋值：
时间戳：HAL_GetTick() 记录事件发生时刻。
等级：传入的 LOG_INFO/LOG_WARN/LOG_ERROR。
魔数：固定 0x55AA，标记有效数据。
字符串拷贝：限制长度，防止超出结构体缓冲区导致内存越界。
循环覆盖逻辑
当「当前指针 + 单条日志大小」超过日志分区总大小 → 指针归零，实现循环日志。
工业设备常用方案：日志不手动删，写满自动覆盖旧日志。
调用分区接口写入：上层完全不碰底层地址。
写指针自增，准备写下一条日志。

 */
/*重要 日志写入 全工程统一入口*/
void Log_Write(Log_Level level, const char *str)
{
    /* 先做空指针校验再取锁：str 为 NULL 时 strlen 会直接 HardFault */
    if (str == NULL) return;
    if (flash_mutex == NULL) return;

    if (flash_mutex) xSemaphoreTake(flash_mutex, portMAX_DELAY);

    Log_Obj log_buf = {0};
    uint16_t str_len;

    // 1. 填充日志结构体
    log_buf.time_stamp = HAL_GetTick();  // 系统运行时间戳
    log_buf.level = level;
    log_buf.magic = LOG_MAGIC;           // 标记为有效日志

    // 拷贝日志内容，防止缓冲区溢出
    str_len = strlen(str);
    if(str_len > LOG_CONTENT_LEN - 1U)
        str_len = LOG_CONTENT_LEN - 1U;
    memcpy(log_buf.content, str, str_len);				//拷贝函数的最后一个参数表示 要拷贝的字节数

    // 2. 日志区循环判断：写满则回到日志区头部
    if((log_write_offset + LOG_OBJ_SIZE) > LOG_PART_SIZE)
    {
        log_write_offset = 0U;
    }
    /* 3. 回绕/跨入新扇区前必须擦除（NOR 写前擦原则），否则 AND 出乱码 */
    if (log_write_offset % 4096U == 0U)
    {
        SPI_FLASH_SectorErase(LOG_PART_ADDR + log_write_offset);
        SPI_FLASH_WaitForWriteEnd();
    }
    // 3. 写入日志分区    基地址     偏移地址
    Flash_Part_Write(LOG_PART_ADDR, log_write_offset, (uint8_t *)&log_buf, LOG_OBJ_SIZE);
    // 4. 偏移自增
    log_write_offset += LOG_OBJ_SIZE;
    if (flash_mutex) xSemaphoreGive(flash_mutex);
}


/**
 * @brief  读取单条日志
 * @param  log_offset 日志区内偏移(读指针)
 * @param  log_obj    日志结构体接收缓存
 * @retval 0=有效日志 1=无效/空日志
 */
/* 临时：清日志区前 32KB（覆盖近期脏数据），上电调用一次后即可移除。
   2026-11-19 修复：原来一次连擦 32 个扇区（128KB）且全程不让出 CPU，
   而调用它的 UI 任务优先级（AboveNormal）高于 AT 任务（Normal）——
   擦除期间 AT 任务被饿死数秒，ESP_SendCmd 的 HAL_GetTick 超时早已到期，
   紧随其后的 AT+CWMODE / AT+CWJAP 全部判失败 → "WiFi rejoin failed"。
   现在：扇区数降到 8（32KB，够清近期历史），每个扇区后主动让出 CPU。 */
void Log_EraseRecent(void)
{
    /* 与 Log_Write 同一把锁：开机擦除期间 MQTT 任务可能正在写 "MQTT online"，
       不取锁会与擦除并发，把新日志写进已擦/未擦扇区产生乱码。 */
    if (flash_mutex) xSemaphoreTake(flash_mutex, portMAX_DELAY);

    for(uint32_t s = 0; s < 8; s++)
    {
        SPI_FLASH_SectorErase(LOG_PART_ADDR + s * 4096U);
        SPI_FLASH_WaitForWriteEnd();
        /* 关键：让出 CPU，保证 AT 任务能继续跑心跳/配网（见上方说明） */
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    log_write_offset = 0U;

    if (flash_mutex) xSemaphoreGive(flash_mutex);
}

uint8_t Log_ReadOne(uint32_t log_offset, Log_Obj *log_obj)
{
    if(log_obj == NULL) return 1;

    if (flash_mutex) xSemaphoreTake(flash_mutex, portMAX_DELAY);
    Flash_Part_Read(LOG_PART_ADDR, log_offset, (uint8_t *)log_obj, LOG_OBJ_SIZE);
    uint8_t bad = (log_obj->magic != LOG_MAGIC);
    if (flash_mutex) xSemaphoreGive(flash_mutex);

    return bad ? 1 : 0;
}


void Log_EraseAll(void)
{
    uint32_t sec_addr;
  // xSemaphoreTake(flash_mutex,portMAX_DELAY);
    //按4KB扇区循环擦除整个日志分区
    for(sec_addr = LOG_PART_ADDR; sec_addr < LOG_PART_ADDR + LOG_PART_SIZE; sec_addr += 4096)
    {
        SPI_FLASH_SectorErase(sec_addr);
        SPI_FLASH_WaitForWriteEnd();
    }
   //xSemaphoreGive(flash_mutex);
    //擦完直接把写指针置0
    log_write_offset = 0;
}


/* ================= 上电清日志（2026-11-19）=================
   需求：每次断电上电，把上一次的日志清掉。

   做法：只擦第 0 扇区 + 写指针归零，**不是**擦整个 4MB 分区
   （1024 个扇区 × 400ms ≈ 7 分钟，开机等不起）。
   这样做为什么就够：
     日志页是从写指针 log_write_offset 倒着读、只显示最新 30 条。
     指针回到 0 之后，后面那些旧数据永远读不到；随着新日志从头往下写，
     每跨进一个新扇区，Log_Write() 里本来就会先擦该扇区，
     旧数据被逐扇区覆盖掉。
   为什么不能只归零指针不擦：NOR Flash 只能 1→0，不擦就写 = 新旧按位与 = 乱码。

   代价：一次扇区擦除约 400ms，发生在调度器启动前，不影响任何任务。

   ⚠ 一个例外：看门狗/软件复位时**不清**。
     那种情况恰恰是最需要看上次日志的时候，清掉等于把事故现场抹了。
     想改成"任何复位都清"，把 LOG_CLEAR_SKIP_ABNORMAL_RESET 定义为 0。 */
#define LOG_CLEAR_SKIP_ABNORMAL_RESET   1

static void Log_ClearOnBoot(void)
{
#if LOG_CLEAR_SKIP_ABNORMAL_RESET
    /* 先判复位来源，再把标志清掉 —— 不清的话下次读到的还是这次的值 */
    uint8_t abnormal = 0;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) ||
        __HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) ||
        __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)  ||
        __HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
    {
        abnormal = 1;
    }
    __HAL_RCC_CLEAR_RESET_FLAGS();

    if (abnormal)
        return;          /* 异常复位：保留上次日志，留给事后排查 */
#endif

    SPI_FLASH_SectorErase(LOG_PART_ADDR);
    SPI_FLASH_WaitForWriteEnd();
    log_write_offset = 0;
}

/*
上电时扫描 Flash 4MB~8MB 缓存分区，
找到上次断电前最后一条有效缓存数据，
定位下一条数据的写入地址，实现断电续写；
同时初始化读写指针与状态标记，让 Flash 缓存模块正常工作。
*/
void FlashCache_Init(void)
{
    /* 先按需求把上一次的日志清掉（正常上电时）；之后下面的扫描会
       从第 0 扇区读起、读不到有效条目 → 写指针停在 0，日志从头开始。 */
    Log_ClearOnBoot();

    /* 日志写指针定位：从分区头扫描到最后一个有效条目，续写其后。
       旧版硬编码 0 会在旧日志数据上直接重写——NOR Flash 只能 1->0，
       不擦除就写 = 新旧数据按位 AND = 乱码（两天乱码问题的根因）。 */
    {
        Log_Obj probe;
        log_write_offset = 0;
        while (log_write_offset + LOG_OBJ_SIZE <= LOG_PART_SIZE)
        {
            Flash_Part_Read(LOG_PART_ADDR, log_write_offset, (uint8_t *)&probe, LOG_OBJ_SIZE);
            if (probe.magic != LOG_MAGIC) break;   /* 第一个无效位置 = 续写点 */
            log_write_offset += LOG_OBJ_SIZE;
        }
        /* 写满则回绕：擦除首个扇区，从头再来（循环日志） */
        if (log_write_offset + LOG_OBJ_SIZE > LOG_PART_SIZE)
        {
            log_write_offset = 0;
            SPI_FLASH_SectorErase(LOG_PART_ADDR);
            SPI_FLASH_WaitForWriteEnd();
        }
    }
    CacheEntry_t entry;
    uint32_t addr = CACHE_START_ADDR;

    write_addr = CACHE_START_ADDR;
    while (addr < CACHE_END_ADDR) {
        SPI_FLASH_BufferRead((uint8_t *)&entry, addr, sizeof(CacheEntry_t));
			/* 已发送(MAGIC_SENT)的条目仍占用空间，扫描必须跳过而不能当作末尾，
			   否则重启后写指针会回退到已发送区，覆盖尚未补传的数据 */
        if (entry.magic == MAGIC_VALID || entry.magic == MAGIC_SENT) {
            write_addr = addr + sizeof(CacheEntry_t);
        } else {
            break;
        }
        addr += sizeof(CacheEntry_t);
    }

    if (write_addr >= CACHE_END_ADDR) {
        write_addr = CACHE_START_ADDR;
    }

    /* ===== 只补传"最近一批"（2026-11-21）=====
       背景：缓存分区里可能积着好几天、上百条历史离线数据（以前每次开机那
       二三十秒也会写几条）。如果一上来从分区头开始补，等于把几天前的旧数据
       当成新闻往云端发，而且按 6 秒一条的速度要补几个小时。
       做法：读指针从写指针往前退 CACHE_REPLAY_BOOT_MAX 条，更早的条目不读
       （它们仍是 VALID，但不会被访问；等写指针绕回来时自然被覆盖）。
       CACHE_REPLAY_BOOT_MAX = 50，约等于最近 8 分钟的数据（离线按 10 秒一条）。 */
#define CACHE_REPLAY_BOOT_MAX   50
    {
        uint32_t back = (uint32_t)CACHE_REPLAY_BOOT_MAX * sizeof(CacheEntry_t);
        if (write_addr > CACHE_START_ADDR + back)
            read_addr = write_addr - back;
        else
            read_addr = CACHE_START_ADDR;
    }
    initialized = 1;
}

// 写入
int FlashCache_Write(const char *json, uint16_t len)
{
    if (!initialized) return -1;
    if (len > CACHE_ENTRY_SIZE - 12) return -2;

	
		    // ========== 加锁 ==========
    if (flash_mutex) xSemaphoreTake(flash_mutex, portMAX_DELAY);
	
    CacheEntry_t entry;
		memset(&entry, 0xFF, sizeof(entry));		//擦除为1111
    entry.magic     = MAGIC_VALID;
    entry.timestamp = HAL_GetTick();
    entry.data_len  = len;
    memcpy(entry.data, json, len);

    // ─── 环形回绕处理 ───
    if ((write_addr +sizeof(CacheEntry_t))>= CACHE_END_ADDR) {   // 到达 8MB 边界
        write_addr = CACHE_START_ADDR;    // 回绕到 4MB
        SPI_FLASH_SectorErase(write_addr);// 擦除起始扇区
    }

    // ─── 扇区首地址擦除 ───
    if (write_addr % CACHE_SECTOR_SIZE == 0) {  // 写指针刚好在扇区边界
        SPI_FLASH_SectorErase(write_addr);      // 擦除整个 4KB 扇区
    }

    SPI_FLASH_BufferWrite((uint8_t *)&entry, write_addr, sizeof(CacheEntry_t));
    write_addr += sizeof(CacheEntry_t);	//固定128
		
		    // ========== 解锁 ==========
    if (flash_mutex) xSemaphoreGive(flash_mutex);
    return 0;
}

// 读取一条，读完直接销毁（最安全，永不重复）

// 是否有未读数据
// 【 fix 】判断是否有未读数据：只看 read < write，
// ======================================================================
int FlashCache_HasData(void)
{
    if (!initialized) return 0;

    /* 单纯判断读指针是否落后于写指针。
       补传条数由调用方（MQTT 任务的 count<5）控制，此处不再重复计数并强制清零，
       否则两处计数互相打架，会在第 5 条后把读指针直接拽到写指针丢数据。 */
    return (read_addr < write_addr) ? 1 : 0;
}

/* 彻底重置缓存区：擦首扇区 + 指针归零。
   为什么只擦首扇区：日志/缓存都是环形写，每跨进一个新扇区 Log_Write/FlashCache_Write
   本来就会先擦该扇区，所以把指针拉回起点 + 擦掉起点即可，不必擦满 4MB（那要 7 分钟）。 */
void FlashCache_Reset(void)
{
    if (!initialized) return;

    if (flash_mutex) xSemaphoreTake(flash_mutex, portMAX_DELAY);

    write_addr = CACHE_START_ADDR;
    read_addr  = CACHE_START_ADDR;

    SPI_FLASH_SectorErase(CACHE_START_ADDR);
    SPI_FLASH_WaitForWriteEnd();

    if (flash_mutex) xSemaphoreGive(flash_mutex);
}


// 清空

//补传到云端

// 【 fix 】标记已发送：只改魔数，不擦除、不乱动指针
// ======================================================================

/* ================= 批量补传（2026-11-21）=================
   为什么必须批量：巴法云免费版约 5~6 秒允许一条消息。离线按 10 秒一条产生
   （6 条/分钟），而交替补传只有 5 条/分钟 —— 数学上永远追不上，积压只会越滚越大。
   把 5 条历史打进球一条消息后，吞吐变成 25 条/分钟，几分钟就能清空；
   消息条数没增加，所以完全不碰频率限制。

   接口拆成两步，是因为"发布失败绝不能丢数据"：
     FlashCache_PeekBatch()   只读、不消耗，把最多 max 条打包成一个 JSON 数组
     FlashCache_CommitBatch() 等 broker 确认收到后才标记 SENT 并推进读指针
   中途失败就直接返回，读指针原地不动，下一轮重试。 */

int FlashCache_PeekBatch(char *out, uint16_t out_size, uint8_t max_cnt, uint8_t *got)
{
    uint32_t addr;
    uint16_t pos;
    uint8_t  n = 0;
    CacheEntry_t entry;

    if (!initialized || out == NULL || got == NULL || max_cnt == 0)
        return -1;

    *got = 0;
    if (flash_mutex) xSemaphoreTake(flash_mutex, portMAX_DELAY);

    pos = (uint16_t)snprintf(out, out_size, "{\"off\":1,\"batch\":[");
    if (pos >= out_size)
    {
        if (flash_mutex) xSemaphoreGive(flash_mutex);
        return -1;
    }

    for (addr = read_addr;
         addr + sizeof(CacheEntry_t) <= write_addr && n < max_cnt;
         addr += sizeof(CacheEntry_t))
    {
        SPI_FLASH_BufferRead((uint8_t *)&entry, addr, sizeof(CacheEntry_t));
        if (entry.magic != MAGIC_VALID)            /* 已发送/无效：跳过，不算进批次 */
            continue;
        if (entry.data_len == 0 || entry.data_len > (uint16_t)(CACHE_ENTRY_SIZE - 12))
            continue;
        if ((uint32_t)pos + entry.data_len + 4U >= out_size)
            break;                                  /* 装不下就少打几条 */

        if (n > 0) out[pos++] = ',';
        memcpy(&out[pos], entry.data, entry.data_len);
        pos += entry.data_len;
        n++;
    }

    if (n == 0)
    {
        if (flash_mutex) xSemaphoreGive(flash_mutex);
        return -2;                                  /* 没有可补的数据 */
    }

    out[pos++] = ']';
    out[pos++] = '}';
    out[pos]   = '\0';
    *got = n;

    if (flash_mutex) xSemaphoreGive(flash_mutex);
    return 0;
}

/* 发布成功后调用：把刚打包的那 cnt 条标记为已发送，并推进读指针。
   只处理 MAGIC_VALID 的条目——MAGIC_SENT 的跳过但读指针要跟着走。 */
void FlashCache_CommitBatch(uint8_t cnt)
{
    uint32_t addr;
    CacheEntry_t entry;
    uint8_t  n = 0;

    if (!initialized || cnt == 0)
        return;

    if (flash_mutex) xSemaphoreTake(flash_mutex, portMAX_DELAY);

    for (addr = read_addr;
         addr + sizeof(CacheEntry_t) <= write_addr && n < cnt;
         addr += sizeof(CacheEntry_t))
    {
        SPI_FLASH_BufferRead((uint8_t *)&entry, addr, sizeof(CacheEntry_t));

        if (entry.magic == MAGIC_SENT)              /* 早已发送过：跳过并前进 */
        {
            read_addr = addr + sizeof(CacheEntry_t);
            continue;
        }
        if (entry.magic != MAGIC_VALID)             /* 擦除态/坏数据：到头了 */
            break;

        entry.magic = MAGIC_SENT;
        SPI_FLASH_WriteEnable();
        SPI_FLASH_BufferWrite((uint8_t *)&entry, addr, sizeof(CacheEntry_t));
        read_addr = addr + sizeof(CacheEntry_t);
        n++;
    }

    if (read_addr > write_addr)
        read_addr = write_addr;

    if (flash_mutex) xSemaphoreGive(flash_mutex);
}

/* ================= 云端按需拉取历史（2026-11-21）=================
   与"自动补传"的区别：补传是设备主动推、占推送配额；这里是云端先问、设备才答，
   用户查一次就发一页，不查就一个字节都不发。
   数据来源用同一块缓存分区（含已补传的 MAGIC_SENT 条目），所以补传过的历史
   依然能被查到，不需要额外分区。

   out 形如：{"off":1,"h":[{...},{...}]}   （数组内按时间从旧到新）
   skip    跳过最新的 skip 条（翻页用）
   max_cnt 本页最多几条（调用方传 CACHE_HIST_PAGE）
   got     实际取到几条；返回 -2 = 没有更多历史 */
int FlashCache_PeekHistory(char *out, uint16_t out_size, uint16_t skip, uint8_t max_cnt, uint8_t *got)
{
    uint32_t addr_list[8];
    uint32_t addr;
    uint16_t pos;
    uint8_t  n = 0, i;
    CacheEntry_t entry;

    if (!initialized || out == NULL || got == NULL || max_cnt == 0)
        return -1;

    *got = 0;
    if (max_cnt > 8) max_cnt = 8;          /* addr_list 的容量 */

    if (flash_mutex) xSemaphoreTake(flash_mutex, portMAX_DELAY);

    /* 从写指针往前倒着找：跳过空洞/坏条目直接停（那条边界之外不会有有效数据） */
    addr = write_addr;
    while (addr >= CACHE_START_ADDR + sizeof(CacheEntry_t) && n < max_cnt)
    {
        addr -= sizeof(CacheEntry_t);
        SPI_FLASH_BufferRead((uint8_t *)&entry, addr, sizeof(CacheEntry_t));

        if (entry.magic != MAGIC_VALID && entry.magic != MAGIC_SENT)
            break;                          /* 到头了（擦除态 = 已写区域的下界） */
        if (entry.data_len == 0 || entry.data_len > (uint16_t)(CACHE_ENTRY_SIZE - 12))
            continue;

        if (skip > 0) { skip--; continue; }  /* 翻页：跳过更新的那几条 */
        addr_list[n++] = addr;
    }

    if (n == 0)
    {
        if (flash_mutex) xSemaphoreGive(flash_mutex);
        return -2;
    }

    pos = (uint16_t)snprintf(out, out_size, "{\"off\":1,\"h\":[");
    if (pos >= out_size)
    {
        if (flash_mutex) xSemaphoreGive(flash_mutex);
        return -1;
    }

    /* addr_list 是"新→旧"，输出要"旧→新"，所以倒着遍历 */
    {
    uint8_t first = 1;
    for (i = n; i > 0; i--)
    {
        SPI_FLASH_BufferRead((uint8_t *)&entry, addr_list[i - 1], sizeof(CacheEntry_t));
        if ((uint32_t)pos + entry.data_len + 4U >= out_size)
            break;
        if (!first) out[pos++] = ',';        /* 第一条不加逗号 */
        first = 0;
        memcpy(&out[pos], entry.data, entry.data_len);
        pos += entry.data_len;
        (*got)++;
    }
    }

    if (*got == 0)
    {
        if (flash_mutex) xSemaphoreGive(flash_mutex);
        return -2;
    }

    out[pos++] = ']';
    out[pos++] = '}';
    out[pos]   = '\0';

    if (flash_mutex) xSemaphoreGive(flash_mutex);
    return 0;
}

/*从读指针位置读一条缓存数据*/
                
