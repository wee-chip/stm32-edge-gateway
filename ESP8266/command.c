#include "command.h"
#include "usart.h"
#include "stm32f4xx.h"
#include "dma.h"
#include "lcd.h"
#include "touch.h"
#include "stdio.h"
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
#include "Taskhandel.h"
#include "cmsis_os2.h"

extern uint8_t a[256];
extern DMA_HandleTypeDef hdma_usart1_rx;
// 循环缓冲区大小

// 循环缓冲区
uint8_t buffer[BUFFER_SIZE];
/* volatile：写索引在 DMA 中断中更新，读侧任务必须每次重读内存，
   否则编译器缓存寄存器导致"数据到了却读不到"的假超时 */
volatile uint8_t readIndex = 0;
volatile uint8_t writeIndex = 0;

/**
 * @brief  清空环形缓冲区（重置读写指针）
 */
void RingBuf_Clear(void)
{
    /* 指针复位必须在临界区内完成：DMA 回调可能在两条赋值之间插入，
       只清一半会让读写索引错位，后续数据整体偏移。 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    readIndex = 0;
    writeIndex = 0;
    if (!primask) __enable_irq();
}

/**
* @brief 增加读索引
* @param length 要增加的长度
*/
void Command_AddReadIndex(uint8_t length) {
    readIndex += length;
    readIndex %= BUFFER_SIZE;
}

/**
* @brief 读取第i位数据 超过缓存区长度自动循环
* @param i 要读取的数据索引
*/

uint8_t Command_Read(uint8_t i) {
    uint8_t index = i % BUFFER_SIZE;
    return buffer[index];
}

/*获取read下标对应的数据*/
uint8_t RingBuf_Read(uint8_t *out) {
    if (readIndex == writeIndex) {
        return 0;   // 缓冲区空
    }
    *out = buffer[readIndex];
    readIndex = (readIndex + 1) % BUFFER_SIZE;
    return 1;
}


uint8_t Command_GetLength() {
    return (writeIndex + BUFFER_SIZE - readIndex) % BUFFER_SIZE;
}


/**
* @brief 计算缓冲区剩余空间
* @return 剩余空间
* @retval 0 缓冲区已满
* @retval 1~BUFFER_SIZE-1 剩余空间
* @retval BUFFER_SIZE 缓冲区为空
*/
/* 返回值必须 > uint8_t：缓冲区空时返回 256，用 uint8_t 会被截断成 0，
   导致 Command_Write 认为"没空间"而永远拒绝写入 */
uint16_t Command_GetRemain() {
    return (uint16_t)(BUFFER_SIZE - Command_GetLength());
}

/**	不会覆盖旧数据 满后停止写入
* @brief 向缓冲区写入数据
* @param data 要写入的数据指针
* @param length 要写入的数据长度
* @return 写入的数据长度
*/


uint8_t Command_Write(uint8_t *data, uint16_t len)
{
    // 入参校验
    if (data == NULL || len == 0)
        return 0;

    uint16_t remain = Command_GetRemain();			//剩余长度
    if (remain < len)
        return 0;

    /* osKernelLock 不能在中断上下文调用（本函数被 DMA 回调调用）——
       换成关中断临界区 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t idx = writeIndex;
    // 分段写入环形缓冲区
    uint16_t part1 = BUFFER_SIZE - idx;					//写索引到末尾的剩余长度
    if (len <= part1)
    {
        memcpy(buffer + idx, data, len);
        writeIndex = (idx + len) % BUFFER_SIZE;
    }
    else			//绕回到开头 但并没有覆盖数据 前面的数据已经被读取
    {
        memcpy(buffer + idx, data, part1);
        memcpy(buffer, data + part1, len - part1);
        writeIndex = (len - part1) % BUFFER_SIZE;
    }

    if (!primask) __enable_irq();   /* 恢复中断 */
    return len;
}




extern uint8_t screen_off_flag;
/**
* @brief 从缓冲区读取所有未处理的数据（不解析指令帧）
* @param command 存储读取数据的指针
* @return 实际读取的字节数（0 表示缓冲区为空）
*/
uint8_t Command_GetCommand(uint8_t *command) {
    uint8_t len = Command_GetLength();  // 获取当前未处理的数据长度
    if (len == 0) {
        return 0;
    }
    // 顺序读取 len 个字节到 command 中
    for (uint8_t i = 0; i < len; i++) {
        command[i] = Command_Read(readIndex + i);
    }
    Command_AddReadIndex(len);  // 移动读索引，表示这些数据已被消费
    return len;
}

extern TickType_t idle_tick;
extern uint8_t lcd_bl_en ; //1亮屏 0熄屏
/*接收完成回调*/
volatile uint32_t g_rx_cb_count = 0;   /* 诊断：DMA 接收回调触发次数 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart==&huart1) g_rx_cb_count++;   /* 诊断计数 */
	if(huart==&huart1)
	{
		Command_Write(a,Size);		//向缓冲区写入数据
		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,a,sizeof(a));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
	}
}

/* UART 错误回调 —— 2026-11-19 新增，解决"ESP 复位后彻底失聪"。
   根因：ESP8266 硬复位时会以 74880bps 吐一段启动日志，而 MCU 按 115200
   接收，极易触发 ORE（溢出）/FE（帧错）/NE（噪声）。HAL 在这种情况下会
   结束 DMA 接收并调用本回调；**原来工程里没有这个回调**，于是 DMA 一停
   就再也没人把它拉起来 → 之后所有 AT 指令都收不到应答，
   现象就是日志无限刷 "ESP no response after reset"、再也连不上。
   处理：清标志 + 把接收状态复位 + 重新武装 DMA（与 RxEventCallback 一致）。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* F4 清 ORE/FE/NE/PE 的正确方式：先读 SR、再读 DR */
        __HAL_UART_CLEAR_OREFLAG(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;

        /* 错误后 HAL 可能仍停在 BUSY_RX，强行置就绪再重新接收，
           否则 HAL_UARTEx_ReceiveToIdle_DMA 会因状态非 READY 直接返回 */
        huart->RxState = HAL_UART_STATE_READY;

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, a, sizeof(a));
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
}
