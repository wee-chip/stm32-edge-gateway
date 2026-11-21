#include "ESP_at.h"
#include "modbus.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "stm32f4xx_hal_def.h"
#include "FreeRTOS.h"
#include "task.h"
#include "flash.h"
#include "Taskhandel.h"
#include "pal.h"          /* 发布主题 / QoS 宏 */
#include "rtc.h"          /* 时间戳：RTC_GetDateTimeStr */

/* ===== 2026-11-17 温湿度并联前必须关闭：光照自动改地址 =====
   原逻辑：上电若 0x01 有应答、0x02 无应答，就向 0x01 发
     01 06 00 64 00 02 49 D4  （功能码06=写单寄存器，把光照地址改成2）
   单接光照时 0x01 就是光照自己，无害；温湿度并联后 0x01 是 XY-MD02，
   这条写命令会打到温湿度、可能改乱它的寄存器导致失联。
   光照地址已实测固定在 0x02（6分钟测试 lp=2 全程稳定），不再需要自动改。
   要重新启用（仅限单独接光照改地址时）改成 1，改完立刻改回 0。 */
#define LIGHT_AUTOFIX_ADDR  0

/* ================================================================
 *	查表法
 *  CRC 高字节表 (高 8 位部分)
 *  由多项式 0x8005 预计算生成，共 256 项
 * ================================================================ */
 
 extern QueueHandle_t     data_queue;

/* ===== 临时调试变量：排查 485 通信问题用，问题解决后删掉 ===== */
volatile uint8_t  g_rx_count   = 0;   // 温湿度(0x04)最后一次收到的字节数
volatile uint8_t  g_rx_light   = 0;   // 光照(0x03)最后一次收到的字节数
volatile uint32_t g_ok_count   = 0;   // 读取成功次数
volatile uint32_t g_fail_count = 0;   // 读取失败次数
volatile uint32_t g_th_tick    = 0;   // 温湿度最近一次读取成功的时刻（算"数据有多旧"）
volatile uint32_t g_tx_fail    = 0;   // 发送环节自身失败的次数（区分"没发出去"和"没人应"）
volatile uint32_t g_light_raw  = 0;   // 光照两个寄存器的原始 32 位值（核对除数用）
 extern UART_HandleTypeDef huart2;

static const uint8_t auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

static const uint8_t auchCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};



/****************************************************手敲*************************************************************/
/*CRC16校验*/

/*
一般情况 低地址在前 小端排序（低位字节在低地址）低字节在前  大端排序 高字节在前
*/
uint16_t CRC_16_MODBUS(uint8_t *data,uint8_t len,uint8_t flag)
{
	if(flag)
	{
		/*按位循环法*/
		uint16_t crc=0xffff;
		while(len--)
		{
			crc^=*data++;
			for(int i=0;i<8;i++)
			{
				if(crc &1)
				{
					crc=(crc>>1)^0xA001;
				}
				else
				{
					crc>>=1;
				}
			}
		}
		return crc;
	}
	else
	{
		/*查表法*/
		uint8_t CRCL=0xff;
		uint8_t CRCH=0xff;
		uint8_t index=0;
		while(len--)
		{
			index=CRCL^*data++;
			CRCL=CRCH^auchCRCHi[index];
			CRCH=auchCRCLo[index];
		}
		return (CRCH<<8)|CRCL;
	}
}


// 结构体数组 多从机配置：地址、功能码、起始寄存器、寄存器数量、错误计数、数据
volatile uint8_t g_echo_cnt = 0xFF;   /* 最后一次发送的回显字节数：8=模块收发正常 0=模块发送失败 */

 ModDev_t dev_list[]={
	{0x01,0x04,0x0001,2,0,0,0},//温湿度 XY-MD02：功能码04读输入寄存器0x0001起=温度/湿度÷10（实测26.3℃/36.6%）
    {0x02,0x03,0x0002,2,0,0,0} //光照 B-RS-L30（地址已实测改为0x02）：功能码03读0x0002起2寄存器=32位÷1000

};
uint8_t dev_cnt = sizeof(dev_list)/sizeof(ModDev_t);
#define MAX_ERR_CNT 5   //连续失败5次临时跳过该设备



/* 把 USART2 接收端读空 + 清掉溢出错误。
   ⚠ 不能再用 __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE|UART_FLAG_ORE)：
   在 STM32F4 上 RXNE / ORE 是只读状态位，那个宏展开是"往 SR 写 0"，对这两位无效。
   真正的清法是读 DR（清 RXNE）和"读 SR 再读 DR"（清 ORE）。
   不清干净的直接后果：上一轮残留的字节会被算进这一帧的开头，
   count 和 data_len 对不上 → CRC 永远校验不过，看起来像"传感器不应答"。 */
static void USART2_RxFlush(void)
{
    volatile uint32_t tmp;
    uint8_t guard = 0;

    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) && guard++ < 64)
        tmp = huart2.Instance->DR;

    tmp = huart2.Instance->SR;   /* 先读 SR */
    tmp = huart2.Instance->DR;   /* 再读 DR：清 ORE/NE/FE/PE */
    (void)tmp;
}

/**
 * @brief 通用Modbus03读寄存器   采集数据
 * @param addr:从机地址 func:功能码 reg:起始地址 regcnt:寄存器数量 buf:接收缓存
 * @retval 0失败 1成功
*/
uint8_t Modbus_Read_Dev(uint8_t addr,uint8_t func,uint16_t reg,uint16_t regcnt,uint16_t *buf)
{
    uint8_t tx[8];
    uint8_t rx[32];
    uint8_t count=0;
    uint16_t crc=0;

    //组问询帧
    tx[0] = addr;
    tx[1] = func;
    tx[2] = (reg>>8)&0xFF;
    tx[3] = reg&0xFF;
    tx[4] = (regcnt>>8)&0xFF;
    tx[5] = regcnt&0xFF;

    crc=CRC_16_MODBUS(tx,6,0);
    tx[6]=crc&0xFF;
    tx[7]=(crc>>8)&0xFF;

    memset(rx,0,sizeof(rx));
	      // 关键：清空UART接收标志，清除上一轮残留
    USART2_RxFlush();
	
    RS485_TX;

    /* 方向切换后必须给驱动器一点建立时间。
       这块模块的 EN 经 10K 下拉，PB0 拉高到 SP3485 真正接管总线之间有一段
       上升沿 + 使能延迟（接触电阻越大越慢）。9600 下一个位只有 104µs，
       如果 EN 还没站稳我们就开始发，**第一个字节的前几位会被切掉**，
       传感器算 CRC 不过直接不回话 —— 表现为"这一帧一个字节都没回来"。
       这里多等 1ms（约 10 个位），代价可以忽略。 */
    vTaskDelay(1);

    /* HAL_UART_Transmit 阻塞返回时已等到 TC（最后一位停止位发完）。 */
    if(HAL_UART_Transmit(&huart2,tx,8,1000) != HAL_OK)
    {
        RS485_RX;
        g_tx_fail++;          /* 诊断：区分"请求没发出去"和"发出去没人应" */
        g_fail_count++;
        return 0;
    }
    RS485_RX;   /* TC 后立即切接收，不再额外延时 */

    /* EN=DE+RE短接的标准模块：发送时接收器禁用（无回显）。
       切接收后清一次标志（滤掉方向切换瞬间的电平抖动），
       随后只收传感器的真实响应。 */
    USART2_RxFlush();
    g_echo_cnt = 0;

    uint32_t start=HAL_GetTick();
    uint32_t upd=HAL_GetTick();
    uint8_t  got_first = 0;
    count=0;
    //沿用你原有阻塞轮询接收逻辑，但修掉两个要命的地方
    while(1)
    {
        if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_RXNE))
        {
            uint8_t ch=huart2.Instance->DR&0xFF;
            if(count<sizeof(rx)) rx[count++]=ch;
            got_first = 1;
            upd=HAL_GetTick();
        }
        if(count>0 && (HAL_GetTick()-upd)>20) break;			//帧间隔3.5个字符 9600下约4ms，留余量取20ms
        if(HAL_GetTick()-start>400) break;								//绝对超时：9600 下正常应答 < 50ms，400ms 已很宽裕

        /* 收到首字节之后**不能**再 vTaskDelay(1)：
           9600 波特率下一个字节只要 1.04ms，睡 1ms 期间会再来一个字节，
           而 USART2 这里没开 DMA/中断，DR 被覆盖就置 ORE、字节直接丢 ——
           表现为"收到了但 CRC 永远不对"。首字节之后改为紧循环，
           一帧最多 9~13 字节、约 14ms，不会饿死别的任务。
           没收到首字节时才让出 CPU，等从机应答。 */
        if(!got_first)
            vTaskDelay(1);
    }

    /* 按设备分开记录：原来只有一个 g_rx_count，轮询顺序是"温湿度→光照"，
       光照失败会把温湿度的 9 字节覆盖成 0，于是云端出现
       "th:OK 但 rx:0" 这种自相矛盾的组合，排查时非常误导人。 */
    if (func == 0x04)      g_rx_count = count;
    else if (func == 0x03) g_rx_light = count;

    /* 边界保护：无响应时 count 可能为 0，count-2 传入 uint8_t 会变成 254，
       导致 CRC 函数越界读 rx（仅 32 字节）——先挡掉短帧 */
    if(count < 5)
    {
        g_fail_count++;
        return 0;
    }

    //校验长度+地址+CRC
    uint8_t data_len = rx[2];			//数据字节数

    /* 寄存器个数上界校验：regcnt 是配置值，若被改错会越界读 rx、越界写 buf，
       i 为 uint8_t 时 regcnt>255 还会死循环 */
    if(regcnt == 0 || regcnt > 8 || (uint16_t)(3 + regcnt * 2 + 2) > count)
    {
        g_fail_count++;
        return 0;
    }

    uint16_t crc_calc = CRC_16_MODBUS(rx,count-2,1);
    if(count == (3+data_len+2) && rx[0]==addr && rx[1]==func
        && (rx[count-2]==(crc_calc&0xFF)) && (rx[count-1]==((crc_calc>>8)&0xFF)))
    {
        //数据转存到buf
        for(uint8_t i=0;i<regcnt;i++)					//一个寄存器是两个字节 一个数据
        {
            buf[i] = (rx[3+i*2]<<8)|rx[4+i*2];
        }
        g_ok_count++;                 /* 调试：成功次数 */
        return 1;
    }
    g_fail_count++;                   /* 调试：失败次数 */
    return 0;
}




/* 临时：把光照传感器（B-RS-L30）地址从 0x01 改为 0x02。
   命令帧来自说明书 2.2 节"修改设备地址"，CRC 已由厂家给出：49 D4。
   成功后传感器回显同一帧。用法：只接光照传感器时上电执行一次。 */
uint8_t Modbus_SetLightAddr02(void)
{
    uint8_t tx[8] = {0x01, 0x06, 0x00, 0x64, 0x00, 0x02, 0x49, 0xD4};
    uint8_t rx[16];
    uint8_t count = 0;

    USART2_RxFlush();

    RS485_TX;
    HAL_UART_Transmit(&huart2, tx, 8, 1000);
    vTaskDelay(1);
    RS485_RX;

    /* 同 Modbus_Read_Dev：切接收后清标志 */
    USART2_RxFlush();

    uint32_t start = HAL_GetTick();
    uint32_t upd   = HAL_GetTick();
    while(1)
    {
        if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))
        {
            uint8_t ch = huart2.Instance->DR & 0xFF;
            if(count < sizeof(rx)) rx[count++] = ch;
            upd = HAL_GetTick();
        }
        if(count > 0 && (HAL_GetTick() - upd) > 50) break;
        if(HAL_GetTick() - start > 800) break;
        vTaskDelay(1);
    }

    /* 成功 = 收到 8 字节且与发送帧一致（06 写命令回显） */
    return (count == 8 && memcmp(rx, tx, 8) == 0) ? 1 : 0;
}

/* 光照探测结果（位域）：bit0=0x01在线 bit1=0x02在线 bit2=补发改地址成功 */
volatile uint8_t g_light_probe = 0;

volatile uint8_t g_rs485_invert = 0;

/* 温湿度真实探测：当前只接 XY-MD02 时，用它判断 485 链路通不通
   g_th_raw[0]=温度原始值(263=26.3℃) g_th_raw[1]=湿度原始值(369=36.9%)
   两个都为 0 表示没读到任何响应 */
uint16_t g_th_raw[2] = {0, 0};
volatile uint8_t g_th_ok = 0;

void Modbus_ProbeLight(void)
{
    uint16_t buf[2];

    /* ===== 先用标准极性探测温湿度 XY-MD02：0x01 / 0x04 / 0x0001 =====
       这是你现在唯一接的传感器，也是判断 485 链路通不通的唯一真实依据。
       注意：老版本的 probe 只发 0x03/0x0002（光照寄存器），
       温湿度传感器根本不认这条命令，所以 01:FAIL 是必然的假信号。 */
    /* 方向极性固定为标准（EN 高 = 发送）：这块 SP3485 模块的 EN 是 10K 下拉，
       不存在"反相版本"，翻极性只会让模块在接收时占住总线。 */
    g_rs485_invert = 0;
    g_th_ok = Modbus_Read_Dev(0x01, 0x04, 0x0001, 2, g_th_raw) ? 1 : 0;

    /* 读不到不换极性，只重发一次：485 上偶发丢帧很常见 */
    if(!g_th_ok)
        g_th_ok = Modbus_Read_Dev(0x01, 0x04, 0x0001, 2, g_th_raw) ? 1 : 0;

    /* ===== 以下保留原光照探测逻辑（极性沿用上面确定的），不影响温湿度 ===== */
    uint8_t at01 = Modbus_Read_Dev(0x01, 0x03, 0x0002, 2, buf) ? 1 : 0;
    uint8_t at02 = Modbus_Read_Dev(0x02, 0x03, 0x0002, 2, buf) ? 1 : 0;
    uint8_t fixed = 0;
#if LIGHT_AUTOFIX_ADDR
    /* ⚠ 温湿度并联后禁用：见文件头 LIGHT_AUTOFIX_ADDR 说明。
       0x01 现在是温湿度 XY-MD02，这条 06 写命令会误伤它。 */
    if(at01 && !at02)
    {
        if(Modbus_SetLightAddr02())
        {
            vTaskDelay(pdMS_TO_TICKS(200));
            at02 = Modbus_Read_Dev(0x02, 0x03, 0x0002, 2, buf) ? 1 : 0;
            fixed = 1;
        }
    }
#endif
    g_light_probe = (at01 ? 1u : 0u) | (at02 ? 2u : 0u) | (fixed ? 4u : 0u) | (g_rs485_invert ? 8u : 0u);
}

void Modbus_Poll_AllDev(void)
{
    uint16_t tmp_buf[8];
    for(uint8_t i=0;i<dev_cnt;i++)		//dev_cnt数组元素个数
    {
        ModDev_t *p = &dev_list[i];
			 memset(tmp_buf, 0, sizeof(tmp_buf)); // 新增：每次读取前清空缓存
        //连续错误超限，跳过本次轮询
//        if(p->err_cnt >= MAX_ERR_CNT)
//        {
//            continue;
//        }

        //发起读取
        uint8_t ok = Modbus_Read_Dev(p->dev_addr,p->func,p->reg_start,p->reg_num,tmp_buf);

        /* ===== 失败重试一次（极性不翻转） =====
           ⚠ 之前这里写的是"翻转 g_rs485_invert 再试"，已经删掉 —— 那是错的。
           实测的这块 SP3485 模块（D:\Desktop\sp3485mod.pdf 电路图）：
             EN(pin2 RE + pin3 DE 并联) 经 R5 10K 下拉到 GND，
             所以 **EN 高 = 发送，EN 低 = 接收**，极性是固定的，不存在"反相版本"。
           而翻转成 invert=1 之后，RS485_RX 会把 EN 顶成高电平 ——
           本该收数据的整段时间里，模块一直占着 A/B 总线（DI 被 MCU 的 TX 拉成空闲高），
           传感器根本没有机会应答，症状就是 rx:0 且永远不会自愈。
           所以这里只做同极性重试（丢帧重发），不动方向极性。 */
        /* 只对温湿度重试（光照本来就没人应答，重试纯浪费 400ms）。
           重试前留 50ms 间隔：万一一帧的应答只是"来得晚"，紧接着重发会和那条
           迟到的应答在总线上撞车，双方都收不干净 —— 会从一个偶发丢帧
           演变成连续一串失败。 */
        if(ok != 1 && p->func == 0x04)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            ok = Modbus_Read_Dev(p->dev_addr,p->func,p->reg_start,p->reg_num,tmp_buf);
        }

        if(ok == 1)
        {
         //   p->err_cnt = 0; //成功清零错误计数
            //根据从机地址解析数据
            switch(p->func)
            {
                case 0x04: //温湿度（实测：01 04 04 01 07 01 71 → 26.3℃/36.7%）
                    /* 改进（GPT 审计）：温度用有符号解析，负温(如-5℃=0xFFCE)才不会变成6553 */
                    p->val1 = (int16_t)tmp_buf[0]/10.0f;
                    p->val2 = tmp_buf[1]/10.0f;
                    /* 关键修复：probe 只在开机跑一次，g_th_raw 会永远冻在开机那帧。
                       这里在每次轮询成功时同步刷新，让巴法云 th 字段反映最新实时读数，
                       不再出现"th:FAIL 但 rx:9/ok:5 一直在成功"的自相矛盾。 */
                    g_th_raw[0] = tmp_buf[0];
                    g_th_raw[1] = tmp_buf[1];
                    g_th_ok = 1;
                    g_th_tick = HAL_GetTick();   /* 记录最近一次成功的时刻 */
                    break;   /* 修复：原缺 break，会贯穿到 case 0x03 把温度算成 ((263<<16|369)/1000)=17236 */
                case 0x03: //光照
                case 0x02: //光照
                    /* 把两个寄存器的原始值也存下来（云端 lr 字段），
                       用来核对"除数到底是 100 还是 1000"：
                       同一个原始值，÷100 和 ÷1000 差 10 倍，肉眼一比就知道。 */
                    g_light_raw = ((uint32_t)tmp_buf[0]<<16) | tmp_buf[1];
                    p->val1 = g_light_raw/1000.0f;
                    p->val2 = 0;
                    break;
//                case 0x03: //继电器状态
//                    p->val1 = tmp_buf[0];
//                    p->val2 = 0;
//                    break;
                default:break;
            }
        }
        else
        {
            /* 温湿度读取失败时立刻把在线标志清零，
               否则传感器掉线后云端仍一直上报 "th":"OK"（诊断字段失真） */
            if(p->func == 0x04)
            {
                g_th_ok = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(30)); //单从机间隔防总线干扰					重点时间不能过大！！！！！！！
    }
}




/*//将采集到的数据赋值给结构体 然后写入队列*/
void Modbus_Upload_Json(void)
{
    Data_t uidata;

    /* 按 Modbus 地址匹配，修复原索引错位（光照值显示到温度卡片） */
    uidata.temp = 0; uidata.shi = 0; uidata.light = 0;
    for(uint8_t i = 0; i < dev_cnt; i++)
    {
        if(dev_list[i].func == 0x04){          /* 温湿度（功能码04，两传感器地址均0x01） */
            uidata.temp = dev_list[i].val1;
            uidata.shi  = dev_list[i].val2;
        }else if(dev_list[i].func == 0x03){    /* 光照（功能码03） */
            uidata.light = dev_list[i].val1;
        }
    }
    xQueueOverwrite(data_queue, &uidata);

    /* 历史记录：每 10 秒一条，与在线/离线无关，是完整时间序列。
       未校时（epoch 拿不到真实时间）不写，宁可缺一条也不写错时间。 */
    {
        static uint32_t last_hist = 0;
        uint32_t now = HAL_GetTick();
        uint32_t ep  = 0;

        if ((now - last_hist) >= 10000u && RTC_GetEpoch(&ep) && ep != 0)
        {
            last_hist = now;
            Hist_Write(ep, uidata.temp, uidata.shi, uidata.light);
        }
    }
}


extern QueueHandle_t lvgl_data_queue;   

void MQTT_SendAllDev_NoWait(void)
{
		Data_t UIdat = {0};
	
    /* 纯 JSON：不再手工把引号转义成 \"、也不再自己拼 AT 命令 ——
       转义与 AT 语法全部交给 pal.c 的 MQTT_Publish() 统一处理。 */
    char payload[256]={0};
    int p=sprintf(payload,"{");
    for(uint8_t i=0;i<dev_cnt;i++)
    {
        ModDev_t *pt=&dev_list[i];
        if(pt->func == 0x04 )
        {
            //01温湿度：温度、湿度
            p += sprintf(payload+p,"\"temp\":%.1f,\"humi\":%.1f,",pt->val1,pt->val2);
						UIdat.temp=pt->val1;
						UIdat.shi=pt->val2;
        }
        else if(pt->func == 0x03)
        {
            //02光照
            p += sprintf(payload+p,"\"light\":%.0f,",pt->val1);
						UIdat.light=pt->val1;
        }
//        else if(pt->dev_addr == 0x03)
//        {
//            //03继电器开关
//            p += sprintf(payload+p,"\"dev3\":{\"state\":%.0f},",pt->val1);
//        }
    }
    /* 追加实时诊断字段（每轮都发，解决"probe 只在上线发一次导致 th 字段冻在开机快照"的问题）。
       用户可每 2 秒在巴法云看到 th/rx/ok 的最新状态，不再被旧 probe 误导。 */
    extern volatile uint8_t g_rx_count;
    extern volatile uint32_t g_ok_count;
    extern volatile uint32_t g_th_tick;
    /* age = 距上一次温湿度读取成功过了多少秒。
       age 一直涨 = 数据是旧的（云端显示的 temp/humi 只是残值，不是实时值）；
       age 归零   = 刚刚真正读到。
       inv = 485 当前方向极性（0 标准 / 1 反相），烧不同接法的模块时一眼能看出来。
       注意：th 的值必须带引号（合法 JSON 字符串），否则整条 JSON 解析失败被巴法云丢弃 */
    /* 告警位：bit0 温度高 / bit1 温度低 / bit2 湿度高 / bit3 湿度低 */
    {
        uint8_t alm = 0;
        if (UIdat.temp > ALM_TEMP_HIGH) alm |= 0x01;
        if (UIdat.temp < ALM_TEMP_LOW)  alm |= 0x02;
        if (UIdat.shi  > ALM_HUMI_HIGH) alm |= 0x04;
        if (UIdat.shi  < ALM_HUMI_LOW)  alm |= 0x08;
        p += sprintf(payload + p, "\"alm\":%u,", (unsigned)alm);
    }

    p += sprintf(payload+p,"\"th\":\"%s\",\"rx\":%u,\"rxl\":%u,\"ok\":%u,\"age\":%u,\"inv\":%u,\"txf\":%u,\"lp\":%u,\"lr\":%u}",
                 g_th_ok ? "OK" : "FAIL", (unsigned)g_rx_count, (unsigned)g_rx_light,
                 (unsigned)g_ok_count,
                 (unsigned)((HAL_GetTick() - g_th_tick) / 1000u),
                 (unsigned)g_rs485_invert, (unsigned)g_tx_fail,
                 (unsigned)g_light_probe, (unsigned)g_light_raw);

    /* 时间戳：SNTP 校时成功后才填，没校过时留空字符串。
       绝不能把 2000-01-01 当成真实时间发给云端 —— 那比没有时间戳更糟，
       因为看起来"有时间"，其实全错。 */
    {
        char tsbuf[24];
        if (RTC_GetDateTimeStr(tsbuf, sizeof(tsbuf)) == 0)
            tsbuf[0] = '\0';
        p += sprintf(payload + p, ",\"ts\":\"%s\"}", tsbuf);
    }

		xQueueOverwrite(lvgl_data_queue, &UIdat);	//给ui写队列 覆盖写

    /* 发布（主题 / QoS / 转义都在 MQTT_Publish 内部）。
       失败只统计、不动作：
       曾经这里连续 3 次失败就置 g_mqtt_connected=0 触发重连，结果非常糟 ——
       AT 任务的重连第一步是 ESP_HardReset()，会把 ESP8266 直接硬复位、TCP 立刻断，
       broker 那边看到的就是"设备突然掉线"（巴法云订阅者从 2 变 1）。
       而"发布失败"并不等于"链路断了" —— 固件在忙的时候回 busy p...，
       命令被拒但连接还好好的，用它去触发硬复位等于自己把自己踢下线。
       现在改为：失败只累加到 g_pub_fail_cnt 供遥测上报，
       真正的断线交给固件的 +MQTTDISCONNECTED 主动上报去发现。 */
    extern volatile uint8_t g_pub_fail_cnt;
    if (MQTT_Publish(payload, 8000) == ESP_OK)
    {
        g_pub_fail_cnt = 0;
    }
    else if (g_pub_fail_cnt < 255)
    {
        g_pub_fail_cnt++;
    }
    vTaskDelay(10);
}

/*																	USART2
手动轮询RXNE：不依赖HAL库的阻塞接收或中断接收，零延迟读取每个字节。

双重超时保护：50ms帧间隔（Modbus标准3.5字符/4ms）和800ms绝对超时，保证不卡死。

vTaskDelay(1)：在FreeRTOS下主动让出CPU，保证UI和MQTT任务不被饿死。

没有开启串口中断接收 检查状态寄存器 有数据就读DR 读完后状态会变为0
*/
int Modbus_Get_Temp(float *temp,float *shi)
{
	uint8_t tx[8];
	uint8_t rx[20];
	uint8_t count=0;
	uint16_t crc=0;
	
	/*问询帧*/
	tx[0]=0x01;
	tx[1]=0x03;
	tx[2]=0x00;
	tx[3]=0x01;
	tx[4]=0x00;
	tx[5]=0x02;
	
	/*小端排序 低字节在前 （低字节在低地址 ，低地址默认在前）*/
	crc=CRC_16_MODBUS(tx,6,0);
	tx[6]=crc &0xff;
	tx[7]=(crc>>8)&0xff;
	
	memset(rx,0,sizeof(rx));
	RS485_TX;
	HAL_UART_Transmit(&huart2,tx,sizeof(tx),1000);			//发送问询帧
	vTaskDelay(1);
	
	RS485_RX;				//切换为接收模式
	
	uint32_t start_time=HAL_GetTick();
	uint32_t up_time=HAL_GetTick();
	
	while(1)
	{
		if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_RXNE))
		{
			uint8_t ch=huart2.Instance->DR &0xff;		//DR寄存器有数据时 状态寄存器为1  读取DR寄存器的值 状态清零
			if(count<sizeof(rx))
			{
				rx[count++]=ch;
				up_time=HAL_GetTick();
			}
		}
		if(count>0 && (HAL_GetTick()-up_time)>50)					//帧与帧间隔3.5个字符 4ms
		{
			break;
		}
		if(HAL_GetTick()-start_time>800)										//绝对超时 不卡死
		{
			break;
		}
		vTaskDelay(1);						//释放CPU防止其他任务饿死
	}
	
	uint16_t crc_re=CRC_16_MODBUS(rx,count-2,1);
	if(count==9 && rx[0]==0x01 && rx[1]==0x03 && rx[2]==0x04 && rx[7]==(crc_re& 0xff) && rx[8]==((crc_re>>8)& 0xff))
	{
		uint16_t t=(rx[3]<<8)|rx[4];
		uint16_t s=(rx[5]<<8)|rx[6];
		*temp=t/10.0;
		*shi=s/10.0;
		return 1;
	}
	return 0;
}
