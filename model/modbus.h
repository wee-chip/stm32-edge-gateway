#ifndef MODBUS_H
#define MODBUS_H
#include "stdint.h"
#include "stm32f4xx.h"

extern volatile uint8_t g_rs485_invert;   /* 0=EN高发送(标准) 1=EN低发送(反相模块) */
/* 模块原理图确认：EN = /RE+DE 短接，R5 10K 下拉。EN 高=发送 / EN 低=接收 */
#define RS485_TX  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, g_rs485_invert ? GPIO_PIN_RESET : GPIO_PIN_SET)
#define RS485_RX  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, g_rs485_invert ? GPIO_PIN_SET : GPIO_PIN_RESET)
/* 正常版本（测试后恢复）：
#define RS485_TX  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, g_rs485_invert ? GPIO_PIN_RESET : GPIO_PIN_SET)
#define RS485_RX  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, g_rs485_invert ? GPIO_PIN_SET : GPIO_PIN_RESET)
*/



uint16_t CRC_16_MODBUS(uint8_t *data,uint8_t len,uint8_t flag);






//单从机配置结构体
typedef struct
{
    uint8_t  dev_addr;    //从机地址
    uint8_t  func;       //功能码
    uint16_t reg_start;  //起始寄存器
    uint16_t reg_num;    //寄存器个数
    uint16_t err_cnt;    //连续失败次数
    float    val1;       //数据1
    float    val2;       //数据2
}ModDev_t;

/* ===== 阈值告警 =====
   位定义：bit0 温度过高 / bit1 温度过低 / bit2 湿度过高 / bit3 湿度过低

   默认区间取工业车间/机房通用值：温度 10~35℃、湿度 20~85%RH。
   参考：GB/T 50174-2017 机房 A 级为 18~27℃ / 35%~75%；
         IEC 60721-3-3 3K3 类为 5~40℃ / 5%~85%（无凝露）。
   运行期可用指令覆盖：{"cmd":"setth","tmax":28,"tmin":18,"hmax":70,"hmin":40} */
#define ALM_TEMP_HIGH   35.0f
#define ALM_TEMP_LOW    10.0f
#define ALM_HUMI_HIGH   85.0f
#define ALM_HUMI_LOW    20.0f

/* 回差：越限置位后，需回到阈值内 1℃ / 5%RH 才清除，避免在阈值附近抖动 */
#define ALM_HYST_T       1.0f
#define ALM_HYST_H       5.0f

void Modbus_SetAlarm(float t_hi, float t_lo, float h_hi, float h_lo);

extern ModDev_t dev_list[];
extern uint8_t dev_cnt;

//通用读寄存器接口（替换原来固定地址Modbus_Get_Temp）
uint8_t Modbus_Read_Dev(uint8_t addr,uint8_t func,uint16_t reg,uint16_t regcnt,uint16_t *buf);

//整表轮询入口
void Modbus_Poll_AllDev(void);

void Modbus_Upload_Json(void);

void MQTT_SendAllDev_NoWait(void);

uint8_t Modbus_SetLightAddr02(void);   /* 临时：光照改地址0x02 */

void Modbus_ProbeLight(void);          /* 上电探测光照0x02是否在线 */

#endif
