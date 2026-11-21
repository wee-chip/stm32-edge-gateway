#ifndef COMMAND_H
#define COMMAND_H

#include "main.h"
#include <string.h>
#include "usart.h"
#include "stm32f4xx.h"
#include "stdio.h"

#define BUFFER_SIZE 256
#define COMMAND_MIN_LENGTH 4
/* length 必须是 uint16_t：DMA 满 256 字节时若用 uint8_t 会被截断成 0，整帧丢失 */
uint8_t Command_Write(uint8_t *data, uint16_t length);

uint8_t Command_GetCommand(uint8_t *command);
uint8_t RingBuf_Read(uint8_t *out) ;
void RingBuf_Clear(void);
#endif /* INC_COMMAND_H_ */
