/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.h
  * @brief   This file contains all the function prototypes for
  *          the rtc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RTC_H__
#define __RTC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_RTC_Init(void);

/* USER CODE BEGIN Prototypes */

/* ===== 时间戳支持（2026-11-21）=====
   本板 RTC 无备份电池，MX_RTC_Init() 每次上电都把时间写成 2000-01-01，
   所以"时间是否可信"必须单独标记，不能直接拿 RTC 当时间戳。
   真实时间由 ESP8266 的 SNTP 提供（pal.c -> ESP_SyncTime）。 */
extern volatile uint8_t g_time_valid;

void    RTC_SetDateTime(uint16_t year, uint8_t mon, uint8_t day,
                        uint8_t hour, uint8_t min, uint8_t sec, uint8_t week);
uint8_t RTC_GetDateTimeStr(char *buf, uint16_t len);   /* 1=已校时 0=未校时 */
/* 取 Unix 时间戳（秒）。返回 1 = 已校时，0 = 时间不可信（此时 *epoch 不写）。 */
uint8_t RTC_GetEpoch(uint32_t *epoch);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __RTC_H__ */

