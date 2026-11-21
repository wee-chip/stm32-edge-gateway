/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
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
/* Includes ------------------------------------------------------------------*/
#include "rtc.h"
#include <stdio.h>   /* snprintf：时间字符串 */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();

    /* RTC interrupt Init */
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();

    /* RTC interrupt Deinit */
    HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* ================= 时间戳支持（2026-11-21 新增）=================
   背景：本板 RTC **没有备份电池**，而且 MX_RTC_Init() 每次上电都把时间强制
   写成 2000-01-01 00:00:00。所以在拿到真实时间之前，RTC 的值不可信 ——
   直接拿它当时间戳，云端看到的永远是"2000年"，比没有还糟。

   真实时间来自 ESP8266 的 SNTP（见 pal.c 的 ESP_SyncTime）：
   联网后读到 "+CIPSNTPTIME:Sat Nov 21 08:37:00 2026" 再写进这里。
   g_time_valid 标记"这次的时间到底是不是真的"，JSON 里靠它决定 ts 字段是否为空。 */

volatile uint8_t g_time_valid = 0;

#define RTC_BCD(x)  ((((x) / 10U) << 4) | ((x) % 10U))

void RTC_SetDateTime(uint16_t year, uint8_t mon, uint8_t day,
                     uint8_t hour, uint8_t min, uint8_t sec, uint8_t week)
{
    RTC_TimeTypeDef t = {0};
    RTC_DateTypeDef d = {0};

    t.Hours   = RTC_BCD(hour);
    t.Minutes = RTC_BCD(min);
    t.Seconds = RTC_BCD(sec);
    t.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    t.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &t, RTC_FORMAT_BCD) != HAL_OK)
        return;

    d.Year    = RTC_BCD((uint8_t)(year % 100U));
    d.Month   = RTC_BCD(mon);
    d.Date    = RTC_BCD(day);
    d.WeekDay = week;
    if (HAL_RTC_SetDate(&hrtc, &d, RTC_FORMAT_BCD) != HAL_OK)
        return;

    g_time_valid = 1;
}

/* 取当前时间字符串："2026-11-21 08:40:12"
   返回 1 = 已校时，0 = 还没校过（内容仍是 2000-01-01，调用方应视为无效）
   ⚠ F4 的 RTC 必须先读 Time 再读 Date：只读 Date 不会解锁影子寄存器，
     读到的是上一次锁存的值。 */
uint8_t RTC_GetDateTimeStr(char *buf, uint16_t len)
{
    RTC_TimeTypeDef t;
    RTC_DateTypeDef d;

    if (buf == NULL || len == 0)
        return 0;

    HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);

    snprintf(buf, len, "20%02u-%02u-%02u %02u:%02u:%02u",
             (unsigned)d.Year, (unsigned)d.Month, (unsigned)d.Date,
             (unsigned)t.Hours, (unsigned)t.Minutes, (unsigned)t.Seconds);

    return g_time_valid;
}

/* Unix 时间戳。历史记录区存 16 字节定长记录，时间用 epoch 数字比字符串省一半。 */
uint8_t RTC_GetEpoch(uint32_t *epoch)
{
    RTC_TimeTypeDef t;
    RTC_DateTypeDef d;
    int y, era, yoe, doy, doe;

    if (epoch == NULL) return 0;

    HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);

    y = 2000 + (int)d.Year;

    /* days from civil（Howard Hinnant 算法），返回 1970-01-01 起的天数 */
    y -= (d.Month <= 2);
    era = (y >= 0 ? y : y - 399) / 400;
    yoe = y - era * 400;
    doy = (153 * (d.Month + (d.Month > 2 ? -3 : 9)) + 2) / 5 + d.Date - 1;
    doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;

    *epoch = (uint32_t)((era * 146097 + doe - 719468) * 86400
                        + t.Hours * 3600 + t.Minutes * 60 + t.Seconds);

    return g_time_valid;
}

/* USER CODE END 1 */
