/**
  ******************************************************************************
  * @file    app_mems_int_pin_interface.c
  * @author  MEMS Application Team
  * @version 3.2.0
  * @date    1-September-2022
  * @brief   This file contains the MEMS INT pin A interface
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_mems_int_pin_interface.h"

/** @addtogroup Applications
  * @{
  */

/** @addtogroup Sigfox_Asset_Tracker
  * @{
  */

/** @addtogroup STM32L4
  * @{
  */

/** @addtogroup Sigfox_Asset_Tracker_Variables
  * @{
  */

/* Private variables ---------------------------------------------------------*/
EXTI_HandleTypeDef hexti5 = {.Line = EXTI_LINE_5};
extern volatile uint8_t MemsEventDetected;

/**
  * @}
  */

/** @addtogroup Sigfox_Asset_Tracker_Functions
  * @{
  */

/* Private function prototypes -----------------------------------------------*/
static void mems_int_pin_hardware_event_isr(void);

/**
  * @brief  Set the EXTI interrupt callback
  * @retval None
  */
void set_mems_int_pin_exti(void)
{
  /* register event irq handler */
  HAL_EXTI_GetHandle(&hexti5, EXTI_LINE_5);
  HAL_EXTI_RegisterCallback(&hexti5, HAL_EXTI_COMMON_CB_ID, mems_int_pin_hardware_event_isr);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/**
  * @brief  EXTI interrupt callback
  * @retval None
  */
static void mems_int_pin_hardware_event_isr(void)
{
  MemsEventDetected = 1;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
