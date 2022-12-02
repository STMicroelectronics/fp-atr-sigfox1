/**
  ******************************************************************************
  * @file    app_mems_int_pin_interface.h
  * @author  MEMS Application Team
  * @version 3.2.0
  * @date    1-September-2022
  * @brief   This file provides code for the configuration
  *          of the STMicroelectronics.MEMS-Library instances.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_MEMS_INT_PIN_INTERFACE_H__
#define __APP_MEMS_INT_PIN_INTERFACE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup Applications
  * @{
  */

/** @addtogroup Sigfox_Asset_Tracker
  * @{
  */

/** @addtogroup STM32L4
  * @{
  */

/** @addtogroup Sigfox_Asset_Tracker_Exported_Variables
  * @{
  */

extern EXTI_HandleTypeDef hexti5;
#define H_EXTI_5 hexti5

/**
 * @}
 */

/** @addtogroup Sigfox_Asset_Tracker_Exported_Functions
  * @{
  */

void set_mems_int_pin_exti(void);

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

#ifdef __cplusplus
}
#endif

#endif /* __APP_MEMS_INT_PIN_INTERFACE_H__*/
