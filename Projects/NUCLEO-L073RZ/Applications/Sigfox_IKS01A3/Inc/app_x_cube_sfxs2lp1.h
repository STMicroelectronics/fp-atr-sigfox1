
/**
  * @file    app_fp_atr_sigfox1.h
  * @author  STM32ODE Team
  * @version 3.2.0
  * @date    1-September-2022
  * @brief   This file contains the main functions headers for FP-ATR-SIGFOX1
  *          STM32Cube function pack
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
#ifndef __APP_X_CUBE_SFXS2LP1_H
#define __APP_X_CUBE_SFXS2LP1_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "s2lp.h"
#include "stm32l0xx_hal.h"
#include "stm32l0xx.h"
#include "stm32l0xx_it.h"
#include "stm32l0xx_nucleo.h"
#include "s2868a1.h"
#include "RTE_Components.h"
#include "sfx_config.h"
#include "nvm_api.h"

/** @addtogroup Applications
  * @{
  */

/** @addtogroup Sigfox_Asset_Tracker
  * @{
  */

/** @addtogroup STM32L0
  * @{
  */

/** @addtogroup Sigfox_Asset_Tracker_Exported_Defines
  * @{
  */

/* Exported Defines --------------------------------------------------------*/
#define RADIO_DEV_VER_REG   (0xF1)

/**
  * @}
  */

/** @addtogroup Sigfox_Asset_Tracker_Exported_Variables
  * @{
  */

/* Exported Variables -------------------------------------------------------*/
extern uint8_t but_pressed;

/**
  * @}
  */

/** @addtogroup Sigfox_Asset_Tracker_Exported_Functions
  * @{
  */

/* Exported Functions --------------------------------------------------------*/
void MX_X_CUBE_SFXS2LP1_Init(void);

void LedBlink(uint8_t times);
void BSP_PB_Callback(Button_TypeDef Button);
void Set_KeyStatus(FlagStatus val);
void Fatal_Error(void);

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
#endif /* __APP_X_CUBE_SFXS2LP1_H*/
