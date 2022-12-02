/**
  * @file    app_bluenrg-ms.h
  * @author  STM32 ODE Team
  * @version 3.2.0
  * @date    1-September-2022
  * @brief   Bluetooth top functions header file
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
#ifndef APP_BLUENRG_MS_H
#define APP_BLUENRG_MS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/** @addtogroup Applications
  * @{
  */

/** @addtogroup Sigfox_Asset_Tracker
  * @{
  */

/** @addtogroup STM32L4
  * @{
  */

/** @addtogroup Sigfox_Asset_Tracker_Exported_Functions
  * @{
  */

/* Exported Functions --------------------------------------------------------*/
void MX_BlueNRG_MS_Init(void);
void MX_BlueNRG_MS_Process(void);
void ble_switch_off(void);

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
#endif /* APP_BLUENRG_MS_H */
