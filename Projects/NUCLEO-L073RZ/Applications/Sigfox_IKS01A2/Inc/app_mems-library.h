/**
  ******************************************************************************
  * @file    app_mems-library.h
  * @author  STM32ODE Team
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
#ifndef __INIT_H
#define __INIT_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported Functions --------------------------------------------------------*/

/** @addtogroup Applications
  * @{
  */

/** @addtogroup Sigfox_Asset_Tracker
  * @{
  */

/** @addtogroup STM32L0
  * @{
  */

/** @addtogroup Sigfox_Asset_Tracker_Exported_Functions
  * @{
  */

void MX_MEMS_Library_Init(void);
void MX_MEMS_Library_Process(void);

void MX_Accelero_Init(void);
void MX_Accelero_WakeUp_Init(int thresh);
void MX_Accelero_Tilt_Init(void);
void MX_Accelero_Orientation_Init(void);
void MX_Accelero_WakeUp_DeInit(void);
void MX_Accelero_Tilt_DeInit(void);
void MX_Accelero_Orientation_DeInit(void);

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
#endif /* __INIT_H */
