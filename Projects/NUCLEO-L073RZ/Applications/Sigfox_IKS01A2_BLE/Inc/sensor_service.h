/**
  ******************************************************************************
  * @file    sensor_service.h
  * @author  STM32ODE Team
  * @version 3.2.0
  * @date    1-September-2022
  * @brief   Header file for sensor_service.c
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
#ifndef _SENSOR_SERVICE_H_
#define _SENSOR_SERVICE_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_gap.h"
#include "string.h"
#include "hci_const.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_aci_const.h"
#include "hci.h"
#include "hci_le.h"
#include "sm.h"
#include <stdlib.h>

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

/* Exported defines ----------------------------------------------------------*/
#define IDB04A1 0
#define IDB05A1 1

#define W2ST_CONSOLE_MAX_CHAR_LEN 20

/**
 * @}
 */

/** @addtogroup Sigfox_Asset_Tracker_Exported_Types
 *  @{
 */
typedef int i32_t;

/**
 * @brief Structure containing acceleration value (in mg) of each axis.
 */
typedef struct {
  i32_t AXIS_X;
  i32_t AXIS_Y;
  i32_t AXIS_Z;
} AxesRaw_t;

/**
 * @}
 */

/** @addtogroup Sigfox_Asset_Tracker_Exported_Functions
 *  @{
 */

tBleStatus Add_Acc_Service(void);
tBleStatus Acc_Update(AxesRaw_t *data);
tBleStatus Add_Environmental_Sensor_Service(void);
tBleStatus Free_Fall_Notify(void);
tBleStatus Temp_Update(int16_t temp);
tBleStatus Press_Update(int32_t press);
tBleStatus Humidity_Update(uint16_t humidity);
void       Read_Request_CB(uint16_t handle);
void       setConnectable(void);
void       GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
void       GAP_DisconnectionComplete_CB(void);
void       user_notify(void * pData);

tBleStatus Term_Update(uint8_t *data,uint8_t length);
tBleStatus Add_ConsoleW2ST_Service(void);

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

#endif /* _SENSOR_SERVICE_H_ */
