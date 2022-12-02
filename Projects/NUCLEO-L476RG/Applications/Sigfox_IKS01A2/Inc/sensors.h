/**
  ******************************************************************************
  * @file    sensors.h
  * @author  STM32ODE Team, Italy
  * @version 3.2.0
  * @date    1-September-2022
  * @brief   This is file contains prototypes for sensors.c
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

#ifndef _SENSORS_H_
#define _SENSORS_H_

#include "app_fp-atr-sigfox1.h"

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

void  sensors_init( uint8_t );
void  sensors_deinit( uint8_t );
void  sensors_enable( uint8_t );
void  sensors_disable( uint8_t );
void sensors_read( sensor_value_type* );
int check_orientation(orientation_axis axis);

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

#endif /* _SENSORS_H_ */
