/**
  * @file    app_fp_atr_sigfox1.c
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
#ifndef __APP_FP_ATR_SIGFOX1_H
#define __APP_FP_ATR_SIGFOX1_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "s2lp.h"
#include "RTE_Components.h"
#include "sfx_config.h"
#include "nvm_api.h"
#ifdef USE_BLE
#include "hci_tl_interface.h"
#endif
#ifdef USE_GNSS
#include "cmsis_os.h"
#endif

/** @addtogroup Applications
  * @{
  */

/** @addtogroup Sigfox_Asset_Tracker
  * @{
  */

/** @addtogroup STM32L4
  * @{
  */

/** @addtogroup Sigfox_Asset_Tracker_Exported_Defines
  * @{
  */

/* Exported Defines --------------------------------------------------------*/
/** Maximum supported number of user-defined thresholds */
#define MAX_THRESHOLDS 8

/** Default value of Sigfox send message frequency in minutes */
#define DEFAULT_SIGFOX_SEND_FREQUENCY 15

/** Default value of sensor read frequency in minutes */
#define DEFAULT_SENSOR_READ_FREQUENCY 1

/**
  * @}
  */

/** @addtogroup Sigfox_Asset_Tracker_Exported_Types
  * @{
  */

/* Public types --------------------------------------------------------*/
typedef enum { TEMP_ID=1, PRES_ID=2, HUM_ID=3, WAKE_ID=4, TILT_ID=5, ORIENT_ID=6 } sensor_id_type;
typedef enum { LESS_FUNC=-1, EQUAL_FUNC=0, GREATER_FUNC=1 } function_type;
typedef struct
{
  sensor_id_type sensor_id;
  function_type function;
  uint16_t value;
} threshold_type;

typedef struct
{
  int16_t temperature;
  int16_t pressure;
  int16_t humidity;

  int32_t latitude;
  int32_t longitude;
} sensor_value_type;

typedef enum { axis_none, axis_xl, axis_xh, axis_yl, axis_yh, axis_zl, axis_zh } orientation_axis;
typedef enum { Trig_None=0, Trig_Timer =1<<0, Trig_Button=1<<1, Trig_Orientation=1<<2, Trig_Tilt=1<<3, Trig_WakeUp=1<<4,
               Trig_Humidity=1<<5, Trig_Pressure=1<<6, Trig_Temperature=1<<7 } Trigger_Flag;

/* Exported Variables -------------------------------------------------------*/

/**
  * @}
  */

/** @addtogroup Sigfox_Asset_Tracker_Exported_Functions
  * @{
  */

/* Exported Functions --------------------------------------------------------*/
void MX_FP_ATR_SIGFOX1_Init(void);
#if (osCMSIS < 0x20000U)
void MX_FP_ATR_SIGFOX1_Process(void const * argument);
#else
void MX_FP_ATR_SIGFOX1_Process(void *argument);
#endif /* osCMSIS */

#ifdef USE_GNSS
#if (osCMSIS < 0x20000U)
void TeseoConsumerTask(void const * argument);
#else
void TeseoConsumerTask(void *argument);
#endif /* osCMSIS */
#endif /* USE_GNSS */

int check_thresholds(int thresholds, threshold_type threshold[],sensor_value_type current_sensor_value,
                     uint8_t *trigger_mask);
int parse_setTracking_command(char* command, int lcommand, int* sigfox_send_frequency, int* sensors_read_frequency,
                              threshold_type threshold[]);
int build_getTracking_reply(char* command, int sigfox_send_frequency, int sensors_read_frequency,
                            threshold_type threshold[], int num_thresholds);
void check_thresholds_reset(void);

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
#endif /* __APP_FP_ATR_SIGFOX1_H */
