/**
  * @file    app_mems-library.c
  * @author  STM32ODE Team
  * @version 3.2.0
  * @date    1-September-2022
  * @brief   This file provides code for the configuration
  *          of the STMicroelectronics.MEMS-Library.0.0.1 instances.
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
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_mems-library.h"
#include "app_mems_int_pin_interface.h"
#include "main.h"
#include "app_x_cube_sfxs2lp1.h"

#ifdef USE_IKS01A2
#include "iks01a2_motion_sensors.h"
#include "iks01a2_motion_sensors_ex.h"
#endif

#ifdef USE_IKS01A3
#include "iks01a3_motion_sensors.h"
#include "iks01a3_motion_sensors_ex.h"
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

/** @addtogroup Sigfox_Asset_Tracker_Defines
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define INDICATION_DELAY  100 /* LED is ON for this period [ms]. */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint8_t PushButtonDetected = 0;

/**
  * @}
  */

/** @addtogroup Sigfox_Asset_Tracker_Functions
  * @{
  */

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  Initialize the MEMS application
  * @retval None
  */
void MX_MEMS_Library_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN MEMS_Library_Init_PreTreatment */

  /* USER CODE END MEMS_Library_Init_PreTreatment */

  /* Initialize the peripherals and the MEMS components */

  MX_Accelero_Init();

  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN MEMS_Library_Init_PostTreatment */

  /* USER CODE END MEMS_Library_Init_PostTreatment */
}

/**
  * @brief  Initialize the LSM6DSL Wake Up Detection application
  * @retval None
  */
void MX_Accelero_Init(void)
{
  set_mems_int_pin_exti();

#ifdef USE_IKS01A2
  if (IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO) == BSP_ERROR_NONE)
  {
    printf("LSM6DSL accelerometer initialized\r\n");
  }
#endif
#ifdef USE_IKS01A3
  if (IKS01A3_MOTION_SENSOR_Init(IKS01A3_LSM6DSO_0, MOTION_ACCELERO | MOTION_GYRO) == BSP_ERROR_NONE)
  {
    printf("LSM6DSO accelerometer initialized\r\n");
  }
#endif
  else
  {
    printf("Accelerometer NOT initialized\r\n");
    Fatal_Error();
  }

  MX_Accelero_WakeUp_DeInit();
  MX_Accelero_Tilt_DeInit();
  MX_Accelero_Orientation_DeInit();
}

void MX_Accelero_WakeUp_Init(int thresh)
{
#ifdef USE_IKS01A2
  if (IKS01A2_MOTION_SENSOR_Enable_Wake_Up_Detection(IKS01A2_LSM6DSL_0, IKS01A2_MOTION_SENSOR_INT1_PIN) == BSP_ERROR_NONE)
  {
    printf("Wake-up detection enabled\r\n");

    IKS01A2_MOTION_SENSOR_SetFullScale(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, 16);
    IKS01A2_MOTION_SENSOR_Set_Wake_Up_Threshold(IKS01A2_LSM6DSL_0, thresh);
  }
#endif
#ifdef USE_IKS01A3
  if (IKS01A3_MOTION_SENSOR_Enable_Wake_Up_Detection(IKS01A3_LSM6DSO_0, IKS01A3_MOTION_SENSOR_INT1_PIN) == BSP_ERROR_NONE)
  {
    printf("Wake-up detection enabled\r\n");

    IKS01A3_MOTION_SENSOR_SetFullScale(IKS01A3_LSM6DSO_0, MOTION_ACCELERO, 16);
    IKS01A3_MOTION_SENSOR_Set_Wake_Up_Threshold(IKS01A3_LSM6DSO_0, thresh);
  }
#endif
  else
  {
    printf("Wake-up detection NOT enabled\r\n");
    Fatal_Error();
  }
}

void MX_Accelero_Tilt_Init(void)
{
#ifdef USE_IKS01A2
  if (IKS01A2_MOTION_SENSOR_Enable_Tilt_Detection(IKS01A2_LSM6DSL_0, IKS01A2_MOTION_SENSOR_INT1_PIN) == BSP_ERROR_NONE)
  {
    printf("Tilt detection enabled\r\n");
    IKS01A2_MOTION_SENSOR_SetFullScale(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, 16);
  }
#endif
#ifdef USE_IKS01A3
  if (IKS01A3_MOTION_SENSOR_Enable_Tilt_Detection(IKS01A3_LSM6DSO_0, IKS01A3_MOTION_SENSOR_INT1_PIN) == BSP_ERROR_NONE)
  {
    printf("Tilt detection enabled\r\n");
    IKS01A3_MOTION_SENSOR_SetFullScale(IKS01A3_LSM6DSO_0, MOTION_ACCELERO, 16);
  }
#endif
  else
  {
    printf("Tilt detection NOT enabled\r\n");
    Fatal_Error();
  }
}

void MX_Accelero_Orientation_Init(void)
{
#ifdef USE_IKS01A2
  if (IKS01A2_MOTION_SENSOR_Enable_6D_Orientation(IKS01A2_LSM6DSL_0, IKS01A2_MOTION_SENSOR_INT1_PIN) == BSP_ERROR_NONE)
  {
    printf("Orientation detection enabled\r\n");
    IKS01A2_MOTION_SENSOR_SetFullScale(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, 16);
  }
#endif
#ifdef USE_IKS01A3
  if (IKS01A3_MOTION_SENSOR_Enable_6D_Orientation(IKS01A3_LSM6DSO_0, IKS01A3_MOTION_SENSOR_INT1_PIN) == BSP_ERROR_NONE)
  {
    printf("Orientation detection enabled\r\n");
    IKS01A3_MOTION_SENSOR_SetFullScale(IKS01A3_LSM6DSO_0, MOTION_ACCELERO, 16);
  }
#endif
  else
  {
    printf("Orientation detection NOT enabled\r\n");
    Fatal_Error();
  }
}

void MX_Accelero_WakeUp_DeInit(void)
{
#ifdef USE_IKS01A2
  if (IKS01A2_MOTION_SENSOR_Disable_Wake_Up_Detection(IKS01A2_LSM6DSL_0) == BSP_ERROR_NONE)
  {
    printf("Wake-up detection disabled\r\n");
  }
#endif
#ifdef USE_IKS01A3
  if (IKS01A3_MOTION_SENSOR_Disable_Wake_Up_Detection(IKS01A3_LSM6DSO_0) == BSP_ERROR_NONE)
  {
    printf("Wake-up detection disabled\r\n");
  }
#endif
  else
  {
    printf("Wake-up detection NOT disabled\r\n");
    Fatal_Error();
  }
}

void MX_Accelero_Tilt_DeInit(void)
{
#ifdef USE_IKS01A2
  if (IKS01A2_MOTION_SENSOR_Disable_Tilt_Detection(IKS01A2_LSM6DSL_0) == BSP_ERROR_NONE)
  {
    printf("Tilt detection disabled\r\n");
  }
#endif
#ifdef USE_IKS01A3
  if (IKS01A3_MOTION_SENSOR_Disable_Tilt_Detection(IKS01A3_LSM6DSO_0) == BSP_ERROR_NONE)
  {
    printf("Tilt detection disabled\r\n");
  }
#endif
  else
  {
    printf("Tilt detection NOT disabled\r\n");
    Fatal_Error();
  }
}

void MX_Accelero_Orientation_DeInit(void)
{
#ifdef USE_IKS01A2
  if (IKS01A2_MOTION_SENSOR_Disable_6D_Orientation(IKS01A2_LSM6DSL_0) == BSP_ERROR_NONE)
  {
    printf("Orientation detection disabled\r\n");
  }
#endif
#ifdef USE_IKS01A3
  if (IKS01A3_MOTION_SENSOR_Disable_6D_Orientation(IKS01A3_LSM6DSO_0) == BSP_ERROR_NONE)
  {
    printf("Orientation detection disabled\r\n");
  }
#endif
  else
  {
    printf("Orientation detection NOT disabled\r\n");
    Fatal_Error();
  }
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

#ifdef __cplusplus
}
#endif
