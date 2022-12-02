/**
  ******************************************************************************
  * @file    sensors.c
  * @author  STM32ODE Team, Italy
  * @version 3.2.0
  * @date    1-September-2022
  * @brief   This file contains functions to manage mems sensors
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

#ifdef USE_IKS01A2
#include "iks01a2_env_sensors.h"
#include "iks01a2_env_sensors_ex.h"
#include "iks01a2_motion_sensors.h"
#include "iks01a2_motion_sensors_ex.h"
#endif

#ifdef USE_IKS01A3
#include "iks01a3_env_sensors.h"
#include "iks01a3_env_sensors_ex.h"
#include "iks01a3_motion_sensors.h"
#include "iks01a3_motion_sensors_ex.h"
#endif

#include "sensors.h"

#ifdef USE_GNSS
#include "gnss_data.h"
#endif

#include "app_fp-atr-sigfox1.h"
#include "app_x_cube_sfxs2lp1.h"

/** @addtogroup Applications
  * @{
  */

/** @addtogroup Sigfox_Asset_Tracker
  * @{
  */

/** @addtogroup STM32L4
  * @{
  */

/** @addtogroup Sigfox_Asset_Tracker_Functions
  * @{
  */
#ifdef USE_GNSS
static float convertCoord( float x, uint8_t sign );

extern GNSSParser_Data_t GNSSParser_Data;
#endif

/**
* @brief  Initializes environmental sensors
* @param  quiet disable printing
* @retval None.
*/
void  sensors_init( uint8_t quiet  )
{
  /* Initialize sensors */
#ifdef USE_IKS01A2
  if ( IKS01A2_ENV_SENSOR_Init(IKS01A2_HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY) == BSP_ERROR_NONE )
  {
    if (!quiet)
      printf("Humidity & Temperature sensor initialized\r\n");
  }
  if ( IKS01A2_ENV_SENSOR_Init(IKS01A2_LPS22HB_0, ENV_PRESSURE) == BSP_ERROR_NONE )
  {
    if (!quiet)
      printf("Pressure sensor initialized\r\n");
  }
#endif
#ifdef USE_IKS01A3
  if ( IKS01A3_ENV_SENSOR_Init(IKS01A3_STTS751_0, ENV_TEMPERATURE) == BSP_ERROR_NONE )
  {
    if (!quiet)
      printf("Temperature sensor initialized\r\n");
  }
  if ( IKS01A3_ENV_SENSOR_Init(IKS01A3_HTS221_0, ENV_HUMIDITY) == BSP_ERROR_NONE )
  {
    if (!quiet)
      printf("Humidity sensor initialized\r\n");
  }
  if ( IKS01A3_ENV_SENSOR_Init(IKS01A3_LPS22HH_0, ENV_PRESSURE) == BSP_ERROR_NONE )
  {
    if (!quiet)
      printf("Pressure sensor initialized\r\n");
  }
#endif
}

/**
* @brief  De-initializes environmental sensors
* @param  quiet disable printing
* @retval None.
*/
void  sensors_deinit( uint8_t quiet )
{
  /* Deinitialize sensors */
#ifdef USE_IKS01A2
  if ( IKS01A2_ENV_SENSOR_DeInit(IKS01A2_HTS221_0 ) == BSP_ERROR_NONE )
  {
    if (!quiet)
      printf("Humidity & Temperature sensor deinitialized\r\n");
  }
  else
  {
    printf("Humidity & Temperature sensor NOT working\r\n");
    Fatal_Error();
  }
  if ( IKS01A2_ENV_SENSOR_DeInit(IKS01A2_LPS22HB_0 ) == BSP_ERROR_NONE )
  {
    if (!quiet)
      printf("Pressure sensor deinitialized\r\n");
  }
  else
  {
    printf("Pressure sensor NOT working\r\n");
    Fatal_Error();
  }
#endif
#ifdef USE_IKS01A3
  if ( IKS01A3_ENV_SENSOR_DeInit(IKS01A3_HTS221_0 ) == BSP_ERROR_NONE )
  {
    if (!quiet)
      printf("Humidity sensor deinitialized\r\n");
  }
  else
  {
    printf("Humidity sensor NOT working\r\n");
    Fatal_Error();
  }
  if ( IKS01A3_ENV_SENSOR_DeInit(IKS01A3_LPS22HH_0 ) == BSP_ERROR_NONE )
  {
    if (!quiet)
      printf("Pressure sensor deinitialized\r\n");
  }
  else
  {
    printf("Pressure sensor NOT working\r\n");
    Fatal_Error();
  }
  if ( IKS01A3_ENV_SENSOR_DeInit(IKS01A3_STTS751_0 ) == BSP_ERROR_NONE )
  {
    if (!quiet)
      printf("Temperature sensor deinitialized\r\n");
  }
  else
  {
    printf("Temperature sensor NOT working\r\n");
    Fatal_Error();
  }
#endif
}

/**
* @brief  Enables environmental sensors
* @param  None
* @retval None.
*/
void  sensors_enable( uint8_t quiet )
{
  /* Enable sensors */
#ifdef USE_IKS01A2
  if ( IKS01A2_ENV_SENSOR_Enable(IKS01A2_HTS221_0, ENV_HUMIDITY ) == BSP_ERROR_NONE )
  {
    if (!quiet)
      printf("Humidity sensor enabled\r\n");
  }
  else
  {
    printf("Humidity sensor NOT enabled\r\n");
    Fatal_Error();
  }

  if ( IKS01A2_ENV_SENSOR_Enable(IKS01A2_HTS221_0, ENV_TEMPERATURE) == BSP_ERROR_NONE )
  {
    if (!quiet)
      printf("Temperature sensor enabled\r\n");
  }
  else
  {
    printf("Temperature sensor NOT enabled\r\n");
    Fatal_Error();
  }

  if ( IKS01A2_ENV_SENSOR_Enable(IKS01A2_LPS22HB_0, ENV_PRESSURE) == BSP_ERROR_NONE )
  {
    if (!quiet)
      printf("Pressure sensor enabled\r\n");
  }
  else
  {
    printf("Pressure sensor NOT enabled\r\n");
    Fatal_Error();
  }
#endif
#ifdef USE_IKS01A3
  if ( IKS01A3_ENV_SENSOR_Enable(IKS01A3_HTS221_0, ENV_HUMIDITY) == BSP_ERROR_NONE )
  {
    printf("Humidity sensor enabled\r\n");
  }
  else
  {
    printf("Humidity sensor NOT enabled\r\n");
    Fatal_Error();
  }

  if ( IKS01A3_ENV_SENSOR_Enable(IKS01A3_LPS22HH_0, ENV_PRESSURE) == BSP_ERROR_NONE )
  {
    printf("Pressure sensor enabled\r\n");
  }
  else
  {
    printf("Pressure sensor NOT enabled\r\n");
    Fatal_Error();
  }

  if ( IKS01A3_ENV_SENSOR_Enable(IKS01A3_STTS751_0, ENV_TEMPERATURE) == BSP_ERROR_NONE )
  {
    printf("Temperature sensor enabled\r\n");
  }
  else
  {
    printf("Temperature sensor NOT enabled\r\n");
    Fatal_Error();
  }
#endif
}

/**
* @brief  Disables environmental sensors
* @param  None
* @retval None.
*/
void  sensors_disable( uint8_t quiet )
{
  /* USER CODE BEGIN 6 */
  int retval = 0;

  /* Disable sensors */
#ifdef USE_IKS01A2
  retval |= IKS01A2_ENV_SENSOR_Disable(IKS01A2_HTS221_0, ENV_HUMIDITY );
  retval |= IKS01A2_ENV_SENSOR_Disable(IKS01A2_HTS221_0, ENV_TEMPERATURE);
  retval |= IKS01A2_ENV_SENSOR_Disable(IKS01A2_LPS22HB_0, ENV_PRESSURE);
#endif
#ifdef USE_IKS01A3
  retval |= IKS01A3_ENV_SENSOR_Disable(IKS01A3_HTS221_0, ENV_HUMIDITY );
  retval |= IKS01A3_ENV_SENSOR_Disable(IKS01A3_LPS22HH_0, ENV_PRESSURE);
  retval |= IKS01A3_ENV_SENSOR_Disable(IKS01A3_STTS751_0, ENV_TEMPERATURE);
#endif

  if ( ( retval == BSP_ERROR_NONE ) && (!quiet) )
      printf("Environmental sensors disabled\r\n");
  else
  {
    printf("Environmental sensors not working\r\n");
    Fatal_Error();
  }
}

/**
* @brief  Read environmental sensors
* @param[out]  value structure with sensor current values
* @retval None.
*/
void sensors_read( sensor_value_type *value )
{
  /* USER CODE BEGIN 5 */
  float HUMIDITY_Value = 0;
  float TEMPERATURE_Value = 0;
  float PRESSURE_Value = 0;
#ifdef USE_GNSS
  float gnss_latitude = 0;
  float gnss_longitude = 0;
#endif

#ifdef USE_IKS01A2
  IKS01A2_ENV_SENSOR_GetValue(IKS01A2_HTS221_0, ENV_HUMIDITY, &HUMIDITY_Value);
  IKS01A2_ENV_SENSOR_GetValue(IKS01A2_HTS221_0, ENV_TEMPERATURE, &TEMPERATURE_Value);
  IKS01A2_ENV_SENSOR_GetValue(IKS01A2_LPS22HB_0, ENV_PRESSURE, &PRESSURE_Value);
#endif
#ifdef USE_IKS01A3
  IKS01A3_ENV_SENSOR_GetValue(IKS01A3_HTS221_0, ENV_HUMIDITY, &HUMIDITY_Value);
  IKS01A3_ENV_SENSOR_GetValue(IKS01A3_STTS751_0, ENV_TEMPERATURE, &TEMPERATURE_Value);
  IKS01A3_ENV_SENSOR_GetValue(IKS01A3_LPS22HH_0, ENV_PRESSURE, &PRESSURE_Value);
#endif

  value->humidity    = (int16_t)(HUMIDITY_Value * 10);
  value->temperature = (int16_t)(TEMPERATURE_Value * 10);
  value->pressure    = (int16_t)(PRESSURE_Value * 100 / 10);

#ifdef USE_GNSS
  gnss_latitude  = convertCoord( GNSSParser_Data.gpgga_data.xyz.lat,
                                GNSSParser_Data.gpgga_data.xyz.ns=='N' ? 0 : 1 );
  gnss_longitude = convertCoord( GNSSParser_Data.gpgga_data.xyz.lon,
                                GNSSParser_Data.gpgga_data.xyz.ew=='E' ? 0 : 1 );

  value->latitude  = (int32_t)(gnss_latitude  * 1000000);
  value->longitude = (int32_t)(gnss_longitude * 1000000);
#endif

  printf("Reading: Hum=%d.%d Temp=%d.%d Pres=%d.%d\r\n",
         value->humidity/10, value->humidity%10,
         value->temperature/10, value->temperature%10,
         value->pressure/10, value->pressure%10);

#ifdef USE_GNSS
  printf("         Lat=%ld.%ld Lon=%ld.%ld\r\n",
         value->latitude/1000000, value->latitude%1000000,
         value->longitude/1000000, value->longitude%1000000);
#endif
}

/**
* @brief  Check axis returned by orientation detection
* @param[in]  axis axis to be checked for match with orientation detection
* @retval 1 if axis match, 0 otherwise
*/
int check_orientation(orientation_axis axis)
{
  uint8_t xl = 0;
  uint8_t xh = 0;
  uint8_t yl = 0;
  uint8_t yh = 0;
  uint8_t zl = 0;
  uint8_t zh = 0;
  int fires = 0;

#ifdef USE_IKS01A2
  if (IKS01A2_MOTION_SENSOR_Get_6D_Orientation_XL(IKS01A2_LSM6DSL_0, &xl) != BSP_ERROR_NONE)
  {
    printf("Error getting 6D orientation XL axis from LSM6DSL accelerometer.\r\n");
    return 0;
  }
  if (IKS01A2_MOTION_SENSOR_Get_6D_Orientation_XH(IKS01A2_LSM6DSL_0, &xh) != BSP_ERROR_NONE)
  {
    printf("Error getting 6D orientation XH axis from LSM6DSL accelerometer.\r\n");
    return 0;
  }
  if (IKS01A2_MOTION_SENSOR_Get_6D_Orientation_YL(IKS01A2_LSM6DSL_0, &yl) != BSP_ERROR_NONE)
  {
    printf("Error getting 6D orientation YL axis from LSM6DSL accelerometer.\r\n");
    return 0;
  }
  if (IKS01A2_MOTION_SENSOR_Get_6D_Orientation_YH(IKS01A2_LSM6DSL_0, &yh) != BSP_ERROR_NONE)
  {
    printf("Error getting 6D orientation YH axis from LSM6DSL accelerometer.\r\n");
    return 0;
  }
  if (IKS01A2_MOTION_SENSOR_Get_6D_Orientation_ZL(IKS01A2_LSM6DSL_0, &zl) != BSP_ERROR_NONE)
  {
    printf("Error getting 6D orientation ZL axis from LSM6DSL accelerometer.\r\n");
    return 0;
  }
  if (IKS01A2_MOTION_SENSOR_Get_6D_Orientation_ZH(IKS01A2_LSM6DSL_0, &zh) != BSP_ERROR_NONE)
  {
    printf("Error getting 6D orientation ZH axis from LSM6DSL accelerometer.\r\n");
    return 0;
  }
#endif
#ifdef USE_IKS01A3
  if (IKS01A3_MOTION_SENSOR_Get_6D_Orientation_XL(IKS01A3_LSM6DSO_0, &xl) != BSP_ERROR_NONE)
  {
    printf("Error getting 6D orientation XL axis from LSM6DSO accelerometer.\r\n");
    return 0;
  }
  if (IKS01A3_MOTION_SENSOR_Get_6D_Orientation_XH(IKS01A3_LSM6DSO_0, &xh) != BSP_ERROR_NONE)
  {
    printf("Error getting 6D orientation XH axis from LSM6DSO accelerometer.\r\n");
    return 0;
  }
  if (IKS01A3_MOTION_SENSOR_Get_6D_Orientation_YL(IKS01A3_LSM6DSO_0, &yl) != BSP_ERROR_NONE)
  {
    printf("Error getting 6D orientation YL axis from LSM6DSO accelerometer.\r\n");
    return 0;
  }
  if (IKS01A3_MOTION_SENSOR_Get_6D_Orientation_YH(IKS01A3_LSM6DSO_0, &yh) != BSP_ERROR_NONE)
  {
    printf("Error getting 6D orientation YH axis from LSM6DSO accelerometer.\r\n");
    return 0;
  }
  if (IKS01A3_MOTION_SENSOR_Get_6D_Orientation_ZL(IKS01A3_LSM6DSO_0, &zl) != BSP_ERROR_NONE)
  {
    printf("Error getting 6D orientation ZL axis from LSM6DSO accelerometer.\r\n");
    return 0;
  }
  if (IKS01A3_MOTION_SENSOR_Get_6D_Orientation_ZH(IKS01A3_LSM6DSO_0, &zh) != BSP_ERROR_NONE)
  {
    printf("Error getting 6D orientation ZH axis from LSM6DSO accelerometer.\r\n");
    return 0;
  }
#endif

  if (xl == 1U && yl == 0U && zl == 0U && xh == 0U && yh == 0U && zh == 0U && axis == axis_xl)
  {
    printf("XL orientation\r\n");
    fires = 1;
  }
  else if (xl == 0U && yl == 1U && zl == 0U && xh == 0U && yh == 0U && zh == 0U && axis == axis_yl)
  {
    printf("YL orientation\r\n");
    fires = 1;
  }
  else if (xl == 0U && yl == 0U && zl == 1U && xh == 0U && yh == 0U && zh == 0U && axis == axis_zl)
  {
    printf("ZL orientation\r\n");
    fires = 1;
  }
  else if (xl == 0U && yl == 0U && zl == 0U && xh == 1U && yh == 0U && zh == 0U && axis == axis_xh)
  {
    printf("XH orientation\r\n");
    fires = 1;
  }
  else if (xl == 0U && yl == 0U && zl == 0U && xh == 0U && yh == 1U && zh == 0U && axis == axis_yh)
  {
    printf("YH orientation\r\n");
    fires = 1;
  }
  else if (xl == 0U && yl == 0U && zl == 0U && xh == 0U && yh == 0U && zh == 1U && axis == axis_zh)
  {
    printf("ZH orientation\r\n");
    fires = 1;
  }
  else
  {
    //printf("None of the 6D orientation axes is set in LSM6DSL - accelerometer.\r\n");
    fires = 0;
  }

  return fires;
}

#ifdef USE_GNSS
/**
  * @brief  Convert latitude/longitude coordinate from sexagesimal to decimal format
  * @param  float x coordinate
  * @param  uint8_t sign 1 for negative 0 for positive
  * @retval coordinate in decimal format
  */
static float convertCoord( float x, uint8_t sign )
{
  int degrees;
  float minutes;
  float ret;

  degrees = (int)(x / 100.0F);
  minutes = x - degrees*100.0F;
  ret = degrees + minutes / 60.0F;
  if (sign==1)
    ret = -ret;

  return ret;
}
#endif

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
