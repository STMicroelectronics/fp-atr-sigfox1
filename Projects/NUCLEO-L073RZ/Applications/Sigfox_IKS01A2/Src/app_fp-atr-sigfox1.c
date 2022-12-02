/**
  * @file    app_fp_atr_sigfox1.c
  * @author  STM32ODE Team
  * @version 3.2.0
  * @date    1-September-2022
  * @brief   This file contains the main functions for FP-ATR-SIGFOX1
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

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "assert.h"
#include "stddef.h"

#include "app_fp-atr-sigfox1.h"
#include "app_x_cube_sfxs2lp1.h"

#include "retriever_api.h"
#include "sigfox_types.h"
#include "sigfox_api.h"
#include "st_rf_api.h"
#include "st_mcu_api.h"
#include "nvm_api.h"
#include "s2lp.h"
#include "sfx_config.h"
#include "mcu_api.h"

#include "app_mems-library.h"
#include "sensors.h"

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

#ifdef USE_GNSS
#include "gnss1a1_conf.h"
#include "teseo_liv3f_conf.h"
#include "gnss1a1_gnss.h"
#include "gnss_data.h"
#endif

#ifdef USE_BLE
#include "app_bluenrg_ms.h"
#endif

/** @addtogroup Applications
  * @{
  */

/** @addtogroup Sigfox_Asset_Tracker
  * @{
  */

/** @addtogroup STM32L0
  * @{
  */

/** @addtogroup Sigfox_Asset_Tracker_Defines
  * @{
  */

/* Private Macros -----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef USE_FLASH
#ifdef __ICCARM__
#pragma data_alignment=FLASH_PAGE_SIZE
#endif
#endif

#define FIRMWARE_VERSION "3.2.0"
#define FIRMWARE_DATE    "1-September-2022"

#define MAIN_THREAD_STACK_SIZE  2048
#define CONSUMER_STACK_SIZE     1024

#if ( USE_HAL_SPI_REGISTER_CALLBACKS == 0U )
#error SPI Register Callbacks must be enabled
#endif

#if ( USE_HAL_UART_REGISTER_CALLBACKS == 0U )
#error UART Register Callbacks must be enabled
#endif

#if ( USE_HAL_TIM_REGISTER_CALLBACKS == 0U )
#error TIM Register Callbacks must be enabled
#endif

/**
  * @}
  */

/** @addtogroup Sigfox_Asset_Tracker_Variables
  * @{
  */

/* Private variables ---------------------------------------------------------*/
static char* orientation_name[] = { "None", "XL axis", "XH axis", "YL axis", "YH axis", "ZL axis", "ZH axis" };
int sigfox_send_frequency = DEFAULT_SIGFOX_SEND_FREQUENCY;      /** Frequency of Sigfox message sending */
int sensors_read_frequency = DEFAULT_SENSOR_READ_FREQUENCY;     /** Frequency of sensor reading */
threshold_type threshold[MAX_THRESHOLDS];                       /** Thresholds for sensors */
int set_thresholds=0;                                           /** Number of thresholds active */
static orientation_axis axis_chosen = axis_none;                /** Axis chosen for orientation detection */
static int first_check = 1;                                     /** Reset checking thresholds first time */
volatile uint8_t MemsEventDetected = 0;
static volatile uint8_t PushButtonDetected = 0;

#ifdef USE_BLE
extern volatile uint8_t BleEventDetected;               /** A flag to understand Bluetooth events */
extern volatile uint8_t BleEventConfirmed;              /** A flag to understand Bluetooth events */
extern volatile uint32_t connected;
volatile uint8_t ble_enable = 1;
#endif

#ifdef USE_GNSS
GNSSParser_Data_t GNSSParser_Data;
#endif

/**
  * @}
  */

/** @addtogroup Sigfox_Asset_Tracker_Functions
  * @{
  */

/* Private Functions ---------------------------------------------------------*/
void sigfox_send(sensor_value_type, uint8_t trigger_mask);

/**
* @brief  Initialize the Sigfox Asset Tracker
* @retval None
*/
void MX_FP_ATR_SIGFOX1_Init(void)
{
  BSP_COM_Init(COM1);
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stdin, NULL, _IONBF, 0);

  printf("\r\n\n******************************\r\n");
  printf(" FP-ATR-SIGFOX1 asset tracker\r\n");
  printf(" V%s %s\r\n",FIRMWARE_VERSION,FIRMWARE_DATE);
  printf("******************************\r\n\n");

  MX_X_CUBE_SFXS2LP1_Init();

  sensors_init(0);
  MX_MEMS_Library_Init();

#ifdef USE_BLE
  MX_BlueNRG_MS_Init();
#endif

#ifndef USE_BLE
  MX_Accelero_WakeUp_Init(1);
#endif

}

/**
* @brief  FP-ATR-SIGFOX1 Main Process
* @retval None
*/
void MX_FP_ATR_SIGFOX1_Process(void)
{
  but_pressed = MemsEventDetected = 0;
#ifdef USE_BLE
  BleEventDetected = BleEventConfirmed = 0;
#endif
  sensor_value_type sensor_value;
  int16_t timer_sigfox=0;
  int16_t timer_sensors=0;
#ifdef USE_IKS01A2
  IKS01A2_MOTION_SENSOR_Event_Status_t motion_status;
#endif
#ifdef USE_IKS01A3
  IKS01A3_MOTION_SENSOR_Event_Status_t motion_status;
#endif
  uint8_t trigger_mask;

#ifdef USE_GNSS
  HAL_Delay(2000);
  printf("Waiting for GNSS fix (press button to skip) ");
#ifdef __GNUC__
  fflush(stdout);
#endif
  while (GNSSParser_Data.gpgga_data.valid != (uint8_t)VALID)
  {
    TeseoConsumerTask(NULL);
    printf(".");
#ifdef __GNUC__
    fflush(stdout);
#endif
    HAL_Delay(1000);
    if (but_pressed)
    {
      but_pressed=0;
      break;
    }
  }
  printf(" OK\r\n");
#endif

  printf("\nWaiting for event\r\n");

  /* application main loop */
  while(1)
  {
#ifdef USE_BLE
    if (ble_enable)
      MX_BlueNRG_MS_Process();
#endif

    /* Go in low power with the STM32 waiting for an external interrupt */
    setGpioLowPower();
    MCU_API_timer_start(10);
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
    MCU_API_timer_stop();
    ST_MCU_API_SetSysClock();
    setGpioRestore();

#ifdef USE_IKS01A2
    IKS01A2_MOTION_SENSOR_Get_Event_Status(IKS01A2_LSM6DSL_0, &motion_status); // !!!!!
#endif
#ifdef USE_IKS01A3
    IKS01A3_MOTION_SENSOR_Get_Event_Status(IKS01A3_LSM6DSO_0, &motion_status); // !!!!!
#endif

    trigger_mask = 0;

    if (but_pressed)
    {
      printf(">> Button event\r\n");
      but_pressed=0;
      trigger_mask |= Trig_Button;

      timer_sensors=timer_sigfox=0;
#ifdef USE_GNSS
      TeseoConsumerTask(NULL);
#endif
      sensors_read(&sensor_value);
      sigfox_send(sensor_value, trigger_mask);
    }
    else if (MemsEventDetected)
    {
      MemsEventDetected=0;

      if (motion_status.WakeUpStatus != 0U)
      {
        printf(">> Mems event (wake up)\r\n");
        trigger_mask |= Trig_WakeUp;
      }
      else if (motion_status.TiltStatus != 0U)
      {
        printf(">> Mems event (tilt)\r\n");
        trigger_mask |= Trig_Tilt;
      }
      else if (motion_status.D6DOrientationStatus != 0U)
      {
        if (!check_orientation(axis_chosen))
          continue;
        printf(">> Mems event (orientation)\r\n");
        trigger_mask |= Trig_Orientation;
      }
      else
      {
        //printf(">> Unknown mems event -- ignored\r\n");
        continue;
      }

      timer_sensors=timer_sigfox=0;
#ifdef USE_GNSS
      TeseoConsumerTask(NULL);
#endif
      sensors_read(&sensor_value);
      sigfox_send(sensor_value,trigger_mask);
    }
#ifdef USE_BLE
    else if (BleEventDetected)
    {
      BleEventDetected=0;

      while(connected)
      {
        MX_BlueNRG_MS_Process();
      }
      if ( !BleEventConfirmed )
        continue;
      timer_sensors=timer_sigfox=0;
      BleEventDetected=0;
      BleEventConfirmed=0;
      printf(">> Ble event\r\n");
    }
#endif

    timer_sigfox++;
    timer_sensors++;

    if ( timer_sensors == sensors_read_frequency*6 )
    {
      printf(">> Timer event (sensors_read)\r\n");
      timer_sensors=0;
#ifdef USE_GNSS
      TeseoConsumerTask(NULL);
#endif
      sensors_read(&sensor_value);

      if (check_thresholds(set_thresholds,threshold,sensor_value,&trigger_mask)!=0)
        sigfox_send(sensor_value,trigger_mask);
    }
    if ( timer_sigfox == sigfox_send_frequency*6 )
    {
      printf(">> Timer event (sigfox_send)\r\n");
      timer_sigfox=0;
      trigger_mask |= Trig_Timer;
      sigfox_send(sensor_value,trigger_mask);
    }

  } // end while(1)
}

/**
* @brief  Send Sigfox frame with provided sensor data
* @param[in]  sensor structure with humidity, temperature and pressure values
* @param[in]  trigger_mask flags with trigger events
* @retval None
*/
void sigfox_send(sensor_value_type sensor, uint8_t trigger_mask)
{
  /* Some variables to store the application data to transmit */
  uint8_t customer_data[12]={0};
  uint8_t customer_resp[8] = {0};
  static int value = 0;
  int data_length;

#ifdef USE_BLE
  if (ble_enable)
  {
    ble_switch_off();
    ble_enable = 0;
    printf("\r\n*** BLE deactivated ***\r\n\n");
  }
#endif

  //LedBlink(6);

  if (value == 0 )
  {
    customer_data[0] = ( sensor.humidity >> 8 ) & 0xFF;
    customer_data[1] = sensor.humidity & 0xFF;
    customer_data[2] = ( sensor.temperature >> 8 ) & 0xFF;
    customer_data[3] = sensor.temperature & 0xFF;
    customer_data[4] = ( sensor.pressure >> 8 ) & 0xFF;
    customer_data[5] = sensor.pressure & 0xFF;
    customer_data[6] = trigger_mask;
    data_length=7;
  }
  else
  {
    customer_data[0] = ( sensor.latitude >> 24 ) & 0xFF;
    customer_data[1] = ( sensor.latitude >> 16 ) & 0xFF;
    customer_data[2] = ( sensor.latitude >>  8 ) & 0xFF;
    customer_data[3] = sensor.latitude & 0xFF;
    customer_data[4] = ( sensor.longitude >> 24 ) & 0xFF;
    customer_data[5] = ( sensor.longitude >> 16 ) & 0xFF;
    customer_data[6] = ( sensor.longitude >>  8 ) & 0xFF;
    customer_data[7] = sensor.longitude & 0xFF;
    customer_data[8] = trigger_mask;
    data_length=9;
  }

  printf("Sending: ");
  for(uint8_t i=0;i<data_length;i++)
    printf("%02x",customer_data[i]);

  if ( value == 0 )
    printf(" (Hum=%d.%d Temp=%d.%d Pres=%d.%d) (Mask=%02X) ... ",
           sensor.humidity/10, sensor.humidity%10,
           sensor.temperature/10, sensor.temperature%10,
           sensor.pressure/10, sensor.pressure%10,
           trigger_mask );
  else
    printf("  (Latitude=%ld.%ld Longitude=%ld.%ld) (Mask=%02X) ... ",
           sensor.latitude/1000000, sensor.latitude%1000000,
           sensor.longitude/1000000, sensor.longitude%1000000,
           trigger_mask );

#ifdef __GNUC__
  fflush(stdout);
#endif

  /* Call the send_frame function */
  SIGFOX_API_send_frame(customer_data,data_length,customer_resp,2,0);

  //LedBlink(6);

  printf("ok\r\n");

#ifdef USE_GNSS
  value = !value;
#endif
}

/**
* @brief  Parse "set tracking" command received from bluetooth
* @param[in]  command       command string received
* @param[in]  lcommand      string length
* @param[out] sigfox_send_frequency     frequency of sigfox send messages
* @param[out] sensor_read_frequency     frequency of sensor read
* @param[out] threshold         array of thresholds
* @retval number of active thresholds in array
*/
int parse_setTracking_command(char* command, int lcommand, int* sigfox_send_frequency, int* sensors_read_frequency,
                              threshold_type threshold[])
{
  char *p;
  int i,ths;

  if (!strstr(command,"setTracking"))
  {
    printf("Wrong command received.\r\n");
    return 0;
  }

#if 0
  p=command;
  printf("Command received [%d]: ",lcommand);
  for (i=0;i<lcommand;i++)
    printf("%d ",*p++);
  printf("\r\n");
#endif

  MX_Accelero_WakeUp_DeInit();
  MX_Accelero_Tilt_DeInit();
  MX_Accelero_Orientation_DeInit();

  p=command;
  for (i=0;i<strlen("setTracking");i++)
    p++;

  *sigfox_send_frequency = *p++;
  *sensors_read_frequency = *p++;

  if (*sigfox_send_frequency==0)
    *sigfox_send_frequency=9999;

  printf("- Sigfox message frequency: %d\r\n", *sigfox_send_frequency);
  printf("- Sensor update frequency: %d\r\n", *sensors_read_frequency);

  ths = *p++;
  printf("- Thresholds received: %d\r\n",ths);
  if (ths>MAX_THRESHOLDS)
  {
    ths=MAX_THRESHOLDS;
    printf("Only %d thresholds will be set\r\n",MAX_THRESHOLDS);
  }

  for (i=0;i<ths;i++)
  {
    threshold[i].sensor_id = (sensor_id_type)*p;
    threshold[i].function = (function_type)*(p+1);
    threshold[i].value = *(p+2) + ((*(p+3))<<8);
    p+=4;

    printf("  - Threshold: #%d\r\n",i);
    printf("    - Sensor id: %d\r\n",threshold[i].sensor_id);
    printf("    - Function: %d\r\n",threshold[i].function);
    printf("    - Value: %d\r\n",threshold[i].value);

    if ( threshold[i].sensor_id == WAKE_ID )
    {
      /* Enable wake up detection with chosen threshold */
      MX_Accelero_WakeUp_Init(threshold[i].value);
    }
    else if ( threshold[i].sensor_id == TILT_ID )
    {
      /* Enable tilt detection */
      MX_Accelero_Tilt_Init();
    }
    else if ( threshold[i].sensor_id == ORIENT_ID )
    {
      /* Enable orientation detection with chosen axis */
      axis_chosen = (orientation_axis)threshold[i].value;
      MX_Accelero_Orientation_Init();
      printf("Activated on %s\r\n",orientation_name[threshold[i].value]);
    }

  }

  check_thresholds_reset();

  return ths;
}

/**
* @brief  Build reply to "get tracking" command received from bluetooth
* @param[out] command      reply string to send
* @param[in]  sigfox_send_frequency     frequency of sigfox send messages
* @param[in]  sensor_read_frequency     frequency of sensor read
* @param[in]  threshold         array of thresholds
* @param[in]  num_thresholds    number of active thresholds
* @retval command length
*/
int build_getTracking_reply(char* command, int sigfox_send_frequency, int sensors_read_frequency,
                            threshold_type threshold[], int num_thresholds)
{
  int i;
  int l = 0;

  if ( sigfox_send_frequency < 256 )
    command[l++] = sigfox_send_frequency;
  else
    command[l++] = 0;

  command[l++] = sensors_read_frequency;

  command[l++] = num_thresholds;

#if 1
  for (i=0; i<num_thresholds; i++)
  {
    command[l++] = threshold[i].sensor_id;
    command[l++] = threshold[i].function;
    command[l++] = threshold[i].value & 0xFF;
    command[l++] = (threshold[i].value >> 8) & 0xFF;
  }
#endif

  printf("Reply built\r\n");
  for (i=0;i<l;i++)
    printf("%d ",command[i]);
  printf("\r\n");

  return l;
}

/**
* @brief  Reset 'first' flag for checking thresholds
*/
void check_thresholds_reset(void)
{
  first_check = 1;
}

/**
* @brief  Check all thresholds on environmental sensor values
* @param[in]  thresholds number of active thresholds
* @param[in]  threshold[] array of all thresholds
* @param[in]  current_sensor_value structure with current environmantal sensor values
* @param[out] trigger_mask flags to signal threshold crossed
* @retval 1 if any threshold has been crossed, 0 otherwise
*/
int check_thresholds(int thresholds, threshold_type threshold[],sensor_value_type current_sensor_value,
                     uint8_t *trigger_mask)
{
  int i;
  int fires=0;
  int16_t current_value, previous_value;
  static sensor_value_type previous_sensor_value = { 0 };
  Trigger_Flag flag;

  for (i=0;i<thresholds;i++)
  {
    switch(threshold[i].sensor_id)
    {
    case TEMP_ID:
      current_value  = current_sensor_value.temperature;
      previous_value = previous_sensor_value.temperature;
      flag = Trig_Temperature;
      break;
    case HUM_ID:
      current_value  = current_sensor_value.humidity;
      previous_value = previous_sensor_value.humidity;
      flag = Trig_Humidity;
      break;
    case PRES_ID:
      current_value  = current_sensor_value.pressure;
      previous_value = previous_sensor_value.pressure;
      flag = Trig_Pressure;
      break;
    default:
      flag=Trig_None;
      continue; // only considers environment sensors
    }

    switch(threshold[i].function)
    {
    case LESS_FUNC:
      if ( (first_check || (previous_value >= threshold[i].value)) && (current_value < threshold[i].value))
      {
        printf("Threshold %d fires\r\n",i);
        *trigger_mask |= flag;
        fires = 1;
      }
      break;
    case GREATER_FUNC:
      if ( (first_check || (previous_value <= threshold[i].value)) && (current_value > threshold[i].value))
      {
        printf("Threshold %d fires\r\n",i);
        *trigger_mask |= flag;
        fires = 1;
      }
      break;
    default:
      printf("Function not implemented\r\n");
      break;
    }
  }

  previous_sensor_value = current_sensor_value;
  first_check = 0;

  return fires;
}

#ifdef USE_GNSS
void TeseoConsumerTask(void const * argument)
{
  GNSSParser_Status_t status, check;
  const GNSS1A1_GNSS_Msg_t *gnssMsg;
#if (configUSE_FEATURE == 1)
  static int config_done = 0;
#endif

  GNSS1A1_GNSS_Init(GNSS1A1_TESEO_LIV3F);

  GNSS_PARSER_Init(&GNSSParser_Data);

  int concluded = 0;
  while (!concluded)
  {
#if (USE_I2C == 1)
    GNSS1A1_GNSS_BackgroundProcess(GNSS1A1_TESEO_LIV3F);
#endif /* USE_I2C */

#if (configUSE_FEATURE == 1)
    /* See CDB-ID 201 - This LOW_BITS Mask enables the following messages:
     * 0x1 $GPGNS Message
     * 0x2 $GPGGA Message
     * 0x4 $GPGSA Message
     * 0x8 $GPGST Message
     * 0x40 $GPRMC Message
     * 0x80000 $GPGSV Message
     * 0x100000 $GPGLL Message
     */
    if(!config_done)
    {
      int lowMask = 0x18004F;
      int highMask = GEOFENCE;
      PRINT_OUT("\n\rConfigure Message List\n\r");
      AppCfgMsgList(lowMask, highMask);
      HAL_Delay(1000);  //Allows to catch the reply from Teseo

      PRINT_OUT("\n\rEnable Geofence\r");
      AppEnFeature("GEOFENCE,1");
      HAL_Delay(500);  //Allows to catch the reply from Teseo

      PRINT_OUT("\n\rConfigure Geofence Circle\n\r");
      AppGeofenceCfg("Geofence-Lecce");

      config_done = 1;
    }
#endif /* configUSE_FEATURE */

    gnssMsg = GNSS1A1_GNSS_GetMessage(GNSS1A1_TESEO_LIV3F);

    if(gnssMsg == NULL)
    {
      continue;
    }

    check = GNSS_PARSER_CheckSanity((uint8_t *)gnssMsg->buf, gnssMsg->len);

#if 0
    printf("got ");
    (check == GNSS_PARSER_OK) ? printf("Good sentence: ") : printf("!!!Bad sentence: ");
    printf((uint8_t *)gnssMsg->buf);
    printf("\n\r");
#endif

    if(check != GNSS_PARSER_ERROR)
    {
      for(int m = 0; m < NMEA_MSGS_NUM; m++)
      {
        status = GNSS_PARSER_ParseMsg(&GNSSParser_Data, (eNMEAMsg)m, (uint8_t *)gnssMsg->buf);

        if((status != GNSS_PARSER_ERROR) && ((eNMEAMsg)m == GPGGA))
        {
          GNSS_DATA_GetValidInfo(&GNSSParser_Data);
          concluded=1;
        }
#if (configUSE_FEATURE == 1)
        if((status != GNSS_PARSER_ERROR) && ((eNMEAMsg)m == PSTMGEOFENCE))
        {
          GNSS_DATA_GetGeofenceInfo(&GNSSParser_Data);
        }
        if((status != GNSS_PARSER_ERROR) && ((eNMEAMsg)m == PSTMSGL))
        {
          GNSS_DATA_GetMsglistAck(&GNSSParser_Data);
        }
        if((status != GNSS_PARSER_ERROR) && ((eNMEAMsg)m == PSTMSAVEPAR))
        {
          GNSS_DATA_GetGNSSAck(&GNSSParser_Data);
        }
#endif /* configUSE_FEATURE */
      }
    }

    GNSS1A1_GNSS_ReleaseMessage(GNSS1A1_TESEO_LIV3F, gnssMsg);
  }
}
#endif // USE_GNSS

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
