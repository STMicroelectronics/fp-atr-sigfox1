
  /**
  * @file    app_fp_atr_sigfox1.c
  * @author  STM32ODE Team
  * @version 3.2.0
  * @date    1-September-2022
  * @brief   This file contains the main functions for X-CUBE-SFXS2LP1
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
#include "main.h"
#include <string.h>
#include "assert.h"
#include "stddef.h"
#include "retriever_api.h"
#include "sigfox_types.h"
#include "sigfox_api.h"
#include "st_rf_api.h"
#include "st_mcu_api.h"
#include "nvm_api.h"
#include "s2lp.h"
#include "app_x_cube_sfxs2lp1.h"
#include "sfx_config.h"
#include "s2868a1.h"
#include "s2868a1_conf.h"
#include "mcu_api.h"
#include "stm32l0xx_nucleo.h"

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

/**
  * @}
  */

/** @addtogroup Sigfox_Asset_Tracker_Variables
  * @{
  */

/* Private variables ---------------------------------------------------------*/
uint8_t but_pressed = 0;
/*Flags declarations*/
volatile int MasterFlag = 0;
__IO uint32_t KEYStatusData = 0x00;
uint16_t exitCounter = 0;
uint16_t txCounter = 0;
uint16_t wakeupCounter = 0;
uint16_t dataSendCounter = 0x00;
static volatile FlagStatus s_xTIMChCompareModeRaised = RESET;
volatile uint8_t interactive = 1;

/* Some local variables to handle the workflow */
uint8_t ret_err, use_public_key = 0;

/* Some variables to store the application data to transmit */
uint32_t cust_counter=0;
uint8_t customer_data[12]={0};
uint8_t customer_resp[8];

/* Private Functions ---------------------------------------------------------*/

/**
  * @}
  */

/** @addtogroup Sigfox_Asset_Tracker_Functions
  * @{
  */

/**
* @brief  Initialize the Sigfox CLI Example
* @retval None
*/
void MX_X_CUBE_SFXS2LP1_Init(void)
{
   /*Local Variables*/
  NVM_BoardDataType sfxConfiguration;
  ST_SFX_ERR stSfxRetErr;

  /* Configure USER Key Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* S2LP ON */
  S2868A1_RADIO_EnterShutdown();
  HAL_Delay(10);
  S2868A1_RADIO_ExitShutdown();

  S2868A1_RADIO_Init();

  S2868A1_SPI_DeInit();
  S2868A1_SPI_Init();
  HAL_SPI_RegisterCallback(&hspi, HAL_SPI_TX_RX_COMPLETE_CB_ID, SFX_SPI_TxRxCpltCallback);
  S2868A1_RADIO_SPI_NSS_PIN_HIGH();
  HAL_Delay(10);

  /* uC IRQ enable */
   RadioIRQEnable(1,0);

  /* Set the EEPROM availability */
  RadioSetHasEeprom(EEPROM_PRESENT);

  /* Auto detect settings, if EEPROM is available */
  if (RadioGetHasEeprom())
  {
    /* Identify the S2-LP RF board reading some production data */
    S2LPManagementIdentificationRFBoard();
  }
  else
  {
    /* Set XTAL frequency with offset */
    S2LP_RADIO_SetXtalFrequency(XTAL_FREQUENCY+XTAL_FREQUENCY_OFFSET);

    /* Set the frequency base */
    S2LP_ManagementSetBand(BOARD_FREQUENCY_BAND);

    /* Configure PA availability */
    S2LP_ManagementSetRangeExtender(DetetctPA());
  }

  /* TCXO Initialization */
  TCXO_Init();

  /* Reset S2LP */
  S2868A1_RADIO_EnterShutdown();
  HAL_Delay(10);
  S2868A1_RADIO_ExitShutdown();

    /* Calibrate RTC in case of STM32*/
  /* The low level driver uses the internal RTC as a timer while the STM32 is in low power.
  This function calibrates the RTC using an auxiliary general purpose timer in order to
  increase its precision. */
  ST_MCU_API_TimerCalibration(500);

  /* FEM Initialization */
  FEM_Operation(FEM_SHUTDOWN);

  /* Init the Sigfox Library and the device for Sigfox communication*/
  stSfxRetErr = ST_Sigfox_Init(&sfxConfiguration, 1);

  if(stSfxRetErr != ST_SFX_ERR_NONE)
    Fatal_Error();

  if(use_public_key)
    enc_utils_set_public_key(1);

  //BSP_LED_Init(LED2);

  printf("Sigfox Device:\r\n  ID  %0X\r\n  PAC ",(int)sfxConfiguration.id);
  for (int p=0;p<8;p++)
    printf("%02X",(int)sfxConfiguration.pac[p]);
  printf("\r\n  RCZ %d\r\n\n",(int)sfxConfiguration.rcz);
}

#if 0
/**
* @brief  Let the application led blinks.
* @param  times Number of toggles.
* @retval None

void LedBlink(uint8_t times)
{
  BSP_LED_Init(LED2);

  for(uint8_t i=0;i<times;i++)
  {
    BSP_LED_Toggle(LED2);
    HAL_Delay(50);
  }
} */
#endif

/**
* @brief  Blink the LED indefinitely stucking the application.
* @param  None
* @retval None
*/
void Fatal_Error(void)
{
  //BSP_LED_Init(LED2);
  while(1)
  {
    HAL_Delay(100);
    //BSP_LED_Toggle(LED2);
  }
}

/**
  * @brief  BSP Push Button callback
  * @param  Button Specifies the pin connected EXTI line
  * @retval None.
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
 if(Button==BUTTON_USER)
  {
   Set_KeyStatus(SET);
  }
}

/**
* @brief  This routine updates the respective status for key press.
* @param  None
* @retval None
*/
void Set_KeyStatus(FlagStatus val)
{
  if(val==SET)
  {
    but_pressed = 1;
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
