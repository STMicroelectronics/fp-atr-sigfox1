/**
  * @file    sfx_config.h
  * @author  STM32ODE Team, Noida
  * @brief   This file defines the manufacturer's MCU functions to be implemented
  * for library usage.External API dependencies to link with this library.
  * Error codes of the MCU API functions are described below.
  * The Manufacturer can add more error code taking care of the limits defined.
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
#ifndef __SFX_CONFIG_H
#define __SFX_CONFIG_H

/* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "s2lp.h"
#include "RTE_Components.h"
#include "bsp_ip_conf.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l4xx_it.h"
#include "s2868a1.h"
#include "nvm_api.h"
#include "sigfox_types.h"
#include "sigfox_api.h"
#include "st_rf_api.h"
#include "st_mcu_api.h"
#include "s2lp.h"
#include "mcu_api.h"
#include "s2lp_management.h"

/* Private defines -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define ABS(x)  (x>0?x:-x)

#define  STM32_GPIO_CLK_DISABLE()   {__GPIOB_CLK_DISABLE();__GPIOC_CLK_DISABLE();__GPIOD_CLK_DISABLE();__GPIOA_CLK_DISABLE();}
#define  STM32_GPIO_CLK_ENABLE()    {__GPIOA_CLK_ENABLE();__GPIOB_CLK_ENABLE();__GPIOC_CLK_ENABLE();__GPIOD_CLK_ENABLE();}

#define RADIO_TIMx_PRIORITY                    1

#define RadioEnterShutdown                      S2868A1_RADIO_EnterShutdown
#define RadioExitShutdown                       S2868A1_RADIO_ExitShutdown

#define EEPROM_YES		       1
#define EEPROM_NO		       0
#define EEPROM_PRESENT                 EEPROM_YES

/* S2LP GPIO - IRQ Pin - GPIO 3 is default */
#if (USE_S2868A1_RADIO_GPIO_0 == 1)
#define S2LP_GPIO_IRQ_PIN                         	0
#define S2LP_RADIO_GPIO                                  S2868A1_RADIO_GPIO_0
#endif
#if (USE_S2868A1_RADIO_GPIO_1 == 1)
#define S2LP_GPIO_IRQ_PIN                         	1
#define S2LP_RADIO_GPIO                                  S2868A1_RADIO_GPIO_1
#endif
#if (USE_S2868A1_RADIO_GPIO_2 == 1)
#define S2LP_GPIO_IRQ_PIN                         	2
#define S2LP_RADIO_GPIO                                  S2868A1_RADIO_GPIO_2
#endif
#if (USE_S2868A1_RADIO_GPIO_3 == 1)
#define S2LP_GPIO_IRQ_PIN                         	3
#define S2LP_RADIO_GPIO                                  S2868A1_RADIO_GPIO_3
#endif

#define S2LP_GPIO_IRQ_EDGE_EVENT			0 /* 0 means falling edge, 1 raising */

/* Exported Macros -----------------------------------------------------------*/
#define RADIO_DEV_VER_REG   (0xF1)

/**
 * @brief  Configures the specified TIMER to raise an interrupt every TIME ms.
 * @param  TIMER: timer to be set.
 *          This parameter can be a pointer to @ref TIM_TypeDef
 * @param  TIME: timer duration in ms.
 *          This parameter is a float.
 * @retval None
 */
#define RadioTimersTimConfig_ms(TIM_HANDLER_P , TIME)      {\
                                                        uint32_t n = (uint32_t)(TIME*CLOCK_FREQUENCY);\
                                                        uint16_t a,b;\
                                                        RadioTimersFindFactors(n,&a,&b);\
                                                        RadioTimersTimConfig(TIM_HANDLER_P,a-1,b-1);\
                                                      }

/**
 * @brief  Enables or Disables a specific Timer with its IRQ.
 * @param  TIMER: timer to be set.
 *          This parameter can be a pointer to @ref TIM_TypeDef
 * @param  NEWSTATE: specifies if a timer has to be enabled or disabled.
 *          This parameter is a float.
 * @retval None
 */
#define RadioTimersState(TIM_HANDLER_P , NEWSTATE)      {((uint8_t)NEWSTATE)?(HAL_TIM_Base_Start_IT(TIM_HANDLER_P)):(HAL_TIM_Base_Stop_IT(TIM_HANDLER_P));}

/**
 * @brief  Set the counter of the specified TIMER.
 * @param  TIMER: timer to be set.
 *          This parameter can be a pointer to @ref TIM_TypeDef
 * @param  VALUE: value to set in the counter.
 *          This parameter is an uint32_t
 * @retval None
 */
#define RadioTimersSetCounter(TIM_HANDLER_P,VALUE)   {__HAL_TIM_SET_COUNTER(TIM_HANDLER_P,VALUE);}

/**
 * @brief  Get the counter of the specified TIMER.
 * @param  TIMER: timer to be set.
 *          This parameter can be a pointer to @ref TIM_TypeDef
 * @retval None
 */
#define RadioTimersGetCounter(TIM_HANDLER_P)   __HAL_TIM_GET_COUNTER(TIM_HANDLER_P)

/**
 * @brief  Resets the counter of a specific timer.
 * @param  TIMER: timer to be reset.
 *          This parameter can be a pointer to @ref TIM_TypeDef
 * @retval None
 */
#define RadioTimersResetCounter(TIM_HANDLER_P)      {__HAL_TIM_SET_COUNTER(TIM_HANDLER_P,0);}

extern uint8_t but_pressed;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef SFX_TIM_HANDLE;

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  FlashRead Status Enum
  */
typedef enum
{
  FLS_RW_OK     			= 0x00,
  FLS_RW_ERROR				= 0x01,
  FLS_RW_OUT_OF_RANGE	= 0x02
} FLS_RW_StatusTypeDef;

/**
  * @brief  Sigfox Error type
  */
typedef enum {
  ST_SFX_ERR_NONE = 0,
  ST_SFX_ERR_OPEN = 1,
  ST_SFX_ERR_CREDENTIALS = 2,
  ST_SFX_ERR_OFFSET = 3,
  ST_SFX_ERR_RC_UNKNOWN = 99
} ST_SFX_ERR;

/**
 * @brief  Opcode for TCXO management
 */
typedef enum
{
  TCXO_OFF        = 0x00,
  TCXO_ON         = 0x01,
} TCXO_OperationType;

/* Exported Functions --------------------------------------------------------*/
void RadioIRQEnable(uint8_t state, uint8_t edge_direction);

void RadioTimersFindFactors(uint32_t lCycles,
                            uint16_t *pnPrescaler,
                            uint16_t *pnCounter);
void RadioSetGpioLowPwr(void);
void RadioRestoreGpio(void);
void RadioTimersTimConfig(TIM_HandleTypeDef* TIM_TimeBaseStructure,
                          uint16_t nPrescaler,
                          uint16_t nPeriod);
uint8_t EepromRead(uint16_t nAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
uint8_t EepromWrite(uint16_t nAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
__weak void RadioSpiRawTC(void);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void RadioSpiRaw(uint8_t cNbBytes,
                        uint8_t* pInBuffer,
                        uint8_t* pOutBuffer,
                        uint8_t can_return_before_tc);
void FEM_Operation(FEM_OperationType operation);
void S2LPSetSpiInUse(uint8_t state);
uint8_t S2LPGetSpiInUse(void);
void Config_RTC_Clk(void);
void HAL_EXTI_SFX_Callback(void);
void TCXO_Operation(TCXO_OperationType operation);
void TCXO_Init(void);
void SystemClock_Config(void);
RangeExtType DetetctPA(void);
ST_SFX_ERR St_Sigfox_Open_RCZ(uint8_t rcz);
ST_SFX_ERR ST_Sigfox_Init(NVM_BoardDataType *sfxConfig, uint8_t openAfterInit);
uint8_t EepromIdentification(void);
uint32_t S2LPManagementGetXtalFrequency(void);
void S2LP_RADIO_SetXtalFrequency(uint32_t lXtalFrequency);
uint8_t RadioGetHasEeprom(void);
void RadioSetHasEeprom(uint8_t eeprom);
void RadioTcxoOn(void);
void ST_MANUF_report_CB(uint8_t status, int32_t rssi);
void RadioComTriggerTx(void);
void updatetxQ(void);
void RadioComBaudrate(uint32_t baudrate);
void SFX_TIM_Callback(void);
void SFX_ClearWKUP_Flag(void);
void SFX_SetWKUP_Timer(uint32_t CtrDiv);
void SFX_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void SFX_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void setGpioLowPower(void);
void setGpioRestore(void);

__weak void TCXO_Init(void);
#ifdef __cplusplus
}
#endif

#endif

