/**
  * @file    sfx_config.c
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

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include "assert.h"
#include "stddef.h"
#include "sfx_config.h"
#include "bsp_ip_conf.h"
#include "retriever_api.h"
#include "s2868a1.h"
#include "s2868a1_conf.h"
#include "stm32l0xx_nucleo_bus.h"
#include "stm32l0xx.h"
#include "stm32l0xx_hal_gpio.h"

/** @defgroup Sigfox configuration file
  * @{
  */

#define DELAY_CS_SCLK			0x70	/* Delay between CSn falling edge & start of SCLK */

/* Private variables ---------------------------------------------------------*/
static volatile RangeExtType s_RfRangeExtender = RANGE_EXT_NONE;
uint32_t  PAGEError = 0;
static volatile uint8_t spi_in_use=0;
static uint8_t rx_buff[128];
static uint8_t waiting_irq=0;
volatile static uint8_t SpiTxCpltFlag = 0;

GpioIrqHandler *GpioIrq[] = {

#if (USE_S2868A1_RADIO_GPIO_0 == 1)
  HAL_EXTI_SFX_Callback,
#endif
#if (USE_S2868A1_RADIO_GPIO_1 == 1)
  HAL_EXTI_SFX_Callback,
#endif
#if (USE_S2868A1_RADIO_GPIO_2 == 1)
  HAL_EXTI_SFX_Callback,
#endif
#if (USE_S2868A1_RADIO_GPIO_3 == 1)
  HAL_EXTI_SFX_Callback,
#endif
};

static volatile uint32_t s_XtalFrequency=50000000;
volatile static uint32_t s_lXtalFrequency=50000000;
static volatile uint32_t s_RfXtalFrequency = 50000000;
static uint8_t s_eeprom  = 0;
/* Private functions -------------------------------------------------------*/
static void SetSpiTxCompleteFlag(void);
static void ResetSpiTxCompleteFlag(void);
static uint8_t GetSpiTxCompleteFlag(void);

/**
 * @brief  Enable the MCU Pin as Interrupt for Radio S2-LP.
 * @param  state : Enable(1) or Disable(0).
 * @param  edge_direction : rising(1) or Falling(0).
 * @retval None.
 */
void RadioIRQEnable(uint8_t state, uint8_t edge_direction)
{
  /*Select Edge*/
  if(edge_direction == 0)
  {
    S2868A1_RADIO_GPIO_Init_Update(S2LP_RADIO_GPIO, RADIO_MODE_EXTI_IN,FALLING);
  }
  else if(edge_direction == 1)
  {
    S2868A1_RADIO_GPIO_Init_Update(S2LP_RADIO_GPIO, RADIO_MODE_EXTI_IN,RISING);
  }
  else
  {
    S2868A1_RADIO_GPIO_Init_Update(S2LP_RADIO_GPIO, RADIO_MODE_EXTI_IN,FALLING);
  }

  /* Handle Interrupt state */
  if(state == 1)
  {
    /* uC IRQ enable */
    S2868A1_RADIO_IoIrqEnable(GpioIrq);
  }
  else if (state == 0)
  {
    /* uC IRQ Disable */
    S2868A1_RADIO_IoIrqDisable(GpioIrq);
  }
}

/**
* @brief  Computes two integer value prescaler and period such that
*         Cycles = prescaler * period.
* @param  lCycles the specified cycles for the desired timer value.
* @param  pnPrescaler prescaler factor.
* @param  pnCounter period factor.
* @retval None.
*/
void RadioTimersFindFactors(uint32_t lCycles,
                            uint16_t *pnPrescaler,
                            uint16_t *pnCounter)
{
  uint16_t b0;
  uint16_t a0;
  long err, err_min=lCycles;

  *pnPrescaler = a0 = ((lCycles-1)/0xffff) + 1;
  *pnCounter = b0 = lCycles / *pnPrescaler;

  for (; *pnPrescaler < 0xffff-1; (*pnPrescaler)++) {
    *pnCounter = lCycles / *pnPrescaler;
    err = (long)*pnPrescaler * (long)*pnCounter - (long)lCycles;
    if (ABS(err) > (*pnPrescaler / 2)) {
      (*pnCounter)++;
      err = (long)*pnPrescaler * (long)*pnCounter - (long)lCycles;
    }
    if (ABS(err) < ABS(err_min)) {
      err_min = err;
      a0 = *pnPrescaler;
      b0 = *pnCounter;
      if (err == 0) break;
    }
  }

  *pnPrescaler = a0;
  *pnCounter = b0;
}

/**
* @brief  Configures the GPIO into Low Power
* @param  None
* @retval None.
*/

void RadioSetGpioLowPwr(void)
{
	/*SAMPLE CODE*/
  /* For STM32, gpio power consumption is reduced when GPIOs are configured as
  no pull - analog, during this configuration we have to sure thath wake-up pins
  still remain as digital inputs*/

  /* Below is the Sample code for Putting the GPIOs in LOW power to
     further reduce the consumption.
  // Build the first part of the init structre for all GPIOs and ports
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Mode       = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull       = GPIO_NOPULL;
  GPIO_InitStructure.Speed      = GPIO_SPEED_HIGH;

  // -------------------------------- PORT A -------------------------------------
  // SDN , CS S2-LP, CS E2PROM, UART RX/TX, BUTTON1
  GPIO_InitStructure.Pin = GPIO_PIN_All & (~GPIO_PIN_8) & (~GPIO_PIN_1) & (~GPIO_PIN_9)\
    & (~GPIO_PIN_2) & (~GPIO_PIN_3) & (~GPIO_PIN_4) & (~GPIO_PIN_0)& (~GPIO_PIN_6) & (~GPIO_PIN_7) & (~GPIO_PIN_13) & (~GPIO_PIN_14);
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  // -------------------------------- PORT B -------------------------------------
  GPIO_InitStructure.Pin = GPIO_PIN_All & (~GPIO_PIN_4) & (~GPIO_PIN_0)& (~GPIO_PIN_3);
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  // -------------------------------- PORT C -------------------------------------
  // IRQ Pin + TCXO Enable
  GPIO_InitStructure.Pin = GPIO_PIN_All & (~GPIO_PIN_0)& (~GPIO_PIN_7);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  // -------------------------------- PORT D -------------------------------------
  GPIO_InitStructure.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

  HAL_SuspendTick();      */
}

/**
* @brief  Configures the GPIO into Low Power
* @param  None
* @retval None.
*/

void RadioRestoreGpio(void)
{
	/*SAMPLE CODE*/
  /* For STM32, restore every gpio, previosly set as analog as digital */

  /*Below is the sample code to restore the
      GPIO of the SPI after exit from Low Power
  // Restore all Gpio CLKs
  STM32_GPIO_CLK_ENABLE();

   // SPI_MOSI/MISO
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Mode       = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull       = GPIO_PULLUP;
  GPIO_InitStructure.Speed      = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Alternate  = BUS_SPI1_MISO_GPIO_AF;
  GPIO_InitStructure.Pin        = BUS_SPI1_MISO_GPIO_PIN;
  HAL_GPIO_Init(BUS_SPI1_MISO_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Mode       = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull       = GPIO_PULLUP;
  GPIO_InitStructure.Speed      = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Alternate  = BUS_SPI1_MOSI_GPIO_AF;
  GPIO_InitStructure.Pin        = BUS_SPI1_MOSI_GPIO_PIN;
  HAL_GPIO_Init(BUS_SPI1_MOSI_GPIO_PORT, &GPIO_InitStructure);

  // SPI_CLK
  GPIO_InitStructure.Mode       = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull       = GPIO_PULLUP;
  GPIO_InitStructure.Alternate  = BUS_SPI1_SCK_GPIO_AF;
  GPIO_InitStructure.Pin        = BUS_SPI1_SCK_GPIO_PIN;
  HAL_GPIO_Init(BUS_SPI1_SCK_GPIO_PORT, &GPIO_InitStructure);

  HAL_ResumeTick();  */
}

/**
* @brief  Configures the specified timer to raise an interrupt every time the counter
*         reaches the nPeriod value counting with a prescaler of nPrescaler.
* @note   The specified timer is configured but not enabled.
* @param  TIM_TimeBaseStructure Timer Handler of the timer to be set.
*          This parameter can be a pointer to @ref TIM_HandleTypeDef .
* @param  nPrescaler prescaler factor.
* @param  nPeriod period factor.
* @retval None.
*/
void RadioTimersTimConfig(TIM_HandleTypeDef* TIM_TimeBaseStructure,
                          uint16_t nPrescaler,
                          uint16_t nPeriod)
{

 if(TIM_TimeBaseStructure == (&SFX_TIM_HANDLE))
  {

    BSP_IP_SFX_TIM_Init();

    /* USER CODE END TIM2_Init 1 */
    SFX_TIM_HANDLE.Instance = BSP_SFX_TIM;
    SFX_TIM_HANDLE.Init.Prescaler = nPrescaler;
    SFX_TIM_HANDLE.Init.CounterMode = TIM_COUNTERMODE_UP;
    SFX_TIM_HANDLE.Init.Period = nPeriod;
    SFX_TIM_HANDLE.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    SFX_TIM_HANDLE.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&SFX_TIM_HANDLE) != HAL_OK)
    {
      Error_Handler();
    }
   HAL_TIM_RegisterCallback(&SFX_TIM_HANDLE, HAL_TIM_PERIOD_ELAPSED_CB_ID, SFX_TIM_PeriodElapsedCallback);
  }
}

/****************************** EEPROM *********************************/
/**
* @brief  Read a page of the EEPROM.
*         A page size is 32 bytes.
*         The pages are 256.
*         Page 0 address: 0x0000
*         Page 1 address: 0x0020
*         ...
*         Page 255 address: 0x1FE0
* @param  None
* @retval None
*/
uint8_t EepromRead(uint16_t nAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t  status;
  status = S2868A1_EEPROM_ReadPage(EEPROM_INSTANCE,nAddress, cNbBytes, pcBuffer);

  return(status);
}

/**
* @brief  Write a page of the EEPROM.
*         A page size is 32 bytes.
*         The pages are 256.
*         Page 0 address: 0x0000
*         Page 1 address: 0x0020
*         ...
*         Page 255 address: 0x1FE0
*         It is allowed to write only a page for each operation. If the bytes
*         exceed the single page location, the other bytes are written at the
*         beginning.
* @param  None
* @retval None
*/
uint8_t EepromWrite(uint16_t nAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t status;
 status = (uint8_t) S2868A1_EEPROM_WritePage(EEPROM_INSTANCE, nAddress, cNbBytes,pcBuffer);

 return(status);
}

/******************************** S2-LP *******************************/
/**
* @brief  This function Indiactes SPI Raw Transfer complete
* @param  None
* @retval None
*/
__weak void RadioSpiRawTC(void)
{

}

/**
* @brief  This function is Tx/Rx complete callback
* @param  hspi: Pointer to the SPI handle
* @retval None
*/
void SFX_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
   ResetSpiTxCompleteFlag();

  if(waiting_irq)
  {
    S2868A1_RADIO_SPI_NSS_PIN_HIGH();
    waiting_irq=0;
    RadioSpiRawTC();
  }
}

/**
* @brief  Perform a raw SPI Write transaction with the passed buffer.
*               To perform read or write transactions:
                  - the 1st byte must be the write bytecode (0x00)
                  - 2nd byte must be the register/FIFO address
                  - from the 3rd bytes on the buffer must contain the data to be written
* @param  cNbBytes: number of bytes to be written into TX FIFO
* @param  pInBuffer: pointer to data to write
* @param  pOutBuffer: pointer to data to read
* @param  can_return_before_tc:  if this flag is 1,
*         it means that the function can be non-blocking returning immediatelly.
*               In this case the SPI uses a combination of DMA+IRQ.
* @retval Device status
*/
void RadioSpiRaw(uint8_t cNbBytes,
                        uint8_t* pInBuffer,
                        uint8_t* pOutBuffer,
                        uint8_t can_return_before_tc)
{

  uint8_t* pOutBuffer_ = pOutBuffer;
  volatile uint32_t ctr;
  uint8_t tmpFlag;

  if(pOutBuffer==NULL)
      pOutBuffer_=rx_buff;

  spi_in_use = 1;

  if(can_return_before_tc)
  {
    waiting_irq=1;
    /*Add code to enable SPI_DMA TX and RX interrupt*/
  }
  else
  {
    waiting_irq = 1;
    SetSpiTxCompleteFlag();
  }

  S2868A1_RADIO_SPI_NSS_PIN_LOW();
    for( ctr=0;ctr<DELAY_CS_SCLK;ctr++);
  S2868A1_SPI_SendRecv(pInBuffer, pOutBuffer_, cNbBytes);

  if(!can_return_before_tc)
  {
      do{
	     tmpFlag  = GetSpiTxCompleteFlag();
	    }while(tmpFlag);
    for( ctr=0;ctr<DELAY_CS_SCLK;ctr++);
    S2868A1_RADIO_SPI_NSS_PIN_HIGH();
  }
  spi_in_use = 0;

}

/**
 * @brief  Sets the Flag to denote the SPI transmission complete
 * @param  None
 * @retval None
 */
static void SetSpiTxCompleteFlag(void)
{
  SpiTxCpltFlag = 1;
}

/**
 * @brief  Resets the Flag after SPI transmission complete
 * @param  None
 * @retval None
 */
static void ResetSpiTxCompleteFlag(void)
{
  SpiTxCpltFlag = 0;

}

/**
 * @brief  Return the SPI transmission complete flag
 * @param  None
 * @retval Flag status
 */
static uint8_t GetSpiTxCompleteFlag(void)
{
  return (SpiTxCpltFlag);
}

/**
 * @brief  Front End Module Operation function.
 * This function configures the PA according to the desired status.
 * This function can be redefined for special needs.
 * @param  operation Specifies the operation to perform.
 *         This parameter can be one of following parameters:
 *         @arg FEM_SHUTDOWN: Shutdown PA
 *         @arg FEM_TX_BYPASS: Bypass the PA in TX
 *         @arg FEM_TX: TX mode
 *         @arg FEM_RX: RX mode
 * @retval None
 */
void FEM_Operation(FEM_OperationType operation)
{
}

/**
* @brief  Set SPI in use Flag.
* @param  None
* @retval None
*/
void S2LPSetSpiInUse(uint8_t state)
{
  spi_in_use = state;
}

/**
* @brief  Get whether SPI in use.
* @param  None
* @retval None
*/
uint8_t S2LPGetSpiInUse(void)
{
  return spi_in_use;
}

/*
* @brief  Configure RTC clock.
* @param  None
* @retval None
*/
void Config_RTC_Clk(void)
{
  RCC_OscInitTypeDef        RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

  HAL_PWR_EnableBkUpAccess();
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

}

/**
* @brief  Detect Power Amplifier
* @retval Range Extender Type
*/
RangeExtType DetetctPA(void)
{
#if S2LP_FEM_PRESENT == S2LP_FEM_NO
  	return RANGE_EXT_NONE;
#else
#ifdef MON_REF_DES
	return RANGE_EXT_SKYWORKS_SKY66420;
#else
	return RANGE_EXT_CUSTOM;
#endif
#endif
}

/**
 * @brief  Opens Sigfox Library according to the zone.
 * @param rcz The Radio Zone
 * @retval Returns 0 if ok.
 */
ST_SFX_ERR St_Sigfox_Open_RCZ(uint8_t rcz)
{
  ST_SFX_ERR open_err = ST_SFX_ERR_NONE;

  switch(rcz)
  {
    case 1:
      {
        /* Turn PA off in RC1/3/5/6/7 */
        ST_RF_API_set_pa(0);
        /* RC1 - open the Sigfox library */
        if(SIGFOX_API_open(&(sfx_rc_t)RC1)!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }
        break;
      }
    case 2:
      {
        /* Turn PA off in RC2 and RC4 */
        ST_RF_API_set_pa(1);
        /* RC2 - open the Sigfox library */
        if(SIGFOX_API_open(&(sfx_rc_t)RC2)!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }

        /* In FCC we can choose the macro channel to use by a 86 bits bitmask
        In this case we use the first 9 macro channels */
        sfx_u32 config_words[3]=RC2_SM_CONFIG;

        /* Set the standard configuration with default channel to 1 */
        if(SIGFOX_API_set_std_config(config_words,0)!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }
        break;
      }
    case 3:
      {
        volatile uint8_t ret;
        /* Turn PA off in RC1/3/5/6/7 */
        ST_RF_API_set_pa(0);
        ret=SIGFOX_API_open(&(sfx_rc_t)RC3C);
        /* RC3 - open the Sigfox library */
        if(ret!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }

        /* In FCC we can choose the macro channel to use by a 86 bits bitmask
        In this case we use 9 consecutive macro channels starting from 63 (920.8MHz) */
        sfx_u32 config_words[3]=RC3C_CONFIG;

        /* Set the standard configuration with default channel to 63 */
        if(SIGFOX_API_set_std_config(config_words,0)!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }
        break;
      }
    case 4:
      {
        volatile uint8_t ret;
        /* Turn PA off in RC2 and RC4 */
        ST_RF_API_set_pa(1);

        ret=SIGFOX_API_open(&(sfx_rc_t)RC4);
        /* RC4 - open the Sigfox library */
        if(ret!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }

        /* In FCC we can choose the macro channel to use by a 86 bits bitmask
        In this case we use 9 consecutive macro channels starting from 63 (920.8MHz) */
        sfx_u32 config_words[3]=RC4_SM_CONFIG;

        /* Set the standard configuration with default channel to 63 */
        if(SIGFOX_API_set_std_config(config_words,1)!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }
        break;
      }
    case 5:
      {
        volatile uint8_t ret;
        /* Turn PA off in RC1/3/5/6/7 */
        ST_RF_API_set_pa(0);
        ret=SIGFOX_API_open(&(sfx_rc_t)RC5);
        /* RC5 - open the Sigfox library */
        if(ret!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }

        /* In FCC we can choose the macro channel to use by a 86 bits bitmask
        In this case we use 9 consecutive macro channels starting from 63 (920.8MHz) */
        sfx_u32 config_words[3]=RC5_CONFIG;

        /* Set the standard configuration with default channel to 63 */
        if(SIGFOX_API_set_std_config(config_words,0)!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }
        break;
      }
    case 6:
      {
        /* Turn PA off in RC1/3/5/6/7 */
        ST_RF_API_set_pa(0);
        /* RC6 - open the Sigfox library */
        if(SIGFOX_API_open(&(sfx_rc_t)RC6)!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }
        break;
      }
    case 7:
      {
        /* Turn PA off in RC1/3/5/6/7 */
        ST_RF_API_set_pa(0);
        /* RC7 - open the Sigfox library */
        if(SIGFOX_API_open(&(sfx_rc_t)RC7)!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }
        break;
      }
    default:
      {
        /* Stuck the application for a out of range number */
        open_err = ST_SFX_ERR_RC_UNKNOWN;
        break;
      }
    }
  return open_err;
}
/**
* @brief  System main function.
* @param  *sfxConfig: Pointer to the xFxConfig
* @param  openAfterInit: Open after init flag
* @retval ST_SFX_ERR : Sigfox Error status
*/
ST_SFX_ERR ST_Sigfox_Init(NVM_BoardDataType *sfxConfig, uint8_t openAfterInit)
{
  ST_SFX_ERR ret_err = ST_SFX_ERR_NONE;

  /* Configure XTAL frequency and offset for the RF Library */
  ST_RF_API_set_xtal_freq(S2LPManagementGetXtalFrequency());

  /* Macro that defines and initializes the nvmconfig structure */
  INIT_NVM_CONFIG(nvmConfig);

  /* Sigfox Credentials Management */

  /* Configure the NVM_API */
  SetNVMInitial(&nvmConfig);

  ST_MCU_API_Shutdown(1);
  HAL_Delay(1);

  /* Set EEPROM CS */
  S2868A1_RADIO_SPI_NSS_PIN_HIGH();

  /* Retrieve Sigfox info from EEPROM */
  if(enc_utils_retrieve_data(&sfxConfig->id, sfxConfig->pac, &sfxConfig->rcz) != 0)
    ret_err = ST_SFX_ERR_CREDENTIALS;
  else
    sfxConfig->freqOffset = S2LPManagementGetXtalFrequency();

  /* If the retriever returns an error (code different from ST_SFX_ERR_NONE) the application will halt */
  /* Otherwise, open the Sigfox Library according to the zone stored in the device */
  if(openAfterInit && ret_err == ST_SFX_ERR_NONE)
    ret_err = St_Sigfox_Open_RCZ(sfxConfig->rcz);

  return ret_err;
}

/**
* @brief  Blink the LED indefinitely stucking the application.
* @param  None
* @retval None
*/
/*static void Fatal_Error(void)
{
  BSP_LED_Init(LED2);
  while(1)
  {
    HAL_Delay(100);
    BSP_LED_Toggle(LED2);
  }
}*/

/**
* @brief  Gets the XTAL frequency.
* @param  None
* @retval The XTAL frequency
*/
uint32_t S2LPManagementGetXtalFrequency(void)
{
  return s_XtalFrequency;
}

/**
* @brief  This function is to query if EEPROM is present or not.
* @param  None
* @retval 1 (yes) or 0 (no).
*/
uint8_t RadioGetHasEeprom(void)
{
#if EEPROM_PRESENT == EEPROM_YES
  return s_eeprom;
#else
  return 0;
#endif
}

/**
* @brief  This function is to set if EEPROM is present or not.
* @param  1 (yes) or 0 (no).
* @retval None
*/
void RadioSetHasEeprom(uint8_t eeprom)
{
#if EEPROM_PRESENT == EEPROM_YES
  s_eeprom = eeprom;
#else
  s_eeprom = 0;
#endif
}

/**
 * @brief  Puts at logic 1 the TCXO pin.
 * @param  None.
 * @retval None.
 */
void RadioTcxoOn(void)
{
  S2868A1_RADIO_EnableTCXO();
}

/**
* @brief  print output result
* @param  status.
* @param  rssi.
* @retval None
*/
void ST_MANUF_report_CB(uint8_t status, int32_t rssi)
{
}

#ifdef __ICCARM__
/**
 * @brief  TCXO initialization function.
 * This function automatically sets the TCXO according to
 *   the information stored in the device EEPROM.
 * This function can be redefined for special needs.
 * @param  None
 * @retval None
 */
__weak void TCXO_Init()
{
	/* Not implemented: at the moment there isn't any eval kit with TCXO */
}

/**
 * @brief  TCXO Operation function.
 * This function configures the TCXO according to the desired status.
 * This function can be redefined for special needs.
 * @param  operation Specifies the operation to perform.
 *         This parameter can be one of following parameters:
 *         @arg TCXO_ON: Turns on TCXO
 *         @arg TCXO_OFF: Turns off TCXO
 * @retval None
 */
__weak void TCXO_Operation(TCXO_OperationType operation)
{
	/* Not implemented: at the moment there isn't any eval kit with TCXO */
}

#endif

#ifdef __CC_ARM
/**
 * @brief  TCXO initialization function.
 * This function automatically sets the TCXO according to
 *   the information stored in the device EEPROM.
 * This function can be redefined for special needs.
 * @param  None
 * @retval None
 */
void __attribute__((weak)) TCXO_Init()
{
	/* Not implemented: at the moment there isn't any eval kit with TCXO */
}

/**
 * @brief  TCXO Operation function.
 * This function configures the TCXO according to the desired status.
 * This function can be redefined for special needs.
 * @param  operation Specifies the operation to perform.
 *         This parameter can be one of following parameters:
 *         @arg TCXO_ON: Turns on TCXO
 *         @arg TCXO_OFF: Turns off TCXO
 * @retval None
 */
void __attribute__((weak)) TCXO_Operation(TCXO_OperationType operation)
{
	/* Not implemented: at the moment there isn't any eval kit with TCXO */
}

#endif

#ifdef __GNUC__
/**
 * @brief  TCXO initialization function.
 * This function automatically sets the TCXO according to
 *   the information stored in the device EEPROM.
 * This function can be redefined for special needs.
 * @param  None
 * @retval None
 */
void __attribute__((weak)) TCXO_Init()
{
	/* Not implemented: at the moment there isn't any eval kit with TCXO */
}

/**
 * @brief  TCXO Operation function.
 * This function configures the TCXO according to the desired status.
 * This function can be redefined for special needs.
 * @param  operation Specifies the operation to perform.
 *         This parameter can be one of following parameters:
 *         @arg TCXO_ON: Turns on TCXO
 *         @arg TCXO_OFF: Turns off TCXO
 * @retval None
 */
void __attribute__((weak)) TCXO_Operation(TCXO_OperationType operation)
{
	/* Not implemented: at the moment there isn't any eval kit with TCXO */
}

#endif

/**
* @brief  Sigfox Time period elasped Callback.
* @param  None
* @retval None
*/
void SFX_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == BSP_SFX_TIM)
  {
    SFX_TIM_Callback();
    RadioTimersState(htim, DISABLE);
  }
}

/**
* @brief  Sigfox TImer Callback.
* @param  None
* @retval None
*/
void SFX_TIM_Callback(void)
{
  ST_RF_API_Timer_Channel_Clear_CB();
}

/**
* @brief  Sigfox Clear Wake up flag.
* @param  None
* @retval None
*/
void SFX_ClearWKUP_Flag(void)
{
  __HAL_RTC_CLEAR_FLAG(RTC_EXTI_LINE_WAKEUPTIMER_EVENT);
}

/**
* @brief  Sigfox Set Wakeup timer.
* @param  None
* @retval None
*/
void SFX_SetWKUP_Timer(uint32_t CtrDiv)
{
  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,CtrDiv,RTC_WAKEUPCLOCK_RTCCLK_DIV16);
}

/**
  * @brief  BSP Push Button callback
  * @param  Button Specifies the pin connected EXTI line
  * @retval None.
  */

#ifdef __cplusplus
}
#endif

