/**
  * @file    bsp_ip_conf.h
  * @author  MEMS Application Team
  * @brief   BSP IP configuration file
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
#ifndef __BSP_IP_CONF_H__
#define __BSP_IP_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Exported variables --------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/

#define BSP_SFX_TIM           TIM3
#define BSP_IP_SFX_TIM_Init   MX_TIM3_Init
#define SFX_TIM_HANDLE        htim3
#define BSP_SFX_TIM_CLK_ENABLE()    __HAL_RCC_TIM3_CLK_ENABLE()

#ifdef __cplusplus
}
#endif

#endif /* __BSP_IP_CONF_H__ */

