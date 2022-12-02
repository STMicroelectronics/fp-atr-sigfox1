/**
 ******************************************************************************
 * @file    gnss.h
 * @author  SRA
 * @brief   This header file contains the functions prototypes for the
 *          gnss driver
 ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GNSS_H
#define GNSS_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup COMMON COMMON
 * @{
 */

/** @addtogroup GNSS GNSS
 * @{
 */

/** @addtogroup GNSS_Public_Types GNSS Public types
 * @{
 */

/**
 * @brief GNSS driver structure definition
 */
typedef struct
{
  int32_t      ( *Init           ) ( void * );
  int32_t      ( *DeInit         ) ( void * );
  const void * ( *GetMessage     ) ( void * );
  int32_t      ( *ReleaseMessage ) ( void *, const void * );
  int32_t      ( *Send           ) ( void *, const void * );
} GNSS_Drv_t;

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

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* GNSS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
