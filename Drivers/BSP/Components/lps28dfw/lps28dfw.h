/**
  ******************************************************************************
  * @file    lps28dfw.h
  * @author  MEMS Software Solutions Team
  * @brief   LPS28DFW header driver file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#ifndef LPS28DFW_H
#define LPS28DFW_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "lps28dfw_reg.h"
#include <string.h>

/** @addtogroup BSP BSP
  * @{
  */

/** @addtogroup Component Component
  * @{
  */

/** @addtogroup LPS28DFW LPS28DFW
  * @{
  */

/** @defgroup LPS28DFW_Exported_Types LPS28DFW Exported Types
  * @{
  */

typedef int32_t (*LPS28DFW_Init_Func)(void);
typedef int32_t (*LPS28DFW_DeInit_Func)(void);
typedef int32_t (*LPS28DFW_GetTick_Func)(void);
typedef int32_t (*LPS28DFW_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*LPS28DFW_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

typedef struct
{
  LPS28DFW_Init_Func          Init;
  LPS28DFW_DeInit_Func        DeInit;
  uint32_t                   BusType; /*0 means I2C, 1 means SPI 4-Wires, 2 means SPI-3-Wires */
  uint8_t                    Address;
  LPS28DFW_WriteReg_Func      WriteReg;
  LPS28DFW_ReadReg_Func       ReadReg;
  LPS28DFW_GetTick_Func       GetTick;
} LPS28DFW_IO_t;

typedef struct
{
  LPS28DFW_IO_t        IO;
  stmdev_ctx_t        Ctx;
  uint8_t             is_initialized;
  uint8_t             press_is_enabled;
  uint8_t             temp_is_enabled;
  lps28dfw_md_t        last_odr;
} LPS28DFW_Object_t;

typedef struct
{
  uint8_t Temperature;
  uint8_t Pressure;
  uint8_t Humidity;
  uint8_t LowPower;
  float   HumMaxOdr;
  float   TempMaxOdr;
  float   PressMaxOdr;
} LPS28DFW_Capabilities_t;

typedef struct
{
  int32_t (*Init)(LPS28DFW_Object_t *);
  int32_t (*DeInit)(LPS28DFW_Object_t *);
  int32_t (*ReadID)(LPS28DFW_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(LPS28DFW_Object_t *, LPS28DFW_Capabilities_t *);
} LPS28DFW_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(LPS28DFW_Object_t *);
  int32_t (*Disable)(LPS28DFW_Object_t *);
  int32_t (*GetOutputDataRate)(LPS28DFW_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LPS28DFW_Object_t *, float);
  int32_t (*GetTemperature)(LPS28DFW_Object_t *, float *);
} LPS28DFW_TEMP_Drv_t;

typedef struct
{
  int32_t (*Enable)(LPS28DFW_Object_t *);
  int32_t (*Disable)(LPS28DFW_Object_t *);
  int32_t (*GetOutputDataRate)(LPS28DFW_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LPS28DFW_Object_t *, float);
  int32_t (*GetPressure)(LPS28DFW_Object_t *, float *);
} LPS28DFW_PRESS_Drv_t;

typedef union
{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} lps28dfw_axis3bit16_t;

typedef union
{
  int16_t i16bit;
  uint8_t u8bit[2];
} lps28dfw_axis1bit16_t;

typedef union
{
  int32_t i32bit[3];
  uint8_t u8bit[12];
} lps28dfw_axis3bit32_t;

typedef union
{
  int32_t i32bit;
  uint8_t u8bit[4];
} lps28dfw_axis1bit32_t;

/**
  * @}
  */

/** @defgroup LPS28DFW_Exported_Constants LPS28DFW Exported Constants
  * @{
  */

#define LPS28DFW_OK                0
#define LPS28DFW_ERROR            -1

#define LPS28DFW_I2C_BUS          0U

/**
  * @}
  */

/** @addtogroup LPS28DFW_Exported_Functions LPS28DFW Exported Functions
  * @{
  */

int32_t LPS28DFW_RegisterBusIO(LPS28DFW_Object_t *pObj, LPS28DFW_IO_t *pIO);
int32_t LPS28DFW_Init(LPS28DFW_Object_t *pObj);
int32_t LPS28DFW_DeInit(LPS28DFW_Object_t *pObj);
int32_t LPS28DFW_ReadID(LPS28DFW_Object_t *pObj, uint8_t *Id);
int32_t LPS28DFW_GetCapabilities(LPS28DFW_Object_t *pObj, LPS28DFW_Capabilities_t *Capabilities);
int32_t LPS28DFW_Get_Init_Status(LPS28DFW_Object_t *pObj, uint8_t *Status);

int32_t LPS28DFW_PRESS_Enable(LPS28DFW_Object_t *pObj);
int32_t LPS28DFW_PRESS_Disable(LPS28DFW_Object_t *pObj);
int32_t LPS28DFW_PRESS_GetOutputDataRate(LPS28DFW_Object_t *pObj, float *Odr);
int32_t LPS28DFW_PRESS_SetOutputDataRate(LPS28DFW_Object_t *pObj, float Odr);
int32_t LPS28DFW_PRESS_GetPressure(LPS28DFW_Object_t *pObj, float *Value);
int32_t LPS28DFW_PRESS_Get_DRDY_Status(LPS28DFW_Object_t *pObj, uint8_t *Status);

int32_t LPS28DFW_TEMP_Enable(LPS28DFW_Object_t *pObj);
int32_t LPS28DFW_TEMP_Disable(LPS28DFW_Object_t *pObj);
int32_t LPS28DFW_TEMP_GetOutputDataRate(LPS28DFW_Object_t *pObj, float *Odr);
int32_t LPS28DFW_TEMP_SetOutputDataRate(LPS28DFW_Object_t *pObj, float Odr);
int32_t LPS28DFW_TEMP_GetTemperature(LPS28DFW_Object_t *pObj, float *Value);
int32_t LPS28DFW_TEMP_Get_DRDY_Status(LPS28DFW_Object_t *pObj, uint8_t *Status);

int32_t LPS28DFW_Read_Reg(LPS28DFW_Object_t *pObj, uint8_t reg, uint8_t *Data);
int32_t LPS28DFW_Write_Reg(LPS28DFW_Object_t *pObj, uint8_t reg, uint8_t Data);

int32_t LPS28DFW_Set_One_Shot(LPS28DFW_Object_t *pObj);
int32_t LPS28DFW_Get_One_Shot_Status(LPS28DFW_Object_t *pObj, uint8_t *Status);

int32_t LPS28DFW_Enable_DRDY_Interrupt(LPS28DFW_Object_t *pObj);
int32_t LPS28DFW_Set_AVG(LPS28DFW_Object_t *pObj, uint8_t avg);
int32_t LPS28DFW_Set_LPF(LPS28DFW_Object_t *pObj, uint8_t lpf);

/**
  * @}
  */

/** @addtogroup LPS28DFW_Exported_Variables LPS28DFW Exported Variables
  * @{
  */
extern LPS28DFW_CommonDrv_t LPS28DFW_COMMON_Driver;
extern LPS28DFW_PRESS_Drv_t LPS28DFW_PRESS_Driver;
extern LPS28DFW_TEMP_Drv_t LPS28DFW_TEMP_Driver;

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
