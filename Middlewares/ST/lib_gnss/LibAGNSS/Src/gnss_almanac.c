/**
  ******************************************************************************
  * @file    gnss_almanac.c
  * @author  SRA
  * @brief   Implements API for Assisted GNSS Almanac 
  *          - LibAGNSS module middleware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "gnss_almanac.h"

/* Defines -------------------------------------------------------------------*/
#define XYBRID_RT_GPS_ALM_SIZE  (31U)
#define XYBRID_RT_GLO_ALM_SIZE  (29U)
#define XYBRID_RT_BEI_ALM_SIZE  (34U)
#define XYBRID_RT_GAL_ALM_SIZE  (21U)

/* Global variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

// GPS
static uint8_t *rt_decode_alm_gps_hdr(uint8_t *rt_payload, assist_almgps_hdr_t *hdr_ptr)
{
  hdr_ptr->week_n = (rt_payload[0] * 256U) + rt_payload[1];
  hdr_ptr->num_sats = rt_payload[2];

  return &(rt_payload[3]);
}

// GLO
static uint8_t *rt_decode_alm_glo_hdr(uint8_t *rt_payload, assist_almglo_hdr_t *hdr_ptr)
{
  hdr_ptr->num_sats = rt_payload[0];
  hdr_ptr->week_n = rt_payload[1];
  hdr_ptr->sToa = rt_payload[2];

  return &(rt_payload[4]);
}

// BEI
static uint8_t *rt_decode_alm_bei_hdr(uint8_t *rt_payload, assist_almbei_hdr_t *hdr_ptr)
{
  hdr_ptr->num_sats = rt_payload[0];
  hdr_ptr->week_n = rt_payload[1];
  hdr_ptr->sToa = rt_payload[2];

  return &(rt_payload[4]);
}

// GAL
static uint8_t *rt_decode_alm_gal_hdr(uint8_t *rt_payload, assist_almgal_hdr_t *hdr_ptr)
{
  hdr_ptr->num_sats = rt_payload[0];
  hdr_ptr->sToa=rt_payload[1];
  hdr_ptr->ioda=rt_payload[5];

  return &(rt_payload[6]);
}

/* GPS */
static void rt_decode_gps_alm(almanac_raw_t *stm_alm, uint8_t *sat_payload, uint16_t week, uint32_t sat_idx)
{
  uint8_t *alm_ptr;
  uint8_t byte;
  uint16_t hword;
  uint32_t dword;
  uint32_t idx = 0;

  alm_ptr = &sat_payload[XYBRID_RT_GPS_ALM_SIZE * sat_idx];
  (void)memset(stm_alm, 0, sizeof(almanac_raw_t));

  extract_byte(&byte, alm_ptr, idx * 8U);
  stm_alm->gps.satid = byte + 1U;
  idx++;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->gps.eccentricity = hword;
  idx += 2U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->gps.toa = hword;
  idx += 2U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->gps.delta_i = hword;
  idx += 2U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->gps.omega_dot = hword;
  idx += 2U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->gps.health = hword;
  idx += 2U;
  extract_dword(&dword, alm_ptr, idx * 8U);
  stm_alm->gps.root_a = dword;
  idx += 4U;
  extract_dword(&dword, alm_ptr, idx * 8U);
  stm_alm->gps.omega_zero = dword;
  idx += 4U;
  extract_dword(&dword, alm_ptr, idx * 8U);
  stm_alm->gps.perigee = dword;
  idx += 4U;
  extract_dword(&dword, alm_ptr, idx * 8U);
  stm_alm->gps.mean_anomaly = dword;
  idx += 4U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->gps.af0 = hword;
  idx += 2U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->gps.af1 = hword;

  stm_alm->gps.week = week;
  stm_alm->gps.available = 1;
}

/* GLONASS */
static uint8_t rt_decode_glo_alm(almanac_raw_t *stm_alm, uint8_t *sat_payload, uint16_t week, uint32_t sat_idx)
{
  uint8_t *alm_ptr;
  uint8_t byte;
  uint16_t hword;
  uint32_t dword;
  uint32_t idx = 0;
  uint8_t can;

  alm_ptr = &sat_payload[XYBRID_RT_GLO_ALM_SIZE * sat_idx];
  (void)memset(stm_alm, 0, sizeof(almanac_raw_t));

  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->glo.NA = hword;
  idx += 2U;
  extract_byte(&byte, alm_ptr, idx * 8U);
  stm_alm->glo.n_A = byte;
  idx++;
  extract_byte(&byte, alm_ptr, idx * 8U);
  stm_alm->glo.H_n_A = byte;
  idx++;
  extract_dword(&dword, alm_ptr, idx * 8U);
  stm_alm->glo.lambda_n_A = sign_extend_1c(dword, 21);
  idx += 4U;
  extract_dword(&dword, alm_ptr, idx * 8U);
  stm_alm->glo.t_lambda_n_A = dword;
  idx += 4U;
  extract_dword(&dword, alm_ptr, idx * 8U);
  stm_alm->glo.delta_i_n_A = sign_extend_1c(dword, 18);
  idx += 4U;
  extract_dword(&dword, alm_ptr, idx * 8U);
  stm_alm->glo.delta_T_n_A = sign_extend_1c(dword, 22);
  idx += 4U;
  extract_byte(&byte, alm_ptr, idx * 8U);
  stm_alm->glo.delta_T_n_dot_A = sign_extend_1c(byte, 7);
  idx++;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->glo.epsilon_n_A = hword;
  idx += 2U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->glo.omega_n_A = sign_extend_1c(hword, 16);
  idx += 2U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->glo.tau_n_A = sign_extend_1c(hword, 10);
  idx += 2U;
  extract_byte(&byte, alm_ptr, idx * 8U);
  can = byte;
  idx++;
  extract_byte(&byte, alm_ptr, idx * 8U);
  stm_alm->glo.M_n_A = byte;

  stm_alm->glo.week = week;
  stm_alm->glo.available = 1;
  return can;
}

/* BEIDOU */
static void rt_decode_bei_alm(almanac_raw_t *stm_alm, uint8_t *sat_payload, uint16_t week, uint32_t sat_idx)
{
  uint8_t *alm_ptr;
  uint8_t byte;
  uint16_t hword;
  uint32_t dword;
  uint32_t idx = 0;

  alm_ptr = &sat_payload[XYBRID_RT_BEI_ALM_SIZE * sat_idx];
  (void)memset(stm_alm, 0, sizeof(almanac_raw_t));

  extract_byte(&byte, alm_ptr, idx * 8U);
  stm_alm->compass.prn = byte + 1U;
  idx++;
  extract_byte(&byte, alm_ptr, idx * 8U);
  stm_alm->compass.toa = byte + 1U;
  idx++;
  extract_dword(&dword, alm_ptr, idx * 8U);
  stm_alm->compass.root_a = dword;
  idx += 4U;
  extract_dword(&dword, alm_ptr, idx * 8U);
  stm_alm->compass.eccentricity = dword;
  idx += 4U;
  extract_dword(&dword, alm_ptr, idx * 8U);
  stm_alm->compass.perigee = dword;
  idx += 4U;
  extract_dword(&dword, alm_ptr, idx * 8U);
  stm_alm->compass.mean_anomaly = dword;
  idx += 4U;
  extract_dword(&dword, alm_ptr, idx * 8U);
  stm_alm->compass.omega_zero = dword;
  idx += 4U;
  extract_dword(&dword, alm_ptr, idx * 8U);
  stm_alm->compass.omega_dot = dword;
  idx += 4U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->compass.delta_i = hword;
  idx += 2U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->compass.af0 = hword;
  idx += 2U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->compass.af1 = hword;
  idx += 2U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->compass.health = (hword >> 8) & (uint16_t)0xff;

  stm_alm->compass.week = week;
  stm_alm->compass.available = 1;
}

/* GALILEO */
static void rt_decode_gal_alm(almanac_raw_t *stm_alm, uint8_t *sat_payload, UINT4 toa, BYTE ioda, uint32_t sat_idx)
{
  uint8_t *alm_ptr;
  uint8_t byte;
  uint16_t hword;
  uint32_t idx = 0;
  
  alm_ptr = &sat_payload[XYBRID_RT_GAL_ALM_SIZE * sat_idx];
  (void)memset(stm_alm, 0, sizeof(almanac_raw_t));

  stm_alm->galileo.toa=toa;
  stm_alm->galileo.ioda_1=ioda;
  stm_alm->galileo.ioda_2=ioda;

  stm_alm->galileo.word_available = 1;
  extract_byte(&byte, alm_ptr, idx * 8U);
  stm_alm->galileo.svid = byte - 1U;//in future it will be svid
  idx++;
  //current svid is incorrect, it start from 1. RXN will correct in future
  stm_alm->galileo.satid = stm_alm->galileo.svid + MIN_GALILEO_SAT_ID - 1U;

  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->galileo.eccentricity = hword;
  idx += 2U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->galileo.delta_i = hword;
  idx += 2U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->galileo.omega_dot = hword;
  idx += 2U;
  extract_byte(&byte, alm_ptr, idx * 8U);
  stm_alm->galileo.E1B_HS = byte & (uint8_t)0x0C;//4 bits string, only 2 MSB here
  stm_alm->galileo.E1B_HS = byte & (uint8_t)0x03;//4 bits string, only 2 LSB here
  idx += 1U;
  extract_byte(&byte, alm_ptr, idx * 8U);
  stm_alm->galileo.health = byte;
  idx += 1U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->galileo.delta_a = hword;
  idx += 2U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->galileo.omega_zero = hword;
  idx += 2U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->galileo.week = hword;//(sW)
  idx += 2U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->galileo.mean_anomaly = hword;
  idx += 2U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->galileo.af0 = hword;
  idx += 2U;
  extract_hword(&hword, alm_ptr, idx * 8U);
  stm_alm->galileo.af1 = hword;

  /* Redundant assignment
  stm_alm->galileo.spare0=0;
  stm_alm->galileo.spare1=0;
  stm_alm->galileo.spare2=0;
  stm_alm->galileo.spare3=0;
  stm_alm->galileo.spare4=0;
  stm_alm->galileo.spare5=0;
  stm_alm->galileo.spare6=0;
  stm_alm->galileo.spare7=0;
  stm_alm->galileo.spare8=0;
  */
}

/* Public functions ----------------------------------------------------------*/

uint16_t A_GNSS_DecodeAlmRT(seed_type_t seed_type,
                            seed_info_t *seed_info,
                            uint16_t week,
                            current_timedate_t *utc_time)
{
  uint16_t w, wn;
  uint32_t sat_id[64];
  uint32_t i;

  // Save the current address of the ALM seed to be recovered at the end of decoding
  uint8_t *temp = seed_info->alm_seed;

  almanac_raw_t *stm_alm = (almanac_raw_t *)seed_info->stm_alm;
  wn = 0;

  // GPS ALM
  if(seed_type == GPS_SEED)
  {
    assist_almgps_hdr_t hdr;

    seed_info->alm_seed = rt_decode_alm_gps_hdr(seed_info->alm_seed, &hdr);
    w = ((uint16_t)(week / 256U) * 256U) + hdr.week_n;
    seed_info->num_sat_alm = hdr.num_sats;

    AGNSS_PRINT_OUT("\n[GPS ALM] (num_sats=%d):\r\n", hdr.num_sats);

    for (i = 0; i < hdr.num_sats; i++)
    {
      rt_decode_gps_alm(&stm_alm[i], seed_info->alm_seed, w, i);
      sat_id[i] = stm_alm[i].gps.satid;
      AGNSS_PRINT_OUT("[GPS ALM] SAT ID: %d\r\n",sat_id[i]);

      if (stm_alm[i].gps.health == 0U)
      {
        if (sat_id[i] > 0U)
        {
          wn = (uint16_t)(stm_alm[i].gps.week);
        }
        else
        {
          AGNSS_PRINT_OUT("[GPS ALM] Warning: unhealty almanac!\n");
        }
      }
    }
  }
  // GLO ALM
  else if(seed_type == GLO_SEED)
  {
    assist_almglo_hdr_t hdr;
    uint8_t can[24];

    w = (uint16_t)(((((utc_time->Year-1U) - 1980U) * 365U) + (((utc_time->Year-1U) - 1980U)/4U) + (day_of_year(utc_time)-1U))/7U);

    seed_info->alm_seed = rt_decode_alm_glo_hdr(seed_info->alm_seed, &hdr);
    w = ((uint16_t)(w / 256U) * 256U) + hdr.week_n;

    seed_info->num_sat_alm = hdr.num_sats;

    AGNSS_PRINT_OUT("\n[GLO ALM] (num_sats=%d):\r\n", hdr.num_sats);

    for (i = 0; i < hdr.num_sats; i++)
    {
      can[i] = rt_decode_glo_alm(&stm_alm[i], seed_info->alm_seed, w, i);
      stm_alm[i].glo.toa = hdr.sToa;
      stm_alm[i].glo.N4 = N4;
      sat_id[i] = (uint32_t) prn2slot[(stm_alm[i].glo.n_A - 1U)];
      stm_alm[i].glo.satid = sat_id[i];
      AGNSS_PRINT_OUT("[GLO ALM] SAT ID: %d\r\n",sat_id[i]);

      if (can[i] == 0U)
      {
        stm_alm[i].glo.health = 1;
        AGNSS_PRINT_OUT("[GLO ALM] Warning: unhealty almanac!\n");
      }
      else
      {
        stm_alm[i].glo.health = 0;
      }
    }
  }
  // BEI ALM
  else if(seed_type == BEI_SEED)
  {
    assist_almbei_hdr_t hdr;
    
    seed_info->alm_seed = rt_decode_alm_bei_hdr(seed_info->alm_seed, &hdr);
    seed_info->num_sat_alm = hdr.num_sats;

    AGNSS_PRINT_OUT("\n[BEI ALM] (num_sats=%d):\r\n", hdr.num_sats);
    
    for (i = 0; i < hdr.num_sats; i++)
    {
      rt_decode_bei_alm(&stm_alm[i], seed_info->alm_seed, week, i);
      sat_id[i] = (((stm_alm[i].compass.prn) + MIN_COMPASS_SAT_ID) - 1U);
      stm_alm[i].compass.toa = hdr.sToa;
      AGNSS_PRINT_OUT("[BEI ALM] SAT ID: %d\r\n",sat_id[i]);

      if(sat_id[i] <= 145U)
      {
        stm_alm[i].compass.is_geo = 1;
      }
      else 
      {
        stm_alm[i].compass.is_geo = 0;
      }
      if (stm_alm[i].compass.health == 0U)
      {
        if (sat_id[i] > 0U)
        {
          wn = (uint16_t)(stm_alm[i].compass.week);
        }
        else
        {
          AGNSS_PRINT_OUT("[BEI ALM] Warning: unhealty almanac!\n");
        }
      }
    }
  }
  // GAL ALM
  else if(seed_type == GAL_SEED)
  {
    assist_almgal_hdr_t hdr;
    
    //w=week>>20;
    
    seed_info->alm_seed = rt_decode_alm_gal_hdr(seed_info->alm_seed, &hdr);
    //week is not available in header
    //w = (unsigned short)(w / 256) * 256 + hdr.week_n;
    seed_info->num_sat_alm = hdr.num_sats;

    AGNSS_PRINT_OUT("\n[GAL ALM] (num_sats=%d):\r\n", hdr.num_sats);

    for (i = 0; i < hdr.num_sats; i++)
    {
      rt_decode_gal_alm(&stm_alm[i], seed_info->alm_seed, hdr.sToa, hdr.ioda, i);
      sat_id[i] = stm_alm[i].galileo.satid;
      AGNSS_PRINT_OUT("[GAL ALM] SAT ID: %d\r\n",sat_id[i]);

      if (stm_alm[i].galileo.health == 0U)
      {
        if (sat_id[i] > 0U)
        {
          wn = (uint16_t)(stm_alm[i].galileo.week);
        }
        else
        {
          AGNSS_PRINT_OUT("[GAL ALM] Warning: unhealty almanac!\n");
        }
      }
    }
  }
  else
  {
    /* do nothing */
  }

  // After conversion, recover the original address of the ALM seed
  seed_info->alm_seed = temp;

  return wn;
}

void A_GNSS_SendAlmRT(seed_type_t seed_type, seed_info_t *seed_info)
{
  uint32_t i;
  int32_t index;
  uint32_t index2;
  uint32_t checksum;
  uint8_t *apointer;
  uint8_t gnss_cmd[256];

  const uint8_t nmea_cmd_alm[] = "$PSTMALMANAC";
  almanac_raw_t *stm_alm = (almanac_raw_t *)seed_info->stm_alm;
  uint32_t num_sat_alm = seed_info->num_sat_alm;
  uint8_t *sat_id = seed_info->sat_id;

  //GPS ALM
  if(seed_type == GPS_SEED)
  {
    for (i = 0; i < num_sat_alm; i++)
    {
      apointer = (uint8_t *)(&stm_alm[i].gps);
      sat_id[i] = stm_alm[i].gps.satid;
      AGNSS_PRINT_OUT("[GPS ALM] SAT ID: %d\r\n",sat_id[i]);

      if (stm_alm[i].gps.health == 0U)
      {
        if (sat_id[i] > 0U)
        {
          index = snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s,%i,%i,", nmea_cmd_alm, sat_id[i], sizeof(gps_almanac_raw_t));
          for (index2 = 0; index2 < sizeof(gps_almanac_raw_t); index2++)
          {
            index += snprintf((char *)(&gnss_cmd[index]), (int32_t)(sizeof(gnss_cmd))-index, "%02x", apointer[index2]);
          }
          checksum = A_GNSS_NMEA_Checksum(gnss_cmd);
          (void)snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s*%02lx\r\n", gnss_cmd, checksum);

          (void)GNSS_Wrapper_Send(gnss_cmd, (uint16_t)(strlen((char *)gnss_cmd)));
          (void)OS_Delay(4*TX_DELAY);
//          AGNSS_PRINT_OUT("%s\r\n",gnss_cmd);
        }
        else
        {
          AGNSS_PRINT_OUT("[GPS ALM] Warning: unhealty almanac!\n");
        }
      }
    }
  }
  //GLO ALM
  else if(seed_type == GLO_SEED)
  {
    for (i = 0; i < num_sat_alm; i++)
    {
      apointer = (uint8_t *)(&stm_alm[i].glo);
      sat_id[i] = stm_alm[i].glo.satid;

      /* Some Glonass almanac parameters are not provided by RT Data:
       * tau_c */

      AGNSS_PRINT_OUT("[GLO ALM] SAT ID: %d\r\n",sat_id[i]);

      if (stm_alm[i].glo.health == 0U)
      {
        if (sat_id[i] > 0U)
        {
          index = snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s,%i,%i,", nmea_cmd_alm, sat_id[i], sizeof(glonass_almanac_raw_t));
          for (index2 = 0; index2 < sizeof(glonass_almanac_raw_t); index2++)
          {
            index += snprintf((char *)&gnss_cmd[index], (int32_t)(sizeof(gnss_cmd))-index, "%02x", apointer[index2]);
          }
          checksum = A_GNSS_NMEA_Checksum(gnss_cmd);
          (void)snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s*%02lx\r\n", gnss_cmd, checksum);

          (void)GNSS_Wrapper_Send(gnss_cmd, (uint16_t)(strlen((char *)gnss_cmd)));
          (void)OS_Delay(4*TX_DELAY);
//          AGNSS_PRINT_OUT("%s\r\n",gnss_cmd);
        }
        else
        {
          AGNSS_PRINT_OUT("[GLO ALM] Warning: unhealty almanac!\n");
        }
      }
    }

    /* NOTE: it is important that only visible sats should be sent to the device */
  }
  //BEI ALM
  else if(seed_type == BEI_SEED)
  {
    for (i = 0; i < num_sat_alm; i++)
    {
      apointer = (uint8_t *)(&stm_alm[i].compass);
      sat_id[i] = (((stm_alm[i].compass.prn) + MIN_COMPASS_SAT_ID) - 1U);

      AGNSS_PRINT_OUT("[BEI ALM] SAT ID: %d\r\n",sat_id[i]);

      if (stm_alm[i].compass.health == 0U)
      {
        if (sat_id[i] > 0U)
        {
          index = snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s,%i,%i,", nmea_cmd_alm, sat_id[i], sizeof(compass_almanac_raw_t));
          for (index2 = 0; index2 < sizeof(compass_almanac_raw_t); index2++)
          {
            index += snprintf((char *)&gnss_cmd[index], (int32_t)(sizeof(gnss_cmd))-index, "%02x", apointer[index2]);
          }
          checksum = A_GNSS_NMEA_Checksum(gnss_cmd);
          (void)snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s*%02lx\r\n", gnss_cmd, checksum);

          (void)GNSS_Wrapper_Send(gnss_cmd, (uint16_t)(strlen((char *)gnss_cmd)));
          (void)OS_Delay(4*TX_DELAY);
//          AGNSS_PRINT_OUT("%s\r\n",gnss_cmd);
        }
        else
        {
          AGNSS_PRINT_OUT("[BEI ALM] Warning: unhealty almanac!\n");
        }
      }
    }
  }
  //GAL ALM
  else if(seed_type == GAL_SEED)
  {
    for (i = 0; i < num_sat_alm; i++)
    {
      apointer = (uint8_t *)(&stm_alm[i].galileo);
      sat_id[i] = stm_alm[i].galileo.satid;
      AGNSS_PRINT_OUT("[GAL ALM] SAT ID: %d\r\n",sat_id[i]);

      if (stm_alm[i].galileo.health == 0U)
      {
        if (sat_id[i] > 0U)
        {
          index = snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s,%i,%i,", nmea_cmd_alm, sat_id[i], sizeof(galileo_almanac_raw_t));
          for (index2 = 0; index2 < sizeof(galileo_almanac_raw_t); index2++)
          {
            index += snprintf((char *)&gnss_cmd[index], (int32_t)(sizeof(gnss_cmd))-index, "%02x", apointer[index2]);
          }
          checksum = A_GNSS_NMEA_Checksum(gnss_cmd);
          (void)snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s*%02lx\r\n", gnss_cmd, checksum);

          (void)GNSS_Wrapper_Send(gnss_cmd, (uint16_t)(strlen((char *)gnss_cmd)));
          (void)OS_Delay(4*TX_DELAY);
//          AGNSS_PRINT_OUT("%s\r\n",gnss_cmd);
        }
        else
        {
          AGNSS_PRINT_OUT("[GAL ALM] Warning: unhealty almanac!\n");
        }
      }
    }
  }
  else
  {
    /* do nothing */
  }
}

