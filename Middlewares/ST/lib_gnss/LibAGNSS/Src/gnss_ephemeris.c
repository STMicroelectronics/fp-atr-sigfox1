/**
  ******************************************************************************
  * @file    gnss_ephemeris.c
  * @author  SRA
  * @brief   Implements API for Assisted GNSS Ephemeris (Real Time case)
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
#include "gnss_ephemeris.h"
#include "gnss_almanac.h"

/* Defines -------------------------------------------------------------------*/
#define XYBRID_RT_GPS_EPH_SIZE          (89U)
#define XYBRID_RT_GLO_EPH_SIZE          (42U)
#define XYBRID_RT_GLO_NKC_EPH_SIZE      (62U)
#define XYBRID_RT_BEI_EPH_SIZE          (81U)

/* Global variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

// GPS
static uint8_t *rt_decode_gps_hdr(uint8_t *rt_payload, assist_hdr_t *hdr_ptr)
{
  hdr_ptr->num_sats = rt_payload[0];
  
  hdr_ptr->non_brc_ind = 0;
  
  return &(rt_payload[1]);
}

// GLO NAC
static uint8_t *rt_decode_glo_hdr_NAC(uint8_t *rt_payload, assist_hdr_t *hdr_ptr)
{
  hdr_ptr->num_sats = rt_payload[0];

  hdr_ptr->non_brc_ind = rt_payload[1];

  return &(rt_payload[2]);
}

// GLO NKC
static uint8_t *rt_decode_glo_hdr_NKC(uint8_t *rt_payload, assist_hdr_t *hdr_ptr)
{
  hdr_ptr->num_sats = rt_payload[0];

  hdr_ptr->non_brc_ind = rt_payload[1];

  return &(rt_payload[2]);
}

// BEI
static uint8_t *rt_decode_bei_hdr(uint8_t *rt_payload, assist_hdr_t *hdr_ptr)
{
  hdr_ptr->num_sats = rt_payload[0];

  hdr_ptr->non_brc_ind = rt_payload[1];

  return &(rt_payload[2]);
}

// GAL
static uint8_t *rt_decode_gal_hdr(uint8_t *rt_payload, assist_hdr_t *hdr_ptr)
{
  hdr_ptr->num_sats = rt_payload[0];

  hdr_ptr->non_brc_ind = rt_payload[1];

  return &(rt_payload[2]);
}

/* GPS */
static uint8_t rt_decode_gps_sat(ephemeris_raw_t *stm_ephem, uint8_t *sat_payload, uint32_t sat_idx)
{
  uint8_t *eph_ptr;
  uint8_t byte;
  uint16_t hword;
  uint32_t dword;
  uint8_t sat_id;
  int32_t idx = 0;

  eph_ptr = &sat_payload[XYBRID_RT_GPS_EPH_SIZE * sat_idx];
  (void)memset(stm_ephem, 0, sizeof(ephemeris_raw_t));

  extract_byte(&sat_id, eph_ptr, idx * 8);
  idx++;
  extract_hword(&hword, eph_ptr, idx * 8);
  stm_ephem->gps.week = hword + (uint16_t)GPS_WEEK_ROLLOVER;
  idx += 7;
  extract_byte(&byte, eph_ptr, idx * 8);
  stm_ephem->gps.accuracy = byte;
  idx++;
  extract_byte(&byte, eph_ptr, idx * 8);
  stm_ephem->gps.health = byte;
  idx++;
  extract_hword(&hword, eph_ptr, idx * 8);
  stm_ephem->gps.iodc = hword;
  idx += 2;
  extract_hword(&hword, eph_ptr, idx * 8);
  stm_ephem->gps.iode1 = hword;
  stm_ephem->gps.iode2 = hword;
  idx += 17;
  extract_byte(&byte, eph_ptr, idx * 8);
  stm_ephem->gps.time_group_delay = byte;
  idx++;
  extract_dword(&dword, eph_ptr, idx * 8);
  stm_ephem->gps.toc = dword;
  idx += 4;
  extract_byte(&byte, eph_ptr, idx * 8);
  stm_ephem->gps.af2 = byte;
  idx++;
  extract_hword(&hword, eph_ptr, idx * 8);
  stm_ephem->gps.af1 = hword;
  idx += 2;
  extract_dword(&dword, eph_ptr, idx * 8);
  stm_ephem->gps.af0 = dword;
  idx += 4;
  extract_hword(&hword, eph_ptr, idx * 8);
  stm_ephem->gps.crs = hword;
  idx += 2;
  extract_hword(&hword, eph_ptr, idx * 8);
  stm_ephem->gps.motion_difference = hword;
  idx += 2;
  extract_dword(&dword, eph_ptr, idx * 8);
  stm_ephem->gps.mean_anomaly = dword;
  idx += 4;
  extract_hword(&hword, eph_ptr, idx * 8);
  stm_ephem->gps.cuc = hword;
  idx += 2;
  extract_dword(&dword, eph_ptr, idx * 8);
  stm_ephem->gps.eccentricity = dword;
  idx += 4;
  extract_hword(&hword, eph_ptr, idx * 8);
  stm_ephem->gps.cus = hword;
  idx += 2;
  extract_dword(&dword, eph_ptr, idx * 8);
  stm_ephem->gps.root_a = dword;
  idx += 4;
  extract_hword(&hword, eph_ptr, idx * 8);
  stm_ephem->gps.toe = hword;
  idx += 4;
  extract_hword(&hword, eph_ptr, idx * 8);
  stm_ephem->gps.cic = hword;
  idx += 2;
  extract_dword(&dword, eph_ptr, idx * 8);
  stm_ephem->gps.omega_zero = dword;
  idx += 4; 
  extract_hword(&hword, eph_ptr, idx * 8);
  stm_ephem->gps.cis = hword;
  idx += 2;
  extract_dword(&dword, eph_ptr, idx * 8);
  stm_ephem->gps.inclination = dword;
  idx += 4;
  extract_hword(&hword, eph_ptr, idx * 8);
  stm_ephem->gps.crc = hword;
  idx += 2;
  extract_dword(&dword, eph_ptr, idx * 8);
  stm_ephem->gps.perigee = dword;
  idx += 4;
  extract_dword(&dword, eph_ptr, idx * 8);
  stm_ephem->gps.omega_dot = dword;
  idx += 4;
  extract_hword(&hword, eph_ptr, idx * 8);
  stm_ephem->gps.i_dot = hword;

  /* Useless assignment
  stm_ephem->gps.predicted = 0;
  */
  stm_ephem->gps.available = 1;
  stm_ephem->gps.subframe1_available = 1;
  stm_ephem->gps.subframe2_available = 1;
  stm_ephem->gps.subframe3_available = 1;

  return sat_id + 1U;
}

/* GLONASS */
static uint8_t rt_decode_glo_sat(ephemeris_raw_t *stm_ephem, uint8_t *sat_payload, uint8_t *sat_payload_nkc, uint32_t sat_idx)
{
  uint8_t *eph_ptr;
  uint8_t byte;
  uint16_t hword;
  uint32_t dword;
  uint8_t sat_id;
  uint32_t idx = 0;
  uint32_t IOD;

  eph_ptr = &sat_payload[XYBRID_RT_GLO_EPH_SIZE * sat_idx];
  (void)memset(stm_ephem, 0, sizeof(ephemeris_raw_t));

  extract_byte(&sat_id, eph_ptr, idx * 8U);
  idx++;
  extract_byte(&byte, eph_ptr, idx * 8U);
  stm_ephem->glonass.health = byte;
  idx++;
  extract_hword(&hword, eph_ptr, idx * 8U);
  IOD = hword;
  idx+=2U;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->glonass.tau_n = sign_extend_1c(dword, 22);
  idx+=4U;
  extract_hword(&hword, eph_ptr, idx * 8U);
  stm_ephem->glonass.gamma_n = sign_extend_1c(hword, 11);
  idx+=3U;
  extract_byte(&byte, eph_ptr, idx * 8U);
  stm_ephem->glonass.E_n = byte;
  idx++;
  extract_byte(&byte, eph_ptr, idx * 8U);
  stm_ephem->glonass.P1 = byte;
  idx++;
  extract_byte(&byte, eph_ptr, idx * 8U);
  stm_ephem->glonass.P2 = byte;
  idx++;
  extract_byte(&byte, eph_ptr, idx * 8U);
  stm_ephem->glonass.M = byte;
  idx++;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->glonass.xn = sign_extend_1c(dword, 27);
  idx+=4U;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->glonass.xn_dot = sign_extend_1c(dword, 24);
  idx+=4U;
  extract_byte(&byte, eph_ptr, idx * 8U);
  stm_ephem->glonass.xn_dot_dot = sign_extend_1c(byte, 5);
  idx++;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->glonass.yn = sign_extend_1c(dword, 27);
  idx+=4U;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->glonass.yn_dot = sign_extend_1c(dword, 24);
  idx+=4U;
  extract_byte(&byte, eph_ptr, idx * 8U);
  stm_ephem->glonass.yn_dot_dot = sign_extend_1c(byte, 5);
  idx++;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->glonass.zn = sign_extend_1c(dword, 27);
  idx+=4U;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->glonass.zn_dot = sign_extend_1c(dword, 24);
  idx+=4U;
  extract_byte(&byte, eph_ptr, idx * 8U);
  stm_ephem->glonass.zn_dot_dot = sign_extend_1c(byte, 5);
  idx++;

  eph_ptr = &sat_payload_nkc[XYBRID_RT_GLO_NKC_EPH_SIZE * sat_idx];

  idx = 0;
  extract_byte(&sat_id, eph_ptr, idx * 8U);
  idx++;
  extract_byte(&byte, eph_ptr, idx * 8U);
  stm_ephem->glonass.health = byte;
  idx+=15U;
  extract_hword(&hword, eph_ptr, idx * 8U);
  stm_ephem->glonass.toe = hword;

  stm_ephem->glonass.tb = IOD;

  stm_ephem->glonass.predicted = 0;
  stm_ephem->glonass.available = 1;

  return sat_id;
}

/* BEIDOU */
static uint8_t rt_decode_bei_sat(ephemeris_raw_t *stm_ephem, uint8_t *sat_payload, uint32_t sat_idx)
{
  uint8_t *eph_ptr;
  uint8_t byte;
  uint16_t hword;
  uint32_t dword;
  uint8_t sat_id;
  uint32_t idx = 0;

  eph_ptr = &sat_payload[XYBRID_RT_BEI_EPH_SIZE * sat_idx];
 (void) memset(stm_ephem, 0, sizeof(ephemeris_raw_t));

  extract_byte(&sat_id, eph_ptr, idx * 8U);
  idx++;
  extract_byte(&byte, eph_ptr, idx * 8U);
  stm_ephem->compass.health = byte;
  idx++;
  extract_byte(&byte, eph_ptr, idx * 8U);
  stm_ephem->compass.aodc = byte;
  idx++;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->compass.toc = dword;
  idx += 4U;
  extract_hword(&hword, eph_ptr, idx * 8U);
  stm_ephem->compass.af2 = hword;
  idx += 2U;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->compass.af1 = dword;
  idx += 4U;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->compass.af0 = dword;
  idx += 4U;
  extract_hword(&hword, eph_ptr, idx * 8U);
  stm_ephem->compass.time_group_delay = hword;
  idx += 2U;
  extract_byte(&byte, eph_ptr, idx * 8U);
  stm_ephem->compass.aode = byte;
  idx++;
  extract_byte(&byte, eph_ptr, idx * 8U);
  stm_ephem->compass.urai = byte;
  idx++;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->compass.toe = dword;
  idx += 4U;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->compass.root_a = dword;
  idx += 4U;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->compass.eccentricity = dword;
  idx += 4U;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->compass.perigee = dword;
  idx += 4U;
  extract_hword(&hword, eph_ptr, idx * 8U);
  stm_ephem->compass.motion_difference = hword;
  idx += 2U;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->compass.mean_anomaly = dword;
  idx += 4U;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->compass.omega_zero = dword;
  idx += 4U;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->compass.omega_dot = dword;
  idx += 4U;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->compass.inclination = dword;
  idx += 4U;
  extract_hword(&hword, eph_ptr, idx * 8U);
  stm_ephem->compass.i_dot = hword;
  idx += 2U;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->compass.cuc = dword;
  idx += 4U;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->compass.cus = dword;
  idx += 4U;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->compass.crc = dword;
  idx += 4U;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->compass.crs = dword;
  idx += 4U;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->compass.cic = dword;
  idx += 4U;
  extract_dword(&dword, eph_ptr, idx * 8U);
  stm_ephem->compass.cis = dword;

  /* Useless assignment
  stm_ephem->compass.predicted = 0;
  stm_ephem->compass.nvm_reliable = 0;
  */
  stm_ephem->compass.available = 1;

  return sat_id + 1U;
}

/* GALILEO */
static uint8_t rt_decode_gal_sat(ephemeris_raw_t *stm_ephem, uint8_t *sat_payload, uint32_t sat_idx, uint32_t *offset)
{
  uint8_t *eph_ptr;
  uint8_t byte;
  uint16_t hword;
  uint32_t dword;
  uint32_t idx = 0;
  uint32_t num_clock_model = 0;
  (void)sat_idx;

  eph_ptr = &sat_payload[*offset];

  (void)memset(stm_ephem, 0, sizeof(ephemeris_raw_t));
  //currently we only evaluate the first time model
  //if sat has two time model the second is discarded

  extract_byte(&byte, eph_ptr,idx*8U);
  stm_ephem->galileo.SVID=byte;
  idx++;

  extract_byte(&byte, eph_ptr,idx*8U);
  stm_ephem->galileo.health=byte;
  idx++;

  //iod
  extract_hword(&hword, eph_ptr,idx*8U);
  stm_ephem->galileo.iod_nav=hword;
  idx+=2U;

  //numclockmodel
  extract_byte((uint8_t *)(&num_clock_model), eph_ptr,idx*8U);
  idx++;
  if(num_clock_model==1U)
  {
    *offset=*offset+67U;
  }
  if(num_clock_model==2U)
  {
    *offset=*offset+82U;
  }

  //stoc
  extract_hword(&hword, eph_ptr,idx*8U);
  stm_ephem->galileo.toc=hword;
  idx+=2U;

  //af2
  extract_byte(&byte, eph_ptr,idx*8U);
  stm_ephem->galileo.af2=byte;
  idx++;

  //af1
  extract_dword(&dword, eph_ptr,idx*8U);
  stm_ephem->galileo.af1=dword;
  idx+=4U;

  //af0
  extract_dword(&dword, eph_ptr,idx*8U);
  stm_ephem->galileo.af0=dword;
  idx+=4U;

  //BGD_E1_E5b
  //this is the unique considered
  extract_hword(&hword, eph_ptr,idx*8U);
  stm_ephem->galileo.BGD_E1_E5b=hword;
  idx+=2U;

  //forced to 0 since the structure doesn't consider 2 time models
  /* Useless assignment
  stm_ephem->galileo.BGD_E1_E5a=0;
  */

  //sisa
  extract_byte(&byte, eph_ptr,idx*8U);
  stm_ephem->galileo.SISA=byte;

  //now we need to jump just after the size of array of time models
  idx = 5U + (num_clock_model * 15U);

  //stoe
  extract_hword(&hword, eph_ptr,idx*8U);
  stm_ephem->galileo.toe=hword;
  idx+=2U;

  //perigee
  extract_dword(&dword, eph_ptr,idx*8U);
  stm_ephem->galileo.perigee=dword;
  idx+=4U;

  //sDn
  extract_hword(&hword, eph_ptr,idx*8U);
  stm_ephem->galileo.motion_difference=hword;
  idx+=2U;

  //sM0
  extract_dword(&dword, eph_ptr,idx*8U);
  stm_ephem->galileo.mean_anomaly=dword;
  idx+=4U;

  //omegadot
  extract_dword(&dword, eph_ptr,idx*8U);
  stm_ephem->galileo.omega_dot=dword;
  idx+=4U;

  //se
  extract_dword(&dword, eph_ptr,idx*8U);
  stm_ephem->galileo.eccentricity=dword;
  idx+=4U;

  //idot
  extract_hword(&hword, eph_ptr,idx*8U);
  stm_ephem->galileo.i_dot=hword;
  idx+=2U;

  //ssqrta
  extract_dword(&dword, eph_ptr,idx*8U);
  stm_ephem->galileo.root_a=dword;
  idx+=4U;

  //si0
  extract_dword(&dword, eph_ptr,idx*8U);
  stm_ephem->galileo.inclination=dword;
  idx+=4U;

  //somega0
  extract_dword(&dword, eph_ptr,idx*8U);
  stm_ephem->galileo.omega_zero=dword;
  idx+=4U;

  //scrs
  extract_hword(&hword, eph_ptr,idx*8U);
  stm_ephem->galileo.crs=hword;
  idx+=2U;

  //scis
  extract_hword(&hword, eph_ptr,idx*8U);
  stm_ephem->galileo.cis=hword;
  idx+=2U;

  //scus
  extract_hword(&hword, eph_ptr,idx*8U);
  stm_ephem->galileo.cus=hword;
  idx+=2U;

  //scrc
  extract_hword(&hword, eph_ptr,idx*8U);
  stm_ephem->galileo.crc=hword;
  idx+=2U;

  //scic
  extract_hword(&hword, eph_ptr,idx*8U);
  stm_ephem->galileo.cic=hword;
  idx+=2U;

  //scuc
  extract_hword(&hword, eph_ptr,idx*8U);
  stm_ephem->galileo.cuc=hword;

  /* Useless assignment
  stm_ephem->galileo.E1BHS=0;
  */
  stm_ephem->galileo.available=1;
  /* Useless assignment
  stm_ephem->galileo.spare0=0;
  stm_ephem->galileo.spare1=0;
  */
  stm_ephem->galileo.word_available=0x1F;
  /* Useless assignment
  stm_ephem->galileo.reserved1=0;
  stm_ephem->galileo.reserved2=0;
  stm_ephem->galileo.reserved3=0;
  stm_ephem->galileo.reserved4=0;
  */

  return (uint8_t)(stm_ephem->galileo.SVID+1U);
}

/* Public functions ----------------------------------------------------------*/

uint16_t A_GNSS_DecodeEphemRT(seed_type_t seed_type,
                              seed_info_t *seed_info,
                              current_timedate_t *utc_time)
{
  assist_hdr_t hdr;
  uint32_t wn[32] = {0};
  uint16_t wn_ret = 0;
  uint32_t slot[MAX_GLO_SLOTS];
  uint32_t i;
  uint8_t *sat_id;

  // Save the current address of the EPH seed to be recovered at the end of decoding
  uint8_t *eph_seed = seed_info->eph_seed;

  ephemeris_raw_t *stm_ephem = (ephemeris_raw_t *)seed_info->stm_ephem;
  (void)memset(&hdr, 0, sizeof(assist_hdr_t));
  sat_id = seed_info->sat_id;

  // GPS EPH
  if(seed_type == GPS_SEED)
  {
    seed_info->eph_seed = rt_decode_gps_hdr(seed_info->eph_seed, &hdr);

    seed_info->num_sat_eph = hdr.num_sats;

    AGNSS_PRINT_OUT("\n[GPS EPH] (num_sats=%d):\r\n", hdr.num_sats);

    for (i = 0; i < hdr.num_sats; i++)
    {
      sat_id[i] = rt_decode_gps_sat(&stm_ephem[i], seed_info->eph_seed, i);
      
      AGNSS_PRINT_OUT("[GPS EPH] SAT ID: %d\r\n",sat_id[i]);

      if (stm_ephem[i].gps.health == 0U)
      {
        if (sat_id[i] > 0U)
        {
          wn[i] = stm_ephem[i].gps.week;
        }
        else
        {
          AGNSS_PRINT_OUT("[GPS EPH] Warning: unhealty ephemeris!\n");
        }
      }
    }
  }
  // GLO EPH
  else if(seed_type == GLO_SEED)
  {
    uint32_t gps_time, glo_time;
    uint32_t ref_glo_day, glo_time_of_day, glo_day;
    uint32_t temp;

    uint32_t NT;

    seed_info->eph_seed = rt_decode_glo_hdr_NAC(seed_info->eph_seed, &hdr);
    seed_info->eph_nkc_seed = rt_decode_glo_hdr_NKC(seed_info->eph_nkc_seed, &hdr);

    seed_info->num_sat_eph = hdr.num_sats;

    gps_time = UTC_ToGPS(utc_time);
    glo_time = gps_time + (3U * SECS_PER_HOUR);

    ref_glo_day = (glo_time / SECS_PER_DAY) * SECS_PER_DAY;
    glo_time_of_day = glo_time % SECS_PER_DAY;

    AGNSS_PRINT_OUT("\n[GLO EPH] (num_sats=%d):\r\n", hdr.num_sats);

    N4 = (((utc_time->Year - 1996U)/4U) + 1U);

    if((utc_time->Year % 4U) == 0U)
    {
      NT = day_of_year(utc_time);
    }
    else
    {
      NT = (365U * (utc_time->Year - (1996U + ((N4 - 1U) * 4U)))) + (day_of_year(utc_time)-1U);
    }

    for (i = 0; i < hdr.num_sats; i++)
    {
      slot[i] = rt_decode_glo_sat(&stm_ephem[i], seed_info->eph_seed, seed_info->eph_nkc_seed, i);

      if (stm_ephem[i].glonass.health == 0U)
      {
        stm_ephem[i].glonass.n = slot[i] + 1U;

        if (prn2slot[slot[i]] == 0U)
        {
          AGNSS_PRINT_OUT("[GLO EPH] Warning: slot %ld not found in slot list!\n", slot[i]);
        }
        else
        {
          sat_id[i] = prn2slot[slot[i]];
        }
        if((stm_ephem[i].glonass.tb >= 48U) && (glo_time_of_day < 43200U))
        {
          glo_day = ref_glo_day - 1U;
        }
        else if((stm_ephem[i].glonass.tb < 48U) && (glo_time_of_day >= 43200U))
        {
          glo_day = ref_glo_day + 1U;
        }
        else
        {
          glo_day = ref_glo_day;
        }
        temp = glo_day + (stm_ephem[i].glonass.tb * 900U);
        stm_ephem[i].glonass.week = temp / SECS_PER_WEEK;
        stm_ephem[i].glonass.toe = (temp % SECS_PER_WEEK) / 16U;
        stm_ephem[i].glonass.toe_lsb = (temp % SECS_PER_WEEK) % 16U;
        stm_ephem[i].glonass.N4 = N4;
        stm_ephem[i].glonass.NT = NT;
        AGNSS_PRINT_OUT("[GLO EPH] SAT ID: %d\r\n",sat_id[i]);

        if (sat_id[i] <= 0U)
        {
          AGNSS_PRINT_OUT("[GLO EPH] Warning: unhealty ephemeris!\n");
        }
      }
    }
  }
  // BEI EPH
  else if(seed_type == BEI_SEED)
  {
    uint32_t week;

    week = ((((utc_time->Year - 1980U) * 365U) - 6U ) + ((utc_time->Year - 1980U)/4U) + day_of_year(utc_time))/7U;
    week = week - COMPASS_INIT_WEEK;

    seed_info->eph_seed = rt_decode_bei_hdr(seed_info->eph_seed, &hdr);

    seed_info->num_sat_eph = hdr.num_sats;

    AGNSS_PRINT_OUT("\n[BEI EPH] (num_sats=%d):\r\n", hdr.num_sats);

    for (i = 0; i < hdr.num_sats; i++)
    {
      sat_id[i] = rt_decode_bei_sat(&stm_ephem[i], seed_info->eph_seed, i);
      sat_id[i] = (((sat_id[i]) + MIN_COMPASS_SAT_ID) - 1U);
      stm_ephem[i].compass.week = week;
      if(sat_id[i] <= 145U)
      {
        stm_ephem[i].compass.is_geo = 1;
      }
      else
      {
        stm_ephem[i].compass.is_geo = 0;
      }
      
      AGNSS_PRINT_OUT("[BEI EPH] SAT ID: %d\r\n",sat_id[i]);
      if (stm_ephem[i].compass.health == 0U)
      {
        if (sat_id[i] > 0U)
        {
          wn[i] = stm_ephem[i].compass.week;
        }
        else
        {
          AGNSS_PRINT_OUT("[BEI EPH] Warning: unhealty ephemeris!\n");
        }
      }
    }
  }
  // GAL EPH
  else if(seed_type == GAL_SEED)
  {
    uint32_t offset=0;
    seed_info->eph_seed = rt_decode_gal_hdr(seed_info->eph_seed, &hdr);

    seed_info->num_sat_eph = hdr.num_sats;

    AGNSS_PRINT_OUT("\n[GAL EPH] (num_sats=%d):\r\n", hdr.num_sats);

    for (i = 0; i < hdr.num_sats; i++)
    {
      sat_id[i] = rt_decode_gal_sat(&stm_ephem[i], seed_info->eph_seed, i, &offset);
      if (stm_ephem[i].galileo.health == 0U)
      {
        AGNSS_PRINT_OUT("[GAL EPH] SAT ID: %d\r\n",sat_id[i]);
        if (sat_id[i] > 0U)
        {
          //weeek is not loaded from rxn server this line is forced to 0
          wn[i] = 0;//stm_ephem[i].galileo.week;
        }
        else
        {
          AGNSS_PRINT_OUT("[GAL EPH] Warning: unhealty ephemeris!\n");
        }
      }
    }
  }
  else
  {
    /* do nothing */
  }

  // After conversion, recover the original address of the EPH seed
  seed_info->eph_seed = eph_seed;

  //week number
  if(hdr.num_sats > 0U)
  {
    wn_ret = (uint16_t)wn[hdr.num_sats - 1U];
  }

  return wn_ret;
}

void A_GNSS_SendEphemRT(seed_type_t seed_type, seed_info_t *seed_info)
{
  uint32_t i;
  int32_t index;
  uint32_t index2;
  uint32_t checksum;
  uint8_t *epointer;
  uint8_t gnss_cmd[256];

  const uint8_t nmea_cmd_eph[] = "$PSTMEPHEM";
  ephemeris_raw_t *stm_ephem = (ephemeris_raw_t *)seed_info->stm_ephem;
  almanac_raw_t *stm_alm = (almanac_raw_t *)seed_info->stm_alm;
  uint32_t num_sat_eph = seed_info->num_sat_eph;
  uint8_t *sat_id = seed_info->sat_id;

  //GPS EPH
  if(seed_type == GPS_SEED)
  {
    for (i = 0; i < num_sat_eph; i++)
    {
      epointer = (uint8_t*)(&stm_ephem[i].gps);
      if (stm_ephem[i].gps.health == 0U)
      {
        AGNSS_PRINT_OUT("[GPS EPH] SAT ID: %d\r\n",sat_id[i]);
        if (sat_id[i] > 0U)
        {
          index = snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s,%i,%i,", nmea_cmd_eph, sat_id[i], sizeof(gps_ephemeris_raw_t));
          for (index2 = 0; index2 < sizeof(gps_ephemeris_raw_t); index2++)
          {
            index += snprintf((char *)(&gnss_cmd[index]), (int32_t)(sizeof(gnss_cmd))-index, "%02x", epointer[index2]);
          }
          checksum = A_GNSS_NMEA_Checksum(gnss_cmd);
          (void)snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s*%02lx\r\n", gnss_cmd, checksum);

          (void)GNSS_Wrapper_Send(gnss_cmd, (uint16_t)(strlen((char *)gnss_cmd)));
          (void)OS_Delay(4*TX_DELAY);
//          AGNSS_PRINT_OUT("%s\r\n",gnss_cmd);
        }
        else
        {
          AGNSS_PRINT_OUT("[GPS EPH] Warning: unhealty ephemeris!\n");
        }
      }
    }
  }
  //GLO EPH
  else if(seed_type == GLO_SEED)
  {
    for (i = 0; i < num_sat_eph; i++)
    {
      epointer = (uint8_t*)(&stm_ephem[i].glonass);
      stm_ephem[i].glonass.NA = stm_alm[i].glo.NA;
      sat_id[i] = (uint32_t) prn2slot[stm_ephem[i].glonass.n - 1U];

      /* Some Glonass ephemeris parameters are not provided by RT Data:
       * KP, Bn, tau_c, tau_GPS, tk, FT */

      AGNSS_PRINT_OUT("[GLO EPH] SAT ID: %d\r\n",sat_id[i]);

      if (stm_ephem[i].glonass.health == 0U)
      {
        if (sat_id[i] > 0U)
        {
          index = snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s,%i,%i,", nmea_cmd_eph, sat_id[i], sizeof(glonass_ephemeris_raw_t));
          for (index2 = 0; index2 < sizeof(glonass_ephemeris_raw_t); index2++)
          {
            index += snprintf((char *)(&gnss_cmd[index]), (int32_t)(sizeof(gnss_cmd))-index, "%02x", epointer[index2]);
          }
          checksum = A_GNSS_NMEA_Checksum(gnss_cmd);
          (void)snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s*%02lx\r\n", gnss_cmd, checksum);

          (void)GNSS_Wrapper_Send(gnss_cmd, (uint16_t)(strlen((char *)gnss_cmd)));
          (void)OS_Delay(4*TX_DELAY);
//          AGNSS_PRINT_OUT("%s\r\n",gnss_cmd);
        }
        else
        {
          AGNSS_PRINT_OUT("[GLO EPH] Warning: unhealty ephemeris!\n");
        }
      }
    }
  }
  //BEI EPH
  else if(seed_type == BEI_SEED)
  {  
    for (i = 0; i < num_sat_eph; i++)
    {
      epointer = (uint8_t *)(&stm_ephem[i].compass);
      stm_ephem[i].compass.week = stm_alm[i].compass.week;

      /* Some BeiDou ephemeris parameters are not provided by RT Data:
       * A0, A1, sow, A2, A3, B2, urai, B3, B0, B1
       */

      AGNSS_PRINT_OUT("[BEI EPH] SAT ID: %d\r\n",sat_id[i]);

      if (stm_ephem[i].compass.health == 0U)
      {
        if (sat_id[i] > 0U)
        {
          index = snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s,%i,%i,", nmea_cmd_eph, sat_id[i], sizeof(compass_ephemeris_raw_t));
          for (index2 = 0; index2 < sizeof(compass_ephemeris_raw_t); index2++)
          {
            index += snprintf((char *)(&gnss_cmd[index]), (int32_t)(sizeof(gnss_cmd))-index, "%02x", epointer[index2]);
          }
          checksum = A_GNSS_NMEA_Checksum(gnss_cmd);
          (void)snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s*%02lx\r\n", gnss_cmd, checksum);

          (void)GNSS_Wrapper_Send(gnss_cmd, (uint16_t)(strlen((char *)gnss_cmd)));
          (void)OS_Delay(4*TX_DELAY);
//          AGNSS_PRINT_OUT("%s\r\n",gnss_cmd);
        }
        else
        {
          AGNSS_PRINT_OUT("[BEI EPH] Warning: unhealty ephemeris!\n");
        }
      }
    }
  }
  //GAL EPH
  else if(seed_type == GAL_SEED)
  {
    for (i = 0; i < num_sat_eph; i++)
    {
      epointer = (uint8_t*)(&stm_ephem[i].galileo);
      AGNSS_PRINT_OUT("[GAL EPH] SAT ID: %d\r\n",sat_id[i]);

      if (stm_ephem[i].galileo.health == 0U)
      {
        if (sat_id[i] > 0U)
        {
          index = snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s,%i,%i,", nmea_cmd_eph, sat_id[i], sizeof(galileo_ephemeris_raw_t));
          for (index2 = 0; index2 < sizeof(galileo_ephemeris_raw_t); index2++)
          {
            index += snprintf((char *)(&gnss_cmd[index]), (int32_t)(sizeof(gnss_cmd))-index, "%02x", epointer[index2]);
          }
          checksum = A_GNSS_NMEA_Checksum(gnss_cmd);
          (void)snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s*%02lx\r\n", gnss_cmd, checksum);

          (void)GNSS_Wrapper_Send(gnss_cmd, (uint16_t)(strlen((char *)gnss_cmd)));
          (void)OS_Delay(4*TX_DELAY);
//          AGNSS_PRINT_OUT("%s\r\n",gnss_cmd);
        }
        else
        {
          AGNSS_PRINT_OUT("[GAL EPH] Warning: unhealty ephemeris!\n");
        }
      }
    }
  }
  else
  {
    /* do nothing */
  }
}

