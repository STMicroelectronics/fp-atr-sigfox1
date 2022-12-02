/**
  ******************************************************************************
  * @file    assisted_gnss.c
  * @author  SRA
  * @brief   Implements API for Assisted GNSS - LibAGNSS module middleware
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
#include "assisted_gnss.h"

/* Defines -------------------------------------------------------------------*/

// Unix epoch time in Julian calendar
// (UnixTime = 00:00:00 01.01.1970 => JDN = 2440588)
#define JULIAN_DATE_BASE 2440588U

/* Global variables ----------------------------------------------------------*/

const uint8_t vendorId[MAX_VENDOR_ID] = "ZYDLLXxEH94dEeX2";
const uint8_t modelId[MAX_MODEL_ID] = "MYST";

// For Real-Time
uint8_t prn2slot[MAX_NUM_GLO_PRNS] = {73, 68, 77, 78, 87, 82, 91, 92, 70, 65, 72, 71, 84, 79, 86, 85, 76, 69, 75, 74, 90, 83, 89, 88}; // updated 2015-07-19 - Please check if it is updated before using it!
uint32_t N4 = 0;

/* Private functions ---------------------------------------------------------*/
static uint32_t UTC_ToUnix(current_timedate_t *cur_time)
{
  uint32_t a;
  uint32_t y;
  uint32_t m;
  uint32_t JDN;

  // These hardcore math's are taken from http://en.wikipedia.org/wiki/Julian_day

  // Calculate some coefficients
  a = (14U - cur_time->Month) / 12U;
  y = cur_time->Year + 4800U - a; // years since 1 March, 4801 BC
  m = cur_time->Month + (12U * a) - 3U; // since 1 March, 4801 BC

  // Gregorian calendar date compute
  JDN  = cur_time->Day;
  JDN += ((153U * m) + 2U) / 5U;
  JDN += 365U * y;
  JDN += y / 4U;
  JDN -= y / 100U;
  JDN += y / 400U;
  JDN  = JDN - 32045U;
  JDN  = JDN - JULIAN_DATE_BASE;    // Calculate from base date
  JDN *= 86400U;                     // Days to seconds
  JDN += (uint32_t)(cur_time->Hours) * 3600U;    // ... and today seconds
  JDN += (uint32_t)(cur_time->Minutes) * 60U;
  JDN += (uint32_t)(cur_time->Seconds);

  return JDN;
}


// Converts a byte to 2-char ASCII hexadecimal format
static void byte2hex(uint8_t hex[2], uint8_t byte)
{
  const uint8_t nibble2hex[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
  uint8_t l, h;

  l = byte % 16U;
  h = byte / 16U;
  hex[0] = nibble2hex[h];
  hex[1] = nibble2hex[l];
}


// Calculates NMEA checksum
uint32_t A_GNSS_NMEA_Checksum(const uint8_t *string)
{
  uint32_t i;
  uint32_t checksum = 0;

  for (i = 1; string[i] != (uint8_t)'\0'; i++)
  {
    checksum ^= (uint32_t)string[i];
  }

  return checksum;
}

// Decodes the seed header returning initial time, current time, clock id, version of the seed and the number of satellites contained in the seed
static uint32_t decode_first_block_seed(uint8_t output_first_blk_seed[(SINGLE_SEED_FIRST_BLOCK_BIT_LEN / 4U) + 3U],
                                        uint8_t *seed_buffer)
{
  uint8_t seed_byte;
  uint32_t i;

  uint32_t current_pos = 0;

  (void)memset(output_first_blk_seed, 0, (SINGLE_SEED_FIRST_BLOCK_BIT_LEN / 4U) + 3U);
  for (i = 0; i < SINGLE_SEED_FIRST_BLOCK_BIT_LEN; i += 8U)
  {
    extract_byte(&seed_byte, seed_buffer, current_pos);
    byte2hex(&output_first_blk_seed[i / 4U], seed_byte);
    if ((SINGLE_SEED_FIRST_BLOCK_BIT_LEN - i) > 8U)
    {
      current_pos += 8U;
    }
    else
    {
      current_pos += SINGLE_SEED_FIRST_BLOCK_BIT_LEN - i;
      break;
    }
  }
  return current_pos;
}

// Decodes succeeding seed blocks
static void decode_next_blocks_seed(uint8_t output_seed[(SINGLE_SEED_BIT_LEN / 4U) + 4U],
                                    uint8_t *seed_buffer,
                                    uint32_t *current_pos)
{
  uint8_t seed_byte;
  uint32_t i;

  (void)memset(output_seed, 0, (SINGLE_SEED_BIT_LEN / 4U) + 4U);
  for (i = 0; i < SINGLE_SEED_BIT_LEN; i += 8U)
  {
    extract_byte(&seed_byte, seed_buffer, *current_pos);
    byte2hex(&output_seed[i / 4U], seed_byte);
    if ((SINGLE_SEED_BIT_LEN - i) > 8U)
    {
      *current_pos += 8U;
    }
    else
    {
      *current_pos += SINGLE_SEED_BIT_LEN - i;
    }
  }
}

/* Public functions ----------------------------------------------------------*/

//Utility for Ephemeris and Almanac
int32_t sign_extend_1c(const uint32_t val, const uint32_t width)
{
  uint32_t sign_bit_mask = 0;
  int32_t ret;

  // Since 'width' is used as RHS, check its value range
  // See MISRA-2012 Rule 12.2
  if((width >= 1U) && (width < 32U))
  {
    sign_bit_mask = 1UL << (width - 1UL);
  }

  if((val & sign_bit_mask) != 0U)
  {
    uint32_t temp = (val & (sign_bit_mask - 1UL));

    ret = -(int32_t)temp;
  }
  else
  {
    ret = (int32_t)val;
  }

  return ret;
}

uint32_t day_of_year(current_timedate_t *utc_time)
{
  uint32_t i;
  uint32_t daymon = 0;
  uint32_t dayday;
  uint32_t mth[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  dayday = utc_time->Day;

  for (i = 0; i < utc_time->Month; i++)
  {
    daymon += mth[i];
  }

  return (daymon + dayday);
}

/* Convert timedate structure to Unix time */
uint32_t UTC_ToGPS(current_timedate_t *utc_time)
{
  uint32_t unix_time;
  uint32_t gps_time;

  unix_time = UTC_ToUnix(utc_time);
  gps_time = unix_time - (((365UL * 10UL) + 2UL + 5UL) * 24UL * 60UL * 60UL); // Current GPS time (approx)

  return gps_time;
}

// Extracts a byte from a byte array at specified position, where the position is expressed in bit
void extract_byte(uint8_t *byte, uint8_t *buffer, uint32_t pos)
{
  uint32_t byte_pos, bit_pos, i;
  uint8_t bit;
  uint32_t shift;

  byte_pos = pos / 8UL;
  bit_pos = pos % 8UL;
  *byte = 0;

  for (i = 0; i < 8U; i++)
  {
    if ((i + bit_pos) < 8U)
    {
      shift = 7U - (i + bit_pos);
    }
    else
    {
      shift = 7U - (i + bit_pos - 8U);
    }

    if(shift < 8U)
    {
      bit = ((buffer[byte_pos] & (1UL << shift)) != 0UL) ? 1U : 0U;

      *byte += bit << (7UL - i);
    }
  }
}

// Extracts a 16-bit word from a byte array at specified position, where the position is expressed in bit
void extract_hword(uint16_t *hword, uint8_t *buffer, uint32_t pos)
{
  uint16_t local_buffer[4];
  int8_t i;
  uint32_t offset = pos;

  *hword = 0;
  for (i = 0; i <= 1; i++)
  {
    extract_byte((uint8_t *)(&local_buffer[i]), buffer, offset);
    offset += 8UL;
  }
//  *hword += (local_buffer[0] * 256) + local_buffer[1];
  *hword += (local_buffer[0] << 8) + local_buffer[1];
}

// Extracts a 32-bit word from a byte array at specified position, where the position is expressed in bit
void extract_dword(uint32_t *dword, uint8_t *buffer, uint32_t pos)
{
  uint32_t local_buffer[4];
  int32_t i;
  uint32_t offset = pos;

  *dword = 0;
  for (i = 0; i <= 3; i++)
  {
    extract_byte((uint8_t *)(&local_buffer[i]), buffer, offset);
    offset += 8UL;
  }
  *dword += (local_buffer[0] << 24) + (local_buffer[1] << 16) + (local_buffer[2] << 8) + local_buffer[3];
}

/* Builds the Json payload for rxn request */
int32_t A_GNSS_BuildJsonReq(uint8_t *jsonData,
                            uint8_t seedMask,
                            int16_t *seedSize,
                            int8_t *reqLen,
                            seed_req_t seedReq)
{
  size_t str_len;
  int32_t ret = AGNSS_OK;

  if(jsonData != NULL)
  {    
    if( (seedMask | GPS_CONSTELLATION) == GPS_CONSTELLATION)  // only GPS
    {
      if(seedReq == RT_SEED)
      {
        const uint8_t gps_req_rt[] = "[ { \"rtAssistance\": {\"format\": \"byte\", \"msgs\": [ \"GPS:1NAC\", \"GPS:1ALM\"] } } ]";
        str_len = strlen((const char *)gps_req_rt);
        
        (void)strncpy((char *)jsonData, (const char *)gps_req_rt, str_len);
        jsonData[str_len] = 0;
        *seedSize = BUF_SIZE_6K;
      }
      else
      {
        const uint8_t gps_req_pr[] = "[ { \"ee\": {\"version\": 8, \"constellations\": [ \"gps\" ], \"seedAge\": 0} } ]";
        str_len = strlen((const char *)gps_req_pr);
        
        (void)strncpy((char *)jsonData, (const char *)gps_req_pr, str_len);
        jsonData[str_len] = 0;
        *seedSize = BUF_SIZE_8K;
      }
    }
    else if( (seedMask | GLO_CONSTELLATION) == GLO_CONSTELLATION)  // only GLO
    {
      if(seedReq == RT_SEED)
      {
        const uint8_t glo_req_rt[] = "[ { \"rtAssistance\": {\"format\": \"byte\", \"msgs\": [ \"GLO:2NAC\", \"GLO:2NKC\", \"GLO:2ALM\"] } } ]";
        str_len = strlen((const char *)glo_req_rt);
        
        (void)strncpy((char *)jsonData, (const char *)glo_req_rt, str_len);
        jsonData[str_len] = 0;
      }
      else
      {
        const uint8_t glo_req_pr[] = "[ { \"ee\": {\"version\": 8, \"constellations\": [ \"glonass\" ], \"seedAge\": 0} } ]";
        str_len = strlen((const char *)glo_req_pr);
        
        (void)strncpy((char *)jsonData, (const char *)glo_req_pr, str_len);
        jsonData[str_len] = 0;
      }
      *seedSize = BUF_SIZE_6K;
    }
    else if( (seedMask | GAL_CONSTELLATION) == GAL_CONSTELLATION)  // only GAL
    {
      if(seedReq == RT_SEED)
      {
        const uint8_t gal_req_rt[] = "[ { \"rtAssistance\": {\"format\": \"byte\", \"msgs\": [ \"GAL:2NAC\", \"GAL:2ALM\"] } } ]";
        str_len = strlen((const char *)gal_req_rt);
        
        (void)strncpy((char *)jsonData, (const char *)gal_req_rt, str_len);
        jsonData[str_len] = 0;
      }
      else
      {
        const uint8_t gal_req_pr[] = "[ { \"ee\": {\"version\": 8, \"constellations\": [ \"galileo\" ], \"seedAge\": 0} } ]";
        str_len = strlen((const char *)gal_req_pr);
        
        (void)strncpy((char *)jsonData, (const char *)gal_req_pr, str_len);
        jsonData[str_len] = 0;
      }
      *seedSize = BUF_SIZE_4K;
    }
    else if( (seedMask | BEI_CONSTELLATION) == BEI_CONSTELLATION) // only BEI
    {
      if(seedReq == RT_SEED)
      {
        const uint8_t bei_req_rt[] = "[ { \"rtAssistance\": {\"format\": \"byte\", \"msgs\": [ \"BDS:2NAC\", \"BDS:2ALM\"] } } ]";
        str_len = strlen((const char *)bei_req_rt);
        
        (void)strncpy((char *)jsonData, (const char *)bei_req_rt, str_len);
        jsonData[str_len] = 0;
      }
      else
      {
        const uint8_t bei_req_pr[] = "[ { \"ee\": {\"version\": 8, \"constellations\": [ \"beidou\" ], \"seedAge\": 0} } ]";
        str_len = strlen((const char *)bei_req_pr);
        
        (void)strncpy((char *)jsonData, (const char *)bei_req_pr, str_len);
        jsonData[str_len] = 0;
      }
      *seedSize = BUF_SIZE_5K;//was BUF_SIZE_4K;
    }
    /* For now, the combined constellation options are only handled for Predictive GNSS */
    else if( (seedMask | (GPS_CONSTELLATION | GLO_CONSTELLATION)) == (GPS_CONSTELLATION | GLO_CONSTELLATION)) // GPS + GLO
    {
      const uint8_t gps_glo_req_pr[] = "[ { \"ee\": {\"version\": 8, \"constellations\": [ \"gps\", \"glonass\" ], \"seedAge\": 0} } ]";
      str_len = strlen((const char *)gps_glo_req_pr);
      
      (void)strncpy((char *)jsonData, (const char *)gps_glo_req_pr, str_len);
      jsonData[str_len] = 0;
      *seedSize = BUF_SIZE_15K;
    }
    else if( (seedMask | (GPS_CONSTELLATION | GAL_CONSTELLATION)) == (GPS_CONSTELLATION | GAL_CONSTELLATION))// GPS + GAL
    {
      const uint8_t gps_gal_req_pr[] = "[ { \"ee\": {\"version\": 8, \"constellations\": [ \"gps\", \"galileo\" ], \"seedAge\": 0} } ]";
      str_len = strlen((const char *)gps_gal_req_pr);
      
      (void)strncpy((char *)jsonData, (const char *)gps_gal_req_pr, str_len);
      jsonData[str_len] = 0;
      *seedSize = BUF_SIZE_12K;
    }
    else if( (seedMask | (GPS_CONSTELLATION | BEI_CONSTELLATION)) == (GPS_CONSTELLATION | BEI_CONSTELLATION)) // GPS + BEI
    {
      const uint8_t gps_bei_req_pr[] = "[ { \"ee\": {\"version\": 8, \"constellations\": [ \"gps\", \"beidou\" ], \"seedAge\": 0} } ]";
      str_len = strlen((const char *)gps_bei_req_pr);
      
      (void)strncpy((char *)jsonData, (const char *)gps_bei_req_pr, str_len);
      jsonData[str_len] = 0;
      *seedSize = BUF_SIZE_12K;
    }
    else if( (seedMask | (GLO_CONSTELLATION | GAL_CONSTELLATION)) == (GLO_CONSTELLATION | GAL_CONSTELLATION)) // GLO + GAL
    {
      const uint8_t glo_gal_req_pr[] = "[ { \"ee\": {\"version\": 8, \"constellations\": [ \"glonass\", \"galileo\" ], \"seedAge\": 0} } ]";
      str_len = strlen((const char *)glo_gal_req_pr);
      
      (void)strncpy((char *)jsonData, (const char *)glo_gal_req_pr, str_len);
      jsonData[str_len] = 0;
      *seedSize = BUF_SIZE_9K;
    }
    else if( (seedMask | (GAL_CONSTELLATION | BEI_CONSTELLATION)) == (GAL_CONSTELLATION | BEI_CONSTELLATION)) // GAL + BEI
    {
      const uint8_t gal_bei_req_pr[] = "[ { \"ee\": {\"version\": 8, \"constellations\": [ \"beidou\", \"galileo\" ], \"seedAge\": 0} } ]";
      str_len = strlen((const char *)gal_bei_req_pr);
      
      (void)strncpy((char *)jsonData, (const char *)gal_bei_req_pr, str_len);
      jsonData[str_len] = 0;
      *seedSize = BUF_SIZE_8K;
    }
    else if( (seedMask | (GPS_CONSTELLATION | GLO_CONSTELLATION | GAL_CONSTELLATION)) == (GPS_CONSTELLATION | GLO_CONSTELLATION | GAL_CONSTELLATION)) // GPS + GLO + GAL
    {
      const uint8_t gps_glo_gal_req_pr[] = "[ { \"ee\": {\"version\": 8, \"constellations\": [ \"gps\", \"glonass\", \"galileo\" ], \"seedAge\": 0} } ]";
      str_len = strlen((const char *)gps_glo_gal_req_pr);
      
      (void)strncpy((char *)jsonData, (const char *)gps_glo_gal_req_pr, str_len);
      jsonData[str_len] = 0;
      *seedSize = BUF_SIZE_16K;
    }
    else if( (seedMask | (GPS_CONSTELLATION | GAL_CONSTELLATION | BEI_CONSTELLATION)) == (GPS_CONSTELLATION | GAL_CONSTELLATION | BEI_CONSTELLATION)) // GPS +  GAL + BEI
    {
      const uint8_t gps_gal_bei_req_pr[] = "[ { \"ee\": {\"version\": 8, \"constellations\": [ \"gps\", \"beidou\", \"galileo\" ], \"seedAge\": 0} } ]";
      str_len = strlen((const char *)gps_gal_bei_req_pr);
      
      (void)strncpy((char *)jsonData, (const char *)gps_gal_bei_req_pr, str_len);
      jsonData[str_len] = 0;
      *seedSize = BUF_SIZE_16K;
    }
    else if((seedMask | DEFAULT_CONSTELLATION) == DEFAULT_CONSTELLATION) 
    {
      const uint8_t default_req_pr[] = "[ { \"ee\": {\"version\": 8, \"constellations\": [ \"gps\", \"glonass\", \"galileo\" ], \"seedAge\": 0} } ]";
      str_len = strlen((const char *)default_req_pr);
      
      (void)strncpy((char *)jsonData, (const char *)default_req_pr, str_len);
      jsonData[str_len] = 0;
      *seedSize = BUF_SIZE_16K;
    }
    
    *reqLen = strlen((const char *)jsonData);
  }
  else
  {
    ret = AGNSS_ERR;
  }

  return ret;
}

int32_t A_GNSS_BuildAuthHeader(GNSSParser_Data_t *pGNSSParser_Data, uint8_t *auth_string)
{
  int32_t ret = AGNSS_OK;

  if(pGNSSParser_Data != NULL)
  {
    (void)snprintf((char *)auth_string, 150, "RXN-SP cId=%s,mId=%s,dId=%s,pw=%s",
                   vendorId,
                   modelId,
                   pGNSSParser_Data->pstmpass_data.deviceId,
                   pGNSSParser_Data->pstmpass_data.pwd);
  }
  else
  {
    ret = AGNSS_ERR;
  }

  return ret;
}

// Finalize decoding and send seed
void A_GNSS_SendSeed(uint8_t constellation, seed_info_t *seed_info)
{
  uint16_t sat;
  uint8_t GNSS_to_ID;
  uint32_t i, k, max_sat;
  int32_t index;
  uint32_t week_number, ref_time;
  int32_t T_A0, T_A1, T_A2;
  int8_t deltaT;
  int8_t slotfreq, blktype;
  glo_slot_freq_t *slot_freq_ptr;
  block_type_t *block_type_ptr;
  uint8_t *time_model_ptr;
  uint8_t gnss_cmd[350];
  uint8_t output_first_blk_seed[(SINGLE_SEED_FIRST_BLOCK_BIT_LEN / 4U) + 3U] = {0};
  uint8_t output_seed[(SINGLE_SEED_BIT_LEN / 4U) + 4U] = {0};
  uint32_t check_sum;
  uint32_t current_pos;

  const uint8_t nmea_seed_pkt[] = "$PSTMSTAGPSSEEDPKT";
  const uint8_t nmea_seed_begin[] = "$PSTMSTAGPSSEEDBEGIN";
  const uint8_t nmea_blk_type[] = "$PSTMSTAGPSBLKTYPE";
  const uint8_t nmea_slot_frq[] = "$PSTMSTAGPSSLOTFRQ";
  const uint8_t nmea_prop_pkt[] = "$PSTMSTAGPSSEEDPROP";

  time_model_ptr = &(seed_info->time_model[0]);
  if(constellation == (uint8_t)GPS_SEED)
  {
    max_sat = MAX_NUM_GPS_PRNS;
  }
  else if(constellation == (uint8_t)GLO_SEED)
  {
    max_sat = MAX_NUM_GLO_PRNS;
  }
  else if(constellation == (uint8_t)GAL_SEED)
  {
    max_sat = MAX_NUM_GAL_PRNS;
  }
  else if(constellation == (uint8_t)BEI_SEED)
  {
    max_sat = MAX_NUM_BEI_PRNS;
    time_model_ptr = &(seed_info->time_model[1]);
  }
  else
  {
    /* do nothing */
    max_sat = 0;
  }

  ref_time = ((uint32_t)time_model_ptr[0] << 8) | (uint32_t)time_model_ptr[1];
  T_A0 = ((uint32_t)time_model_ptr[2] << 24) + ((uint32_t)time_model_ptr[3] << 16) + ((uint32_t)time_model_ptr[4] << 8) + (uint32_t)time_model_ptr[5];
  T_A1 = ((uint32_t)time_model_ptr[6] << 24) + ((uint32_t)time_model_ptr[7] << 16) + ((uint32_t)time_model_ptr[8] << 8) + (uint32_t)time_model_ptr[9];
  T_A2 = (uint32_t)time_model_ptr[10];
  GNSS_to_ID = time_model_ptr[11];
  week_number = ((uint32_t)time_model_ptr[12] << 8) + (uint32_t)time_model_ptr[13];

  if(constellation == (uint8_t)BEI_SEED)
  {
    deltaT = time_model_ptr[14];
  }
  else
  {
    deltaT = 0;
  }

  /************************
   * STEP 1 ***************
   ************************/
  // Send $PSTMSTAGPSSEEDBEGIN command
  AGNSS_PRINT_OUT("Send $PSTMSTAGPSSEEDBEGIN command\n\n");

  current_pos = decode_first_block_seed(output_first_blk_seed, seed_info->seed);
  (void)snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%d,%ld,%d,%s",
		  nmea_seed_begin, constellation, seed_info->curr_secs, seed_info->next_gps_time,
		  seed_info->next_secs, ref_time, T_A0, T_A1, T_A2, GNSS_to_ID, week_number, deltaT,
                  output_first_blk_seed);
  check_sum = A_GNSS_NMEA_Checksum(gnss_cmd);
  (void)snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s*%02lx\r\n", gnss_cmd, check_sum);
  (void)GNSS_Wrapper_Send(gnss_cmd, (uint16_t)(strlen((char *)gnss_cmd)));
  (void)OS_Delay(4*TX_DELAY);
//  Enable for DEBUG
//  AGNSS_PRINT_OUT(gnss_cmd);

  /************************
   * STEP 2 ***************
   ************************/
  // Send $PSTMSTAGPSBLKTYPE command
  AGNSS_PRINT_OUT("Send $PSTMSTAGPSBLKTYPE command\n\n");

  block_type_ptr = (block_type_t *)&(seed_info->block_types[1]);
  index = snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s", nmea_blk_type);
  for(i = 0; i < max_sat; i++)
  {
    blktype = 0;
    for(k = 0; k < max_sat; k++)
    {
      if((block_type_ptr[k].SV_ID) == i)
      {
        blktype = block_type_ptr[k].block_type;
        break;
      }
      else
      {
        blktype = 0;
      }
    }
    index += snprintf((char *)(&gnss_cmd[index]), (int32_t)(sizeof(gnss_cmd))-index, ",%d", blktype);
  }
  check_sum = A_GNSS_NMEA_Checksum(gnss_cmd);
  (void)snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s*%02lx\r\n", gnss_cmd, check_sum);
  (void)GNSS_Wrapper_Send(gnss_cmd, (uint16_t)(strlen((char *)gnss_cmd)));
  (void)OS_Delay(4*TX_DELAY);
//  Enable for DEBUG
//  AGNSS_PRINT_OUT(gnss_cmd);

  /************************
   * STEP 3 ***************
   ************************/
  // Send $PSTMSTAGPSSLOTFRQ command
  if (constellation == (uint8_t)GLO_SEED)
  {
    AGNSS_PRINT_OUT("Send $PSTMSTAGPSSLOTFRQ command\n\n");

    slot_freq_ptr = (glo_slot_freq_t *)&(seed_info->slot_freq[1]);
    index = snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s", nmea_slot_frq);

    for(i = 0; i < max_sat; i++)
    {
      if(slot_freq_ptr[i].ch_num != 0)
      {
        slotfreq = slot_freq_ptr[i].ch_num;
      }
      else
      {
        slotfreq = 0;
      }
      index += snprintf((char *)(&gnss_cmd[index]), (int32_t)(sizeof(gnss_cmd))-index, ",%d", slotfreq);
    }
    check_sum = A_GNSS_NMEA_Checksum(gnss_cmd);
    (void)snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s*%02lx\r\n", gnss_cmd, check_sum);
    (void)GNSS_Wrapper_Send(gnss_cmd, (uint16_t)(strlen((char *)gnss_cmd)));
    (void)OS_Delay(4*TX_DELAY);
//  Enable for DEBUG
//  AGNSS_PRINT_OUT(gnss_cmd);
  }

  /************************
   * STEP 4 ***************
   ************************/
  AGNSS_PRINT_OUT("Send $PSTMSTAGPSSEEDPKT commands\n\n");

  for(sat = seed_info->nsat; sat < seed_info->max_satid; sat++)
  {
    decode_next_blocks_seed(output_seed, seed_info->seed, &current_pos);
    AGNSS_PRINT_OUT("Sending PKT %d\n", sat);
    if (output_seed[0] != 0U)
    {
      // Send $PSTMSTAGPSSEEDPKT command
      (void)snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s,%s", nmea_seed_pkt, output_seed);
      check_sum = A_GNSS_NMEA_Checksum(gnss_cmd);
      (void)snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s*%02lx\r\n", gnss_cmd, check_sum);
      (void)GNSS_Wrapper_Send(gnss_cmd, (uint16_t)(strlen((char *)gnss_cmd)));
      (void)OS_Delay(4*TX_DELAY);
//      Enable for DEBUG
//      AGNSS_PRINT_OUT(gnss_cmd);
    }
  }

  /************************
   * STEP 5 ***************
   ************************/
  AGNSS_PRINT_OUT("Send $PSTMSTAGPSSEEDPROP command\n\n");

  // Send $PSTMSTAGPSSEEDPROP command
  (void)snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s*%02lx\r\n", (char*)nmea_prop_pkt, A_GNSS_NMEA_Checksum(nmea_prop_pkt));
  (void)OS_Delay(4*TX_DELAY);
  (void)GNSS_Wrapper_Send(gnss_cmd, (uint16_t)(strlen((char *)gnss_cmd)));
//  Enable for DEBUG
//  AGNSS_PRINT_OUT(gnss_cmd);

//  AGNSS_PRINT_OUT("\n\n*********************sat = %d\n\n", sat);
//  AGNSS_PRINT_OUT("\n\n*********************seed_info->nsat = %d\n\n", seed_info->nsat);
//  AGNSS_PRINT_OUT("\n\n*********************seed_info->max_satid = %d\n\n", seed_info->max_satid);

  return;
}

// Send Inittime command
void A_GNSS_SendCurrentTime(current_timedate_t *utc_time)
{
  const uint8_t nmea_inittime_pkt[] = "$PSTMINITTIME";
  uint8_t gnss_cmd[64];
  uint32_t check_sum;

  AGNSS_PRINT_OUT("Send $PSTMINITTIME command\n\n");

  (void)snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s,%02ld,%02ld,%04ld,%02d,%02d,%02d",
                 nmea_inittime_pkt,
                 utc_time->Day,
                 utc_time->Month,
                 utc_time->Year,
                 utc_time->Hours,
                 utc_time->Minutes,
                 utc_time->Seconds);
  check_sum = A_GNSS_NMEA_Checksum(gnss_cmd);

  (void)snprintf((char *)gnss_cmd, sizeof(gnss_cmd), "%s*%02lx\r\n", gnss_cmd, check_sum);
  (void)GNSS_Wrapper_Send(gnss_cmd, (uint16_t)(strlen((char *)gnss_cmd)));
  (void)OS_Delay(TX_DELAY);
//  Enable for DEBUG
//  AGNSS_PRINT_OUT(gnss_cmd);
}

