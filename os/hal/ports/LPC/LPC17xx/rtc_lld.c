/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/
/*
   Concepts and parts of this file have been contributed by Uladzimir Pylinsky
   aka barthess.
 */

/**
 * @file    STM32/RTCv1/rtc_lld.c
 * @brief   STM32 RTC subsystem low level driver header.
 *
 * @addtogroup RTC
 * @{
 */
#include "ch.h"
#include "hal.h"

#include "time.h"
#include <string.h>  

#if HAL_USE_RTC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief RTC driver identifier.
 */
RTCDriver RTCD1;

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   RTC interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(Vector84) {
  CH_IRQ_PROLOGUE();
  LPC_RTC->ILR |= ILR_RTCCIF;   /* clear interrupt flag */
#if RTC_SUPPORTS_CALLBACKS
  if(RTCD1.callback)
  RTCD1.callback(rtcp, event);
#endif
  CH_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initialize RTC.
 *
 * @notapi
 */
void rtc_lld_init(void){
  /* Enable CLOCK into RTC */
  LPC_SC->PCONP |= (1 << 9);

  /* If RTC is stopped, clear STOP bit. */
  if ( LPC_RTC->RTC_AUX & (0x1<<4) )
  {
  LPC_RTC->RTC_AUX |= (0x1<<4); 
  }
  
  /*--- Initialize registers ---*/    
  LPC_RTC->AMR = 0;
  LPC_RTC->CIIR = 0;
  LPC_RTC->CCR = 0;
#if RTC_SUPPORTS_CALLBACKS
  rtc_lld_set_callback(&RTC1, rtc_cb);
#endif /* RTC_SUPPORTS_CALLBACKS */
  /*--- Start RTC counters ---*/
  LPC_RTC->CCR |= CCR_CLKEN;
  LPC_RTC->ILR = ILR_RTCCIF;
  /* IRQ vector permanently assigned to this driver.*/
  NVICEnableVector(RTC_IRQn,
                       CORTEX_PRIORITY_MASK(LPC17XX_RTC_IRQ_PRIORITY));
}

/**
 * @brief   Set current time.
 * @note    Fractional part will be silently ignored. There is no possibility
 *          to change it on STM32F1xx platform.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] timespec  pointer to a @p RTCTime structure
 *
 * @notapi
 */
void rtc_lld_set_time(RTCDriver *rtcp, const RTCTime *timespec);
  if (&RTC1 == rtcp) {
    LPC_RTC->SEC = timespec->RTC_Sec;
    LPC_RTC->MIN = timespec->RTC_Min;
    LPC_RTC->HOUR = timespec->RTC_Hour;
    LPC_RTC->DOM = timespec->RTC_Mday;
    LPC_RTC->DOW = timespec->RTC_Wday;
    LPC_RTC->DOY = timespec->RTC_Yday;
    LPC_RTC->MONTH = timespec->RTC_Mon;
    LPC_RTC->YEAR = timespec->RTC_Year;
  }
}

/**
 * @brief   Get current time.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] timespec  pointer to a @p RTCTime structure
 *
 * @notapi
 */
void rtc_lld_get_time(RTCDriver *rtcp, RTCTime *timespec){
if (&RTC1 == rtcp) {
    RTC1.timespec->RTC_Sec = LPC_RTC->SEC;
    RTC1.timespec->RTC_Min = LPC_RTC->MIN;
    RTC1.timespec->RTC_Hour = LPC_RTC->HOUR;
    RTC1.timespec->RTC_Mday = LPC_RTC->DOM;
    RTC1.timespec->RTC_Wday = LPC_RTC->DOW;
    RTC1.timespec->RTC_Yday = LPC_RTC->DOY;
    RTC1.timespec->RTC_Mon = LPC_RTC->MONTH;
    RTC1.timespec->RTC_Year = LPC_RTC->YEAR;
  }
}


#if RTC_ALARMS > 0
/**
 * @brief   Set alarm time.
 *
 * @note      Default value after BKP domain reset is 0xFFFFFFFF
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] alarm     alarm identifier
 * @param[in] alarmspec pointer to a @p RTCAlarm structure
 *
 * @notapi
 */
  void rtc_lld_set_alarm(RTCDriver *rtcp,
                         rtcalarm_t alarm,
                         const RTCAlarm *alarmspec){
  (void)alarm;
  if (&RTC1 == rtcp) {
  LPC_RTC->ALSEC = alarmspec->RTC_Sec;
  LPC_RTC->ALMIN = alarmspec->RTC_Min;
  LPC_RTC->ALHOUR = alarmspec->RTC_Hour;
  LPC_RTC->ALDOM = alarmspec->RTC_Mday;
  LPC_RTC->ALDOW = alarmspec->RTC_Wday;
  LPC_RTC->ALDOY = alarmspec->RTC_Yday;
  LPC_RTC->ALMON = alarmspec->RTC_Mon;
  LPC_RTC->ALYEAR = alarmspec->RTC_Year; 
  }
}

/**
 * @brief   Get current alarm.
 * @note    If an alarm has not been set then the returned alarm specification
 *          is not meaningful.
 *
 * @note    Default value after BKP domain reset is 0xFFFFFFFF.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] alarm     alarm identifier
 * @param[out] alarmspec pointer to a @p RTCAlarm structure
 *
 * @notapi
 */
void rtc_lld_get_alarm(RTCDriver *rtcp,
                         rtcalarm_t alarm,
                         RTCAlarm *alarmspec){
  (void)alarm;
  if (&RTC1 == rtcp) {
  RTC1.alarmspec->RTC_Sec = LPC_RTC->ALSEC;
  RTC1.alarmspec->RTC_Min = LPC_RTC->ALMIN;
  RTC1.alarmspec->RTC_Hour = LPC_RTC->ALHOUR;
  RTC1.alarmspec->RTC_Mday = LPC_RTC->ALDOM;
  RTC1.alarmspec->RTC_Wday = LPC_RTC->ALDOW;
  RTC1.alarmspec->RTC_Yday = LPC_RTC->ALDOY;
  RTC1.alarmspec->RTC_Mon = LPC_RTC->ALMON;
  RTC1.alarmspec->RTC_Year = LPC_RTC->ALYEAR; 
  }
}

#if RTC_SUPPORTS_CALLBACKS
void rtc_lld_set_callback(RTCDriver *rtcp, rtccb_t callback){
  if (&RTC1 == rtcp) {
  RTCD1.rtc_cfg.callback = callback;
  }
}
#endif /* RTC_SUPPORTS_CALLBACKS */
#endif /* RTC_ALARMS */

#endif /* HAL_USE_RTC */

/** @} */
