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
 * @file    LPC17XX/rtc_lld.h
 * @brief   LPC17xx RTC subsystem low level driver header.
 *
 * @addtogroup RTC
 * @{
 */

#ifndef _RTC_LLD_H_
#define _RTC_LLD_H_

#if HAL_USE_RTC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/
#define IMSEC     0x00000001
#define IMMIN     0x00000002
#define IMHOUR    0x00000004
#define IMDOM     0x00000008
#define IMDOW     0x00000010
#define IMDOY     0x00000020
#define IMMON     0x00000040
#define IMYEAR    0x00000080

#define AMRSEC    0x00000001  /* Alarm mask for Seconds */
#define AMRMIN    0x00000002  /* Alarm mask for Minutes */
#define AMRHOUR   0x00000004  /* Alarm mask for Hours */
#define AMRDOM    0x00000008  /* Alarm mask for Day of Month */
#define AMRDOW    0x00000010  /* Alarm mask for Day of Week */
#define AMRDOY    0x00000020  /* Alarm mask for Day of Year */
#define AMRMON    0x00000040  /* Alarm mask for Month */
#define AMRYEAR   0x00000080  /* Alarm mask for Year */

#define PREINT_RTC  0x000001C8  /* Prescaler value, integer portion, 
            PCLK = 15Mhz */
#define PREFRAC_RTC 0x000061C0  /* Prescaler value, fraction portion, 
            PCLK = 15Mhz */
#define ILR_RTCCIF  0x01
#define ILR_RTCALF  0x02

#define CCR_CLKEN   0x01
#define CCR_CTCRST  0x02
#define CCR_CLKSRC  0x10
/**
 * @brief   This RTC implementation supports callbacks.
 */
#define RTC_SUPPORTS_CALLBACKS      TRUE

/**
 * @brief   One alarm comparator available.
 */
#define RTC_ALARMS                  1

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/*
 * RTC driver system settings.
 */
#define LPC17XX_RTC_IRQ_PRIORITY      15
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

// #if HAL_USE_RTC && !LPC17XX_HAS_RTC
// #error "RTC not present in the selected device"
// #endif

// #if LPC17XX_RTCCLK == 0
// #error "RTC clock not enabled"
// #endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/
/**
 * @brief   Type of a structure representing an RTC callbacks config.
 */
typedef struct RTCConfig RTCConfig;

/**
 * @brief   Type of an RTC event.
 */
typedef enum {
  RTC_EVENT_SECOND = 0,                 /** Triggered every second.         */
  RTC_EVENT_ALARM = 1,                  /** Triggered on alarm.             */
  RTC_EVENT_OVERFLOW = 2                /** Triggered on counter overflow.  */
} rtcevent_t;

/**
 * @brief   Type of a generic RTC callback.
 */
typedef void (*rtccb_t)(RTCDriver *rtcp, rtcevent_t event);

/**
 * @brief   Structure representing an RTC callbacks config.
 */
struct RTCConfig{
  /**
   * @brief Generic RTC callback pointer.
   */
  rtccb_t           callback;
};

/**
 * @brief   Structure representing an RTC alarm time stamp.
 */
 #if RTC_ALARMS > 0
typedef struct RTCAlarm {
    uint32_t RTC_Sec;     /* Second value - [0,59] */
    uint32_t RTC_Min;     /* Minute value - [0,59] */
    uint32_t RTC_Hour;    /* Hour value - [0,23] */
    uint32_t RTC_Mday;    /* Day of the month value - [1,31] */
    uint32_t RTC_Mon;     /* Month value - [1,12] */
    uint32_t RTC_Year;    /* Year value - [0,4095] */
    uint32_t RTC_Wday;    /* Day of week value - [0,6] */
    uint32_t RTC_Yday;    /* Day of year value - [1,365] */
}RTCAlarm;
#endif
/**
 * @brief   Structure representing an RTC time stamp.
 */
typedef struct RTCTime{
    uint32_t RTC_Sec;     /* Second value - [0,59] */
    uint32_t RTC_Min;     /* Minute value - [0,59] */
    uint32_t RTC_Hour;    /* Hour value - [0,23] */
    uint32_t RTC_Mday;    /* Day of the month value - [1,31] */
    uint32_t RTC_Mon;     /* Month value - [1,12] */
    uint32_t RTC_Year;    /* Year value - [0,4095] */
    uint32_t RTC_Wday;    /* Day of week value - [0,6] */
    uint32_t RTC_Yday;    /* Day of year value - [1,365] */

} RTCTime;

/**
 * @brief   Structure representing an RTC driver.
 */
struct RTCDriver{

  RTCTime           rtc_time;
#if RTC_ALARMS > 0
  RTCAlarm          rtc_alarm;
#if RTC_SUPPORTS_CALLBACKS
  rtccb_t           callback;
#endif  
#endif  
};
typedef uint32_t rtcalarm_t;
/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
extern RTCDriver RTCD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void rtc_lld_init(void);
  void rtc_lld_set_time(RTCDriver *rtcp, const RTCTime *timespec);
  void rtc_lld_get_time(RTCDriver *rtcp, RTCTime *timespec);
#if RTC_ALARMS > 0
  void rtc_lld_set_alarm(RTCDriver *rtcp,
                         rtcalarm_t alarm,
                         const RTCAlarm *alarmspec);
  void rtc_lld_get_alarm(RTCDriver *rtcp,
                         rtcalarm_t alarm,
                         RTCAlarm *alarmspec);
#if RTC_SUPPORTS_CALLBACKS
  void rtc_lld_set_callback(RTCDriver *rtcp, rtccb_t callback);
#endif /* RTC_SUPPORTS_CALLBACKS */
#endif /* RTC_ALARMS */
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_RTC */

#endif /* _RTC_LLD_H_ */

/** @} */
