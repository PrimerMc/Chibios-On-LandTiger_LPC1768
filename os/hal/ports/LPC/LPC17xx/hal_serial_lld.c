/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011 Giovanni Di Sirio.

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
*/

/**
 * @file    LPC17xx/serial_lld.c
 * @brief   LPC17xx low level serial driver code.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/
SerialDriver SD2;


/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/** @brief Driver default configuration.*/
static const SerialConfig default_config = {
  SERIAL_DEFAULT_BITRATE,
  LCR_WL8 | LCR_STOP1 | LCR_NOPARITY,
  FCR_TRIGGER0
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   UART initialization.
 *
 * @param[in] sdp       communication channel associated to the UART
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void uart_init(SerialDriver *sdp, const SerialConfig *config) {
  LPC_UART_TypeDef *u = sdp->uart;

  uint32_t div = LPC17xx_SERIAL_UART0_PCLK / (config->sc_speed << 4);
  u->LCR = config->sc_lcr | LCR_DLAB;
  u->DLL = div;
  u->DLM = div >> 8;
  u->LCR = config->sc_lcr;
  u->FCR = FCR_ENABLE | FCR_RXRESET | FCR_TXRESET | config->sc_fcr;
  u->ACR = 0;
  u->FDR = 0x10;
  u->TER = TER_ENABLE;
  u->IER = IER_RBR | IER_STATUS;
}

/**
 * @brief   UART de-initialization.
 *
 * @param[in] u         pointer to an UART I/O block
 */
static void uart_deinit(LPC_UART_TypeDef *u) {

  u->LCR = LCR_DLAB;
  u->DLL = 1;
  u->DLM = 0;
  u->LCR = 0;
  u->FDR = 0x10;
  u->IER = 0;
  u->FCR = FCR_RXRESET | FCR_TXRESET;
  u->ACR = 0;
  u->TER = TER_ENABLE;
}

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       communication channel associated to the UART
 * @param[in] err       UART LSR register value
 */
static void set_error(SerialDriver *sdp, IOREG32 err) {
  eventflags_t sts = 0;

  if (err & LSR_OVERRUN)
    sts |= SD_OVERRUN_ERROR;
  if (err & LSR_PARITY)
    sts |= SD_PARITY_ERROR;
  if (err & LSR_FRAMING)
    sts |= SD_FRAMING_ERROR;
  if (err & LSR_BREAK)
    sts |= SD_BREAK_DETECTED;
  osalSysLockFromISR();
  chnAddFlagsI(sdp, sts);
  osalSysUnlockFromISR();
}

/**
 * @brief   Common IRQ handler.
 * @note    Tries hard to clear all the pending interrupt sources, we don't
 *          want to go through the whole ISR and have another interrupt soon
 *          after.
 *
 * @param[in] u         pointer to an UART I/O block
 * @param[in] sdp       communication channel associated to the UART
 */
static void serve_interrupt(SerialDriver *sdp) {
  LPC_UART_TypeDef *u = sdp->uart;

  while (TRUE) {
    switch (u->IIR & IIR_SRC_MASK) {
    case IIR_SRC_NONE:
      return;
    case IIR_SRC_ERROR:
      set_error(sdp, u->LSR);
      break;
    case IIR_SRC_TIMEOUT:
    case IIR_SRC_RX:
      osalSysLockFromISR();
      if (iqIsEmptyI(&sdp->iqueue))
        chnAddFlagsI(sdp, CHN_INPUT_AVAILABLE);
      osalSysUnlockFromISR();
      while (u->LSR & LSR_RBR_FULL) {
        osalSysLockFromISR();
        if (iqPutI(&sdp->iqueue, u->RBR) < Q_OK)
          chnAddFlagsI(sdp, SD_OVERRUN_ERROR);
        osalSysUnlockFromISR();
      }
      break;
    case IIR_SRC_TX:
      {
        int i = LPC17xx_SERIAL_FIFO_PRELOAD;
        do {
          msg_t b;

          osalSysLockFromISR();
          b = oqGetI(&sdp->oqueue);
          osalSysUnlockFromISR();
          if (b < Q_OK) {
            u->IER &= ~IER_THRE;
            osalSysLockFromISR();
            chnAddFlagsI(sdp, CHN_INPUT_AVAILABLE);
            osalSysUnlockFromISR();
            break;
          }
          u->THR = b;
        } while (--i);
      }
      break;
    default:
      (void) u->THR;
      (void) u->RBR;
    }
  }
}

/**
 * @brief   Attempts a TX FIFO preload.
 */
static void preload(SerialDriver *sdp) {
  LPC_UART_TypeDef *u = sdp->uart;

  if (u->LSR & LSR_THRE) {
    int i = LPC17xx_SERIAL_FIFO_PRELOAD;
    do {
      msg_t b = oqGetI(&sdp->oqueue);
      if (b < Q_OK) {
        chnAddFlagsI(sdp, CHN_INPUT_AVAILABLE);
        return;
      }
      u->THR = b;
    } while (--i);
  }
  u->IER |= IER_THRE;
}

/**
 * @brief   Driver SD1 output notification.
 */


static void notify2(io_queue_t *qp) {

  (void)qp;
  preload(&SD2);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

CH_IRQ_HANDLER(Vector58) {  
 
  CH_IRQ_PROLOGUE();

  serve_interrupt(&SD2);

  CH_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
 //MAYBE TODO
void sd_lld_init(void) {


  sdObjectInit(&SD2, NULL, notify2);
  SD2.uart = LPC_UART1;
  /* RDX without resistors.       */
  /* TDX without resistors.       */
  //modify PINSEL to change pin
  LPC_PINCON->PINSEL0 |= (1<<30) ;
  LPC_PINCON->PINSEL1 |= (1<<0);

}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 *
 * @notapi
 */

 //MAYBE TODO
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

  if (config == NULL)
    config = &default_config;

    if (&SD2 == sdp) {
      uint32_t d;
      switch(LPC17xx_SERIAL_UART1CLKDIV)
      {
          default: d = 1; break;
          case 2: d = 2; break;
          case 4: d = 0; break;
          case 8: d = 3; break;
      }
      LPC_SC->PCLKSEL0 = (LPC_SC->PCLKSEL0 & ~(3<<8)) | (d<<8);
      
      nvicEnableVector(UART1_IRQn, LPC17xx_SERIAL_UART1_IRQ_PRIORITY);
    }

  uart_init(sdp, config);
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the UART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
//MAYBE TODO
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
    uart_deinit(sdp->uart);

    if (&SD2 == sdp) {
      // LPC_SC->PCLKSEL0 = (LPC_SC->PCLKSEL0 & ~(1<<8)) & (LPC17xx_SERIAL_UART0CLKDIV<<8);
      nvicDisableVector(UART1_IRQn);
      return;
    }

  }
}

#endif /* HAL_USE_SERIAL */

/** @} */
