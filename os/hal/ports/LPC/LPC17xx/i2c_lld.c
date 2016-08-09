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
//wei tiao tong 
/**
 * @file    templates/i2c_lld.c
 * @brief   I2C Driver subsystem low level driver source template.
 *
 * @addtogroup I2C
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_I2C || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/


/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/
I2CDriver I2CD1;
/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/
#if I2C_USE_WAIT || defined(__DOXYGEN__)
/**
 * @brief   Waits for operation completion.
 * @details This function waits for the driver to complete the current
 *          operation.
 * @pre     An operation must be running while the function is invoked.
 * @note    No more than one thread can wait on a I2C driver using
 *          this function.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
#define _i2c_wait_s(i2cp) {                                                 \
  chDbgAssert((i2cp)->id_thread == NULL,                                    \
              "_i2c_wait(), #1", "already waiting");                        \
  chSysLock();                                                              \
  (i2cp)->id_thread = chThdSelf();                                          \
  chSchGoSleepS(THD_STATE_SUSPENDED);                                       \
  chSysUnlock();                                                            \
}

/**
 * @brief   Wakes up the waiting thread.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
#define _i2c_wakeup_isr(i2cp) {                                             \
  if ((i2cp)->id_thread != NULL) {                                          \
    Thread *tp = (i2cp)->id_thread;                                         \
    (i2cp)->id_thread = NULL;                                               \
    chSysLockFromIsr();                                                     \
    chSchReadyI(tp);                                                        \
    chSysUnlockFromIsr();                                                   \
  }                                                                         \
}
#else /* !I2C_USE_WAIT */
#define _i2c_wait_s(i2cp)
#define _i2c_wakeup_isr(i2cp)
#endif /* !I2C_USE_WAIT */

/**
 * @brief   Common ISR code.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Waiting thread wakeup, if any.
 *          - Driver state transitions.
 *
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
// #define _i2c_isr_code(i2cp, i2cscfg) {                                 \
//   if(((i2cp)->id_slave_config)->id_callback) {                         \
//     ((i2cp)->id_slave_config)->id_callback(i2cp, i2cscfg);             \
//     (i2cp)->id_state = I2C_READY;                                      \
//   }                                                                    \
//   else                                                                 \
//     (i2cp)->id_state = I2C_READY;                                      \
//   _i2c_wakeup_isr(i2cp);                                               \
// }

/**
 * @brief   Error ISR code.
 * @details This code handles the portable part of the ISR code:
 *          - Error callback invocation.
 *          - Waiting thread wakeup, if any.
 *          - Driver state transitions.
 *
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
// #define _i2c_isr_err_code(i2cp, i2cscfg) {                             \
//   if(((i2cp)->id_slave_config)->id_err_callback) {                     \
//     ((i2cp)->id_slave_config)->id_err_callback(i2cp, i2cscfg);         \
//     (i2cp)->id_state = I2C_READY;                                      \
//   }                                                                    \
//   else                                                                 \
//     (i2cp)->id_state = I2C_READY;                                      \
//   _i2c_wakeup_isr(i2cp);                                               \
// }

/**
 * @brief   Aborts an I2C transaction.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_abort_operation(I2CDriver *i2cp) {
  LPC_I2C_TypeDef *dp = i2cp->i2c;
  if (&I2CD1 == i2cp) {
    dp->I2CONSET = I2CONSET_STO;      /* Set Stop flag */ 
    dp->I2CONCLR = I2CONCLR_SIC;      /* Clear SI flag */
    /*--- Wait for STOP detected ---*/
    while (dp->I2CONSET & I2CONSET_STO);
  }
  
}

/**
 * @brief   Handling of stalled I2C transactions.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_safety_timeout(void *p) {
  I2CDriver *i2cp = (I2CDriver *)p;

  chSysLockFromIsr();
  if (i2cp->thread) {
    Thread *tp = i2cp->thread;
    i2c_lld_abort_operation(i2cp);
    i2cp->thread = NULL;
    tp->p_u.rdymsg = RDY_TIMEOUT;
    chSchReadyI(tp);
  }
  chSysUnlockFromIsr();

}
/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
i2c0_lld_interrupt(I2CDriver *i2cp){
  LPC_I2C_TypeDef *dp = i2cp->i2c;
  //TODO: 
  //interrupt you wenti  zai zhongduan nei siji 
  uint8_t StatValue;

  /* this handler deals with master read and master write only */
    StatValue = dp->I2STAT;
    sdWrite(&SD1, B, 2);
    switch ( StatValue )
    {
    case 0x08:      /* A Start condition is issued. */
    i2cp->WrIndex = 0;
    // dp->I2DAT = (i2cp->addr | 0x01);
    dp->I2DAT = (i2cp->WrIndex < i2cp->txbytes) ? i2cp->addr : (i2cp->addr | 0x01);//todo
    dp->I2CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
    break;
    
    case 0x10:      /* A repeated started is issued */
    i2cp->RdIndex = 0;
    /* Send SLA with R bit set, */
    dp->I2DAT =  (i2cp->WrIndex < i2cp->txbytes) ? i2cp->addr : (i2cp->addr | 0x01);//todo
    dp->I2CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
    break;
    
    case 0x18:      /* Regardless, it's a ACK */
    if ( i2cp->txbytes == 0 )
    {
      dp->I2CONSET = I2CONSET_STO;      /* Set Stop flag */
      i2cp->irq_state = I2C_NO_DATA;
    }
    else
    { 
      dp->I2DAT = i2cp->txbuf[(i2cp->WrIndex)++];
    }
    dp->I2CONCLR = I2CONCLR_SIC;
    break;

    case 0x28:  /* Data byte has been transmitted, regardless ACK or NACK */
    if ( i2cp->WrIndex < i2cp->txbytes )
    {   
      dp->I2DAT = i2cp->txbuf[(i2cp->WrIndex)++]; /* this should be the last one */
    }
    else
    {
      if ( i2cp->rxbytes != 0 )
      {
        i2cp->RdIndex = 0;
        dp->I2CONSET = I2CONSET_STA;  /* Set Repeated-start flag */
      }
      else
      {
        dp->I2CONSET = I2CONSET_STO;      /* Set Stop flag */
        i2cp->irq_state = I2C_OK;
      }
    }
    dp->I2CONCLR = I2CONCLR_SIC;
    break;

    case 0x30:
    dp->I2CONSET = I2CONSET_STO;      /* Set Stop flag */
    dp->I2CONCLR = I2CONCLR_SIC;
    i2cp->irq_state = I2C_NACK_ON_DATA;

    break;
    
    case 0x40:  /* Master Receive, SLA_R has been sent */
    if ( (i2cp->RdIndex + 1) < i2cp->rxbytes )
    {
      /* Will go to State 0x50 */
      dp->I2CONSET = I2CONSET_AA; /* assert ACK after data is received */
    }
    else
    {
      /* Will go to State 0x58 */
      dp->I2CONCLR = I2CONCLR_AAC;  /* assert NACK after data is received */
    }
    dp->I2CONCLR = I2CONCLR_SIC;
    break;
    
    case 0x50:  /* Data byte has been received, regardless following ACK or NACK */
    i2cp->rxbuf[(i2cp->RdIndex)++] = dp->I2DAT;
    if ( (i2cp->RdIndex + 1) < i2cp->rxbytes )
    {   
      dp->I2CONSET = I2CONSET_AA; /* assert ACK after data is received */
    }
    else
    {
      dp->I2CONCLR = I2CONCLR_AAC;  /* assert NACK on last byte */
    }
    dp->I2CONCLR = I2CONCLR_SIC;
    break;
    
    case 0x58:
    i2cp->rxbuf[(i2cp->RdIndex)++] = dp->I2DAT;
    i2cp->irq_state = I2C_OK;
    dp->I2CONSET = I2CONSET_STO;  /* Set Stop flag */ 
    dp->I2CONCLR = I2CONCLR_SIC;  /* Clear SI flag */
    break;

    case 0x20:    /* regardless, it's a NACK */
    case 0x48:
    dp->I2CONSET = I2CONSET_STO;      /* Set Stop flag */
    i2cp->irq_state = I2C_NACK_ON_ADDRESS;
    dp->I2CONCLR = I2CONCLR_SIC;
    break;
    
    case 0x38:    /* Arbitration lost, in this example, we don't
            deal with multiple master situation */
    default:
    i2cp->irq_state = I2C_ARBITRATION_LOST;
    dp->I2CONCLR = I2CONCLR_SIC;  
    break;
    }
}
/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/
CH_IRQ_HANDLER(Vector68) {

  CH_IRQ_PROLOGUE();

  i2c0_lld_interrupt(&I2CD1);

  CH_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level I2C driver initialization.
 *
 * @notapi
 */
void i2c_lld_init(void) {
  i2cObjectInit(&I2CD1);
  I2CD1.thread = NULL;
  I2CD1.i2c    = LPC_I2C0;

  LPC_SC->PCONP |= (1 << 7);
    
  /* set PIO0.27 and PIO0.28 to I2C0 SDA and SCL */
  /* function to 01 on both SDA and SCL. */
  // LPC_PINCON->PINSEL1 &= ~((0x03<<22)|(0x03<<24));
  // LPC_PINCON->PINSEL1 |= ((0x01<<22)|(0x01<<24)); 
  LPC_PINCON->PINSEL1 &= ~0x03C00000;
  LPC_PINCON->PINSEL1 |= 0x01400000;

  LPC_SC->PCLKSEL0 |= (3<<14);
 
  /*--- Clear flags ---*/
  LPC_I2C0->I2CONCLR = I2CONCLR_AAC | I2CONCLR_SIC | I2CONCLR_STAC | I2CONCLR_I2ENC;    
  
  /*--- Reset registers ---*/
// #if FAST_MODE_PLUS
//   LPC_PINCON->I2CPADCFG |= ((0x1<<0)|(0x1<<2));
//   LPC_I2C0->I2SCLL   = I2SCLL_HS_SCLL;
//   LPC_I2C0->I2SCLH   = I2SCLH_HS_SCLH;
// #else
  LPC_PINCON->I2CPADCFG &= ~((0x1<<0)|(0x1<<2));
  LPC_I2C0->I2SCLL   = I2SCLL_SCLL;
  LPC_I2C0->I2SCLH   = I2SCLH_SCLH;
// #endif
}

/**
 * @brief   Configures and activates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_start(I2CDriver *i2cp) {
  LPC_I2C_TypeDef *dp = i2cp->i2c;
  if (i2cp->state == I2C_STOP) {
    /* Clock activation.*/
    if (&I2CD1 == i2cp) {

      // LPC_SC->PCLKSEL0 = (3<<14);
      NVICEnableVector(I2C0_IRQn, CORTEX_PRIORITY_MASK(LPC17xx_IIC0_IRQ_PRIORITY));
      dp->I2CONSET = I2CONSET_I2EN;
    
    }
  }
    /* Install interrupt handler */
}

/**
 * @brief   Deactivates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_stop(I2CDriver *i2cp) {
  // LPC_I2C_TypeDef *dp = i2cp->i2c;
  if (i2cp->state != I2C_STOP) {
    /* I2C disable.*/
    i2c_lld_abort_operation(i2cp);
    if (&I2CD1 == i2cp) {
      NVICDisableVector(I2C0_IRQn);
    }
  }
}

/**
 * @brief   Master transmission.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] n         number of bytes to be transmitted
 * @param[in] txbuf     transmit data buffer pointer
 *
 * @notapi
 */
  msg_t i2c_lld_master_transmit_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                        const uint8_t *txbuf, size_t txbytes,
                                        uint8_t *rxbuf, size_t rxbytes,
                                        systime_t timeout){
 
    if (&I2CD1 == i2cp) {
      LPC_I2C_TypeDef *dp = i2cp->i2c;
      VirtualTimer vt;

      /* Global timeout for the whole operation.*/
      if (timeout != TIME_INFINITE)
        chVTSetI(&vt, timeout, i2c_lld_safety_timeout, (void *)i2cp);

      i2cp->txbuf = txbuf;
      i2cp->txbytes = txbytes;
      i2cp->rxbuf = rxbuf;
      i2cp->rxbytes = rxbytes;
      i2cp->addr = addr;

      i2cp->WrIndex = 0;
      i2cp->RdIndex = 0;

      i2cp->irq_state = I2C_IDLE;

      // Starts the operation.
      dp->I2CONSET = I2CONSET_STA;

      // Waits for the operation completion or a timeout.
      i2cp->thread = chThdSelf();
 
      chSchGoSleepS(THD_STATE_SUSPENDED);
      if ((timeout != TIME_INFINITE) && chVTIsArmedI(&vt))
          chVTResetI(&vt);

      dp->I2CONCLR = I2CONCLR_STAC;

      return chThdSelf()->p_u.rdymsg; 
  }
  return RDY_RESET;
}


/**
 * @brief   Master receive.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] n         number of bytes to be transmitted
 * @param[in] rxbuf     receive data buffer pointer
 *
 * @notapi
 */
  msg_t i2c_lld_master_receive_timeout(I2CDriver *i2cp, i2caddr_t addr,
         uint8_t *rxbuf, size_t rxbytes,
         systime_t timeout){
  return i2c_lld_master_transmit_timeout(i2cp, addr, NULL, 0, rxbuf, rxbytes, timeout);
 
  }

#endif /* HAL_USE_I2C */

/** @} */
