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

#include <stdlib.h>

#include <ch.h>
#include <hal.h>
#include "ch_test.h"


static THD_WORKING_AREA(led0_thread_wa, 128);
static THD_FUNCTION(led0_thread, arg)
{
    (void)arg;
    chRegSetThreadName("blinker0");
    while(TRUE)
    {
        palSetPort(GPIO2, GPIO2_LED0);
        chThdSleepMilliseconds(100);
        palClearPort(GPIO2, GPIO2_LED0);
        chThdSleepMilliseconds(200);
    }
     
}

static THD_WORKING_AREA(led1_thread_wa, 128);
static THD_FUNCTION(led1_thread, arg)
{
    (void)arg;

    while(TRUE)
    {
        palSetPort(GPIO2, GPIO2_LED1);
        chThdSleepMilliseconds(300);
        palClearPort(GPIO2, GPIO2_LED1);
        chThdSleepMilliseconds(400);

    }
     
}
static THD_WORKING_AREA(led2_thread_wa, 128);
static THD_FUNCTION(led2_thread, arg)
{
    (void)arg;

    while(TRUE)
    {
         palSetPort(GPIO2, GPIO2_LED2);
        chThdSleepMilliseconds(500);
        palClearPort(GPIO2, GPIO2_LED2);
        chThdSleepMilliseconds(600);
    }
     
}

static THD_WORKING_AREA(led3_thread_wa, 128);
static THD_FUNCTION(led3_thread, arg)
{
    (void)arg;

    while(TRUE)
    {
        palSetPort(GPIO2, GPIO2_LED3);
        chThdSleepMilliseconds(700);
        palClearPort(GPIO2, GPIO2_LED3);
        chThdSleepMilliseconds(800);

    }
     
}
static THD_WORKING_AREA(led4_thread_wa, 128);
static THD_FUNCTION(led4_thread, arg)
{
    (void)arg;

    while(TRUE)
    {
         palSetPort(GPIO2, GPIO2_LED4);
        chThdSleepMilliseconds(900);
        palClearPort(GPIO2, GPIO2_LED4);
        chThdSleepMilliseconds(1000);
    }
     
}

static THD_WORKING_AREA(led5_thread_wa, 128);
static THD_FUNCTION(led5_thread, arg)
{
    (void)arg;

    while(TRUE)
    {
        palSetPort(GPIO2, GPIO2_LED5);
        chThdSleepMilliseconds(1100);
        palClearPort(GPIO2, GPIO2_LED5);
        chThdSleepMilliseconds(1200);

    }
     
}
static THD_WORKING_AREA(led6_thread_wa, 128);
static THD_FUNCTION(led6_thread, arg)
{
    (void)arg;

    while(TRUE)
    {
        palSetPort(GPIO2, GPIO2_LED6);
        chThdSleepMilliseconds(1300);
        palClearPort(GPIO2, GPIO2_LED6);
        chThdSleepMilliseconds(1400);
    }
     
}


/*
 * Application entry point.
 */
int main(void) {
    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */
    halInit();
    chSysInit();

    sdStart(&SD2, NULL);
    /*
   * Creating blinkers threads.
   */
    chThdCreateStatic(led0_thread_wa, sizeof(led0_thread_wa),
                      NORMALPRIO + 1, led0_thread, NULL);
    chThdCreateStatic(led1_thread_wa, sizeof(led1_thread_wa),
                      NORMALPRIO + 1, led1_thread, NULL);
    chThdCreateStatic(led2_thread_wa, sizeof(led2_thread_wa),
                      NORMALPRIO + 1, led2_thread, NULL);
    chThdCreateStatic(led3_thread_wa, sizeof(led3_thread_wa),
                      NORMALPRIO + 1, led3_thread, NULL);
    chThdCreateStatic(led4_thread_wa, sizeof(led4_thread_wa),
                      NORMALPRIO + 1, led4_thread, NULL);
    chThdCreateStatic(led5_thread_wa, sizeof(led5_thread_wa),
                      NORMALPRIO + 1, led5_thread, NULL);
    chThdCreateStatic(led6_thread_wa, sizeof(led6_thread_wa),
                      NORMALPRIO + 1, led6_thread, NULL);



  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the buttons state and run test procedure.
   */
    test_execute((BaseSequentialStream *)&SD2);
    while (TRUE)
    {

        palSetPort(GPIO2, GPIO2_LED7);
        chThdSleepMilliseconds(1500);
        palClearPort(GPIO2, GPIO2_LED7);
        chThdSleepMilliseconds(1600);

    }
     
}
