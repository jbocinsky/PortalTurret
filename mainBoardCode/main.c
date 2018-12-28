/*****************************************************************************
 brain main file
 All code written for MSP432 in Code Composer Studio 7
 ****************************************************************************/

#include "msp.h"
#include <stdlib.h>
//#include <string.h>
//#include <stdint.h>
#include "drivers.h"
#include "gf.h"

void main(void){

    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer
    WDT_A->CTL = WDT_A_CTL_PW |             // Stop watchdog timer
            WDT_A_CTL_HOLD;
    dOutInit(GUN_CTRL);
    dOutInit(TURR_0_CTRL);
    dOutInit(TURR_1_CTRL);

    dOutNeg(GUN_CTRL);
    dOutPos(TURR_1_CTRL);
    dOutPos(TURR_0_CTRL);

    initClk();
    stepperMotorInit();
    pwmInit();

    spiInit();
    initUART();

    while (1)
    {
//        updatePos();
//        movPos();
//        demo(1);
//          shoot();
        pause();
    }
}

