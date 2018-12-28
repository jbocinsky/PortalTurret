/*
 * gf.cpp
 *
 *  Created on: Sep 18, 2017
 *      Author: Mason Rawson
 */

#include "msp.h"
//#include <stdlib.h>
//#include <string.h>
//#include <stdint.h>
//#include <iostream>
//#include <math.h>
#include "config.h"
#include "drivers.h"



/*
 * Turret Control
 * */

// Full 360 movement
void shoot(void){
    dOutPos(GUN_CTRL);
    int i;
    for(i = 0; i<FULL_STEP; i++){
        turns(SMALL_STEP,M1,1.0*DEFAULT_MOTOR_SPEED);
        turns(SMALL_STEP,M2,1.0*DEFAULT_MOTOR_SPEED);
    }
    dOutNeg(GUN_CTRL);
//    turns(1,M2,1.1*DEFAULT_MOTOR_SPEED);
}

//Spins all servos on both turrets
void demo(int cycles){
    int j;
    for(j = 0; j<cycles; j++)
    {
        shoot();
        int i;
        for (i = 0; i < 40; i++)
        {
            pwmMove(1, 5);
            pwmMove(3, 5);
            pwmMove(2, 5);
            pwmMove(4, 5);
            swDelay(40);
        }

        for (i = 0; i < 40; i++)
        {
            pwmMove(1, -5);
            pwmMove(3, -5);
            pwmMove(2, -5);
            pwmMove(4, -5);
            swDelay(40);
        }
    }
}



