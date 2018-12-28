/*
 * config.h
 *
 *  Created on: Sep 14, 2017
 *      Author: Mason Rawson
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/*
 * System Wide Defines
 *
 * */
#define CLK_FREQ 6000000
typedef enum {false, true} bool;

/*
 * Camera to PWM - Constants
 *
 * */
#define MAX_HEIGHT 0
#define MIN_HEIGHT 200

#define MAX_WIDTH 640
#define MIN_WIDTH 0

#define height_slope ((float)((MAX_TILT - MIN_TILT))/(float)(MAX_HEIGHT - MIN_HEIGHT))
#define width_slope ((float)(MAX_PAN - MIN_PAN)/(float)(MAX_WIDTH - MIN_WIDTH))

/*
 * GPIO:
 *
 * Uses Port 8
 *
 * */
#define GUN_CTRL BIT5
#define TURR_0_CTRL BIT6
#define TURR_1_CTRL BIT7

#define D_OUT BIT5
#define D_IN_0 BIT1


/*
 * PWM:
 * PWM via output compare with Timer A
 *
 * Basic PWM Math
 * 3M/50Hz=60k=xEA60
 * xEA60/10=x1770 -> 2ms
 * xEA60/5=xBB8 -> 1ms
 *
 * Uses Port 2
 *
 * */

//PWM pins
#define CCR1 BIT4
#define CCR2 BIT5
#define CCR3 BIT6
#define CCR4 BIT7

//PWM constants
#define CCR0_PERIOD 0xEA60
#define CCR_2MS (0x1770<<1)
#define CCR_1MS (0xBB8<<1)
#define CCR_P5MS CCR_1MS/2
#define CCR_1P5MS (CCR_P5MS + CCR_1MS)
#define CCR_P75MS CCR_1P5MS/2
#define CCR_1P25MS ((CCR_P75MS+CCR_1MS)-CCR_P5MS)

#define CCR_100US CCR_1MS/10
#define CCR_10US CCR_100US/10
#define CCR_5US CCR_10US/2

//PWM positions
//PAN SERVOS
#define MAX_PAN CCR_2MS
#define MIN_PAN CCR_1MS

#define CCR1_INIT (MAX_PAN+MIN_PAN)/2
#define CCR3_INIT (MAX_PAN+MIN_PAN)/2
#define CCR1_MAX MAX_PAN
#define CCR3_MAX MAX_PAN
#define CCR1_MIN MIN_PAN
#define CCR3_MIN MIN_PAN

//TILT SERVOS
#define MAX_TILT CCR_1P25MS
#define MIN_TILT CCR_1MS

#define CCR2_INIT MIN_TILT
#define CCR4_INIT MIN_TILT
#define CCR4_MAX MAX_TILT
#define CCR2_MAX MAX_TILT
#define CCR4_MIN MIN_TILT
#define CCR2_MIN MIN_TILT



/*
 * Stepper Motor
 *
 * Implemented with 4 GPIO output pins
 * Port 4 for one and Port 6 for another
 *
 * There is no homing. Instead we will manually calibrate once and only do 360s
 * */

//Port 4
#define M2IN1  BIT0
#define M2IN2  BIT2
#define M2IN3  BIT4
#define M2IN4  BIT5

//Port 6
#define M1IN1  BIT0
#define M1IN2  BIT1
#define M1IN3  BIT4
#define M1IN4  BIT5

//STEPPER MOTORS
#define M2 1
#define M1 0
#define CYCLES_PER_ROTATION 512
#define totPos 1.125
#define zeroPos 0.09375*1.125
#define tenPos (1+zeroPos)
//#define FULL_STEP 250
#define FULL_STEP 500
#define SMALL_STEP 1.0/FULL_STEP
#define DEFAULT_MOTOR_SPEED 0.13
//#define DEFAULT_MOTOR_SPEED 0.25



/*
 * UART
 * Using Port 3
 * Pins 2 (Rx) and 3 (Tx)
 *
 * */
#define RX_UART BIT2
#define TX_UART BIT3



/*
 * SPI
 * Use Port 1
 * Pins 5-7
 *
 * */
#define SPICLK BIT5
#define MOSI BIT6
#define MISO BIT7
#define SPI_CNT_SHOOT 10


#endif /* BRAIN_H_ */
