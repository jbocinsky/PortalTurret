/*
 * drivers.cpp
 *
 *  Created on: Sep 14, 2017
 *      Author: Mason Rawson
 */


#include "msp.h"
//#include <stdlib.h>
//#include <string.h>
#include <stdint.h>
//#include <iostream>
//#include <stdio.h>
//#include <math.h>
#include "config.h"
#include "drivers.h"
//#include "drivers.h"

/*
 * Global
 *
 * */

//STEPPER MOTORS
//position in rotation
float posM2 = 1;
float posM1 = 1;
float newPosM1 = 1;
float newPosM2 = 1;

//SPI COMM
static volatile uint16_t spi_rx[4];
static volatile uint8_t spi_idx = 0;
static volatile unsigned char spi_cnt = 0;
static volatile uint16_t width = MAX_WIDTH;
static volatile uint16_t height = MAX_HEIGHT;
static volatile uint16_t width_old = 0;
static volatile uint16_t height_old = 0;

//GLOBAL POS
static volatile uint16_t pt1_pos = 0;
static volatile uint16_t pt2_pos = 0;
static volatile uint16_t pt3_pos = 0;
static volatile uint16_t pt4_pos = 0;

//volatile float width_slope = (float)(MAX_TILT - MIN_TILT) / (float)(MAX_HEIGHT - MIN_HEIGHT);
//volatile float height_slope = (float)(MAX_PAN - MIN_PAN) / (float)(MAX_WIDTH - MIN_WIDTH);

static volatile uint16_t w_diff = 0;
static volatile uint16_t h_diff = 0;
static volatile char h_thresh = 0;
static volatile char w_thresh = 0;


static volatile char spiMov = 0;
static volatile char step = 5;

//WIFI COMMAND
//static volatile char grabMessage = 0;
static volatile char messageInd = 0;
static volatile char message[8];


static volatile unsigned char resetBool = 0;
static volatile char auto_mode = 1;

static volatile int h_delta = 0;
static volatile int w_delta = 0;
static volatile char udCnt = 5;
/*
 * Functions
 *
 * */

//Software Delay
void swDelay(float ms){
    int const cycles= (int) CLK_FREQ*ms/1000;
    unsigned int i;
    for(i =0; i<cycles/2; i++);
}

//DIGITAL IO PORT 8
void dOutInit(char dOut){
    //gpio config for d_out pin
    P8SEL0 &= ~dOut;
    P8SEL1 &= ~dOut;

    //out dir for d_out pin
    P8DIR |= dOut;
}

void dOutPos(char dOut){
    P8OUT |= dOut;
}

void dOutNeg(char dOut){
    P8OUT &= ~dOut;
}

void dInInit(char pin){
    if(!pin){
        P1SEL0 &= ~D_IN_0;
        P1SEL1 &= ~D_IN_0;
        P1->DIR &= ~D_IN_0;
    }
}

char dInRead(char pin){
    if(!pin){
        if(P1->IN & D_IN_0){
            return 1;
        }else{
            return 0;
        }
    }
    return 0;
}

void initClk(){
    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
//    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
    CS->CTL0 = CS_CTL0_DCORSEL_2;           // Set DCO to 6MHz
    CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
            CS_CTL1_SELS_3 |                // SMCLK = DCO
            CS_CTL1_SELM_3;                 // MCLK = DCO
    CS->KEY = 0;                            // Lock CS module from unintended accesses
}


//PWM via output compare with Timer A
void pwmInit(void){
    //Inverted PWM (because the amplifier is inverting)

    //timer in up mode: MC 01
    //counts to taxCCR0n
    //capture mode not selected, compare mode selected cap = 0
    //CCIFG interrupt flag
    //EQUn effects output based on output mode,
    //Output mode reset/set outmodx 111

    //clock src = timera0, TASSEL = 00
    //first clock div ID
    //second clock div TAIDEX

    //requires 16 bit accesses
    //uses timera0, and output module based on input

    //    NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31);
    //


    // Configure GPIO
    P2->DIR |= CCR1 | CCR2 | CCR3 | CCR4;
    P2->SEL0 |= CCR1 | CCR2 | CCR3 | CCR4;

    // Configure Timer_A
    // TIMER_A0->CCR[0] toggle, interrupt not enabled
//    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_OUTMOD_2 | TIMER_A_CCTLN_CCIE;
    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_OUTMOD_2;

    //P2.4, TACCR1 toggle, interrupt not enabled
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_2;

    //P2.5, TACCR2 toggle, interrupt not enabled
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_2;

    //P2.6, TACCR2 toggle, interrupt not enabled
    TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_2;

    //P2.7, TACCR2 toggle, interrupt not enabled
    TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_2;

    //Initial positions
    TIMER_A0->CCR[0] = CCR0_PERIOD;
    TIMER_A0->CCR[1] = CCR1_INIT;
    TIMER_A0->CCR[2] = CCR2_INIT;
    TIMER_A0->CCR[3] = CCR3_INIT;
    TIMER_A0->CCR[4] = CCR4_INIT;

    // 3M clk SMCLK, up mode, no clk dividing, timer overflow interrupt enabled
    //this starts the timer
    TIMER_A0->CTL = TIMER_A_CTL_TASSEL_2 | ID__1 | TIMER_A_CTL_MC__UP | TIMER_A_CTL_IE;

    // Enable global interrupt
    //NVIC_EnableIRQ(TA0_N_IRQn);
    //NVIC_EnableIRQ(TA0_0_IRQn);
    //NVIC->ISER[0] |= 1 << ((TA0_N_IRQn) & 31);
//    NVIC->ISER[0] |= 1 << ((TA0_0_IRQn) & 31);
//    __enable_irq();

}

void pwmInitPos(void){
    //Initial positions
    TIMER_A0->CCR[0] = CCR0_PERIOD;
    TIMER_A0->CCR[1] = CCR1_INIT;
    TIMER_A0->CCR[2] = CCR2_INIT;
    TIMER_A0->CCR[3] = CCR3_INIT;
    TIMER_A0->CCR[4] = CCR4_INIT;
}


// Timer A0 interrupt service routine
//__interrupt void TA0_0_IRQHandler(void){
//    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
//}

//PWM
void pwmGo(char ccrSel, uint16_t amt){

    //check validity of ccrSel
    if(ccrSel<=0 || ccrSel>4){
        return;
    }

    //shift pwm
    uint16_t oldPos = TIMER_A0->CCR[ccrSel];

    uint16_t newPos = oldPos;
    uint16_t delta = (newPos-amt)/step;
    char i;
    for(i=0;i<step;i++){

        if(i==step-1){
            newPos=amt;
        }else{
            newPos-=delta;
        }

        switch(ccrSel){
        case 1:
            if(newPos < CCR1_MIN){
                pt1_pos = CCR1_MIN;
                TIMER_A0->CCR[1] = CCR1_MIN;
                break;
//                return true;
            }else if(newPos > CCR1_MAX){
                pt1_pos = CCR1_MAX;
                TIMER_A0->CCR[1] = CCR1_MAX;
                break;
//                return true;
            }else{
                pt1_pos = newPos;
                TIMER_A0->CCR[1] = newPos;
//                return true;
                break;
            }
        case 2:
            if(newPos < CCR2_MIN){
                pt2_pos = CCR2_MIN;
                TIMER_A0->CCR[2] = CCR2_MIN;
//                return true;
                break;
            }else if(newPos > CCR2_MAX){
                pt2_pos = CCR2_MAX;
                TIMER_A0->CCR[2] = CCR2_MAX;
//                return true;
                break;
            }else{
                pt2_pos = newPos;
                TIMER_A0->CCR[2] = newPos;
//                return true;
                break;
            }
        case 3:
            if(newPos < CCR3_MIN){
                pt3_pos = CCR3_MIN;
                TIMER_A0->CCR[3] = CCR3_MIN;
//                return true;
                break;
            }else if(newPos > CCR3_MAX){
                pt3_pos = CCR3_MAX;
                TIMER_A0->CCR[3] = CCR3_MAX;
//                return true;
                break;
            }else{
                pt3_pos = newPos;
                TIMER_A0->CCR[3] = newPos;
//                return true;
                break;
            }
        case 4:
            if(newPos < CCR4_MIN){
                pt4_pos = CCR4_MIN;
                TIMER_A0->CCR[4] = CCR4_MIN;
//                return true;
                break;
            }else if(newPos > CCR4_MAX){
                pt4_pos = CCR4_MAX;
                TIMER_A0->CCR[4] = CCR4_MAX;
//                return true;
                break;
            }else{
                pt4_pos = newPos;
                TIMER_A0->CCR[4] = newPos;
//                return true;
                break;
            }
        default:
//            return false;
            break;
        }
        swDelay(5);
    }
}

bool pwmMove(char ccrSel, int amt){
    //amount to be entered in tens of microseconds (x5 us)
    //check validity of ccrSel
    if(ccrSel<=0 || ccrSel>4){
        return false;
    }

    //shift pwm
    uint16_t oldPos = TIMER_A0->CCR[ccrSel];
    uint16_t newPos = oldPos + (uint16_t) (amt*CCR_5US);
    switch(ccrSel){
    case 1:
        if(newPos < CCR1_MIN){
            pt1_pos = CCR1_MIN;
            TIMER_A0->CCR[1] = CCR1_MIN;
            return true;
        }else if(newPos > CCR1_MAX){
            pt1_pos = CCR1_MAX;
            TIMER_A0->CCR[1] = CCR1_MAX;
            return true;
        }else{
            pt1_pos = newPos;
            TIMER_A0->CCR[1] = newPos;
            return true;
        }
    case 2:
        if(newPos < CCR2_MIN){
            pt2_pos = CCR2_MIN;
            TIMER_A0->CCR[2] = CCR2_MIN;
            return true;
        }else if(newPos > CCR2_MAX){
            pt2_pos = CCR2_MAX;
            TIMER_A0->CCR[2] = CCR2_MAX;
            return true;
        }else{
            pt2_pos = newPos;
            TIMER_A0->CCR[2] = newPos;
            return true;
        }
    case 3:
        if(newPos < CCR3_MIN){
            pt3_pos = CCR3_MIN;
            TIMER_A0->CCR[3] = CCR3_MIN;
            return true;
        }else if(newPos > CCR3_MAX){
            pt3_pos = CCR3_MAX;
            TIMER_A0->CCR[3] = CCR3_MAX;
            return true;
        }else{
            pt3_pos = newPos;
            TIMER_A0->CCR[3] = newPos;
            return true;
        }
    case 4:
        if(newPos < CCR4_MIN){
            pt4_pos = CCR4_MIN;
            TIMER_A0->CCR[4] = CCR4_MIN;
            return true;
        }else if(newPos > CCR4_MAX){
            pt4_pos = CCR4_MAX;
            TIMER_A0->CCR[4] = CCR4_MAX;
            return true;
        }else{
            pt4_pos = newPos;
            TIMER_A0->CCR[4] = newPos;
            return true;
        }
    default:
        return false;
    }
}




// Stepper Motors GPIO Initialization
void stepperMotorInit(void){

    P4->IE = 0;
    P6->IE = 0;

    //M1 CTRL GPIO
    P6->SELC &= ~(M1IN1 | M1IN2 | M1IN3 | M1IN4);
    P6->DIR |= (M1IN1 | M1IN2 | M1IN3 | M1IN4);

    //M2 CTRL GOIO
    P4->SELC &= ~(M2IN1 | M2IN2 | M2IN3 | M2IN4);
    P4->DIR |= (M2IN1 | M2IN2 | M2IN3 | M2IN4);

}

void phaseSelect(int phase, int motor, float delay){
    if(motor == M1){
        P6->OUT &= ~(M1IN4 | M1IN3 | M1IN2 | M1IN1);
        switch(phase){
         case 0:
           P6->OUT |= (M1IN4);
           break;
         case 1:
           P6->OUT |= (M1IN3) | (M1IN4);
           break;
         case 2:
             P6->OUT |= (M1IN3);
           break;
         case 3:
             P6->OUT |= (M1IN3) | (M1IN2);
           break;
         case 4:
             P6->OUT |= (M1IN2);
           break;
         case 5:
             P6->OUT |= (M1IN1) | (M1IN2);
           break;
         case 6:
             P6->OUT |= (M1IN1);
           break;
         case 7:
             P6->OUT |= (M1IN1) | (M1IN4);
           break;
         default:
             P6->OUT |= 0x0;
           break;
        }
    }else{
        P4->OUT &= ~(M2IN4 | M2IN3 | M2IN2 | M2IN1);
        switch(phase){
         case 0:
           P4->OUT |= (M2IN4);
           break;
         case 1:
           P4->OUT |= (M2IN3) | (M2IN4);
           break;
         case 2:
           P4->OUT |= (M2IN3);
           break;
         case 3:
           P4->OUT |= (M2IN3) | (M2IN2);
           break;
         case 4:
           P4->OUT |= (M2IN2);
           break;
         case 5:
           P4->OUT |= (M2IN1) | (M2IN2);
           break;
         case 6:
           P4->OUT |= (M2IN1);
           break;
         case 7:
           P4->OUT |= (M2IN1) | (M2IN4);
           break;
         default:
           P4->OUT |= 0x0;
           break;
        }
    }
    // pause so the stepper has time to react
    //__delay_cycles((1/3M)*50000);
    swDelay(delay);
}

// Turning calc for steppers
void turns(float rotations, int motor, float delay){
  // if the rotation count is -ve then it is CCW, otherwise it is positive
  int clockwise;
  if(rotations >= 0){
      clockwise = 1;
  }else{
      clockwise = 0;
  }

  // calculate how many cycles the stepper will have to make
  // force the cycle count to be positive
  int cycles = (int)abs(rotations * CYCLES_PER_ROTATION);

  if(motor == M1){
      posM1 += rotations;
      fabs(posM1);

  }else{
      posM2 += rotations;
      fabs(posM2);
  }

  // only move if the user specified an actual movement
  if(rotations != 0){
    if(clockwise == 1){
      // for each cycle
      int x,y;
      for (x=0; x<cycles; x++){
        // for each phase
        for(y=0; y<8; y++){
          // go to phase y
          phaseSelect(y, motor, delay);
        }
      }
    }else {
      // for each cycle
      int x,y;
      for (x=0; x<cycles; x++){
        // for each phase (backwards for CCW rotation)
        for(y=7; y>=0; y--){
          // go to phase y
          phaseSelect(y, motor, delay);
        }
      }
    }
  }
  // go to the default state (all poles off) when finished
  phaseSelect(8, motor, delay);
}

/*
 * SPI Divers
 */
//Initial SPI to slave, low clk polarity
void spiInit(void){
    P1->SEL0 |= BIT5 | BIT6 | BIT7;         // Set P1.5, P1.6, and P1.7 as
                                            // SPI pins functionality

    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST; // Put state machine in reset
    EUSCI_B0->CTLW0 = EUSCI_B_CTLW0_SWRST | // Keep the state machine in reset
            EUSCI_B_CTLW0_SYNC |            // Set as synchronous mode
            EUSCI_B_CTLW0_MSB |              // MSB first and Slave
            UCSSEL_2;
            //EUSCI_A_CTLW0_CKPL;         // Set clock polarity high
//            UCSSEL_2;

    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SSEL__ACLK; // ACLK
    EUSCI_B0->BRW = 0x00;                   // /2,fBitClock = fBRCLK/(UCBRx+1).
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;// Initialize USCI state machine
    EUSCI_B0->IE |= EUSCI_B_IE_RXIE;        // Enable USCI_B0 RX interrupt

    // Enable global interrupt
    __enable_irq();

    // Enable eUSCI_B0 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIB0_IRQn) & 31);
}

//Update global spi position
void updatePos(void){

    int a = (uint16_t)spi_rx[0];
    int b = (uint16_t)spi_rx[1];
    int c = (uint16_t)spi_rx[2];
    int d = (uint16_t)spi_rx[3];

    uint16_t h_tmp = (a<<8) + b;
    uint16_t w_tmp = (c<<8) + d;
    h_tmp+=10;
//    w_tmp = (c<<8) + d;

//    h_diff = abs(height - h_tmp);
//    w_diff = abs(width - w_tmp);

//    if(h_diff>h_thresh)

//    if(h_tmp<MIN_HEIGHT){
//        height = MIN_HEIGHT;
//    }else if(h_tmp>MAX_HEIGHT){
//
//    }else{
    h_delta = abs(height-h_tmp);
    w_delta = abs(width-w_tmp);

    height = h_tmp;
//    if(w_diff>w_thresh)
    width = w_tmp;

}


/*
 * Moving Turrets to Pos
 * */
uint16_t widthToUs(uint16_t w){
    w = MAX_WIDTH - w;
//    volatile float width_slope = (float)(MAX_PAN - MIN_PAN) / (float)(MAX_HEIGHT - MIN_HEIGHT);
    //converts amount to move in int to microsecond us changes to PWM
//    float slope = ((float)CCR_1MS)/MAX_WIDTH;
    return (uint16_t)(width_slope * (w - MIN_WIDTH) + MIN_PAN);
}

uint16_t heightToUs(uint16_t h){

//    uint16_t hTmp = MAX_HEIGHT - h;

//    h = hTmp;
//    volatile float height_slope = (float)((MAX_TILT - MIN_TILT)) / (float)(MAX_WIDTH - MIN_WIDTH);
    //converts amount to move in int to microsecond us changes to PWM
//    float slope = ((float)CCR_P5MS)/MAX_HEIGHT;
    uint16_t hTmp = (uint16_t)(height_slope * (h - MIN_HEIGHT) + MIN_TILT);
    return hTmp;
}

void movPos(void){

    //finds the amount to move from the most recent face
    //only execute if a significant diff noticed

    if(height>480 || width>640){
        height = 0;
        width = 0;
        spi_idx = 0;
        //kill spi
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST; // Put state machine in reset
        resetCenter();
        spiInit();
//        spiInit();
//        swDelay(100);
    }else{

    //    if(h_diff>h_thresh){
            pwmGo(2, heightToUs(height-10));
            pwmGo(4, heightToUs(height));
    //    }
    //    if(w_diff>w_thresh){
            if (width<125){
                pwmGo(3, widthToUs(width));
                pwmGo(1, widthToUs(width+25));
            }else if(width >= 450){
                pwmGo(3, widthToUs(width-75));
                pwmGo(1, widthToUs(width-25));
            }else{
                pwmGo(3, widthToUs(width-50));
                pwmGo(1, widthToUs(width));
            }
    //    }

    }
    spiMov = 0;
}


//SPI ISR
void EUSCIB0_IRQHandler(void)
{
    //check ISR flag and RX flag
    if (EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG){

        spi_rx[spi_idx++] = EUSCI_B0->RXBUF;

        if(spi_idx==4){
            //have received full data for height and width
            spi_idx = 0;
            //Update global spi position
            updatePos();
            //go to position in main
//            movPos();
                spiMov = 1;
            //shoot
            spi_cnt++;
        }
    }
    //kill ISR flag
    EUSCI_B0->IFG &= ~(EUSCI_B_IFG_RXIFG);
}



/*
 * WIFI CODE
 * */
void initUART(){
//    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
//    CS->CTL0 = 0;                           // Reset tuning parameters
//    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
//    CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
//            CS_CTL1_SELS_3 |                // SMCLK = DCO
//            CS_CTL1_SELM_3;                 // MCLK = DCO
//    CS->KEY = 0;                            // Lock CS module from unintended accesses

    // Configure UART pins
    //P1->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function
    P3->SEL0 |= BIT2 | BIT3;

    // Configure UART
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset
            EUSCI_B_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK

    // Baud Rate calculation
    // 12000000/(16*9600) = 78.125
    // Fractional portion = 0.125
    // User's Guide Table 21-4: UCBRSx = 0x10
    // UCBRFx = int ( (78.125-78)*16) = 2
//    EUSCI_A2->BRW = 78;                     // 12000000/16/9600
//    EUSCI_A2->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) |
//            EUSCI_A_MCTLW_OS16;

    // Baud Rate calculation
    // 6,000,000/(16*9600) = 39.0625
    // Fractional portion = 0.125
    // User's Guide Table 21-4: UCBRSx = 0x10
    // UCBRFx = int ( (39.0625-39)*16) = 1
    EUSCI_A2->BRW = 39;                     //6,000,000/(16*9600)
    EUSCI_A2->MCTLW = (1 << EUSCI_A_MCTLW_BRF_OFS) |
            EUSCI_A_MCTLW_OS16;

    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
    EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A2 RX interrupt

    // Enable global interrupt
    __enable_irq();

    // Enable eUSCIA2 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIA2_IRQn) & 31);
}

// UART interrupt service routine
void EUSCIA2_IRQHandler(void){
    if (EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG){

        char serialInput = EUSCI_A2->RXBUF;

        if(serialInput == 's'){
            messageInd = 0;
        }else if(serialInput == 'e'){
            messageInd = 0;
            movWifi();
        }else{
            message[messageInd] = serialInput;
            messageInd += 1;
        }
    }
}

void movWifi(void){

    char idx = 0;
    for(idx=0;idx<3;idx++){
        switch(message[idx]){
        case 'u':
            //Up
            udCnt++;
            if(udCnt==5){
                udCnt=0;
                pwmMove(2,1);
                pwmMove(4,1);
            }
            break;
        case 'd':
            //Down
            udCnt++;
            if(udCnt==5){
                udCnt=0;
                pwmMove(2,-1);
                pwmMove(4,-1);
            }
            break;
        case 'l':
            //Left
            pwmMove(1,1);
            pwmMove(3,1);
            break;
        case 'r':
            //Right
            pwmMove(1,-1);
            pwmMove(3,-1);
            break;
        case 'M':
            //manual mode
            resetBool = 1;
            auto_mode = 0;
            udCnt = 0;
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST; // Put state machine in reset
            break;
        case 'A':
            //autonomous mode
            resetBool = 1;
            auto_mode = 1;
            spiInit();
            break;
        default:
            break;
        }
    }

    if(message[3] == '1'){
        //Reset Center
        resetBool = 1;
    }else if(message[5] == '1'){
        //Shoot right
        shootSingle(M2);
    }else if(message[6] == '1'){
        //shoot left
        shootSingle(M1);
    }else if(message[7] == '1'){
        //shoot both
        shootDouble();
    }
}


// Full 360 movement
void shootDouble(void){
    dOutPos(GUN_CTRL);
    int i;
    for(i = 0; i<FULL_STEP; i++){
        turns(SMALL_STEP,M1,1.0*DEFAULT_MOTOR_SPEED);
        turns(SMALL_STEP,M2,1.0*DEFAULT_MOTOR_SPEED);
    }
    dOutNeg(GUN_CTRL);
}

// Full 360 movement
void shootSingle(int motor){
    dOutPos(GUN_CTRL);
    int i;
    for(i = 0; i<FULL_STEP; i++){
        turns(SMALL_STEP,motor,1.0*DEFAULT_MOTOR_SPEED);
    }
    dOutNeg(GUN_CTRL);
}


//Reset Center/Init
void resetCenter(void){

    resetBool = 0;

    pwmGo(1,CCR1_INIT);
    pwmGo(2,CCR2_INIT);
    pwmGo(3,CCR3_INIT);
    pwmGo(4,CCR4_INIT);

}


void spiShoot(void){
//    if(spiCnt == 10){
//
//    }
}


void pause(){
    while(1){

        //Reset based on the from uart communication from WIFI/Remote Controller
        if(resetBool == 1){
            resetCenter();
        }

        if(auto_mode){
            //Move based on spi communication from pi
            if(spiMov == 1){
                movPos();
                if(h_delta<5 && w_delta<10){
                    shootSingle(M1);
                    shootSingle(M2);
//                    h_delta =0;
//                    w_delta =0;
                }
            }
        }
    }
}

//    int ccrOld[4];
//    int ccrDelta[4];
//    char step_size = 20;
//
//    ccrOld[0] = TIMER_A0->CCR[1];
//    ccrDelta[0] = (int)((CCR1_INIT-ccrOld[0])/step_size);
//
//    ccrOld[1] = TIMER_A0->CCR[2];
//    ccrDelta[1] = (int)((CCR2_INIT-ccrOld[1])/step_size);
//
//    ccrOld[2] = TIMER_A0->CCR[3];
//    ccrDelta[2] = (int)((CCR3_INIT-ccrOld[2])/step_size);
//
//    ccrOld[3] = TIMER_A0->CCR[4];
//    ccrDelta[3] = (int)((CCR4_INIT-ccrOld[3])/step_size);
//
//    char i,j;
//    for(i=1; i!=(step_size); i++){
//        for(j=0; j!=4; j++){
//            pwmGo(j+1, ccrOld[j] + i*ccrDelta[j]);
//        }
//        swDelay(10);
//    }
