/*
 * PeripheralLib.c
 *
 *  Created on: Oct 8, 2017
 *      Author: Owner
 */

#include "msp.h"
#include "PeripheralLib.h"

void flashLEDS(){
    setLEDS(0xFF);
    int i = 0;
    for(i = 300000; i > 0; i--);        // Delay
    setLEDS(0x00);
    for(i = 100000; i > 0; i--);        // Delay
    setLEDS(0xFF);
    for(i = 100000; i > 0; i--);        // Delay
    setLEDS(0x00);
    for(i = 100000; i > 0; i--);        // Delay
    setLEDS(0xFF);
    for(i = 100000; i > 0; i--);        // Delay
    setLEDS(0x00);
    for(i = 100000; i > 0; i--);        // Delay
}

void flashRightLEDS(){
    int i = 0;
    for(i = 60000; i > 0; i--);        // Delay
    setLEDS(0x80);
    for(i = 60000; i > 0; i--);        // Delay
    setLEDS(0x40);
    for(i = 60000; i > 0; i--);        // Delay
    setLEDS(0x20);
    for(i = 60000; i > 0; i--);        // Delay
    setLEDS(0x10);
    for(i = 60000; i > 0; i--);        // Delay
    setLEDS(0x00);
    for(i = 60000; i > 0; i--);        // Delay
    setLEDS(0x08);
    for(i = 60000; i > 0; i--);        // Delay
    setLEDS(0x04);
    for(i = 60000; i > 0; i--);        // Delay
    setLEDS(0x02);
    for(i = 60000; i > 0; i--);        // Delay
    setLEDS(0x01);
    for(i = 600000; i > 0; i--);        // Delays

}

void flashLeftLEDS(){
    int i = 0;
    for(i = 60000; i > 0; i--);        // Delay
    setLEDS(0x01);
    for(i = 60000; i > 0; i--);        // Delay
    setLEDS(0x02);
    for(i = 60000; i > 0; i--);        // Delay
    setLEDS(0x04);
    for(i = 60000; i > 0; i--);        // Delay
    setLEDS(0x08);
    for(i = 60000; i > 0; i--);        // Delay
    setLEDS(0x00);
    for(i = 60000; i > 0; i--);        // Delay
    setLEDS(0x10);
    for(i = 60000; i > 0; i--);        // Delay
    setLEDS(0x20);
    for(i = 60000; i > 0; i--);        // Delay
    setLEDS(0x40);
    for(i = 60000; i > 0; i--);        // Delay
    setLEDS(0x80);
    for(i = 600000; i > 0; i--);        // Delay
}

void flashMiddleLEDS(){
    int i = 0;
    for(i = 120000; i > 0; i--);        // Delay
    setLEDS(0x81);
    for(i = 120000; i > 0; i--);        // Delay
    setLEDS(0x42);
    for(i = 120000; i > 0; i--);        // Delay
    setLEDS(0x24);
    for(i = 120000; i > 0; i--);        // Delay
    setLEDS(0x18);
    for(i = 600000; i > 0; i--);        // Delay
    setLEDS(0x00);
}

void initGPIO(){
    initLEDS();
    initButtons();
}

void initADC(){

    P5->SEL1 |= BIT5;                       // Configure P5.5 for ADC A0
    P5->SEL0 |= BIT5;

    P5->SEL1 |= BIT4;                       // Configure P5.4 for ADC A1
    P5->SEL0 |= BIT4;

    // Enable global interrupt
    __enable_irq();

    // Enable ADC interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31);
    NVIC->ISER[1] = 1 << ((ADC14_IRQn) & 31);

    // Sampling time, S&H=16, ADC14 on
    ADC14->CTL0 = ADC14_CTL0_SHT0__32 | ADC14_CTL0_CONSEQ_1 | ADC14_CTL0_MSC | ADC14_CTL0_SHP | ADC14_CTL0_ON;
    ADC14->CTL1 = ADC14_CTL1_RES_2;         // Use sampling timer, 12-bit conversion results

    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_0;   // A0 ADC input select; Vref=AVCC
    ADC14->MCTL[1] |= ADC14_MCTLN_INCH_1;   // A1 ADC input select; Vreg=AVCC

    ADC14->IER0 |= ADC14_IER0_IE1;          // Enable ADC conv complete interrupt
    ADC14->IER0 |= ADC14_IER0_IE0;          // Enable ADC conv complete interrupt
}

void initLEDS(){
    P2->OUT = 0x00;                       // Clear LEDS to start
    P2->DIR = 0xFF;                       // Set Port as outputs
}

void initButtons(){

    //Set SEL0 and SEL1 to 0 to set Port 6 to GPIO
    P6SEL0 = 0;
    P6SEL1 = 0;
    //Set P6DIR to 1 to be an output
    P6DIR = 1;

    P6OUT = 0x0;

    //Set SEL0 and SEL1 to 0 to set Port 4 to GPIO
    P4SEL0 = 0;
    P4SEL1 = 0;

    //Set P4DIR to 0 to be an input
    P4DIR = 0;
}

void initUART(){
    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
            CS_CTL1_SELS_3 |                // SMCLK = DCO
            CS_CTL1_SELM_3;                 // MCLK = DCO
    CS->KEY = 0;                            // Lock CS module from unintended accesses

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
    EUSCI_A2->BRW = 78;                     // 12000000/16/9600
    EUSCI_A2->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) |
            EUSCI_A_MCTLW_OS16;

    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
    EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A2 RX interrupt

    // Enable global interrupt
    __enable_irq();

    // Enable eUSCIA2 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIA2_IRQn) & 31);
}

void getADC(){
    // Enable and Start conversion (must be done at the same time)
    ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;
}

char getButtons(){
    char button1 = P4IN & 0x01; //Pin 4.0
    char button2 = P4IN & 0x02; //Pin 4.1
    char button3 = P4IN & 0x04; //Pin 4.2
    char button4 = P4IN & 0x08; //Pin 4.3
    char button5 = P4IN & 0x10; //Joy stick button Pin 4.4
    button5 ^= 1 << 4;    //Joy stick button inverted (toggle this bit for button5)

    char buttons = button5 | button4 | button3 | button2 | button1;
    return buttons;
}

void setLEDS(char LEDS){
    P2->OUT = LEDS;
}

void charOut(char ch){
    while(!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A2->TXBUF = ch;
}

void serialStringOut(char str[]){
    int i;
    for(i = 0; str[i] != '\0'; i++){
        // Check if the TX buffer is empty first
        while(!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
        EUSCI_A2->TXBUF = str[i];
    }
}

void messageOut(char message[]){
    int i;
    for(i = 0; message[i] != '!'; i++){
        // Check if the TX buffer is empty first
        while(!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
        char ch = message[i];
        EUSCI_A2->TXBUF = ch;
    }
}

void resetDefaultMessage(char *message){

    message[0] = 's';
    message[1] = '0';
    message[2] = '0';
    message[3] = '0';
    message[4] = '0';
    message[5] = '0';
    message[6] = '0';
    message[7] = '0';
    message[8] = '0';
    message[9] = 'e';
    message[10] = '!';
}

void setMessageField(char *message, int messageBit, char value){
    message[messageBit] = value;
}

