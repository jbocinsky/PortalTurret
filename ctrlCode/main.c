//*****************************************************************************
//
// MSP432 main.c template - Empty main
//
//****************************************************************************

#include "msp.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "PeripheralLib.h"

void main(void)
{

    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer

    initGPIO();
    flashLEDS();
    initADC();
    initUART();
    controlState = 0;

    char buttons;
    char message[11];
    //Message BitMask:
    //0: 's'
    //1: U/D
    //2: L/R
    //3: M/A
    //4: B4
    //5: 0
    //6: B3
    //7: B2
    //8: B1
    //9: 'e'
    //10: '!'

    //Mason's Message Received:
    //0: U/D
    //1: L/R
    //2: M/A
    //3: B4
    //4: 0
    //5: B3
    //6: B2
    //7: B1

    while (1)
    {
        resetDefaultMessage(message);
        sendMessage = 0;

        getADC(); //joystickX and joyStickY set in interrupt when conversion done
        buttons = getButtons();

        if(buttons == 0x01){
            setMessageField(message, 8, '1');
            sendMessage = 1;
        }
        else if(buttons == 0x02){
            setMessageField(message, 7, '1');
            sendMessage = 1;
        }
        else if(buttons == 0x04){
            setMessageField(message, 6, '1');
            sendMessage = 1;
        }
        else if(buttons == 0x08){
            if(controlState == 0){
                controlState = 1;
                setLEDS(0xFF);
                //Set Main Board in Manual Mode
                setMessageField(message, 3, 'M');
                int i = 0;
                for(i = 300000; i > 0; i--);        // Delay
                flashLEDS();
                for(i = 300000; i > 0; i--);        // Delay
                flashLEDS();
                for(i = 300000; i > 0; i--);        // Delay
                flashLEDS();
                sendMessage = 1;
            }
            else if(controlState == 1){
                controlState = 0;
                setLEDS(0xFF);
                //Set Main Board in Autonomous mode
                setMessageField(message, 3, 'A');
                int i = 0;
                for(i = 300000; i > 0; i--);        // Delay
                flashLEDS();
                sendMessage = 1;
            }
        }
        else if(buttons == 0x10){
            setLEDS(0x00);
            setMessageField(message, 4, '1');
            int i = 0;
            for(i = 250000; i > 0; i--);        // Delay
            setLEDS(0xFF);
            sendMessage = 1;
        }

        if(dX != '0'){
            setMessageField(message, 2, dX);
            sendMessage = 1;
        }

        if(dY != '0'){
            setMessageField(message, 1, dY);
            sendMessage = 1;
        }

        if((sendMessage && controlState) || message[3] == 'A'){
//            //Used for debugging:
//            serialStringOut("\r\n");
            messageOut(message);
            if(message[8] == '1'){
                flashMiddleLEDS();
            }
            if(message[7] == '1'){
                flashLeftLEDS();
            }
            if(message[6] == '1'){
                flashRightLEDS();
            }
        }
        if(controlState == 0){
            flashLeftLEDS();
            flashRightLEDS();
        }
        else if(controlState == 1){
            setLEDS(0xFF);
        }
    }
}

//Interrupts
//
// ADC14 interrupt service routine
void ADC14_IRQHandler(void) {
    if(ADC14->IV == 0x0000000C){
        joyStickX = ADC14->MEM[0];                  // A0 >= .5 AVCC?  pin 5.5

        if(joyStickX < 1000){
            dX = 'l';
        }
        else if(joyStickX > 3000){
            dX = 'r';
        }
        else{
            dX = '0';
        }
    }
    else if(ADC14->IV == 0x0000000E){
        joyStickY = ADC14->MEM[1];                  // A1 >= .5 AVCC? pin 5.4

        //joyStickY is inverted (up has a low pot value)
        if(joyStickY < 1000){
            dY = 'u';
        }
        else if(joyStickY > 3000){
            dY = 'd';
        }
        else{
            dY = '0';
        }
    }

}

// UART interrupt service routine
void EUSCIA2_IRQHandler(void)
{
    if (EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG)
    {
        // Check if the TX buffer is empty first
        //while(!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));

        serialInput = EUSCI_A2->RXBUF; //read the uart buffer to clear interrupt flag, so UART can proceed like usual and send messages on a UART write
        //serialStringOut("Input Character:  ");
        // Echo the received character back
        //EUSCI_A2->TXBUF = serialInput;
    }

}
