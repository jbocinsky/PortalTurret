/*
 * PeripheralLib.h
 *
 *  Created on: Oct 8, 2017
 *      Author: Owner
 */

#ifndef PERIPHERALLIB_H_
#define PERIPHERALLIB_H_

//global variables
static long joyStickX = 2000; //-> pin 5.5, analog value of joystick
static long joyStickY = 2000; //-> pin 5.4, analog value of joystick
static char dX = 0; //Movement in X direction, l = left, r = right, 0 = none
static char dY = 0; //Movement in Y direction, d = down, u = up, 0 = none
static char serialInput = '_';
static char controlState = 0;
static char sendMessage = 0;

//function prototypes
void flashLEDS(void);
void flashRightLEDS(void);
void flashLeftLEDS(void);
void flashMiddleLEDS(void);
void initADC(void);
void initGPIO(void);
void initLEDS(void);
void initButtons(void);
void initUART(void);
char getButtons(void);
void getADC(void);
void setLEDS(char LEDS);
void charOut(char ch);
void serialStringOut(char serialOut[]);
void messageOut(char message[]);
void resetDefaultMessage(char *message);
void setDefaultMessage(char *message);
void setMessageField(char *message, int messageBit, char value);

#endif /* PERIPHERALLIB_H_ */
