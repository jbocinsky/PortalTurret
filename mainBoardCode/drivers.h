/*
 * drivers.h
 *
 *  Created on: Sep 16, 2017
 *      Author: Mason Rawson
 */

#ifndef DRIVERS_H_
#define DRIVERS_H_

#include "config.h"

/*
 * Prototypes
 *
 * */

void swDelay(float ms);
void dOutInit(char dOut);
void dOutPos(char dOut);
void dOutNeg(char dOut);
void dInInit(char pin);
char dInRead(char pin);
void initClk();
void pwmInit(void);
bool pwmMove(char ccrSel, int amt);
void pwmGo(char ccrSel, uint16_t amt);
void stepperMotorInit(void);
void turns(float rotations, int motor, float delay);
void phaseSelect(int phase, int motor, float delay);

void spiInit(void);
void updatePos(void);

void movPos(void);
uint16_t widthToUs(uint16_t w);
uint16_t heightToUs(uint16_t h);
void pwmInitPos(void);



void initUART();
void updateWifi();
void movWifi();

void shootDouble(void);
void shootSingle(int motor);

void spiShoot(void);
void resetCenter(void);
void pause();

#endif /* DRIVERS_H_ */
