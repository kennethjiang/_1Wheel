#ifndef _MEGA_MOTO_H
#define _MEGA_MOTO_H

int enablePin = 8;
int duty;
int pWMPinA = 11;  // Timer2
int pWMPinB = 3;

// Should be called from main program's setup()
void megaMotoSetup();

#endif