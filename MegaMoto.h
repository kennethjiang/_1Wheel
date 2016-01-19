#ifndef _MEGA_MOTO_H
#define _MEGA_MOTO_H

// Should be called from main program's setup()
void megaMotoSetup();


// duty: -255 - 255
//    0: stop
//  255: forward, max speed
// -255: reverse, max speed
void driveMotor(int duty);

void shutoff();

#endif
