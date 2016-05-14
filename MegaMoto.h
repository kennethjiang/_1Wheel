#ifndef _MEGA_MOTO_H
#define _MEGA_MOTO_H

#define DIRECTION -1

#define enablePin 8
#define pWMPinA 11  // Timer2
#define pWMPinB 3

// Motor doesn't output same torque on both directions. Adjust output accordingly
#define POS_GAIN 1.2
#define NEG_GAIN 1.0

// Should be called from main program's setup()
void megaMotoSetup();


// duty: -255 - 255
//    0: stop
//  255: forward, max speed
// -255: reverse, max speed
void driveMotor(int duty);

void shutoff();

#endif
