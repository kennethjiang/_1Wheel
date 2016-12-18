#ifndef CYTRON_H
#define CYTRON_H

#define DIRECTION 1

#define pWMPin 7
#define dirPin 6
#define enablePin 12

// Motor doesn't output same torque on both directions. Adjust output accordingly
#define POS_GAIN 1.0
#define NEG_GAIN 1.0

// Should be called from main program's setup()
void driverSetup();


// duty: -255 - 255
//    0: stop
//  255: forward, max speed
// -255: reverse, max speed
void drive(int duty);

void disableMotor();

#endif
