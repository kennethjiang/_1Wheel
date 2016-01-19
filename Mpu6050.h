#ifndef _MPU6050_H
#define _MPU6050_H

#include "I2Cdev.h"

// supply your own gyro offsets by redefining them.
#define X_GYRO_OFFSET 220
#define Y_GYRO_OFFSET 76
#define Z_GYRO_OFFSET -85
#define Z_ACCEL_OFFSET 1788  // 1688 factory default for my test chip

// Should be called from main program's setup()
void mpu6050Setup();

// MPU 6050's reading tends to drift significantly at the beginning.
// This is a helper method to determine if reading has stabilized, which is 
// defined as <1% change within 1s.
// The sensor should be kept absolutely stable when this function is called
// for it to function correctly.
bool hasSensorStabilized();

// return NULL if anything has gone wrong
float *readYpr();

int16_t *lastGyroReadking();

#endif
