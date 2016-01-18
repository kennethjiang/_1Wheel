#include "Motor.h"



const byte CPin = 0;  // analog input channel
int CRaw;      // raw A/D value
float CVal;    // adjusted Amps value

void setup() {                
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards:
  pinMode(EnablePin, OUTPUT);     
  pinMode(PWMPin, OUTPUT);
  pinMode(PWMPin2, OUTPUT);
  setPwmFrequency(PWMPin, 8);  // change Timer2 divisor to 8 gives 3.9kHz PWM freq
}

void loop() {
    // To drive the motor in H-bridge mode
    // the power chip inputs must be opposite polarity
    // and the Enable input must be HIGH
    digitalWrite(EnablePin, HIGH);
    analogWrite(PWMPin2, 0);
    for(duty = 0; duty <= 255; duty += 5){
      analogWrite(PWMPin, duty);
      delay(200);
    }
    analogWrite(PWMPin, 255);
    CRaw = analogRead(CPin);
    delay(2000);
    for(duty = 255; duty>=0; duty -= 5){
      analogWrite(PWMPin, duty);   
      delay(200);   
    }
    analogWrite(PWMPin, 0);
    delay(500);
    // Toggle enable to reset the power chips if we have had 
    // an overcurrent or overtemp fault
    digitalWrite(EnablePin, LOW);
    delay(500);
    
    // Swap pins to make the motor reverse
    if(PWMPin == 11) {
      PWMPin = 3;
      PWMPin2 = 11;
    } else {
      PWMPin = 11;
      PWMPin2 = 3;
    }
}
/*
 * Divides a given PWM pin frequency by a divisor.
 * 
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024.
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024.
 * 
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired (Timer0)
 *   - Pins 9 and 10 are paired (Timer1)
 *   - Pins 3 and 11 are paired (Timer2)
 * 
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 5, 6 may cause the delay() and
 *     millis() functions to stop working. Other timing-related
 *     functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 * 
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1235060559/0#4
 */
 
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) { // Timer0 or Timer1
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) { 
      TCCR0B = TCCR0B & 0b11111000 | mode; // Timer0
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode; // Timer1
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode; // Timer2
  }
}
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

}



// ================================================================
// ===        READING AND PROCESSING MPU 6050 DATA              ===
// ================================================================

bool isSensorDataReady() {
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data

    return mpuInterrupt || fifoCount >= packetSize;
}

// Return NULL if DMP is not ready or anything has gone wrong
Quaternion *readQuaternion() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return NULL;

    // wait for MPU interrupt or extra packet(s) available
    while (!isSensorDataReady()) ;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

        return NULL;
    } 
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_GYRO
            // display gyro in easy matrix form: x y z
            mpu.dmpGetGyro(gyro, fifoBuffer);
            Serial.print("gyro\t");
            Serial.print(gyro[0]);
            Serial.print("\t");
            Serial.print(gyro[1]);
            Serial.print("\t");
            Serial.println(gyro[2]);
        #endif
        
        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    }

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    return &q;
}


// return NULL if anything has gone wrong
float *readYpr() {
    Quaternion *quaternion = readQuaternion();
    if (quaternion == NULL) {
      return NULL;
    }
    
    mpu.dmpGetGravity(&gravity, quaternion);
    mpu.dmpGetYawPitchRoll(ypr, quaternion, &gravity);
    return ypr;
}

// ================================================================
// ===        SENSOR STABILIZATION                              ===
// ================================================================

long lastSampleTime = 0;
Quaternion lastQ;
bool readingHasStablized = false;
int continuousSamplesBelowThreshold = 0;

bool hasSensorStabilized() {
    if (readingHasStablized) {
        return true;
    }

    if (!isSensorDataReady()) {
        return false;
    }

    Quaternion *quaternion = readQuaternion();
    if (quaternion == NULL) {
        return false;
    }
    if ((millis() - lastSampleTime) < 100) {
        return false;  // <100ms since last sample, not ready for new sample to compare
    }

    lastSampleTime = millis();
    double delta = abs(quaternion->w-lastQ.w) + abs(quaternion->x-lastQ.x) + abs(quaternion->y-lastQ.y) + abs(quaternion->z-lastQ.z);
    lastQ = *quaternion;
    delta *= 1000;
    Serial.println(delta);
    
    if (delta > 1.5) {  // experimential. You may need to adjust it.
        continuousSamplesBelowThreshold = 0;
    } else {
        continuousSamplesBelowThreshold ++;
    }

    if (continuousSamplesBelowThreshold >= 10) { // sensor is considered stablized when delta < threshold for longer than 1s
        readingHasStablized = true;
    }
    return readingHasStablized;
}

