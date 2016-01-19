#include <Arduino.h>

#include "MegaMoto.h"

int enablePin = 8;
int pWMPinA = 11;  // Timer2
int pWMPinB = 3;

// ================================================================
// ===        SETUP                                             ===
// ================================================================


void megaMotoSetup() {                
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards:
  pinMode(pWMPinA, OUTPUT);
  pinMode(pWMPinB, OUTPUT);
//  setPwmFrequency(pWMPinA, 8);  // change Timer2 divisor to 8 gives 3.9kHz PWM freq
}



// ================================================================
// ===        CAP MOTOR OUTPUT CHANGE                           ===
// ================================================================

#define MAX_DUTY_CHANGE_RATE 10.0  // Change of duty should not exceed 10 per second
int lastDuty = 0;
long lastDutyTime = 0;

int cappedDuty(int duty) {
  long elapsedTime = millis() - lastDutyTime;
  float maxAllowedChange = ((float) elapsedTime) / 1000.0 * MAX_DUTY_CHANGE_RATE;
  
  if (abs(duty-lastDuty) > maxAllowedChange) {
    int posOrNeg = (duty-lastDuty) > 0 ? 1.0 : -1.0;
    duty = lastDuty + (int) (maxAllowedChange + posOrNeg);
  }

  lastDuty = duty;
  lastDutyTime = millis();

  return duty;
}

// ================================================================
// ===        MOTOR DRIVER                                      ===
// ================================================================


// duty: 0-255, 0: stop; 255: maximum speed
void forward(int duty) {
  analogWrite(pWMPinB, 0);
  analogWrite(pWMPinA, duty);
}

// duty: 0-255, 0: stop; 255: maximum speed
void reverse(int duty) {
  analogWrite(pWMPinA, 0);
  analogWrite(pWMPinB, duty);
}

void driveMotor(int duty) {
  duty = cappedDuty(duty);

  if (duty >= 0) {
    forward(duty);
  } else {
    reverse(duty*-1);
  }
}

void shutoff() {
  analogWrite(pWMPinA, 0);
  analogWrite(pWMPinB, 0);
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
