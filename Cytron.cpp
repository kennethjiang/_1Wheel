#include <Arduino.h>

#include "Cytron.h"

// ================================================================
// ===        SETUP                                             ===
// ================================================================


void driverSetup() {
  pinMode(enablePullDownPin,OUTPUT);
  pinMode(enablePin,INPUT_PULLUP); 
  pinMode(pWMPin,OUTPUT);
  pinMode(dirPin,OUTPUT);
  
  digitalWrite(enablePullDownPin, LOW);
}



// ================================================================
// ===        CAP MOTOR OUTPUT CHANGE                           ===
// ================================================================

#define MAX_DUTY 100.0 // CAP max duty for testing

int capped(int duty) {
  if (duty > MAX_DUTY) {
    duty = MAX_DUTY;
    Serial.println("CAPPED duty");
  }

  if (duty < MAX_DUTY * -1) {
    duty = MAX_DUTY * -1;
    Serial.println("CAPPED duty");
  }
  
  return duty;
}

// ================================================================
// ===        MOTOR DRIVER                                      ===
// ================================================================


// duty: 0-255, 0: stop; 255: maximum speed

void drive(int duty) {
  if (digitalRead(enablePin) == HIGH) {
    disableMotor();
  }

  duty *= DIRECTION;
  duty = capped(duty);

  if (duty >= 0) {
    duty = duty * POS_GAIN;
  } else {
    duty = duty * NEG_GAIN;
  }

  duty *= 0.5;
  duty += 128;

  digitalWrite(pWMPin,HIGH);
  analogWrite(dirPin,duty);
}

void disableMotor() {
  digitalWrite(pWMPin,LOW);
  analogWrite(dirPin,128);
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
