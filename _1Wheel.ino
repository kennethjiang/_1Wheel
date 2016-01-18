#include "Sensor.h"
  
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    mpu6050Setup();

}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

    while(!hasSensorStabilized()) {
      blink();
    }

    ledOn();
    readYpr();
}



// ================================================================
// ===                    LED                                   ===
// ================================================================


// LED
enum LED_STATUS {
  ON,
  OFF,
  BLINK,
} ledStatus;

bool blinkState = LOW;
long lastSwitchingTime = 0;

void blink() {
    ledStatus = BLINK;
    led();
}

void ledOn() {
    ledStatus = ON;
    led();
}

void led() {
    switch (ledStatus) {
        case BLINK:
            if ((millis() - lastSwitchingTime) > 500) {
                blinkState = !blinkState;
                digitalWrite(LED_PIN, blinkState);
                lastSwitchingTime = millis();
            }
          break;
        case ON:
            digitalWrite(LED_PIN, HIGH);
            break;
        case OFF:
            digitalWrite(LED_PIN, LOW);
            break;
    }
}

