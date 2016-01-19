#include "Mpu6050.h"
#include "MegaMoto.h"

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

#define ACCEL_GAIN 18.0
#define GYRO_GAIN 0.2
#define CYCLE_TIME 10   //10ms per cycle of adjusting speed

float desiredSpeed = 0.0;

float overallGain = 0.3;

long lastCycle = 0;

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

    mpu6050Setup();
    megaMotoSetup();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

    while(!hasSensorStabilized()) {
      blink();
    }

    ledOn();
    float *ypr = readYpr();

    if (ypr == NULL) {
       shutoff();  // something is wrong, shut off motor
       return;
    }
    
    float angle = ypr[2]; // pitch is how much the board tilts
    int16_t *gyro = lastGyroReadking();
    int16_t angleRate = gyro[0];  // gyro_x is the rate of tilting

    // balance_torque is the output to motor just to keep it balanced
    float balanceTorque = (float) (ACCEL_GAIN * angle) + (GYRO_GAIN * angleRate);

    //this is not current speed. We do not know actual speed as we have no wheel rotation encoders. This is a type of accelerator pedal effect:
    //this variable increases with each loop of the program IF board is deliberately held at an angle (by rider for example)
    //So it means "if we are STILL tilted, speed up a bit" and it keeps accelerating as long as you hold it tilted.
    //You do NOT need this to just balance, but to go up a slight incline for example you would need it: if board hits incline and then stops - if you hold it
    //tilted for long eneough, it will eventually go up the slope (so long as motors powerfull enough and motor controller powerful enough)
    //Why the 0.999 value? I got this from the SeWii project code - thanks!
    //If you have built up a large cur_speed value and you tilt it back to come to a standstill, you will have to keep it tilted back even when you have come to rest
    //i.e. board will stop moving OK but will now not be level as you are tiliting it back other way to counteract this large cur_speed value
    //The 0.999 means that if you bring board level after a long period tilted forwards, the cur_speed value magically decays away to nothing and your board
    //is now not only stationary but also level!
  
    desiredSpeed = (float) (desiredSpeed + (angle * 6 * CYCLE_TIME)) * 0.999;

    float level = (float)(balanceTorque + desiredSpeed) * overallGain;

    Serial.println(level);
    driveMotor((int) level);

    long remainingCycle = millis() - lastCycle;
    if (remainingCycle > 0) {
      delay(remainingCycle);
    }
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

