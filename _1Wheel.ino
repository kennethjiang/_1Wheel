#include "Mpu6050.h"
#include "MegaMoto.h"

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

//#define SPEED_GAIN 0.006
#define SPEED_GAIN 0.000
#define ANGLE_GAIN 96.0
#define ANGLE_RATE_GAIN 15.0

#define ANGLE_OFFSET 0.03054  // Sensor is not perfectly level and needs offset. Caliberate your own and set accordingly

float desiredSpeed = 0.0;

float overallGain = 16.0;

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

    float angle = ypr[2] + ANGLE_OFFSET; // pitch is how much the board tilts, in radians
    float angleRate = float(readGyro()[0]) * M_PI / 180.0;  // gyro_x is the rate of tilting, in radians

    if (!shouldBeActivated(angle)) {
      Serial.println("NO!!!");
      desiredSpeed = 0.0;
      shutoff();
      return;
    }

    // balanceTorque is the output to motor just to keep it balanced
    float balanceTorque = ANGLE_GAIN * angle + ANGLE_RATE_GAIN * angleRate;

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

    if (lastCycle != 0) {
      desiredSpeed = (float) (desiredSpeed + (angle * SPEED_GAIN * (millis() - lastCycle))) * 0.999;
    }
    lastCycle = millis();
      
    float duty = (float)(balanceTorque + desiredSpeed) * overallGain;

    Serial.println(duty, 6);

    driveMotor((int) duty);

}


// ================================================================
// ===                    ACTIVATION                           ====
// ================================================================

#define ACTIVE_ANGLE 10 * M_PI / 180.0
#define ACTIVE_DUR 100
#define DEACTIVE_ANGLE 16 * M_PI / 180.0
#define DEACTIVE_DUR 1000

bool activated = false;
long lastChangedAt = 0;

// Motor should be activated only when angle is < ACTIVE_ANGLE for > ACTIVE_DUR ms,
//   or be deactivated when angle is > DEACTIVE_ANGLE for > DEACTIVE_DUR ms
bool shouldBeActivated(float angle) {
  
  if (!activated) {

    if (abs(angle) > ACTIVE_ANGLE)
      return false;

    if (lastChangedAt == 0)
      lastChangedAt = millis();
    else {
      if ((millis() - lastChangedAt) >= ACTIVE_DUR) {
        activated = true;
        lastChangedAt = 0;
      }
    }

  } else {

    if (abs(angle) < ACTIVE_ANGLE)
      return true;

    if (lastChangedAt == 0)
      lastChangedAt = millis();
    else {
      if ((millis() - lastChangedAt) > DEACTIVE_DUR) {
        activated = false;
        lastChangedAt = 0;
      }
    }
  }

  return activated;
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

