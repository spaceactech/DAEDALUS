// #include <Arduino.h>
// #include <Servo.h>
#include "UserPins.h"

constexpr int RA_SERVO_MIN = 500;
constexpr int RA_SERVO_MAX = 2450;
constexpr int SERVO_CEN = (RA_SERVO_MIN + RA_SERVO_MAX) / 2;

// Servo servo_a;
// String inputString = "";

// void setup() {
//   servo_a.attach(USER_GPIO_PWM3, SERVO_MIN, SERVO_MAX, SERVO_MAX);
//   Serial.begin();

//   delay(3000);
//   Serial.println("Hello!");
// }

// void loop() {
//     if (Serial.available()) {
//     inputString = Serial.readStringUntil('\n');   // read serial line
//     int angle = inputString.toInt();              // convert to integer

//     if (angle >= 0 && angle <= 180) {
//       servo_a.write(angle);
//       Serial.print("Servo moved to: ");
//       Serial.println(angle);
//     } else {
//       Serial.println("Invalid angle. Enter 0-180.");
//     }
//   }
// }

/* BEGIN INCLUDE SYSTEM LIBRARIES */
#include <Arduino.h>  // Arduino Framework
#include <STM32Servo.h>
/* END INCLUDE SYSTEM LIBRARIES */

STM32ServoList servos(TIM15);

/* END USER THREADS */
void tick() {
  digitalWrite(PC15, !digitalRead(PC15));
}

HardwareTimer *htim;

void setup() {
  // pinMode(PC15, OUTPUT);
  // digitalWrite(PC15, HIGH);

  // htim = new HardwareTimer(TIM15);
  // htim->setPrescaleFactor(htim->getTimerClkFreq() / 1'000'000ul);
  // htim->setOverflow(2, HERTZ_FORMAT);
  // htim->attachInterrupt(tick);
  // htim->resume();
  Serial.begin(115200);
  servos.attach(USER_GPIO_PWM3, RA_SERVO_MIN, RA_SERVO_MAX, RA_SERVO_MAX);
}

void loop() {
  servos[0].write(0);
  servos[0].write(180);
}
