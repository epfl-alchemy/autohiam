#include <Servo.h>

Servo pump;  // Create Servo object to control the pump

#define PUMPPIN 10             // Connect pump control pin to Arduino digital pin 9
// #define CLOCKWISE_SPEED 0     // 0° for maximum clockwise speed
#define RUN_TIME_MS 180000   // 180 seconds in milliseconds (30 * 1000)

void setup() {
  pump.attach(PUMPPIN);       // Attach the pump control pin
  // pump.write(CLOCKWISE_SPEED); // Start clockwise at full speed
}

void loop() {
  pump.write(0);              // Clockwise maximum speed rotation
  // delay(RUN_TIME_MS);         // Run for 30 seconds
  // pump.write(90);             // Stop the pump (90°)
  // while (true) {
    // Do nothing, permanently stop the pump
  // }
}
