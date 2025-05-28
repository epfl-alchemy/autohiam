#include <Servo.h>

Servo pump1;
Servo pump2;

#define PUMP1_PIN 9
#define PUMP2_PIN 11
#define RUN_TIME_MS 200000   // 200 seconds in milliseconds

void setup() {
  pump1.attach(PUMP1_PIN);
  pump2.attach(PUMP2_PIN);
}

void loop() {
  pump1.write(0);
  // pump2.write(0);
  // delay(RUN_TIME_MS); 
  // pump1.write(90); 
  // pump2.write(90);

  // while (true) {
    // Do nothing, permanently stop the pump
  // }
}
