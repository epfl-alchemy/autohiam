#include <Servo.h>

Servo pump;

#define PUMPPIN 11  
#define RUN_TIME_MS 30000

void setup() {
  pump.attach(PUMPPIN);       // Attach the pump control pin
}

void loop() {
  pump.write(0); 
  // delay(RUN_TIME_MS);
  // pump.write(90);             // Stop the pump (90°)
  // while (true) {
  //   // Do nothing, permanently stop the pump
  // }
}
