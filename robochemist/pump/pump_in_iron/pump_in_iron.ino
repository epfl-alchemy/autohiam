// #include <Servo.h>

// Servo pump;

// #define PUMPPIN 13  // Use 9 or another pin if 13 causes issues
// #define RUN_TIME_MS 80000
// #define LOG_INTERVAL_MS 1000

// void setup() {
//   delay(2000);
//   Serial.begin(9600);
//   while (!Serial);  // Wait for serial connection (especially useful for Leonardo/Micro)
  
//   pump.attach(PUMPPIN);
//   pump.write(0);  // Start the pump
//   Serial.println("🚰 Pump started.");

//   unsigned long startTime = millis();
//   while (millis() - startTime < RUN_TIME_MS) {
//     int secondsLeft = (RUN_TIME_MS - (millis() - startTime)) / 1000;
//     Serial.print("⏳ Seconds left: ");
//     Serial.println(secondsLeft);
//     delay(LOG_INTERVAL_MS);
//   }

//   pump.write(90);  // Stop the pump
//   Serial.println("✅ Pump stopped.");
// }

// void loop() {
//   // Freeze execution to keep the pump stopped
//   while (true) {}
// }

#include <Servo.h>

Servo pump;

#define PUMPPIN 13  
#define RUN_TIME_MS 80000

void setup() {
  pump.attach(PUMPPIN);       // Attach the pump control pin
}

void loop() {
  pump.write(0); 
  delay(RUN_TIME_MS);
  pump.write(90);             // Stop the pump (90°)
  while (true) {
    // Do nothing, permanently stop the pump
  }
}
