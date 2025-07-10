void setup() {
  // Set all digital pins as inputs with pull-down or outputs set to LOW
  for (int pin = 0; pin <= 13; pin++) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW); // Ensure output is LOW
  }

  // Optionally, do the same for analog pins A0-A5 if needed
  for (int pin = A0; pin <= A5; pin++) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  // Disable PWM by setting all relevant timers to normal mode (optional)
}

void loop() {
  // Do nothing to keep board idle and safe
}
