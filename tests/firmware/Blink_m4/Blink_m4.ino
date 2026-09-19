// Blink for the generic STM32F411 board, built with
//
//   STMicroelectronics:stm32:GenF4:pnum=GENERIC_F411CEUX
//
// The generic variant defines no LED_BUILTIN, so PC13 is used -- the LED on the
// "Black Pill" F411 boards.  The simulator does not care which pin it is; what
// matters is that the firmware drives the STM32 GPIO block and SysTick.
#define LED_PIN PC13

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
}
