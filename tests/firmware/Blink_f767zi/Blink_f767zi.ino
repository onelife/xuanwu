// Blink for the ST Nucleo-F767ZI (STM32F767ZI, Cortex-M7), built with
//
//   STMicroelectronics:stm32:Nucleo_144:pnum=NUCLEO_F767ZI
//
// There is no chip description for the STM32F767 yet, so the test suite does not
// run this image.  Note that its flash lives at 0x08000000, the same range the
// STM32F411 firmware loads at, which is why `board.yaml` -- not the load address --
// decides which chip an image belongs to.
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
}
