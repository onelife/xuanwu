// Blink for the Arduino MKR Zero (ATSAMD21G18, Cortex-M0+), built with
//
//   arduino:samd:mkrzero
//
// There is no chip description for the SAMD21 yet, so the test suite does not run
// this image; it is here to be ready when `samd21.yaml` exists.
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
}
