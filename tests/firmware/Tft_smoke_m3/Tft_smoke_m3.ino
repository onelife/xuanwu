/* ===========================================================================
   A small TFT acceptance sketch for the simulated Arduino Due + Adafruit 2.8"
   TFT Touch Shield v2 (product 1651).

   It exists because `graphicstest` -- the example it is modelled on -- draws
   megabytes of pixels and takes minutes of host time, which is too much for a
   test that runs on every change.  This one uses the same library calls on a
   known, small set of shapes, prints what it did, and stops: everything it
   draws can be checked pixel by pixel from the host, and it finishes in a
   second or two.

   The full `graphicstest` run is still a test of its own, marked `milestone`.
   =========================================================================== */

#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

// The Adafruit shield: D10 is the panel's chip select, D9 its data/command pin.
#define TFT_CS 10
#define TFT_DC 9

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

void setup() {
  Serial.begin(9600);
  Serial.println("TFT smoke");

  tft.begin();

  // The register diagnostics a real panel answers.
  Serial.print("power mode ");
  Serial.println(tft.readcommand8(ILI9341_RDMODE), HEX);
  Serial.print("madctl ");
  Serial.println(tft.readcommand8(ILI9341_RDMADCTL), HEX);
  Serial.print("pixfmt ");
  Serial.println(tft.readcommand8(ILI9341_RDPIXFMT), HEX);

  tft.setRotation(0);
  Serial.print("size ");
  Serial.print(tft.width());
  Serial.print("x");
  Serial.println(tft.height());

  // Portrait shapes: one filled rectangle, one outline, one line, one pixel and
  // one run of text.  All coordinates are chosen so the host can check them.
  tft.fillScreen(ILI9341_BLACK);
  tft.fillRect(10, 20, 30, 40, ILI9341_RED);
  tft.drawRect(50, 60, 40, 30, ILI9341_GREEN);
  tft.drawFastHLine(0, 0, 240, ILI9341_BLUE);
  tft.drawPixel(200, 300, ILI9341_WHITE);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(4, 200);
  tft.print("xuanwu");

  // Landscape: the library swaps the axes, and a marker put at (5, 6) has to come
  // back at (5, 6) -- the rotation must not move what was drawn.
  tft.setRotation(1);
  Serial.print("size ");
  Serial.print(tft.width());
  Serial.print("x");
  Serial.println(tft.height());
  tft.fillRect(5, 6, 2, 2, ILI9341_CYAN);

  tft.setRotation(0);
  Serial.println("smoke done");
}

void loop() {
  // Nothing: the test stops at "smoke done".
  delay(1000);
}
