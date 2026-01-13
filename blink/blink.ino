#include <Adafruit_NeoPixel.h>

#define PIN 48 // Or 38, depending on your S3 DevKit version!
#define NUMPIXELS 1

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500

void setup() {
  pixels.begin();
}

void loop() {
  pixels.clear();
  pixels.show();
  delay(DELAYVAL);

  for(int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 5, 0));
    pixels.show();
    delay(DELAYVAL);
  }
}



