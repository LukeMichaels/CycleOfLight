#include <Arduino.h>
#include <FastLED.h>
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
#include "lib8tion.h"
#include "noise.h"
const int hallSensorPin = 32;
unsigned long lastTime = 0;        // the last time the sensor was triggered
unsigned long debounceDelay = 250; // debounce delay in milliseconds
int rpm = 0;                       // revolutions per minute
int hue = 0;
#define NUM_STRIPS 7
#define NUM_LEDS_PER_STRIP 165
#define NUM_LEDS_LAST_STRIP 193
#define NUM_LEDS_TOTAL (NUM_STRIPS * NUM_LEDS_PER_STRIP + NUM_LEDS_LAST_STRIP)
#define NUM_ROWS 43
#define NUM_COLS 28
#define BRIGHTNESS 127 // 127 for 50%, 204 for 80%
#define LAST_VISIBLE_LED 1182
CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];
CRGB lastStrip[NUM_LEDS_LAST_STRIP];
const uint8_t kMatrixWidth = 28;
const uint8_t kMatrixHeight = 43;
#define CENTER_X (kMatrixWidth / 2)
#define CENTER_Y (kMatrixHeight / 2)
#define MAX_DISTANCE sqrt(sq(CENTER_X) + sq(CENTER_Y))
#define HUE_MAX 255
#define SATURATION 255
#define BRIGHTNESS 127
float distances[NUM_LEDS_TOTAL];
uint8_t ballX, ballY;
uint16_t XY(uint16_t x, uint16_t y)
{
  // any out of bounds address maps to the first hidden pixel
  if ((x >= kMatrixWidth) || (y >= kMatrixHeight))
  {
    return (LAST_VISIBLE_LED + 1);
  }

  const uint16_t XYTable[] = {};

  uint16_t i = (y * kMatrixWidth) + x;
  uint16_t j = XYTable[i];
  return j;
}

void setup()
{
  pinMode(hallSensorPin, INPUT);
  Serial.begin(115200);
  FastLED.addLeds<NEOPIXEL, 0>(leds[0], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<NEOPIXEL, 1>(leds[1], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<NEOPIXEL, 2>(leds[2], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<NEOPIXEL, 3>(leds[3], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<NEOPIXEL, 4>(leds[4], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<NEOPIXEL, 5>(leds[5], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<NEOPIXEL, 12>(lastStrip, NUM_LEDS_LAST_STRIP); // code for last strip with 28 extra LEDs
  FastLED.setBrightness(127);                                    // 127 for 50%, 204 for 80%                                 // 128 for 50%, 204 for 80%
  FastLED.setDither(0);
  // Calculate LED distances
  for (uint16_t i = 0; i < NUM_LEDS_TOTAL; i++)
  {
    uint8_t strip, led;
    float x, y;

    if (i < NUM_STRIPS * NUM_LEDS_PER_STRIP)
    {
      strip = i / NUM_LEDS_PER_STRIP;
      led = i % NUM_LEDS_PER_STRIP;
      x = led % kMatrixWidth;
      y = i / kMatrixWidth;
    }
    else
    {
      strip = NUM_STRIPS;
      led = i - NUM_STRIPS * NUM_LEDS_PER_STRIP;
      x = led % kMatrixWidth;
      y = (i - NUM_LEDS_PER_STRIP) / (float)kMatrixWidth;
    }

    distances[i] = fastSqrt(sq(x - ballX) + sq(y - ballY));
  }
}

void loop()
{
  int sensorValue = digitalRead(hallSensorPin);

  // If the sensor is triggered (magnetic field detected) and enough time has passed since the last trigger (debounce)
  if (sensorValue == LOW && millis() - lastTime > debounceDelay)
  {
    // Calculate the revolutions per minute
    rpm = 60000 / (millis() - lastTime);
    lastTime = millis();
    Serial.print("Cadence: ");
    Serial.print(rpm);
    Serial.println(" RPM");
  }

  // available patterns
  // fireFlies();
  // ballWaves();
  // tunnelIn();
  // tunnelOut(); // use
  // plasma();
  // psychWaves(); // use
  // psychWavesReversed(); // kind of bugged, don't use for now
  // psychWaterfall();
  // psychWaterfallReverse(); // use this one
  // brainWaves();
  // pyramids();
  // rorschach();
  // vaporPyramid();
  // psychSpin();
  // pinWheel();
  // trippyClouds();
  // galaxy();
  if (rpm >= 0 && rpm < 10)
  {
    clearDisplay();
    rorschach();
    //   Serial.println("Cadence 0 - 20");
  }
  else if (rpm >= 11 && rpm < 20)
  {
    clearDisplay();
    pyramids();
    //  Serial.println("Cadence 11 - 20");
  }
  else if (rpm >= 21 && rpm < 30)
  {
    clearDisplay();
    tunnelOut();
    // Serial.println("Cadence 21 - 30");
  }
  else if (rpm >= 31 && rpm < 40)
  {
    clearDisplay();
    psychWaterfallReverse();
    // Serial.println("Cadence 31 - 40");
  }

  else if (rpm >= 41 && rpm < 50)
  {
    clearDisplay();
    psychWaves();
    //  Serial.println("Cadence 41 - 50");
  }
  else if (rpm >= 51 && rpm < 60)
  {
    clearDisplay();
    vaporPyramid();
    //  Serial.println("Cadence 51 - 60");
  }
  else if (rpm >= 61 && rpm < 70)
  {
    clearDisplay();
    fireFlies();
    // Serial.println("Cadence 61 - 70");
  }
  else if (rpm >= 71 && rpm < 80)
  {
    clearDisplay();
    psychSpin();
    // Serial.println("Cadence 71 - 80");
  }
  else if (rpm >= 81 && rpm < 90)
  {
    clearDisplay();
    ballWaves();
    // Serial.println("Cadence 81 - 90");
  }
  else if (rpm >= 91 && rpm < 100)
  {
    clearDisplay();
    galaxy();
    // Serial.println("Cadence 91 - 100");
  }
  else if (rpm >= 101 && rpm < 110)
  {
    clearDisplay();
    trippyClouds();
    // Serial.println("Cadence 101 - 110");
  }
  else if (rpm >= 111 && rpm < 120)
  {
    clearDisplay();
    brainWaves();
    // Serial.println("Cadence 111 - 120");
  }
  else if (rpm >= 121)
  {
    clearDisplay();
    plasma();
    // Serial.println("Cadence greater than 121");
  }
}

void clearDisplay()
{
  // Clear the leds array
  for (int i = 0; i < NUM_STRIPS; i++)
  {
    fill_solid(leds[i], NUM_LEDS_PER_STRIP, CRGB::Black);
  }
  // Clear the lastStrip array
  fill_solid(lastStrip, NUM_LEDS_LAST_STRIP, CRGB::Black);
}

void plasma()
{
  static uint8_t start = 0;
  start += 1;

  for (int strip = 0; strip < NUM_STRIPS; strip++)
  {
    for (int i = 0; i < NUM_LEDS_PER_STRIP; i++)
    {
      uint8_t index = (sin8(i + start) + sin8(i * 16 + start)) / 2; // Average of two sine waves
      CHSV hsv = CHSV(index, 255, BRIGHTNESS);
      leds[strip][i] = hsv;
    }
  }

  for (int i = 0; i < NUM_LEDS_LAST_STRIP; i++)
  {
    uint8_t index = (sin8(i + start) + sin8(i * 16 + start)) / 2; // Average of two sine waves
    CHSV hsv = CHSV(index, 255, BRIGHTNESS);
    lastStrip[i] = hsv;
  }

  FastLED.show();
}

void psychWaves()
{
  static uint8_t hueShift = 0;

  for (uint16_t i = 0; i < NUM_LEDS_TOTAL; i++)
  {
    uint8_t strip, led;
    float x, y;

    if (i < NUM_STRIPS * NUM_LEDS_PER_STRIP)
    {
      strip = i / NUM_LEDS_PER_STRIP;
      led = i % NUM_LEDS_PER_STRIP;
      x = i % kMatrixWidth;
      y = i / kMatrixWidth;
    }
    else
    {
      strip = NUM_STRIPS;
      led = i - NUM_STRIPS * NUM_LEDS_PER_STRIP;
      x = led % kMatrixWidth;
      y = led / kMatrixWidth + NUM_STRIPS;
    }

    float distance = sqrt(sq(x - CENTER_X) + sq(y - CENTER_Y));
    uint8_t hue = map(distance, 0, MAX_DISTANCE, hueShift, hueShift + HUE_MAX);

    if (strip < NUM_STRIPS)
    {
      leds[strip][led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
    else
    {
      lastStrip[led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
  }

  FastLED.show();

  hueShift += 1;
}

void psychWavesReversed()
{
  static uint8_t hueShift = 0;

  for (uint16_t i = 0; i < NUM_LEDS_TOTAL; i++)
  {
    uint8_t strip, led;
    float x, y;

    if (i < NUM_STRIPS * NUM_LEDS_PER_STRIP)
    {
      strip = i / NUM_LEDS_PER_STRIP;
      led = i % NUM_LEDS_PER_STRIP;
    }
    else
    {
      strip = NUM_STRIPS;
      led = i - NUM_STRIPS * NUM_LEDS_PER_STRIP;
    }

    x = i % kMatrixWidth;
    y = i / kMatrixWidth;

    float distance = sqrt(sq(x - CENTER_X) + sq(y - CENTER_Y));
    uint8_t hue = map(distance, 0, MAX_DISTANCE, hueShift + HUE_MAX, hueShift);

    if (strip < NUM_STRIPS)
    {
      leds[strip][led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
    else
    {
      lastStrip[led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
  }

  FastLED.show();

  hueShift += 1;
}

void psychWaterfall()
{
  static uint8_t hueShift = 0;

  for (uint16_t i = 0; i < NUM_LEDS_TOTAL; i++)
  {
    uint8_t strip, led;
    float x, y;

    if (i < NUM_STRIPS * NUM_LEDS_PER_STRIP)
    {
      strip = i / NUM_LEDS_PER_STRIP;
      led = i % NUM_LEDS_PER_STRIP;
      x = led % kMatrixWidth;
      y = strip;
    }
    else
    {
      strip = NUM_STRIPS;
      led = i - NUM_STRIPS * NUM_LEDS_PER_STRIP;
      x = led % NUM_COLS; // Use NUM_COLS for the last strip
      y = NUM_STRIPS;     // Use NUM_STRIPS for the last strip
    }

    float distance = sqrt(sq(x - CENTER_X) + sq(y - CENTER_Y));
    uint8_t hue = map(distance, 0, MAX_DISTANCE, hueShift + HUE_MAX, hueShift);

    if (strip < NUM_STRIPS)
    {
      leds[strip][led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
    else
    {
      lastStrip[led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
  }

  FastLED.show();

  hueShift += 1;
}

void psychWaterfallReverse()
{
  static uint8_t hueShift = 0;

  for (uint16_t i = 0; i < NUM_LEDS_TOTAL; i++)
  {
    uint8_t strip, led;
    float x, y;

    if (i < NUM_STRIPS * NUM_LEDS_PER_STRIP)
    {
      strip = i / NUM_LEDS_PER_STRIP;
      led = i % NUM_LEDS_PER_STRIP;
      x = led % kMatrixWidth;
      y = NUM_STRIPS - 1 - strip; // Reverse the y-coordinate
    }
    else
    {
      strip = NUM_STRIPS;
      led = i - NUM_STRIPS * NUM_LEDS_PER_STRIP;
      x = led % NUM_COLS; // Use NUM_COLS for the last strip
      y = 0;              // Use 0 for the last strip
    }

    float distance = sqrt(sq(x - CENTER_X) + sq(y - CENTER_Y));
    uint8_t hue = map(distance, 0, MAX_DISTANCE, hueShift + HUE_MAX, hueShift);

    if (strip < NUM_STRIPS)
    {
      leds[strip][led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
    else
    {
      lastStrip[led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
  }

  FastLED.show();

  hueShift += 1;
}

void tunnelIn()
{
  static uint8_t hueShift = 0;

  for (uint16_t i = 0; i < NUM_LEDS_TOTAL; i++)
  {
    uint8_t strip, led;
    float x, y;

    if (i < NUM_STRIPS * NUM_LEDS_PER_STRIP)
    {
      strip = i / NUM_LEDS_PER_STRIP;
      led = i % NUM_LEDS_PER_STRIP;
      x = led % kMatrixWidth;
      y = i / kMatrixWidth; // Use the LED's position within the entire matrix
    }
    else
    {
      strip = NUM_STRIPS;
      led = i - NUM_STRIPS * NUM_LEDS_PER_STRIP;
      x = led % kMatrixWidth;                             // Use kMatrixWidth for the last strip
      y = (i - NUM_LEDS_PER_STRIP) / (float)kMatrixWidth; // Subtract the number of LEDs in a strip from the total number of LEDs
    }

    float distance = sqrt(sq(x - CENTER_X) + sq(y - CENTER_Y));
    uint8_t hue = map(distance, 0, MAX_DISTANCE, hueShift, hueShift + HUE_MAX);

    if (strip < NUM_STRIPS)
    {
      leds[strip][led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
    else
    {
      lastStrip[led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
  }

  FastLED.show();

  hueShift += 1;
}

void tunnelOut()
{
  static uint8_t hueShift = 0;

  for (uint16_t i = 0; i < NUM_LEDS_TOTAL; i++)
  {
    uint8_t strip, led;
    float x, y;

    if (i < NUM_STRIPS * NUM_LEDS_PER_STRIP)
    {
      strip = i / NUM_LEDS_PER_STRIP;
      led = i % NUM_LEDS_PER_STRIP;
      x = led % kMatrixWidth;
      y = i / kMatrixWidth; // Use the LED's position within the entire matrix
    }
    else
    {
      strip = NUM_STRIPS;
      led = i - NUM_STRIPS * NUM_LEDS_PER_STRIP;
      x = led % kMatrixWidth;                             // Use kMatrixWidth for the last strip
      y = (i - NUM_LEDS_PER_STRIP) / (float)kMatrixWidth; // Subtract the number of LEDs in a strip from the total number of LEDs
    }

    float distance = sqrt(sq(x - CENTER_X) + sq(y - CENTER_Y));
    uint8_t hue = map(distance, 0, MAX_DISTANCE, hueShift + HUE_MAX, hueShift); // Reverse the mapping of the hue

    if (strip < NUM_STRIPS)
    {
      leds[strip][led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
    else
    {
      lastStrip[led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
  }

  FastLED.show();

  hueShift += 1;
}

float fastSqrt(float x)
{
  union
  {
    int i;
    float x;
  } u;
  u.x = x;
  u.i = (1 << 29) + (u.i >> 1) - (1 << 22);

  u.x = u.x + x / u.x;
  u.x = 0.25f * u.x + x / u.x;

  return u.x;
}

void ballWaves()
{
  static uint8_t hueShift = 0;
  static CRGB previousLeds[NUM_LEDS_TOTAL];

  // Update the ball position
  ballX = beatsin8(5, 0, kMatrixWidth - 1);
  ballY = beatsin8(7, 0, kMatrixHeight - 1);

  for (uint16_t i = 0; i < NUM_LEDS_TOTAL; i++)
  {
    uint8_t strip, led;
    float x, y;

    if (i < NUM_STRIPS * NUM_LEDS_PER_STRIP)
    {
      strip = i / NUM_LEDS_PER_STRIP;
      led = i % NUM_LEDS_PER_STRIP;
      x = led % kMatrixWidth;
      y = i / kMatrixWidth;
    }
    else
    {
      strip = NUM_STRIPS;
      led = i - NUM_STRIPS * NUM_LEDS_PER_STRIP;
      x = led % kMatrixWidth;
      y = (i - NUM_LEDS_PER_STRIP) / (float)kMatrixWidth;
    }

    // Use the precalculated distance
    float distance = distances[i];

    // Calculate the hue based on the distance
    uint8_t hue = sin8(hueShift + distance * 8);

    // Create the new color
    CHSV newColor = CHSV(hue, SATURATION, BRIGHTNESS);

    // Convert the new color to CRGB
    CRGB newColorRgb;
    hsv2rgb_spectrum(newColor, newColorRgb);

    // Interpolate between the previous color and the new color
    CRGB interpolatedColor = blend(previousLeds[i], newColorRgb, 128);

    if (strip < NUM_STRIPS)
    {
      leds[strip][led] = interpolatedColor;
    }
    else
    {
      lastStrip[led] = interpolatedColor;
    }

    // Store the new color for the next frame
    previousLeds[i] = newColorRgb;
  }

  FastLED.show();

  hueShift += 1;
}

#define NUM_FLIES 1
#define FADE_RATE 1
void fireFlies()
{
  static uint8_t hueShift = 0;
#define NUM_LEDS_TOTAL (NUM_STRIPS * NUM_LEDS_PER_STRIP + NUM_LEDS_LAST_STRIP)
  static uint8_t dropIntensity[NUM_LEDS_TOTAL] = {0};

  // Set the background to nautical blue
  CRGB nauticalBlue;
  hsv2rgb_spectrum(CHSV(170, 255, 255), nauticalBlue); // Convert CHSV to CRGB
  for (uint8_t strip = 0; strip < NUM_STRIPS; strip++)
  {
    fill_solid(leds[strip], NUM_LEDS_PER_STRIP, nauticalBlue);
  }
  fill_solid(lastStrip, NUM_LEDS_LAST_STRIP, nauticalBlue);

  // Randomly select LEDs to light up in yellow
  for (uint8_t i = 0; i < NUM_FLIES; i++)
  {
    uint16_t led = random16(NUM_LEDS_TOTAL);
    dropIntensity[led] = 255; // Set the intensity of the drop to the maximum
  }

  // Update the LEDs
  for (uint16_t i = 0; i < NUM_LEDS_TOTAL; i++)
  {
    if (dropIntensity[i] > 0)
    {
      // If this LED is a drop, set its color to yellow and decrease its intensity
      uint8_t strip;
      uint8_t led;
      if (i < NUM_STRIPS * NUM_LEDS_PER_STRIP)
      {
        strip = i / NUM_LEDS_PER_STRIP;
        led = i % NUM_LEDS_PER_STRIP;
      }
      else
      {
        strip = NUM_STRIPS;
        led = i - NUM_STRIPS * NUM_LEDS_PER_STRIP;
      }

      if (strip < NUM_STRIPS)
      {
        leds[strip][led] = CHSV(42, 255, dropIntensity[i]); // 42 is the hue for yellow in the HSV color space
      }
      else
      {
        if (led < NUM_LEDS_LAST_STRIP)
        {
          lastStrip[led] = CHSV(42, 255, dropIntensity[i]);
        }
      }

      dropIntensity[i] -= FADE_RATE;
    }
  }

  FastLED.show();

  hueShift += 1;
}

#define BRAIN_WAVES_NOISE_SCALE 80000000.0 // Adjusts the scale of the noise. Higher values create larger patterns.
#define BRAIN_WAVES_SPEED 1                // Adjusts the speed of the animation. Lower values create slower animations.
void brainWaves()
{
  static uint16_t t = 0;

  for (uint8_t strip = 0; strip < NUM_STRIPS; strip++)
  {
    uint16_t numLedsInStrip = (strip == NUM_STRIPS - 1) ? NUM_LEDS_LAST_STRIP : NUM_LEDS_PER_STRIP;

    for (uint16_t led = 0; led < numLedsInStrip; led++)
    {
      uint16_t i = strip * NUM_LEDS_PER_STRIP + led;

      // Calculate the x and y coordinates for the current LED
      uint16_t x = i % kMatrixWidth;
      uint16_t y = i / kMatrixWidth;

      // If we're on the last strip, adjust the y coordinate to account for the extra LEDs
      if (strip == NUM_STRIPS - 1)
      {
        x = i % kMatrixWidth; // Use 'i' instead of 'led'
        y = i / kMatrixWidth; // Use 'i' instead of 'NUM_ROWS - 1'
      }

      // Calculate the noise value based on Perlin noise
      uint8_t noise = inoise8(x * BRAIN_WAVES_NOISE_SCALE, y * BRAIN_WAVES_NOISE_SCALE, t);

      // Calculate the hue based on the noise value and add a unique offset for each LED
      uint8_t hue = noise + i;

      // Set the brightness to a constant value
      uint8_t brightness = 255;

      // Set the color of the current LED
      if (strip < NUM_STRIPS - 1)
      {
        leds[strip][led] = CHSV(hue, SATURATION, brightness);
      }
      else
      {
        lastStrip[led] = CHSV(hue, SATURATION, brightness);
      }
    }
  }

  // Show the LEDs
  FastLED.show();

  // Increment the time for the next frame
  t += BRAIN_WAVES_SPEED;
}

void pyramids()
{
  static uint16_t t = 0;

  for (uint8_t strip = 0; strip < NUM_STRIPS; strip++)
  {
    uint16_t numLedsInStrip = (strip == NUM_STRIPS - 1) ? NUM_LEDS_LAST_STRIP : NUM_LEDS_PER_STRIP;

    for (uint16_t led = 0; led < numLedsInStrip; led++)
    {
      uint16_t i = strip * NUM_LEDS_PER_STRIP + led;

      // Calculate the x and y coordinates for the current LED
      uint16_t x = i % kMatrixWidth;
      uint16_t y = i / kMatrixWidth;

      // If we're on the last strip, adjust the y coordinate to account for the extra LEDs
      if (strip == NUM_STRIPS - 1)
      {
        x = i % kMatrixWidth; // Use 'i' instead of 'led'
        y = i / kMatrixWidth; // Use 'i' instead of 'NUM_ROWS - 1'
      }

      // Calculate the noise value
      uint8_t noise = inoise8(x * 10, y * 10, t);

      // Map the noise value to a hue
      uint8_t hue = map(noise, 0, 255, 0, HUE_MAX);

      // Set the color of the current LED
      if (strip < NUM_STRIPS - 1)
      {
        leds[strip][led] = CHSV(hue, SATURATION, BRIGHTNESS);
      }
      else
      {
        lastStrip[led] = CHSV(hue, SATURATION, BRIGHTNESS);
      }
    }
  }

  // Show the LEDs
  FastLED.show();

  // Increment the time for the next frame
  t += 1;
}

void rorschach()
{
  static uint16_t t = 0;

  for (uint8_t strip = 0; strip < NUM_STRIPS; strip++)
  {
    uint16_t numLedsInStrip = (strip == NUM_STRIPS - 1) ? NUM_LEDS_LAST_STRIP : NUM_LEDS_PER_STRIP;

    for (uint16_t led = 0; led < numLedsInStrip; led++)
    {
      uint16_t i = strip * NUM_LEDS_PER_STRIP + led;

      // Calculate the x and y coordinates for the current LED
      uint16_t x = i % kMatrixWidth;
      uint16_t y = i / kMatrixWidth;

      // If we're on the last strip, adjust the y coordinate to account for the extra LEDs
      if (strip == NUM_STRIPS - 1)
      {
        x = i % kMatrixWidth; // Use 'i' instead of 'led'
        y = i / kMatrixWidth; // Use 'i' instead of 'NUM_ROWS - 1'
      }

      // Calculate the noise value with a larger scale factor
      uint8_t noise = inoise8(x * 30, y * 30, t);

      // Map the noise value to a hue
      uint8_t hue = map(noise, 0, 255, 0, HUE_MAX);

      // Set the color of the current LED
      if (strip < NUM_STRIPS - 1)
      {
        leds[strip][led] = CHSV(hue, SATURATION, BRIGHTNESS);
      }
      else
      {
        lastStrip[led] = CHSV(hue, SATURATION, BRIGHTNESS);
      }
    }
  }

  // Show the LEDs
  FastLED.show();

  // Increment the time for the next frame
  t += 1;
}

void vaporPyramid()
{
  static uint8_t hueShift = 0;

  for (uint16_t i = 0; i < NUM_LEDS_TOTAL; i++)
  {
    uint8_t strip = i / NUM_LEDS_PER_STRIP;
    uint8_t led = i % NUM_LEDS_PER_STRIP;

    // Calculate the x and y coordinates for the current LED
    uint16_t x = i % kMatrixWidth;
    uint16_t y = i / kMatrixWidth;

    // If we're on the last strip, adjust the y coordinate to account for the extra LEDs
    if (strip == NUM_STRIPS - 1)
    {
      x = i % kMatrixWidth; // Use 'i' instead of 'led'
      y = i / kMatrixWidth; // Use 'i' instead of 'NUM_ROWS - 1'
    }

    // Calculate the angle from the center
    float angle = atan2(y - CENTER_Y, x - CENTER_X);

    // Map the angle to a hue
    uint8_t hue = map8(angle * 255 / (2 * PI), 0, HUE_MAX) + hueShift;

    // Set the color of the current LED
    if (strip < NUM_STRIPS)
    {
      leds[strip][led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
    else
    {
      lastStrip[led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
  }

  // Show the LEDs
  FastLED.show();

  // Increment the hue shift for the next frame
  hueShift += 1;
}

void psychSpin()
{
  static uint8_t hueShift = 0;

  for (uint16_t i = 0; i < NUM_LEDS_TOTAL; i++)
  {
    uint8_t strip, led;
    float x, y;

    if (i < NUM_STRIPS * NUM_LEDS_PER_STRIP)
    {
      strip = i / NUM_LEDS_PER_STRIP;
      led = i % NUM_LEDS_PER_STRIP;
      x = led % kMatrixWidth;
      y = i / kMatrixWidth; // Use the LED's position within the entire matrix
    }
    else
    {
      strip = NUM_STRIPS;
      led = i - NUM_STRIPS * NUM_LEDS_PER_STRIP;
      x = led % kMatrixWidth;                             // Use kMatrixWidth for the last strip
      y = (i - NUM_LEDS_PER_STRIP) / (float)kMatrixWidth; // Subtract the number of LEDs in a strip from the total number of LEDs
    }

    // Calculate the angle of the LED relative to the center
    float angle = atan2(y - CENTER_Y, x - CENTER_X);

    // Adjust the angle to be in the range 0 to 2π
    if (angle < 0)
    {
      angle += 2 * PI;
    }

    // Map the angle to a hue
    uint8_t hue = map(angle, 0, 2 * PI, hueShift, hueShift + HUE_MAX);

    if (strip < NUM_STRIPS)
    {
      leds[strip][led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
    else
    {
      lastStrip[led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
  }

  FastLED.show();

  hueShift += 1;
}

void pinWheel()
{
  static uint8_t hueShift = 0;

  for (uint16_t i = 0; i < NUM_LEDS_TOTAL; i++)
  {
    uint8_t strip, led;
    float x, y;

    if (i < NUM_STRIPS * NUM_LEDS_PER_STRIP)
    {
      strip = i / NUM_LEDS_PER_STRIP;
      led = i % NUM_LEDS_PER_STRIP;
      x = led % kMatrixWidth;
      y = i / kMatrixWidth; // Use the LED's position within the entire matrix
    }
    else
    {
      strip = NUM_STRIPS;
      led = i - NUM_STRIPS * NUM_LEDS_PER_STRIP;
      x = led % kMatrixWidth;                             // Use kMatrixWidth for the last strip
      y = (i - NUM_LEDS_PER_STRIP) / (float)kMatrixWidth; // Subtract the number of LEDs in a strip from the total number of LEDs
    }

    // Calculate the angle of the LED relative to the center
    float angle = atan2(y - CENTER_Y, x - CENTER_X);

    // Adjust the angle to be in the range 0 to 2π
    if (angle < 0)
    {
      angle += 2 * PI;
    }

    // Map the angle to a hue, adding the hueShift to create a rotating effect
    uint8_t hue = map(angle, 0, 2 * PI, 0, 255) + hueShift;

    // Assign the color to the LED
    if (strip < NUM_STRIPS)
    {
      leds[strip][led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
    else
    {
      lastStrip[led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
  }

  FastLED.show();

  // Increment the hueShift every 20 milliseconds to create a smoother spinning effect
  EVERY_N_MILLISECONDS(20)
  {
    hueShift++;
  }
}

void trippyClouds()
{
  static uint8_t hueShift = 0;

  for (uint16_t i = 0; i < NUM_LEDS_TOTAL; i++)
  {
    uint8_t strip, led;
    float x, y;

    if (i < NUM_STRIPS * NUM_LEDS_PER_STRIP)
    {
      strip = i / NUM_LEDS_PER_STRIP;
      led = i % NUM_LEDS_PER_STRIP;
      x = led % kMatrixWidth;
      y = i / kMatrixWidth; // Use the LED's position within the entire matrix
    }
    else
    {
      strip = NUM_STRIPS;
      led = i - NUM_STRIPS * NUM_LEDS_PER_STRIP;
      x = led % kMatrixWidth;                             // Use kMatrixWidth for the last strip
      y = (i - NUM_LEDS_PER_STRIP) / (float)kMatrixWidth; // Subtract the number of LEDs in a strip from the total number of LEDs
    }

    // Calculate the angle of the LED relative to the center
    float angle = atan2(y - CENTER_Y, x - CENTER_X);

    // Adjust the angle to be in the range 0 to 2π
    if (angle < 0)
    {
      angle += 2 * PI;
    }

    // Calculate the distance of the LED from the center
    float distance = sqrt(sq(x - CENTER_X) + sq(y - CENTER_Y));

    // Adjust the phase of the hue based on the distance
    uint8_t hue = (angle + sin(distance) * 2 * PI) * 255 / (2 * PI) + hueShift;

    if (strip < NUM_STRIPS)
    {
      leds[strip][led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
    else
    {
      lastStrip[led] = CHSV(hue, SATURATION, BRIGHTNESS);
    }
  }

  FastLED.show();

  // Increment the hueShift to create the rotating effect
  hueShift++;
}

static const uint8_t exp_gamma[256] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3,
    4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 7, 7,
    7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 12, 12,
    12, 13, 13, 14, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19,
    19, 20, 20, 21, 21, 22, 23, 23, 24, 24, 25, 26, 26, 27, 28,
    28, 29, 30, 30, 31, 32, 32, 33, 34, 35, 35, 36, 37, 38, 39,
    39, 40, 41, 42, 43, 44, 44, 45, 46, 47, 48, 49, 50, 51, 52,
    53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67,
    68, 70, 71, 72, 73, 74, 75, 77, 78, 79, 80, 82, 83, 84, 85,
    87, 89, 91, 92, 93, 95, 96, 98, 99, 100, 101, 102, 105, 106, 108,
    109, 111, 112, 114, 115, 117, 118, 120, 121, 123, 125, 126, 128, 130, 131,
    133, 135, 136, 138, 140, 142, 143, 145, 147, 149, 151, 152, 154, 156, 158,
    160, 162, 164, 165, 167, 169, 171, 173, 175, 177, 179, 181, 183, 185, 187,
    190, 192, 194, 196, 198, 200, 202, 204, 207, 209, 211, 213, 216, 218, 220,
    222, 225, 227, 229, 232, 234, 236, 239, 241, 244, 246, 249, 251, 253, 254,
    255};
void galaxy()
{
  int a = millis() / 32;
  for (uint16_t i = 0; i < NUM_LEDS_TOTAL; i++)
  {
    uint8_t strip, led;
    float x, y;

    if (i < NUM_STRIPS * NUM_LEDS_PER_STRIP)
    {
      strip = i / NUM_LEDS_PER_STRIP;
      led = i % NUM_LEDS_PER_STRIP;
      x = led % kMatrixWidth;
      y = i / kMatrixWidth; // Use the LED's position within the entire matrix
    }
    else
    {
      strip = NUM_STRIPS;
      led = i - NUM_STRIPS * NUM_LEDS_PER_STRIP;
      x = led % kMatrixWidth;                             // Use kMatrixWidth for the last strip
      y = (i - NUM_LEDS_PER_STRIP) / (float)kMatrixWidth; // Subtract the number of LEDs in a strip from the total number of LEDs
    }

    if (strip < NUM_STRIPS)
    {
      leds[strip][led].b = exp_gamma[sin8((x - 8) * cos8((y + 20) * 4) / 4 + a)];
      leds[strip][led].g = exp_gamma[(sin8(x * 16 + a / 3) + cos8(y * 8 + a / 2)) / 2];
      leds[strip][led].r = exp_gamma[sin8(cos8(x * 8 + a / 3) + sin8(y * 8 + a / 4) + a)];
    }
    else
    {
      lastStrip[led].b = exp_gamma[sin8((x - 8) * cos8((y + 20) * 4) / 4 + a)];
      lastStrip[led].g = exp_gamma[(sin8(x * 16 + a / 3) + cos8(y * 8 + a / 2)) / 2];
      lastStrip[led].r = exp_gamma[sin8(cos8(x * 8 + a / 3) + sin8(y * 8 + a / 4) + a)];
    }
  }

  FastLED.show();
}