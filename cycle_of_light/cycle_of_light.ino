#include <Arduino.h>
#include <FastLED.h>
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
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
#define BRIGHTNESS 255
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
  if (rpm >= 0 && rpm < 20)
  {
    clearDisplay();
    psychWaterfallReverse();
    //  Serial.println("Cadence 0 - 20");
  }
  else if (rpm >= 21 && rpm < 40)
  {
    clearDisplay();
    tunnelOut();
    // Serial.println("Cadence 21 - 40");
  }
  else if (rpm >= 41 && rpm < 60)
  {
    clearDisplay();
    psychWaves();
    //  Serial.println("Cadence 41 - 60");
  }
  else if (rpm >= 61 && rpm < 80)
  {
    clearDisplay();
    fireFlies();
    // Serial.println("Cadence 61 - 80");
  }
  else if (rpm >= 81 && rpm < 100)
  {
    clearDisplay();
    ballWaves();
    // Serial.println("Cadence 81 - 100");
  }
  else if (rpm >= 101)
  {
    clearDisplay();
    plasma();
    // Serial.println("Cadence greater than 100");
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