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
int rpm = 0;
const int nextButtonPin = 34;
const int prevButtonPin = 35;
int hue = 0;
#define NUM_STRIPS 7
#define NUM_LEDS_PER_STRIP 165
#define NUM_LEDS_LAST_STRIP 193
#define NUM_LEDS_TOTAL (NUM_STRIPS * NUM_LEDS_PER_STRIP + NUM_LEDS_LAST_STRIP)
#define NUM_ROWS 43
#define NUM_COLS 28
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
#define BRIGHTNESS 204 // 127 for 50%, 204 for 80%
#define NUM_PATTERNS 13
float distances[NUM_LEDS_TOTAL];
uint8_t ballX, ballY;
uint16_t XY(uint16_t x, uint16_t y)
{
  // any out of bounds address maps to the first hidden pixel
  if ((x >= kMatrixWidth) || (y >= kMatrixHeight))
  {
    return (LAST_VISIBLE_LED + 1);
  }
  const uint16_t XYTable[] = {
      1155, 1156, 1157, 1158, 1159, 1160, 1161, 1162, 1163, 1164, 1165, 1166, 1167, 1168, 1169, 1170, 1171, 1172, 1173, 1174, 1175, 1176, 1177, 1178, 1179, 1180, 1181, 1182,
      1203, 1154, 1153, 1152, 1151, 1150, 1149, 1148, 1147, 1146, 1145, 1144, 1143, 1142, 1141, 1140, 1139, 1138, 1137, 1136, 1135, 1134, 1133, 1132, 1131, 1130, 1129, 1128,
      1100, 1101, 1102, 1103, 1104, 1105, 1106, 1107, 1108, 1109, 1110, 1111, 1112, 1113, 1114, 1115, 1116, 1117, 1118, 1119, 1120, 1121, 1122, 1123, 1124, 1125, 1126, 1127,
      1202, 1099, 1098, 1097, 1096, 1095, 1094, 1093, 1092, 1091, 1090, 1089, 1088, 1087, 1086, 1085, 1084, 1083, 1082, 1081, 1080, 1079, 1078, 1077, 1076, 1075, 1074, 1073,
      1045, 1046, 1047, 1048, 1049, 1050, 1051, 1052, 1053, 1054, 1055, 1056, 1057, 1058, 1059, 1060, 1061, 1062, 1063, 1064, 1065, 1066, 1067, 1068, 1069, 1070, 1071, 1072,
      1201, 1044, 1043, 1042, 1041, 1040, 1039, 1038, 1037, 1036, 1035, 1034, 1033, 1032, 1031, 1030, 1029, 1028, 1027, 1026, 1025, 1024, 1023, 1022, 1021, 1020, 1019, 1018,
      990, 991, 992, 993, 994, 995, 996, 997, 998, 999, 1000, 1001, 1002, 1003, 1004, 1005, 1006, 1007, 1008, 1009, 1010, 1011, 1012, 1013, 1014, 1015, 1016, 1017,
      1200, 989, 988, 987, 986, 985, 984, 983, 982, 981, 980, 979, 978, 977, 976, 975, 974, 973, 972, 971, 970, 969, 968, 967, 966, 965, 964, 963,
      935, 936, 937, 938, 939, 940, 941, 942, 943, 944, 945, 946, 947, 948, 949, 950, 951, 952, 953, 954, 955, 956, 957, 958, 959, 960, 961, 962,
      1199, 934, 933, 932, 931, 930, 929, 928, 927, 926, 925, 924, 923, 922, 921, 920, 919, 918, 917, 916, 915, 914, 913, 912, 911, 910, 909, 908,
      880, 881, 882, 883, 884, 885, 886, 887, 888, 889, 890, 891, 892, 893, 894, 895, 896, 897, 898, 899, 900, 901, 902, 903, 904, 905, 906, 907,
      1198, 879, 878, 877, 876, 875, 874, 873, 872, 871, 870, 869, 868, 867, 866, 865, 864, 863, 862, 861, 860, 859, 858, 857, 856, 855, 854, 853,
      825, 826, 827, 828, 829, 830, 831, 832, 833, 834, 835, 836, 837, 838, 839, 840, 841, 842, 843, 844, 845, 846, 847, 848, 849, 850, 851, 852,
      1197, 824, 823, 822, 821, 820, 819, 818, 817, 816, 815, 814, 813, 812, 811, 810, 809, 808, 807, 806, 805, 804, 803, 802, 801, 800, 799, 798,
      770, 771, 772, 773, 774, 775, 776, 777, 778, 779, 780, 781, 782, 783, 784, 785, 786, 787, 788, 789, 790, 791, 792, 793, 794, 795, 796, 797,
      1196, 769, 768, 767, 766, 765, 764, 763, 762, 761, 760, 759, 758, 757, 756, 755, 754, 753, 752, 751, 750, 749, 748, 747, 746, 745, 744, 743,
      715, 716, 717, 718, 719, 720, 721, 722, 723, 724, 725, 726, 727, 728, 729, 730, 731, 732, 733, 734, 735, 736, 737, 738, 739, 740, 741, 742,
      1195, 714, 713, 712, 711, 710, 709, 708, 707, 706, 705, 704, 703, 702, 701, 700, 699, 698, 697, 696, 695, 694, 693, 692, 691, 690, 689, 688,
      660, 661, 662, 663, 664, 665, 666, 667, 668, 669, 670, 671, 672, 673, 674, 675, 676, 677, 678, 679, 680, 681, 682, 683, 684, 685, 686, 687,
      1194, 659, 658, 657, 656, 655, 654, 653, 652, 651, 650, 649, 648, 647, 646, 645, 644, 643, 642, 641, 640, 639, 638, 637, 636, 635, 634, 633,
      605, 606, 607, 608, 609, 610, 611, 612, 613, 614, 615, 616, 617, 618, 619, 620, 621, 622, 623, 624, 625, 626, 627, 628, 629, 630, 631, 632,
      1193, 604, 603, 602, 601, 600, 599, 598, 597, 596, 595, 594, 593, 592, 591, 590, 589, 588, 587, 586, 585, 584, 583, 582, 581, 580, 579, 578,
      550, 551, 552, 553, 554, 555, 556, 557, 558, 559, 560, 561, 562, 563, 564, 565, 566, 567, 568, 569, 570, 571, 572, 573, 574, 575, 576, 577,
      1192, 549, 548, 547, 546, 545, 544, 543, 542, 541, 540, 539, 538, 537, 536, 535, 534, 533, 532, 531, 530, 529, 528, 527, 526, 525, 524, 523,
      495, 496, 497, 498, 499, 500, 501, 502, 503, 504, 505, 506, 507, 508, 509, 510, 511, 512, 513, 514, 515, 516, 517, 518, 519, 520, 521, 522,
      1191, 494, 493, 492, 491, 490, 489, 488, 487, 486, 485, 484, 483, 482, 481, 480, 479, 478, 477, 476, 475, 474, 473, 472, 471, 470, 469, 468,
      440, 441, 442, 443, 444, 445, 446, 447, 448, 449, 450, 451, 452, 453, 454, 455, 456, 457, 458, 459, 460, 461, 462, 463, 464, 465, 466, 467,
      1190, 439, 438, 437, 436, 435, 434, 433, 432, 431, 430, 429, 428, 427, 426, 425, 424, 423, 422, 421, 420, 419, 418, 417, 416, 415, 414, 413,
      385, 386, 387, 388, 389, 390, 391, 392, 393, 394, 395, 396, 397, 398, 399, 400, 401, 402, 403, 404, 405, 406, 407, 408, 409, 410, 411, 412,
      1189, 384, 383, 382, 381, 380, 379, 378, 377, 376, 375, 374, 373, 372, 371, 370, 369, 368, 367, 366, 365, 364, 363, 362, 361, 360, 359, 358,
      330, 331, 332, 333, 334, 335, 336, 337, 338, 339, 340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357,
      1188, 329, 328, 327, 326, 325, 324, 323, 322, 321, 320, 319, 318, 317, 316, 315, 314, 313, 312, 311, 310, 309, 308, 307, 306, 305, 304, 303,
      275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286, 287, 288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298, 299, 300, 301, 302,
      1187, 274, 273, 272, 271, 270, 269, 268, 267, 266, 265, 264, 263, 262, 261, 260, 259, 258, 257, 256, 255, 254, 253, 252, 251, 250, 249, 248,
      220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247,
      1186, 219, 218, 217, 216, 215, 214, 213, 212, 211, 210, 209, 208, 207, 206, 205, 204, 203, 202, 201, 200, 199, 198, 197, 196, 195, 194, 193,
      165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192,
      1185, 164, 163, 162, 161, 160, 159, 158, 157, 156, 155, 154, 153, 152, 151, 150, 149, 148, 147, 146, 145, 144, 143, 142, 141, 140, 139, 138,
      110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137,
      1184, 109, 108, 107, 106, 105, 104, 103, 102, 101, 100, 99, 98, 97, 96, 95, 94, 93, 92, 91, 90, 89, 88, 87, 86, 85, 84, 83,
      55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82,
      1183, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28,
      0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27};

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
  FastLED.setBrightness(204);                                    // 127 for 50%, 204 for 80%                                 // 128 for 50%, 204 for 80%
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
  // Set the pin mode for the buttons
  pinMode(nextButtonPin, INPUT_PULLUP);
  pinMode(prevButtonPin, INPUT_PULLUP);
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

void fadeOutAndClear(void (*pattern)())
{
  for (int j = 255; j >= 0; j -= 1) // Decrease brightness by 1 in each iteration
  {
    FastLED.setBrightness(j);
    pattern(); // Call the pattern function in each iteration
    FastLED.show();
    delay(20); // Increase delay to 20 ms
  }
  clearDisplay();
}

void fadeIn(void (*pattern)())
{
  for (int j = 0; j <= 255; j += 1) // Increase brightness by 1 in each iteration
  {
    FastLED.setBrightness(j);
    pattern(); // Call the pattern function in each iteration
    FastLED.show();
    delay(20); // Increase delay to 20 ms
  }
}

void fadeTransition(void (*oldPattern)(), void (*newPattern)())
{
  CRGB oldLeds[NUM_STRIPS][NUM_LEDS_PER_STRIP];
  CRGB newLeds[NUM_STRIPS][NUM_LEDS_PER_STRIP];

  // Generate the old pattern with full brightness
  FastLED.setBrightness(255);
  oldPattern(); // Call the old pattern function

  // Save the old pattern
  memcpy(oldLeds, leds, sizeof(oldLeds));

  // Generate the new pattern with full brightness
  newPattern(); // Call the new pattern function

  // Save the new pattern
  memcpy(newLeds, leds, sizeof(newLeds));

  for (int j = 0; j <= 255; j += 5) // Increase brightness by 5 in each iteration
  {
    // Blend the old and new patterns
    for (int i = 0; i < NUM_STRIPS; i++)
    {
      for (int k = 0; k < NUM_LEDS_PER_STRIP; k++)
      {
        leds[i][k] = blend(oldLeds[i][k], newLeds[i][k], j);
      }
    }

    FastLED.show();
    delay(10); // Decrease delay to 10 ms
  }
}

// availible patterns
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
void (*patternFunctions[])(void) = {
    psychWaves,
    pyramids,
    tunnelOut,
    psychWaterfallReverse,
    fireFlies,
    ballWaves,
    plasma,
    psychSpin,
    vaporPyramid,
    galaxy,
    trippyClouds,
    brainWaves,
    rorschach};

enum Pattern
{
  NONE,
  PSYCH_WAVES,
  PYRAMIDS,
  TUNNEL_OUT,
  PSYCH_WATERFALL_REVERSE,
  FIRE_FLIES,
  BALL_WAVES,
  PLASMA,
  PSYCH_SPIN,
  VAPOR_PYRAMID,
  GALAXY,
  TRIPPY_CLOUDS,
  BRAIN_WAVES,
  RORSCHACH
};

// Initialize the current pattern to NONE
Pattern currentPattern = NONE;

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

  Pattern newPattern;
  if (rpm >= 0 && rpm < 15)
  {
    // newPattern = GALAXY;
    // newPattern = PYRAMIDS;
    // newPattern = TUNNEL_OUT;
    // newPattern = PSYCH_WATERFALL_REVERSE;
    // newPattern = TRIPPY_CLOUDS;
    // newPattern = VAPOR_PYRAMID;
    // newPattern = RORSCHACH; // currently broken :(
    newPattern = BALL_WAVES;
  }
  else if (rpm >= 16 && rpm < 30)
  {
    newPattern = RORSCHACH;
    // newPattern = PYRAMIDS;
  }
  else if (rpm >= 46 && rpm < 60)
  {
    newPattern = TUNNEL_OUT;
  }
  else if (rpm >= 61 && rpm < 75)
  {
    newPattern = RORSCHACH;
  }
  else if (rpm >= 76 && rpm < 90)
  {
    newPattern = PSYCH_WATERFALL_REVERSE;
  }
  else if (rpm >= 91 && rpm < 106)
  {
    newPattern = VAPOR_PYRAMID;
  }
  else if (rpm >= 106 && rpm < 135)
  {
    newPattern = BRAIN_WAVES;
  }
  else if (rpm >= 136 && rpm < 150)
  {
    newPattern = BALL_WAVES;
  }
  else if (rpm >= 151 && rpm < 165)
  {
    newPattern = PSYCH_WAVES;
  }
  else if (rpm >= 166 && rpm < 180)
  {
    newPattern = TRIPPY_CLOUDS;
  }
  else if (rpm >= 181 && rpm < 195)
  {
    newPattern = PLASMA;
  }
  else if (rpm >= 196 && rpm < 210)
  {
    newPattern = PSYCH_SPIN;
  }
  else if (rpm >= 211)
  {
    newPattern = FIRE_FLIES;
  }

  // If the pattern has changed, transition from the old pattern to the new one
  if (newPattern != currentPattern)
  {
    switch (currentPattern)
    {
    case NONE:
      switch (newPattern)
      {
      case PSYCH_WAVES:
        fadeTransition(patternFunctions[currentPattern], patternFunctions[newPattern]);
        break;
      case PYRAMIDS:
        fadeTransition(patternFunctions[currentPattern], patternFunctions[newPattern]);
        break;
      case TUNNEL_OUT:
        fadeTransition(patternFunctions[currentPattern], patternFunctions[newPattern]);
        break;
      case PSYCH_WATERFALL_REVERSE:
        fadeTransition(patternFunctions[currentPattern], patternFunctions[newPattern]);
        break;
      case FIRE_FLIES:
        fadeTransition(patternFunctions[currentPattern], patternFunctions[newPattern]);
        break;
      case BALL_WAVES:
        fadeTransition(patternFunctions[currentPattern], patternFunctions[newPattern]);
        break;
      case PLASMA:
        fadeTransition(patternFunctions[currentPattern], patternFunctions[newPattern]);
        break;
      case PSYCH_SPIN:
        fadeTransition(patternFunctions[currentPattern], patternFunctions[newPattern]);
        break;
      case VAPOR_PYRAMID:
        fadeTransition(patternFunctions[currentPattern], patternFunctions[newPattern]);
        break;
      case GALAXY:
        fadeTransition(patternFunctions[currentPattern], patternFunctions[newPattern]);
        break;
      case TRIPPY_CLOUDS:
        fadeTransition(patternFunctions[currentPattern], patternFunctions[newPattern]);
        break;
      case BRAIN_WAVES:
        fadeTransition(patternFunctions[currentPattern], patternFunctions[newPattern]);
        break;
      case RORSCHACH:
        fadeTransition(patternFunctions[currentPattern], patternFunctions[newPattern]);
        break;
      }
      break;
      // Add other cases here for each pattern in the enum
    }

    // Update the current pattern
    currentPattern = newPattern;
  }

  // Display the current pattern
  switch (currentPattern)
  {
  case NONE:
    break;
  case PSYCH_WAVES:
    psychWaves();
    break;
  case PYRAMIDS:
    pyramids();
    break;
  case TUNNEL_OUT:
    tunnelOut();
    break;
  case PSYCH_WATERFALL_REVERSE:
    psychWaterfallReverse();
    break;
  case FIRE_FLIES:
    fireFlies();
    break;
  case BALL_WAVES:
    ballWaves();
    break;
  case PLASMA:
    plasma();
    break;
  case PSYCH_SPIN:
    psychSpin();
    break;
  case VAPOR_PYRAMID:
    vaporPyramid();
    break;
  case GALAXY:
    galaxy();
    break;
  case TRIPPY_CLOUDS:
    trippyClouds();
    break;
  case BRAIN_WAVES:
    brainWaves();
    break;
  case RORSCHACH:
    rorschach();
    break;
  }

  FastLED.show();
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