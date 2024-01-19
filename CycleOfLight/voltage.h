#include <Arduino.h>

float readVcc() {
  analogSetAttenuation(ADC_0db); // Set attenuation to 0dB for full-scale voltage reading
  analogSetWidth(12); // Set ADC resolution to 12 bits

  // Read the internal reference voltage (1.1V) using ADC1
  int rawValue = analogRead(33); // ADC1 channel 5 corresponds to GPIO 33 on ESP32 DevKit

  // Calculate Vcc in millivolts
  float voltage = rawValue / 4095.0 * 1100.0;

  return voltage;
}
