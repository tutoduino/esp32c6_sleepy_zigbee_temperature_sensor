//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/**
 * @brief Zigbee temperature and humidity sensor Zigbee Sleepy End Device.
 *
 * https://tutoduino.fr/tutoriels/esp32c6-zigbee/
 * This code is based on example "Zigbee temperature and humidity sensor Sleepy device" created by Jan Procházka 
 * https://github.com/espressif/arduino-esp32/tree/master/libraries/Zigbee/examples/Zigbee_Temp_Hum_Sensor_Sleepy
 */
#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif
// Uncomment the following line to display debug traces in serial monitor of Arduino IDE
//#define DEBUG_TRACE
#include "Zigbee.h"
#include <BME280I2C.h>
#include <Wire.h>
/* Zigbee temperature + humidity sensor configuration */
#define TEMP_SENSOR_ENDPOINT_NUMBER 10
#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 600         /* Sleep for 10 minutes */
ZigbeeTempSensor zbTempSensor = ZigbeeTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER);
/* BME280 sensor */
BME280I2C sensor;
// 3.7 V Li-Ion battery voltage
const float minVoltage = 3.0;
const float maxVoltage = 4.0;
// Mapp float values to percentage
uint8_t mapFloat(float x, float in_min, float in_max) {
  float val;
  val = (x - in_min) * (100) / (in_max - in_min);
  if (val < 0) {
    val = 0;
  } else if (val > 100) {
    val = 100;
  }
  return (uint8_t)val;
}
// Get battery voltage en V
float getVbatt() {
  uint32_t Vbatt = 0;
  for (int i = 0; i < 16; i++) {
    Vbatt += analogReadMilliVolts(A0);  // Read and accumulate ADC voltage
  }
  return (2 * Vbatt / 16 / 1000.0);  // Adjust for 1:2 divider and convert to volts
}
// Get data from BME280 sensor and go to deep sleep mode
void meausureAndSleep() {
  // Measure temperature sensor value
  float temperature(NAN), humidity(NAN), pressure(NAN);
  uint8_t percentage;
  float vBat;
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_hPa);
  // Read temperature and humidity on BME280 sensor
  sensor.read(pressure, temperature, humidity, tempUnit, presUnit);
  // Measure battery voltage
  vBat = getVbatt();
  percentage = mapFloat(vBat, minVoltage, maxVoltage);
#ifdef DEBUG_TRACE
  Serial.printf("Battery: %.2fV (%d%%)\n", vBat, percentage);
#endif
  // Update battery percentage
  zbTempSensor.setBatteryPercentage(percentage);
  zbTempSensor.setBatteryVoltage(vBat * 10);  // voltage in 100mV
  // Update temperature and humidity values in Temperature sensor EP
  zbTempSensor.setTemperature(temperature);
  zbTempSensor.setHumidity(humidity);
  // Report values
  zbTempSensor.report();
  zbTempSensor.reportBatteryPercentage();
#ifdef DEBUG_TRACE
  Serial.printf("Reported temperature: %.2f°C, Humidity: %.2f%%\r\n", temperature, humidity);
#endif
  // Turn on the builtin LED for a very short time
  flashLED();
  // Add small delay to allow the data to be sent before going to sleep
  delay(500);
  // Put device to deep sleep
#ifdef DEBUG_TRACE
  Serial.println("Going to sleep now");
#endif
  esp_deep_sleep_start();
}
// Internal Led flash
void flashLED() {
  // Turn on LED for 100ms
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
}
/********************* Arduino functions **************************/
void setup() {
#ifdef DEBUG_TRACE
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("Tutoduino Zigbee temperature sensor start!");
#endif
  // Configure use of external antenna
  pinMode(WIFI_ENABLE, OUTPUT);    // pinMode(3, OUTPUT);
  digitalWrite(WIFI_ENABLE, LOW);  // digitalWrite(3, LOW); // Activate RF switch control
  delay(100);
  pinMode(WIFI_ANT_CONFIG, OUTPUT);     // pinMode(14, OUTPUT);
  digitalWrite(WIFI_ANT_CONFIG, HIGH);  // digitalWrite(14, HIGH); // Use external antenna
  // Configure builtin LED and turn it OFF (HIGH)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // Init BME280 sensor
  Wire.begin();
  while (!sensor.begin()) {
#ifdef DEBUG_TRACE
    Serial.println("Could not find BME280 sensor!");
#endif
    delay(1000);
  }
  // Configure A0 as ADC input for reading battery voltage
  pinMode(A0, INPUT);
  // Configure the wake up source and set to wake up every 5 seconds
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  // Optional: set Zigbee device name and model
  zbTempSensor.setManufacturerAndModel("Tutoduino", "ESP32C6TempSensor");
  // Set minimum and maximum temperature measurement value
  zbTempSensor.setMinMaxValue(-20, 80);
  // Set tolerance for temperature measurement in °C (lowest possible value is 0.01°C)
  zbTempSensor.setTolerance(1);
  // Set power source to battery, battery percentage and battery voltage (now 100% and 3.5V for demonstration)
  // The value can be also updated by calling zbTempSensor.setBatteryPercentage(percentage) or zbTempSensor.setBatteryVoltage(voltage) anytime after Zigbee.begin()
  zbTempSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY, 100, 35);
  // Add humidity cluster to the temperature sensor device with min, max and tolerance values
  zbTempSensor.addHumiditySensor(0, 100, 1);
  // Add endpoint to Zigbee Core
  Zigbee.addEndpoint(&zbTempSensor);
  // Create a default Zigbee configuration for End Device
  esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();
#ifdef DEBUG_TRACE
  Serial.println("Starting Zigbee");
#endif
  // When all EPs are registered, start Zigbee in End Device mode
  if (!Zigbee.begin(&zigbeeConfig, false)) {
    // If Zigbee does not start with 30s default timeout (ZB_BEGIN_TIMEOUT_DEFAULT) then restart
#ifdef DEBUG_TRACE
    Serial.println("Zigbee failed to start!");
    Serial.println("Rebooting ESP32!");
#endif
    ESP.restart();  // If Zigbee failed to start, reboot the device and try again
  }
#ifdef DEBUG_TRACE
  Serial.println("Connecting to network");
#endif
  while (!Zigbee.connected()) {
#ifdef DEBUG_TRACE
    Serial.print(".");
#endif
    delay(100);
  }
#ifdef DEBUG_TRACE
  Serial.println("Successfully connected to Zigbee network");
#endif
  // Delay approx 1s (may be adjusted) to allow establishing proper connection with coordinator, needed for sleepy devices
  delay(1000);
  // Call the function to measure temperature and put the device to deep sleep
  meausureAndSleep();
}
void loop() {
  // No actions are performed in the loop (the ESP32C6 enters the setup function when it exits deep sleep).
}
