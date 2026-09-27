#include "Tof.h"
#include "config.h"

#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

namespace {

SX1509 expander;
VL53L0X shortSensors[TofConfig::MAX_SENSOR_COUNT];
VL53L1X longSensors[TofConfig::MAX_SENSOR_COUNT];
Tof::SensorDefinition sensorDefinitions[TofConfig::MAX_SENSOR_COUNT] = {};
bool sensorReady[TofConfig::MAX_SENSOR_COUNT] = {};
uint8_t sensorCount = 0;
bool expanderReady = false;

// Holds every sensor's XSHUT low so they all boot at the same default I2C
// address (0x29) without colliding, ready to be brought up one at a time.
void resetAllSensors() {
  // Hold every CPU-board XSHUT connector low. This prevents an unlisted sensor
  // remaining at the shared factory address 0x29 and colliding during setup.
  for (uint8_t pin = 0; pin < TofConfig::MAX_SENSOR_COUNT; ++pin) {
    expander.pinMode(pin, OUTPUT);
    expander.digitalWrite(pin, LOW);
  }
}

} // namespace

bool Tof::begin(const SensorDefinition* definitions, uint8_t count) {
  sensorCount = count > TofConfig::MAX_SENSOR_COUNT ? TofConfig::MAX_SENSOR_COUNT : count;
  for (uint8_t i = 0; i < TofConfig::MAX_SENSOR_COUNT; ++i) sensorReady[i] = false;
  // Keep definitions even if the expander is absent on this attempt, so a
  // later retry can recover without rebuilding the JSON configuration.
  for (uint8_t i = 0; i < sensorCount; ++i) sensorDefinitions[i] = definitions[i];

  Wire.begin();
  Wire.setClock(400000);
  expanderReady = expander.begin(TofConfig::EXPANDER_I2C_ADDRESS);
  if (!expanderReady) {
    return false;
  }

  resetAllSensors();
  delay(10);
  bool allReady = true;

  for (uint8_t i = 0; i < sensorCount; ++i) {
    expander.digitalWrite(sensorDefinitions[i].xshutPin, HIGH);
    delay(10);

    if (sensorDefinitions[i].type == SensorType::Short) {
      shortSensors[i].setTimeout(500);
      if (!shortSensors[i].init()) {
        allReady = false;
        expander.digitalWrite(sensorDefinitions[i].xshutPin, LOW);
        continue;
      }
      shortSensors[i].setAddress(sensorDefinitions[i].address);
      shortSensors[i].startContinuous(50);
    } else {
      longSensors[i].setTimeout(500);
      if (!longSensors[i].init()) {
        allReady = false;
        expander.digitalWrite(sensorDefinitions[i].xshutPin, LOW);
        continue;
      }
      longSensors[i].setAddress(sensorDefinitions[i].address);
      longSensors[i].startContinuous(50);
    }
    sensorReady[i] = true;
  }

  return allReady;
}

bool Tof::retry() {
  return begin(sensorDefinitions, sensorCount);
}

bool Tof::expanderAvailable() {
  return expanderReady;
}

// Latest continuous-mode reading, mm.
uint8_t Tof::count() {
  return sensorCount;
}

int16_t Tof::read(uint8_t index) {
  if (index >= sensorCount || !sensorReady[index]) return -1;
  if (sensorDefinitions[index].type == SensorType::Short)
    return shortSensors[index].readRangeContinuousMillimeters();
  return longSensors[index].read();
}

bool Tof::available(uint8_t index) {
  return index < sensorCount && sensorReady[index];
}

bool Tof::timedOut(uint8_t index) {
  if (index >= sensorCount || !sensorReady[index]) return true;
  if (sensorDefinitions[index].type == SensorType::Short)
    return shortSensors[index].timeoutOccurred();
  return longSensors[index].timeoutOccurred();
}
