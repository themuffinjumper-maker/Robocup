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
bool sensorTimedOut[TofConfig::MAX_SENSOR_COUNT] = {};
uint8_t sensorCount = 0;
bool expanderReady = false;

constexpr uint8_t DEFAULT_SENSOR_ADDRESS = 0x29;
constexpr uint16_t BOOT_SETTLE_MS = 40;

// Do not call a sensor library's init() until the just-enabled device has
// actually acknowledged on its factory address.  Besides making failures
// quick, this avoids a failed XSHUT socket holding up the rest of the I2C bus.
bool sensorAcknowledgesAtDefaultAddress() {
  Wire.beginTransmission(DEFAULT_SENSOR_ADDRESS);
  return Wire.endTransmission() == 0;
}

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
  for (uint8_t i = 0; i < TofConfig::MAX_SENSOR_COUNT; ++i) {
    sensorReady[i] = false;
    sensorTimedOut[i] = false;
  }
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
  delay(BOOT_SETTLE_MS);
  bool allReady = true;

  for (uint8_t i = 0; i < sensorCount; ++i) {
    expander.digitalWrite(sensorDefinitions[i].xshutPin, HIGH);
    delay(BOOT_SETTLE_MS);

    if (!sensorAcknowledgesAtDefaultAddress()) {
      allReady = false;
      expander.digitalWrite(sensorDefinitions[i].xshutPin, LOW);
      continue;
    }

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

bool Tof::retry(uint8_t index) {
  if (index >= sensorCount || !expanderReady)
    return false;

  const SensorDefinition& definition = sensorDefinitions[index];
  expander.pinMode(definition.xshutPin, OUTPUT);
  expander.digitalWrite(definition.xshutPin, LOW);
  delay(10);
  expander.digitalWrite(definition.xshutPin, HIGH);
  delay(BOOT_SETTLE_MS);

  if (!sensorAcknowledgesAtDefaultAddress()) {
    sensorReady[index] = false;
    sensorTimedOut[index] = true;
    expander.digitalWrite(definition.xshutPin, LOW);
    return false;
  }

  bool ready = false;
  if (definition.type == SensorType::Short) {
    shortSensors[index].setTimeout(120);
    ready = shortSensors[index].init();
    if (ready) {
      shortSensors[index].setAddress(definition.address);
      shortSensors[index].startContinuous(50);
    }
  } else {
    longSensors[index].setTimeout(120);
    ready = longSensors[index].init();
    if (ready) {
      longSensors[index].setAddress(definition.address);
      longSensors[index].startContinuous(50);
    }
  }

  sensorReady[index] = ready;
  sensorTimedOut[index] = !ready;
  if (!ready)
    expander.digitalWrite(definition.xshutPin, LOW);
  return ready;
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
  const int16_t range = sensorDefinitions[index].type == SensorType::Short
      ? static_cast<int16_t>(shortSensors[index].readRangeContinuousMillimeters())
      : static_cast<int16_t>(longSensors[index].read());
  sensorTimedOut[index] = sensorDefinitions[index].type == SensorType::Short
      ? shortSensors[index].timeoutOccurred()
      : longSensors[index].timeoutOccurred();
  return sensorTimedOut[index] ? -1 : range;
}

bool Tof::readIfReady(uint8_t index, int16_t& rangeMm) {
  if (index >= sensorCount || !sensorReady[index]) return false;

  if (sensorDefinitions[index].type == SensorType::Short) {
    // Pololu's VL53L0X continuous-read helper waits in a loop (up to its
    // timeout). Check RESULT_INTERRUPT_STATUS first so a missing sensor or a
    // not-yet-finished 50 ms conversion can never stall the robot loop.
    if ((shortSensors[index].readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0)
      return false;
    rangeMm = static_cast<int16_t>(shortSensors[index].readRangeContinuousMillimeters());
    sensorTimedOut[index] = shortSensors[index].timeoutOccurred();
  } else {
    if (!longSensors[index].dataReady())
      return false;
    rangeMm = static_cast<int16_t>(longSensors[index].read(false));
    sensorTimedOut[index] = longSensors[index].timeoutOccurred();
  }
  if (sensorTimedOut[index])
    rangeMm = -1;
  return true;
}

bool Tof::available(uint8_t index) {
  return index < sensorCount && sensorReady[index];
}

bool Tof::timedOut(uint8_t index) {
  return index >= sensorCount || !sensorReady[index] || sensorTimedOut[index];
}
