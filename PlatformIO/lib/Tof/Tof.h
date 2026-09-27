#pragma once

#include <stdint.h>

namespace Tof {

enum class SensorType : uint8_t {
    Short,
    Long
};

struct SensorDefinition {
    SensorType type;
    uint8_t xshutPin;
    uint8_t address;
};

// Brings up the SX1509 expander and all configured short/long TOF sensors.
// Returns true when every configured sensor was found. A failed sensor is
// marked unavailable rather than halting the rest of the robot firmware.
bool begin(const SensorDefinition* definitions, uint8_t count);

// Reboot the configured XSHUT bank and reassign each sensor address. Safe to
// call again after a failed initial begin; configuration is retained.
bool retry();
bool expanderAvailable();

uint8_t count();
int16_t read(uint8_t index);
bool available(uint8_t index);

// True if the last read on that sensor timed out (treat the reading as stale).
bool timedOut(uint8_t index);

} // namespace Tof
