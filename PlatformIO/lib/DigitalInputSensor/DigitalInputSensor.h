#pragma once

#include <Arduino.h>

// Debounced digital sensor input. The reported detected state can be inverted
// independently of the electrical pin level, which supports active-low NPN /
// open-collector sensors without hiding the raw signal during commissioning.
class DigitalInputSensor {
public:
    void begin(uint8_t pin, bool activeLow = true, bool usePullup = true,
               uint16_t debounceMs = 20);
    void update(uint32_t nowMs = millis());

    bool rawHigh() const { return stableRawHigh_; }
    bool detected() const { return activeLow_ ? !stableRawHigh_ : stableRawHigh_; }
    uint32_t transitionCount() const { return transitionCount_; }
    uint8_t pin() const { return pin_; }

private:
    uint8_t pin_ = 0;
    bool activeLow_ = true;
    uint16_t debounceMs_ = 20;
    bool candidateRawHigh_ = false;
    bool stableRawHigh_ = false;
    uint32_t candidateSinceMs_ = 0;
    uint32_t transitionCount_ = 0;
    bool begun_ = false;
};
