#include "DigitalInputSensor.h"

void DigitalInputSensor::begin(uint8_t pin, bool activeLow, bool usePullup,
                               uint16_t debounceMs)
{
    pin_ = pin;
    activeLow_ = activeLow;
    debounceMs_ = debounceMs;
    pinMode(pin_, usePullup ? INPUT_PULLUP : INPUT);
    stableRawHigh_ = digitalRead(pin_) == HIGH;
    candidateRawHigh_ = stableRawHigh_;
    candidateSinceMs_ = millis();
    transitionCount_ = 0;
    begun_ = true;
}

void DigitalInputSensor::update(uint32_t nowMs)
{
    if (!begun_)
        return;

    const bool rawHigh = digitalRead(pin_) == HIGH;
    if (rawHigh != candidateRawHigh_)
    {
        candidateRawHigh_ = rawHigh;
        candidateSinceMs_ = nowMs;
        return;
    }

    if (rawHigh != stableRawHigh_ && nowMs - candidateSinceMs_ >= debounceMs_)
    {
        stableRawHigh_ = rawHigh;
        ++transitionCount_;
    }
}
