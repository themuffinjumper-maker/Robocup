#pragma once

#include <Arduino.h>

// Non-blocking trigger/echo ultrasonic ranging. Call update() frequently.
class UltrasoundSensor {
public:
    void begin(uint8_t triggerPin, uint8_t echoPin,
               uint32_t intervalMs = 100, uint32_t timeoutUs = 30000);
    bool readyToTrigger(uint32_t nowUs = micros()) const;
    void trigger(uint32_t nowUs = micros());
    void update(uint32_t nowUs = micros());

    bool valid() const { return valid_; }
    bool timedOut() const { return timedOut_; }
    uint16_t distanceMm() const { return distanceMm_; }
    uint32_t echoUs() const { return echoUs_; }
    bool echoHigh() const { return digitalRead(echoPin_) == HIGH; }
    uint32_t triggerCount() const { return triggerCount_; }
    uint32_t riseCount() const { return riseCount_; }
    uint32_t fallCount() const { return fallCount_; }
    uint16_t currentEchoAdc() const { return currentEchoAdc_; }
    uint16_t lastMinEchoAdc() const { return lastMinEchoAdc_; }
    uint16_t lastMaxEchoAdc() const { return lastMaxEchoAdc_; }
    uint16_t triggerLowAdc() const { return triggerLowAdc_; }
    uint16_t triggerHighAdc() const { return triggerHighAdc_; }
    void sampleEchoLevel();
    uint8_t triggerPin() const { return triggerPin_; }
    uint8_t echoPin() const { return echoPin_; }
    bool busy() const { return state_ != State::Idle; }

private:
    enum class State : uint8_t { Idle, WaitRise, WaitFall };

    uint8_t triggerPin_ = 0;
    uint8_t echoPin_ = 0;
    uint32_t intervalUs_ = 100000;
    uint32_t timeoutUs_ = 30000;
    uint32_t lastTriggerUs_ = 0;
    uint32_t waitStartedUs_ = 0;
    uint32_t echoStartedUs_ = 0;
    uint32_t echoUs_ = 0;
    uint16_t distanceMm_ = 0;
    bool valid_ = false;
    bool timedOut_ = false;
    bool begun_ = false;
    State state_ = State::Idle;
    uint8_t interruptSlot_ = 0xFF;
    uint32_t triggerCount_ = 0;
    volatile uint32_t riseCount_ = 0;
    volatile uint32_t fallCount_ = 0;
    volatile uint32_t capturedEchoUs_ = 0;
    volatile uint32_t interruptRiseUs_ = 0;
    volatile bool captureArmed_ = false;
    volatile bool captureSawRise_ = false;
    volatile bool captureReady_ = false;
    uint16_t currentEchoAdc_ = 0;
    uint16_t measurementMinEchoAdc_ = 4095;
    uint16_t measurementMaxEchoAdc_ = 0;
    uint16_t lastMinEchoAdc_ = 0;
    uint16_t lastMaxEchoAdc_ = 0;
    uint16_t triggerLowAdc_ = 0;
    uint16_t triggerHighAdc_ = 0;

    static UltrasoundSensor* interruptInstances_[2];
    static void echoInterrupt0();
    static void echoInterrupt1();
    void handleEchoInterrupt();
};
