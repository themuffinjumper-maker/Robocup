#include "UltrasoundSensor.h"

UltrasoundSensor* UltrasoundSensor::interruptInstances_[2] = {nullptr, nullptr};

void UltrasoundSensor::echoInterrupt0()
{
    if (interruptInstances_[0] != nullptr)
        interruptInstances_[0]->handleEchoInterrupt();
}

void UltrasoundSensor::echoInterrupt1()
{
    if (interruptInstances_[1] != nullptr)
        interruptInstances_[1]->handleEchoInterrupt();
}

void UltrasoundSensor::handleEchoInterrupt()
{
    if (!captureArmed_)
        return;
    const uint32_t nowUs = micros();
    if (digitalRead(echoPin_) == HIGH)
    {
        interruptRiseUs_ = nowUs;
        captureSawRise_ = true;
        ++riseCount_;
    }
    else if (captureSawRise_)
    {
        capturedEchoUs_ = nowUs - interruptRiseUs_;
        captureReady_ = true;
        captureArmed_ = false;
        ++fallCount_;
    }
}

void UltrasoundSensor::begin(uint8_t triggerPin, uint8_t echoPin,
                             uint32_t intervalMs, uint32_t timeoutUs)
{
    triggerPin_ = triggerPin;
    echoPin_ = echoPin;
    intervalUs_ = max(50000UL, intervalMs * 1000UL);
    timeoutUs_ = constrain(timeoutUs, 1000UL, 50000UL);
    pinMode(triggerPin_, OUTPUT);
    digitalWrite(triggerPin_, LOW);
    pinMode(echoPin_, INPUT);
    analogReadResolution(12);
    currentEchoAdc_ = analogRead(echoPin_);
    lastMinEchoAdc_ = currentEchoAdc_;
    lastMaxEchoAdc_ = currentEchoAdc_;
    for (uint8_t slot = 0; slot < 2; ++slot)
    {
        if (interruptInstances_[slot] != nullptr)
            continue;
        interruptInstances_[slot] = this;
        interruptSlot_ = slot;
        attachInterrupt(digitalPinToInterrupt(echoPin_),
                        slot == 0 ? echoInterrupt0 : echoInterrupt1, CHANGE);
        break;
    }
    lastTriggerUs_ = micros() - intervalUs_;
    state_ = State::Idle;
    begun_ = true;
}

void UltrasoundSensor::sampleEchoLevel()
{
    if (!begun_)
        return;
    currentEchoAdc_ = analogRead(echoPin_);
    if (!busy())
        return;
    measurementMinEchoAdc_ = min(measurementMinEchoAdc_, currentEchoAdc_);
    measurementMaxEchoAdc_ = max(measurementMaxEchoAdc_, currentEchoAdc_);
}

bool UltrasoundSensor::readyToTrigger(uint32_t nowUs) const
{
    return begun_ && state_ == State::Idle && nowUs - lastTriggerUs_ >= intervalUs_;
}

void UltrasoundSensor::trigger(uint32_t nowUs)
{
    if (!readyToTrigger(nowUs))
        return;
    // Only the 12 us trigger pulse is synchronous. Echo waiting is fully
    // non-blocking, unlike pulseIn().
    noInterrupts();
    captureReady_ = false;
    captureSawRise_ = false;
    captureArmed_ = interruptSlot_ != 0xFF;
    interrupts();
    ++triggerCount_;
    measurementMinEchoAdc_ = 4095;
    measurementMaxEchoAdc_ = 0;
    digitalWrite(triggerPin_, LOW);
    delayMicroseconds(2);
    triggerLowAdc_ = analogRead(triggerPin_);
    digitalWrite(triggerPin_, HIGH);
    delayMicroseconds(2);
    triggerHighAdc_ = analogRead(triggerPin_);
    // ADC conversion time already keeps the pulse above the required 10 us.
    delayMicroseconds(8);
    digitalWrite(triggerPin_, LOW);
    lastTriggerUs_ = micros();
    waitStartedUs_ = lastTriggerUs_;
    state_ = State::WaitRise;
}

void UltrasoundSensor::update(uint32_t nowUs)
{
    if (!begun_ || state_ == State::Idle)
        return;

    bool captured = false;
    uint32_t capturedUs = 0;
    noInterrupts();
    if (captureReady_)
    {
        capturedUs = capturedEchoUs_;
        captureReady_ = false;
        captured = true;
    }
    interrupts();

    if (captured)
    {
        lastMinEchoAdc_ = measurementMinEchoAdc_;
        lastMaxEchoAdc_ = measurementMaxEchoAdc_;
        echoUs_ = capturedUs;
        const uint32_t millimetres = (echoUs_ * 343UL + 1000UL) / 2000UL;
        distanceMm_ = static_cast<uint16_t>(min(millimetres, 65535UL));
        valid_ = distanceMm_ >= 20 && distanceMm_ <= 5000;
        timedOut_ = false;
        state_ = State::Idle;
        return;
    }

    if (nowUs - waitStartedUs_ >= timeoutUs_)
    {
        noInterrupts();
        captureArmed_ = false;
        captureSawRise_ = false;
        interrupts();
        lastMinEchoAdc_ = measurementMinEchoAdc_;
        lastMaxEchoAdc_ = measurementMaxEchoAdc_;
        valid_ = false;
        timedOut_ = true;
        echoUs_ = 0;
        state_ = State::Idle;
    }
}
