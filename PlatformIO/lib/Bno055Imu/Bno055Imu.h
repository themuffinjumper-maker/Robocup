#pragma once

#include <Arduino.h>
#include <Wire.h>

// Small BNO055 driver for the robot's two exposed I2C buses. The sensor's
// fused Euler heading is used for display/localisation, not motor control.
class Bno055Imu {
public:
    bool begin();
    bool update();
    bool available() const { return available_; }
    uint8_t bus() const { return bus_; }
    uint8_t address() const { return address_; }
    float headingDeg() const { return headingDeg_; }
    float rollDeg() const { return rollDeg_; }
    float pitchDeg() const { return pitchDeg_; }
    float quaternionW() const { return quaternionW_; }
    float quaternionX() const { return quaternionX_; }
    float quaternionY() const { return quaternionY_; }
    float quaternionZ() const { return quaternionZ_; }
    float linearAccelX() const { return linearAccelX_; }
    float linearAccelY() const { return linearAccelY_; }
    float linearAccelZ() const { return linearAccelZ_; }
    float gravityX() const { return gravityX_; }
    float gravityY() const { return gravityY_; }
    float gravityZ() const { return gravityZ_; }
    int8_t temperatureC() const { return temperatureC_; }
    uint8_t systemCalibration() const { return (calibration_ >> 6) & 3; }
    uint8_t gyroCalibration() const { return (calibration_ >> 4) & 3; }
    uint8_t accelCalibration() const { return (calibration_ >> 2) & 3; }
    uint8_t magCalibration() const { return calibration_ & 3; }
    uint8_t systemStatus() const { return systemStatus_; }
    uint8_t systemError() const { return systemError_; }
    uint8_t operationMode() const { return operationMode_; }
    uint8_t selfTestResult() const { return selfTestResult_; }
    bool selfTestPassed() const { return (selfTestResult_ & 0x0F) == 0x0F; }
    // SYS_ERR is only an active error code while SYS_STATUS is 0x01.
    bool systemErrorActive() const { return systemStatus_ == 1 && systemError_ != 0; }
    bool fusionRunning() const { return systemStatus_ == 5; }

private:
    TwoWire* wire_ = nullptr;
    uint8_t bus_ = 0;
    uint8_t address_ = 0;
    bool available_ = false;
    uint8_t calibration_ = 0;
    float headingDeg_ = 0;
    float rollDeg_ = 0;
    float pitchDeg_ = 0;
    float quaternionW_ = 1;
    float quaternionX_ = 0;
    float quaternionY_ = 0;
    float quaternionZ_ = 0;
    float linearAccelX_ = 0;
    float linearAccelY_ = 0;
    float linearAccelZ_ = 0;
    float gravityX_ = 0;
    float gravityY_ = 0;
    float gravityZ_ = 0;
    int8_t temperatureC_ = 0;
    uint8_t systemStatus_ = 0;
    uint8_t systemError_ = 0;
    uint8_t operationMode_ = 0;
    uint8_t selfTestResult_ = 0;
    uint8_t failures_ = 0;
    uint32_t lastProbeMs_ = 0;

    bool readBytes(uint8_t reg, uint8_t* output, uint8_t size);
    bool writeRegister(uint8_t reg, uint8_t value);
};
