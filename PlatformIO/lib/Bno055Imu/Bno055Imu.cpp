#include "Bno055Imu.h"

namespace {
constexpr uint8_t CHIP_ID_REG = 0x00;
constexpr uint8_t CHIP_ID = 0xA0;
constexpr uint8_t PAGE_ID_REG = 0x07;
constexpr uint8_t EULER_REG = 0x1A;
constexpr uint8_t QUATERNION_REG = 0x20;
constexpr uint8_t LINEAR_ACCEL_REG = 0x28;
constexpr uint8_t GRAVITY_REG = 0x2E;
constexpr uint8_t TEMPERATURE_REG = 0x34;
constexpr uint8_t CALIBRATION_REG = 0x35;
constexpr uint8_t SELF_TEST_REG = 0x36;
constexpr uint8_t SYSTEM_STATUS_REG = 0x39;
constexpr uint8_t SYSTEM_ERROR_REG = 0x3A;
constexpr uint8_t UNIT_REG = 0x3B;
constexpr uint8_t POWER_REG = 0x3E;
constexpr uint8_t MODE_REG = 0x3D;
constexpr uint8_t NDOF_MODE = 0x0C;

int16_t signedWord(uint8_t low, uint8_t high) {
    return static_cast<int16_t>(static_cast<uint16_t>(low) |
                                (static_cast<uint16_t>(high) << 8));
}
}

bool Bno055Imu::readBytes(uint8_t reg, uint8_t* output, uint8_t size) {
    if (!wire_) return false;
    wire_->beginTransmission(address_);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0) return false;
    if (wire_->requestFrom(address_, size) != size) return false;
    for (uint8_t i = 0; i < size; ++i) output[i] = wire_->read();
    return true;
}

bool Bno055Imu::writeRegister(uint8_t reg, uint8_t value) {
    wire_->beginTransmission(address_);
    wire_->write(reg);
    wire_->write(value);
    return wire_->endTransmission() == 0;
}

bool Bno055Imu::begin() {
    available_ = false;
    lastProbeMs_ = millis();
    // Robot wiring has BNO055 and SEN0628 together on I2C1. Check that bus
    // first, but retain I2C0 as a recovery option if the plug is moved.
    TwoWire* buses[] = {&Wire1, &Wire};
    for (uint8_t index = 0; index < 2; ++index) {
        buses[index]->begin();
        for (uint8_t address : {uint8_t(0x28), uint8_t(0x29)}) {
            wire_ = buses[index];
            bus_ = index == 0 ? 1 : 0;
            address_ = address;
            uint8_t id = 0;
            if (!readBytes(CHIP_ID_REG, &id, 1) || id != CHIP_ID) continue;
            if (!writeRegister(MODE_REG, 0x00)) continue;
            delay(25);
            if (!writeRegister(PAGE_ID_REG, 0x00) ||
                !writeRegister(POWER_REG, 0x00) ||
                !writeRegister(UNIT_REG, 0x00) ||
                !writeRegister(MODE_REG, NDOF_MODE)) continue;
            // Bosch specifies 7 ms when switching from CONFIG_MODE to an
            // operation mode. Leave additional margin before the first read.
            delay(30);
            failures_ = 0;
            available_ = true;
            return update();
        }
    }
    wire_ = nullptr;
    address_ = 0;
    return false;
}

bool Bno055Imu::update() {
    if (!available_) {
        if (millis() - lastProbeMs_ >= 5000) begin();
        return available_;
    }
    uint8_t euler[6] = {};
    uint8_t quaternion[8] = {};
    uint8_t linearAcceleration[6] = {};
    uint8_t gravity[6] = {};
    uint8_t calibration = 0;
    uint8_t systemStatus = 0;
    uint8_t systemError = 0;
    uint8_t operationMode = 0;
    uint8_t selfTestResult = 0;
    uint8_t temperature = 0;
    if (!readBytes(EULER_REG, euler, 6) ||
        !readBytes(QUATERNION_REG, quaternion, 8) ||
        !readBytes(LINEAR_ACCEL_REG, linearAcceleration, 6) ||
        !readBytes(GRAVITY_REG, gravity, 6) ||
        !readBytes(TEMPERATURE_REG, &temperature, 1) ||
        !readBytes(CALIBRATION_REG, &calibration, 1) ||
        !readBytes(SELF_TEST_REG, &selfTestResult, 1) ||
        !readBytes(SYSTEM_STATUS_REG, &systemStatus, 1) ||
        !readBytes(SYSTEM_ERROR_REG, &systemError, 1) ||
        !readBytes(MODE_REG, &operationMode, 1)) {
        if (++failures_ >= 3) available_ = false;
        return false;
    }
    failures_ = 0;
    headingDeg_ = signedWord(euler[0], euler[1]) / 16.0f;
    rollDeg_ = signedWord(euler[2], euler[3]) / 16.0f;
    pitchDeg_ = signedWord(euler[4], euler[5]) / 16.0f;
    quaternionW_ = signedWord(quaternion[0], quaternion[1]) / 16384.0f;
    quaternionX_ = signedWord(quaternion[2], quaternion[3]) / 16384.0f;
    quaternionY_ = signedWord(quaternion[4], quaternion[5]) / 16384.0f;
    quaternionZ_ = signedWord(quaternion[6], quaternion[7]) / 16384.0f;
    linearAccelX_ = signedWord(linearAcceleration[0], linearAcceleration[1]) / 100.0f;
    linearAccelY_ = signedWord(linearAcceleration[2], linearAcceleration[3]) / 100.0f;
    linearAccelZ_ = signedWord(linearAcceleration[4], linearAcceleration[5]) / 100.0f;
    gravityX_ = signedWord(gravity[0], gravity[1]) / 100.0f;
    gravityY_ = signedWord(gravity[2], gravity[3]) / 100.0f;
    gravityZ_ = signedWord(gravity[4], gravity[5]) / 100.0f;
    temperatureC_ = static_cast<int8_t>(temperature);
    calibration_ = calibration;
    systemStatus_ = systemStatus;
    systemError_ = systemError;
    operationMode_ = operationMode;
    selfTestResult_ = selfTestResult;
    return true;
}
