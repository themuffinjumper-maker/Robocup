#pragma once

#include <Arduino.h>
#include <DFRobot_MatrixLidar.h>
#include <Wire.h>

class Tof8x8 {
public:
    static constexpr uint8_t WIDTH = 8;
    static constexpr uint8_t HEIGHT = 8;
    static constexpr uint8_t ZONE_COUNT = WIDTH * HEIGHT;

    explicit Tof8x8(uint8_t address = 0x52, TwoWire& wire = Wire);

    // Selects I2C 8x8 ranging mode. Returns false instead of blocking forever
    // when the sensor is absent or configured for UART.
    bool begin();

    // Advances a non-blocking I2C request. Returns true only when a new
    // complete row-major frame has arrived; call frequently from loop().
    bool read();

    bool available() const;
    bool lastReadSucceeded() const;
    uint32_t frameNumber() const;
    uint32_t lastFrameMs() const;
    uint8_t detectedAddress() const;
    uint8_t addressAckMask() const;
    bool address52Acknowledged() const;
    bool addressAcknowledged(uint8_t address) const;
    uint16_t distanceMm(uint8_t row, uint8_t column) const;
    const uint16_t* data() const;

private:
    TwoWire& wire_;
    uint8_t preferredAddress_;
    DFRobot_MatrixLidar_I2C* sensor_ = nullptr;
    uint16_t frame_[ZONE_COUNT] = {};
    bool available_ = false;
    bool lastReadSucceeded_ = false;
    uint32_t frameNumber_ = 0;
    uint32_t lastFrameMs_ = 0;
    uint32_t pendingSinceMs_ = 0;
    uint32_t nextRequestMs_ = 0;
    uint32_t lastPollMs_ = 0;
    bool awaitingFrame_ = false;
    uint8_t detectedAddress_ = 0;
    uint8_t addressAckMask_ = 0;
    bool address52Acknowledged_ = false;
    uint8_t i2cAckBits_[16] = {};
    bool readBytes(uint8_t* destination, uint16_t length);
};
