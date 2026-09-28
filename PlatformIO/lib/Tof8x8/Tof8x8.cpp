#include "Tof8x8.h"

#include <new>

Tof8x8::Tof8x8(uint8_t address, TwoWire& wire)
    : wire_(wire), preferredAddress_(address)
{
}

bool Tof8x8::begin()
{
    wire_.begin();
    addressAckMask_ = 0;
    address52Acknowledged_ = false;
    memset(i2cAckBits_, 0, sizeof(i2cAckBits_));
    detectedAddress_ = 0;

    // Scan the complete usable 7-bit range. Besides locating the SEN0628, this
    // proves whether the selected CPU socket is electrically on this Wire bus.
    for (uint8_t address = 0x08; address <= 0x77; ++address)
    {
        wire_.beginTransmission(address);
        if (wire_.endTransmission() == 0)
            i2cAckBits_[address >> 3] |= static_cast<uint8_t>(1U << (address & 7));
    }

    for (uint8_t address = 0x30; address <= 0x33; ++address)
        if (addressAcknowledged(address))
            addressAckMask_ |= static_cast<uint8_t>(1U << (address - 0x30));

    address52Acknowledged_ = addressAcknowledged(0x52);

    uint8_t address = 0;
    if (preferredAddress_ == 0x52 && address52Acknowledged_)
        address = 0x52;
    else if (preferredAddress_ >= 0x30 && preferredAddress_ <= 0x33 &&
        (addressAckMask_ & (1U << (preferredAddress_ - 0x30))))
        address = preferredAddress_;
    else
        for (uint8_t index = 0; index < 4 && address == 0; ++index)
            if (addressAckMask_ & (1U << index))
                address = 0x30 + index;

    if (address != 0)
    {
        sensor_ = new (std::nothrow) DFRobot_MatrixLidar_I2C(address, &wire_);
        if (sensor_ != nullptr && sensor_->begin() == 0 &&
            sensor_->setRangingMode(eMatrix_8X8) == 0)
        {
            available_ = true;
            detectedAddress_ = address;
            return true;
        }
    }

    available_ = false;
    return false;
}

bool Tof8x8::read()
{
    if (!available_)
        return false;
    const uint32_t now = millis();
    if (!awaitingFrame_)
    {
        if (static_cast<int32_t>(now - nextRequestMs_) < 0)
            return false;
        // DFRobot's CMD_ALLData packet is 0x55, argument-count 1, command 2.
        // Its synchronous getAllData() waits up to seconds for a response;
        // waiting here would starve motor control and odometry.
        const uint8_t request[4] = {0x55, 0x00, 0x01, 0x02};
        wire_.beginTransmission(detectedAddress_);
        const size_t written = wire_.write(request, sizeof(request));
        const uint8_t transmissionError = wire_.endTransmission();
        if (written != sizeof(request) || transmissionError != 0)
        {
            lastReadSucceeded_ = false;
            nextRequestMs_ = now + 100U;
            return false;
        }
        awaitingFrame_ = true;
        pendingSinceMs_ = now;
        lastPollMs_ = now;
        return false;
    }
    if (now - pendingSinceMs_ > 2500U)
    {
        awaitingFrame_ = false;
        lastReadSucceeded_ = false;
        nextRequestMs_ = now + 100U;
        return false;
    }
    if (now - lastPollMs_ < 10U)
        return false;
    lastPollMs_ = now;

    uint8_t status = 0xFF;
    if (!readBytes(&status, 1))
        return false;
    if (status == 0xFF) // The module has not finished the ranging frame.
        return false;

    uint8_t header[3] = {};
    if ((status != 0x53 && status != 0x63) ||
        !readBytes(header, sizeof(header)))
    {
        awaitingFrame_ = false;
        lastReadSucceeded_ = false;
        nextRequestMs_ = now + 100U;
        return false;
    }
    const uint16_t length = static_cast<uint16_t>(header[1]) |
                            (static_cast<uint16_t>(header[2]) << 8);
    if (header[0] != 0x02 || status != 0x53 || length != 128U)
    {
        // Consume a short error payload so the next request starts at a
        // packet boundary. Refuse unexpected lengths rather than overflowing
        // the 64-zone output buffer.
        if (length > 0 && length <= 256U)
        {
            uint8_t discard[32];
            uint16_t remaining = length;
            while (remaining > 0)
            {
                const uint16_t chunk = remaining < sizeof(discard)
                    ? remaining : sizeof(discard);
                if (!readBytes(discard, chunk)) break;
                remaining -= chunk;
            }
        }
        awaitingFrame_ = false;
        lastReadSucceeded_ = false;
        nextRequestMs_ = now + 100U;
        return false;
    }

    uint8_t raw[128];
    if (!readBytes(raw, sizeof(raw)))
    {
        awaitingFrame_ = false;
        lastReadSucceeded_ = false;
        nextRequestMs_ = now + 100U;
        return false;
    }
    for (uint8_t i = 0; i < ZONE_COUNT; ++i)
        frame_[i] = static_cast<uint16_t>(raw[2U * i]) |
                    (static_cast<uint16_t>(raw[2U * i + 1U]) << 8);
    awaitingFrame_ = false;
    lastReadSucceeded_ = true;
    lastFrameMs_ = now;
    nextRequestMs_ = now + 500U;
    ++frameNumber_;
    return true;
}

bool Tof8x8::readBytes(uint8_t* destination, uint16_t length)
{
    uint16_t offset = 0;
    while (offset < length)
    {
        const uint16_t remaining = length - offset;
        const uint8_t chunk = remaining < 32U
            ? static_cast<uint8_t>(remaining) : 32U;
        const bool last = offset + chunk == length;
        if (wire_.requestFrom(detectedAddress_, chunk, last) != chunk)
            return false;
        for (uint8_t i = 0; i < chunk; ++i)
        {
            if (wire_.available() <= 0) return false;
            destination[offset + i] = static_cast<uint8_t>(wire_.read());
        }
        offset += chunk;
    }
    return true;
}

bool Tof8x8::available() const { return available_; }
bool Tof8x8::lastReadSucceeded() const { return lastReadSucceeded_; }
uint32_t Tof8x8::frameNumber() const { return frameNumber_; }
uint32_t Tof8x8::lastFrameMs() const { return lastFrameMs_; }
uint8_t Tof8x8::detectedAddress() const { return detectedAddress_; }
uint8_t Tof8x8::addressAckMask() const { return addressAckMask_; }
bool Tof8x8::address52Acknowledged() const { return address52Acknowledged_; }

bool Tof8x8::addressAcknowledged(uint8_t address) const
{
    return address < 128 && (i2cAckBits_[address >> 3] & (1U << (address & 7))) != 0;
}

uint16_t Tof8x8::distanceMm(uint8_t row, uint8_t column) const
{
    if (row >= HEIGHT || column >= WIDTH)
        return 0;
    return frame_[row * WIDTH + column];
}

const uint16_t* Tof8x8::data() const { return frame_; }
