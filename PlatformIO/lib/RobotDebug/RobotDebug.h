#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

class RobotDebug {
public:
    using MessageHandler = void (*)(JsonDocument& message, void* context);

    RobotDebug(Stream& stream, MessageHandler handler, void* context = nullptr);
    void update();
    void log(const char* level, const char* message);
    void error(const char* message);
    void state(bool debugMode, bool stopped, bool fault = false);
    void parameterValue(const char* name, float value);
    void telemetry(const char* name, float value);
    bool send(JsonDocument& message, size_t reserveBytes = 0);
    uint32_t droppedMessages() const { return droppedMessages_; }
    uint32_t droppedBytes() const { return droppedBytes_; }

private:
    // Includes the combined encoder, TOF, motor, navigation and IMU frame.
    // Normal telemetry now includes navigation phase/heading and can exceed
    // the former 3072-byte allocation once encoder counts gain more digits.
    static constexpr size_t TX_BUFFER_SIZE = 6144;
    Stream& stream_;
    MessageHandler handler_;
    void* context_;
    char buffer_[768] = {};
    char txBuffer_[TX_BUFFER_SIZE] = {};
    size_t length_ = 0;
    bool droppingLine_ = false;
    uint32_t droppedMessages_ = 0;
    uint32_t droppedBytes_ = 0;

    void finishLine();
};
