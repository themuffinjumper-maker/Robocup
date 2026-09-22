#include "RobotDebug.h"

RobotDebug::RobotDebug(Stream& stream, MessageHandler handler, void* context)
    : stream_(stream), handler_(handler), context_(context)
{
}

void RobotDebug::update()
{
    while (stream_.available() > 0)
    {
        const char c = static_cast<char>(stream_.read());
        if (c == '\n')
        {
            finishLine();
            continue;
        }
        if (c == '\r' || droppingLine_)
            continue;
        if (length_ >= sizeof(buffer_) - 1)
        {
            length_ = 0;
            droppingLine_ = true;
            continue;
        }
        buffer_[length_++] = c;
    }
}

void RobotDebug::finishLine()
{
    if (droppingLine_)
    {
        droppingLine_ = false;
        error("Incoming message exceeded 767 bytes");
        return;
    }
    if (length_ == 0)
        return;

    buffer_[length_] = '\0';
    JsonDocument message;
    const DeserializationError result = deserializeJson(message, buffer_);
    length_ = 0;
    if (result)
    {
        error("Malformed JSON message");
        return;
    }
    if (!message.is<JsonObject>())
    {
        error("Message must be a JSON object");
        return;
    }
    if (handler_ != nullptr)
        handler_(message, context_);
}

bool RobotDebug::send(JsonDocument& message, size_t reserveBytes)
{
    const size_t required = measureJson(message);
    if (required >= sizeof(txBuffer_))
    {
        ++droppedMessages_;
        droppedBytes_ += required + 1;
        return false;
    }

    const size_t length = serializeJson(message, txBuffer_, sizeof(txBuffer_));
    // Never begin a line unless the complete JSON, newline, and requested
    // command-response reserve fit in the UART ring. HardwareSerial::write()
    // may otherwise accept a prefix, and the next send then joins two partial
    // documents into one corrupt line. Dropping one complete bulk frame is far
    // safer than wedging the CH9143 and preserves room for Stop/parameter ACKs.
    const size_t requiredSpace = length + 1 + reserveBytes;
    if (stream_.availableForWrite() < static_cast<int>(requiredSpace))
    {
        ++droppedMessages_;
        droppedBytes_ += length + 1;
        return false;
    }
    const size_t written = stream_.write(
        reinterpret_cast<const uint8_t*>(txBuffer_), length);
    if (written != length)
    {
        ++droppedMessages_;
        droppedBytes_ += length + 1 - written;
        return false;
    }
    stream_.write('\n');
    return true;
}

void RobotDebug::log(const char* level, const char* text)
{
    JsonDocument message;
    message["type"] = "log";
    message["level"] = level;
    message["message"] = text;
    send(message);
}

void RobotDebug::error(const char* text)
{
    JsonDocument message;
    message["type"] = "error";
    message["message"] = text;
    send(message);
}

void RobotDebug::state(bool debugMode, bool stopped, bool fault)
{
    JsonDocument message;
    message["type"] = "state";
    message["debug_mode"] = debugMode;
    message["stopped"] = stopped;
    message["fault"] = fault;
    message["uptime_ms"] = millis();
    send(message);
}

void RobotDebug::parameterValue(const char* name, float value)
{
    JsonDocument message;
    message["type"] = "parameter_value";
    message["name"] = name;
    message["value"] = value;
    send(message);
}

void RobotDebug::telemetry(const char* name, float value)
{
    JsonDocument message;
    message["type"] = "telemetry";
    message["time"] = millis();
    message["name"] = name;
    message["value"] = value;
    send(message);
}
