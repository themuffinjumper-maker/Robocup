#include "BluetoothDebugWorkflow.h"

#include <config.h>
#include <debug_config.generated.h>

namespace {
// BNO055 does not provide absolute velocity. Detect the beginning of a real
// chassis-motion event from linear acceleration at the 50 ms IMU sample rate,
// then expose an event counter so the desktop map cannot miss a short launch
// acceleration between slower telemetry frames.
bool imuMotionActive = false;
uint8_t imuMotionHighSamples = 0;
uint8_t imuMotionLowSamples = 0;
uint32_t imuMotionEventCount = 0;
}

BluetoothDebugWorkflow::BluetoothDebugWorkflow(
    HardwareSerialIMXRT& bluetoothPort,
    HardwareSerial& herkulexPort
)
    : bluetoothPort_(bluetoothPort),
      servos_(herkulexPort, HerkulexConfig::BAUD),
      dcMotor203_(Pins::DC_MOTOR_203_CHANNEL_A, Pins::DC_MOTOR_203_CHANNEL_B),
      dcMotor203Second_(Pins::DC_MOTOR_203_SECOND_CHANNEL_A, Pins::DC_MOTOR_203_SECOND_CHANNEL_B),
      encoders_(Pins::ENCODER_1_A, Pins::ENCODER_1_B, Pins::ENCODER_2_A, Pins::ENCODER_2_B),
      hx12kA_(Pins::HX12K_OUTPUT_A),
      hx12kB_(Pins::HX12K_OUTPUT_B),
      hx12kC_(Pins::HX12K_OUTPUT_C),
      hx12kD_(Pins::HX12K_OUTPUT_D),
      link_(bluetoothPort, dispatch, this),
      tof8x8_(Tof8x8Config::I2C_ADDRESS, Wire1)
{
}

void BluetoothDebugWorkflow::begin()
{
    Serial.begin(115200);
    delay(1500);
    Serial.println("BOOT: workflow begin");
    // Teensy's default hardware-serial RX storage is too small for the longer
    // JSON motion commands when telemetry transmission temporarily delays the
    // parser. Extra storage prevents complete newline-terminated commands from
    // being truncated before link_.update() can consume them.
    bluetoothPort_.addMemoryForRead(bluetoothRxBuffer_, sizeof(bluetoothRxBuffer_));
    // Large telemetry and matrix lines must be queued without blocking the
    // navigation loop while the 115200-baud UART physically shifts them out.
    bluetoothPort_.addMemoryForWrite(bluetoothTxBuffer_, sizeof(bluetoothTxBuffer_));
    bluetoothPort_.begin(BluetoothConfig::BAUD);
    Serial.println("BOOT: Serial1 ready");
    servos_.begin();
    servos_.torqueOff(0xFE);
    Serial.println("BOOT: servo buses ready");
    dcMotor203_.begin();
    dcMotor203Second_.begin();
    encoders_.begin();
    hx12kA_.begin();
    hx12kB_.begin();
    hx12kC_.begin();
    hx12kD_.begin();
    const DeserializationError error = deserializeJson(config_, EmbeddedDebugConfig::JSON);
    if (error)
    {
        link_.error("Embedded debug_config.json is invalid");
        return;
    }

    Serial.println("BOOT: config parsed");

    initialiseDigitalInputs();
    initialiseUltrasoundSensors();
    Serial.println("BOOT: digital sensors ready");
    initialiseTofSensors();
    Serial.println("BOOT: point TOF ready");
    const bool tof8x8Ready = tof8x8_.begin();
    Serial.println("BOOT: matrix TOF begin returned");
    const bool imuReady = imu_.begin();
    Serial.println("BOOT: IMU begin returned");

    JsonObject interval = findParameter("debug.telemetry_interval_ms");
    if (!interval.isNull())
        telemetryIntervalMs_ = constrain(interval["value"] | 1000UL, 750UL, 2000UL);

    delay(100);
    link_.log("INFO", "Teensy Bluetooth workflow ready");
    link_.log(tof8x8Ready ? "INFO" : "WARNING",
              tof8x8Ready ? "SEN0628 8x8 TOF ready" : "SEN0628 8x8 TOF not detected");
    link_.log(imuReady ? "INFO" : "WARNING",
              imuReady ? "BNO055 IMU ready" : "BNO055 IMU not detected on I2C0/I2C1");
    sendState();
    Serial.println("BOOT: workflow ready");
}

void BluetoothDebugWorkflow::update()
{
    static uint32_t lastUsbHeartbeatMs = 0;
    if (millis() - lastUsbHeartbeatMs >= 1000U)
    {
        lastUsbHeartbeatMs = millis();
        Serial.printf("LOOP: uptime=%lu tx_free=%d dropped=%lu\n",
                      millis(), bluetoothPort_.availableForWrite(),
                      link_.droppedMessages());
    }
    const uint32_t updateStartedUs = micros();
    if (lastUpdateStartedUs_ != 0)
        maxUpdateGapUs_ = max(maxUpdateGapUs_, updateStartedUs - lastUpdateStartedUs_);
    lastUpdateStartedUs_ = updateStartedUs;
    link_.update();
    const uint32_t now = millis();
    for (uint8_t i = 0; i < digitalInputCount_; ++i)
        digitalInputs_[i].update(now);
    if (dcMotor203SecondDeadman_ && dcMotor203SecondActive_ &&
        millis() - lastDcMotor203SecondCommandMs_ > KEYBOARD_DRIVE_TIMEOUT_MS)
    {
        dcMotor203Second_.stop();
        dcMotor203SecondActive_ = false;
        dcMotor203SecondDeadman_ = false;
        link_.log("WARNING", "Keyboard drive heartbeat lost; motor bank 2 stopped");
    }
    updateAutomaticServoRead();
    updateTof8x8();
    updateMotionAndRangeSensors();
    // Make the safety decision before potentially lengthy JSON transmission.
    updateNavigation();
    sendTelemetry();
    // Start/sample echo after the potentially slower I2C/telemetry work so a
    // fresh trigger cannot be hidden inside those operations.
    updateUltrasoundSensors();
}

void BluetoothDebugWorkflow::dispatch(JsonDocument& message, void* context)
{
    static_cast<BluetoothDebugWorkflow*>(context)->handleMessage(message);
}

void BluetoothDebugWorkflow::handleMessage(JsonDocument& message)
{
    ++receivedMessages_;
    const char* type = message["type"] | "";

    if (strcmp(type, "hello") == 0)
    {
        link_.log("INFO", "Robot Debug GUI connected");
        sendState();
        sendDefinitions();
    }
    else if (strcmp(type, "request_definitions") == 0)
    {
        if (millis() - lastDefinitionsMs_ >= 500)
            sendDefinitions();
    }
    else if (strcmp(type, "command") == 0)
    {
        handleCommand(message);
    }
    else if (strcmp(type, "drive") == 0)
    {
        // Compact, heartbeat-driven packet used by desktop arrow-key drive.
        // Keeping this packet short makes it much less vulnerable to losses
        // on the half-duplex CH9143 Bluetooth bridge than a full command JSON.
        if (!debugMode_ || stopped_)
        {
            link_.error("Keyboard drive requires Debug Mode and Run state");
            return;
        }
        if (!message["a"].is<int>() || !message["b"].is<int>())
        {
            link_.error("Keyboard drive requires integer a and b fields");
            return;
        }
        const int channelA = message["a"].as<int>();
        const int channelB = message["b"].as<int>();
        if (channelA < -100 || channelA > 100 ||
            channelB < -100 || channelB > 100)
        {
            link_.error("Keyboard drive values must be from -100 to 100");
            return;
        }
        const bool changed = channelA != dcMotor203Second_.channelAPercent() ||
                             channelB != dcMotor203Second_.channelBPercent();
        dcMotor203Second_.setPercent(channelA, channelB);
        dcMotor203SecondActive_ = channelA != 0 || channelB != 0;
        dcMotor203SecondDeadman_ = true;
        lastDcMotor203SecondCommandMs_ = millis();
        if (changed)
            link_.log(dcMotor203SecondActive_ ? "WARNING" : "INFO",
                      dcMotor203SecondActive_ ? "Keyboard drive applied" :
                                                "Keyboard drive stopped");
    }
    else if (strcmp(type, "parameter") == 0)
    {
        handleParameter(message);
    }
    else if (strcmp(type, "parameter_request") == 0)
    {
        const char* name = message["name"] | "";
        JsonObject parameter = findParameter(name);
        if (parameter.isNull())
            link_.error("Unknown parameter");
        else
            link_.parameterValue(name, parameter["value"].as<float>());
    }
    else
    {
        link_.error("Unknown message type");
    }
}

void BluetoothDebugWorkflow::handleCommand(JsonDocument& message)
{
    const char* commandName = message["command"] | "";

    // This command is built into the desktop GUI, so it does not need to be
    // repeated in debug_config.json.
    if (strcmp(commandName, "set_debug_mode") == 0)
    {
        debugMode_ = message["enabled"] | false;
        if (!debugMode_)
        {
            stopNavigation();
            stopped_ = true;
            servos_.torqueOff(0xFE);
            dcMotor203_.stop();
            dcMotor203Active_ = false;
            dcMotor203Second_.stop();
            dcMotor203SecondActive_ = false;
            dcMotor203SecondDeadman_ = false;
            hx12kA_.disable();
            hx12kB_.disable();
            hx12kC_.disable();
            hx12kD_.disable();
        }
        sendState();
        link_.log("INFO", debugMode_ ? "Debug mode enabled" : "Debug mode disabled; torque off");
        return;
    }

    // Mission-plan upload is a built-in desktop/firmware protocol rather
    // than a debug_config command. This keeps large editable layouts out of
    // the generated command-definition table while still enforcing Debug Mode.
    if (strcmp(commandName, "mission_plan_clear") == 0)
    {
        if (!debugMode_)
        {
            link_.error("Mission plan requires Debug Mode");
            return;
        }
        if (navigationActive_)
        {
            link_.error("Stop autonomous navigation before replacing the mission plan");
            return;
        }
        missionWaypointCount_ = 0;
        missionWaypointIndex_ = 0;
        missionTargetCount_ = 0;
        missionTargetsVisited_ = 0;
        missionPlanValid_ = false;
        missionPoseInitialised_ = false;
        link_.log("INFO", "Pre-laid mission plan cleared");
        return;
    }
    if (strcmp(commandName, "mission_plan_set") == 0)
    {
        if (!debugMode_)
        {
            link_.error("Mission plan requires Debug Mode");
            return;
        }
        if (navigationActive_)
        {
            link_.error("Stop autonomous navigation before replacing the mission plan");
            return;
        }
        if (!message["points"].is<JsonArray>())
        {
            link_.error("Mission plan requires a flat points array");
            return;
        }
        JsonArray points = message["points"].as<JsonArray>();
        if (points.size() == 0 || points.size() % 3 != 0 ||
            points.size() / 3 > MAX_MISSION_WAYPOINTS)
        {
            link_.error("Mission plan must contain 1..64 x,y,flags waypoint triples");
            return;
        }

        const int startX = message["start_x_mm"] | -1;
        const int startY = message["start_y_mm"] | -1;
        const float startHeading = message["start_heading_deg"] | 0.0f;
        const int fallback = message["fallback_strategy"] | 0;
        if (startX < 0 || startX > 12000 || startY < 0 || startY > 12000 ||
            fallback < 0 || fallback > 2)
        {
            link_.error("Mission start/fallback values are outside the supported range");
            return;
        }

        uint8_t count = 0;
        uint8_t targets = 0;
        for (size_t i = 0; i < points.size(); i += 3)
        {
            const int x = points[i].as<int>();
            const int y = points[i + 1].as<int>();
            const int flags = points[i + 2].as<int>();
            if (x < 0 || x > 12000 || y < 0 || y > 12000 || flags < 0 || flags > 255)
            {
                link_.error("Mission waypoint is outside the supported range");
                missionPlanValid_ = false;
                return;
            }
            missionWaypoints_[count].xMm = static_cast<int16_t>(x);
            missionWaypoints_[count].yMm = static_cast<int16_t>(y);
            missionWaypoints_[count].flags = static_cast<uint8_t>(flags);
            if ((flags & 0x01) != 0)
                ++targets;
            ++count;
        }

        missionStartXmm_ = static_cast<float>(startX);
        missionStartYmm_ = static_cast<float>(startY);
        missionStartArenaHeadingDeg_ = startHeading;
        while (missionStartArenaHeadingDeg_ >= 360.0f) missionStartArenaHeadingDeg_ -= 360.0f;
        while (missionStartArenaHeadingDeg_ < 0.0f) missionStartArenaHeadingDeg_ += 360.0f;
        missionFallbackStrategy_ = static_cast<uint8_t>(fallback);
        missionWaypointCount_ = count;
        missionWaypointIndex_ = 0;
        missionTargetCount_ = targets;
        missionTargetsVisited_ = 0;
        missionPlanValid_ = count > 0 && targets > 0;
        missionPoseInitialised_ = false;
        missionBlockedSinceMs_ = 0;
        link_.log(missionPlanValid_ ? "INFO" : "WARNING",
                  missionPlanValid_
                      ? "Pre-laid mission plan loaded"
                      : "Mission plan loaded without any real-weight targets");
        return;
    }

    JsonObject definition = findCommand(commandName);
    if (definition.isNull())
    {
        link_.error("Unknown command");
        return;
    }

    if ((definition["debug_mode_required"] | false) && !debugMode_)
    {
        link_.error("Command requires Debug Mode");
        return;
    }

    const char* action = definition["action"] | "";
    if (strcmp(action, "ping") == 0)
    {
        link_.log("INFO", "Pong from Teensy");
        sendState();
    }
    else if (strcmp(action, "stop") == 0)
    {
        stopNavigation();
        servos_.torqueOff(0xFE);
        continuousVelocityActive_ = false;
        commandedVelocity_ = 0;
        dcMotor203_.stop();
        dcMotor203Active_ = false;
        dcMotor203Second_.stop();
        dcMotor203SecondActive_ = false;
        dcMotor203SecondDeadman_ = false;
        hx12kA_.disable();
        hx12kB_.disable();
        hx12kC_.disable();
        hx12kD_.disable();
        stopped_ = true;
        sendState();
        link_.log("WARNING", "STOP received; servo torque disabled");
    }
    else if (strcmp(action, "run") == 0)
    {
        stopped_ = false;
        trackingFault_ = false;
        consecutiveTrackingErrors_ = 0;
        sendState();
        link_.log("INFO", "Run mode enabled; servo remains still until commanded");
    }
    else if (strcmp(action, "herkulex_angle") == 0)
    {
        if (!message["id"].is<int>() ||
            (!message["angle_deg"].is<float>() && !message["angle_deg"].is<int>()) ||
            !message["move_time_ms"].is<int>())
        {
            link_.error("Servo command requires id, angle_deg, and move_time_ms");
            return;
        }

        const int id = message["id"].as<int>();
        const float angle = message["angle_deg"].as<float>();
        const int moveTime = message["move_time_ms"].as<int>();
        if (id < 1 || id > 253 ||
            angle < HerkulexConfig::MIN_ANGLE_DEG || angle > HerkulexConfig::MAX_ANGLE_DEG ||
            moveTime < 50 || moveTime > 2850)
        {
            link_.error("Servo command is outside its safe range");
            return;
        }

        if (!servoZeroed_[id])
        {
            link_.error("Servo must be zeroed before it can move");
            return;
        }

        if (stopped_)
        {
            link_.error("Robot is stopped; press Run Robot before moving");
            return;
        }

        const float absoluteAngle = servoZeroOffsetsDeg_[id] + angle;
        if (absoluteAngle < HerkulexConfig::MIN_ANGLE_DEG ||
            absoluteAngle > HerkulexConfig::MAX_ANGLE_DEG)
        {
            link_.error("Relative target exceeds the servo's absolute range");
            return;
        }

        lastServoId_ = static_cast<uint8_t>(id);
        lastServoAngleDeg_ = angle;
        lastMoveStartMs_ = millis();
        lastMoveDurationMs_ = static_cast<uint16_t>(moveTime);
        trackingFault_ = false;
        consecutiveTrackingErrors_ = 0;
        // The DRS-0101 must be torque-free while changing from continuous-turn
        // mode back to position mode. The first JOG selects the mode; the
        // second executes after torque is restored.
        servos_.torqueOff(lastServoId_);
        delay(5);
        servos_.clearError(lastServoId_);
        servos_.moveAngle(lastServoId_, absoluteAngle, static_cast<uint16_t>(moveTime), HerkulexTeensy::LED_BLUE);
        delay(5);
        servos_.torqueOn(lastServoId_);
        delay(5);
        servos_.moveAngle(lastServoId_, absoluteAngle, static_cast<uint16_t>(moveTime), HerkulexTeensy::LED_BLUE);
        continuousVelocityActive_ = false;
        commandedVelocity_ = 0;
        stopped_ = false;
        sendState();
        link_.log("INFO", "Herkulex angle command sent");
    }
    else if (strcmp(action, "herkulex_read_angle") == 0)
    {
        const int id = message["id"] | -1;
        if (id < 1 || id > 253)
        {
            link_.error("Read angle requires a servo id from 1 to 253");
            return;
        }

        const float angle = servos_.readAngle(static_cast<uint8_t>(id), 30);
        if (isnan(angle))
        {
            link_.error("No valid angle response from Herkulex servo");
            return;
        }

        lastServoId_ = static_cast<uint8_t>(id);
        measuredServoAngleDeg_ = servoZeroed_[id]
            ? angle - servoZeroOffsetsDeg_[id]
            : angle;
        autoReadServoId_ = static_cast<uint8_t>(id);
        autoReadEnabled_ = true;
        servoResponding_ = true;
        consecutiveReadFailures_ = 0;
        link_.telemetry("servo.measured_angle_deg", measuredServoAngleDeg_);
        link_.log("INFO", "Herkulex angle read successfully");
    }
    else if (strcmp(action, "herkulex_zero_here") == 0)
    {
        const int id = message["id"] | -1;
        if (id < 1 || id > 253)
        {
            link_.error("Zero command requires a servo id from 1 to 253");
            return;
        }
        if (continuousVelocityActive_ && id == lastServoId_)
        {
            link_.error("Stop continuous rotation before setting zero");
            return;
        }

        // Reading does not enable torque or move the mechanism.
        const float currentAngle = servos_.readAngle(static_cast<uint8_t>(id), 30);
        if (isnan(currentAngle))
        {
            link_.error("Cannot zero: no valid response from Herkulex servo");
            return;
        }

        servoZeroOffsetsDeg_[id] = currentAngle;
        servoZeroed_[id] = true;
        lastServoId_ = static_cast<uint8_t>(id);
        lastServoAngleDeg_ = 0.0f;
        measuredServoAngleDeg_ = 0.0f;
        link_.telemetry("servo.measured_angle_deg", 0.0f);
        link_.telemetry("servo.zero_offset_deg", currentAngle);
        link_.log("INFO", "Current Herkulex position stored as software zero");
    }
    else if (strcmp(action, "herkulex_velocity") == 0)
    {
        if (stopped_)
        {
            link_.error("Robot is stopped; press Run Robot before turning");
            return;
        }

        const int id = message["id"] | -1;
        const int speed = message["speed"] | 0;
        if (id < 1 || id > 253 || speed < -1023 || speed > 1023)
        {
            link_.error("Velocity command requires id 1..253 and speed -1023..1023");
            return;
        }

        lastServoId_ = static_cast<uint8_t>(id);
        commandedVelocity_ = static_cast<int16_t>(speed);
        servos_.torqueOff(lastServoId_);
        delay(5);
        servos_.clearError(lastServoId_);
        servos_.moveVelocity(lastServoId_, 0, 0, HerkulexTeensy::LED_GREEN);
        delay(5);
        servos_.torqueOn(lastServoId_);
        delay(5);
        servos_.moveVelocity(lastServoId_, commandedVelocity_, 0, HerkulexTeensy::LED_GREEN);
        continuousVelocityActive_ = commandedVelocity_ != 0;
        trackingFault_ = false;
        consecutiveTrackingErrors_ = 0;
        sendState();
        link_.log("WARNING", continuousVelocityActive_
            ? "Herkulex continuous rotation started"
            : "Herkulex continuous velocity set to zero");
    }
    else if (strcmp(action, "herkulex_stop_velocity") == 0)
    {
        const int id = message["id"] | -1;
        if (id < 1 || id > 253)
        {
            link_.error("Stop turn requires a servo id from 1 to 253");
            return;
        }

        servos_.moveVelocity(static_cast<uint8_t>(id), 0);
        delay(5);
        servos_.torqueOff(static_cast<uint8_t>(id));
        if (id == lastServoId_)
        {
            continuousVelocityActive_ = false;
            commandedVelocity_ = 0;
        }
        link_.log("INFO", "Herkulex continuous rotation stopped; torque off");
    }
    else if (strcmp(action, "dc_motor_203_speed") == 0)
    {
        if (stopped_)
        {
            link_.error("Robot is stopped; press Run Robot before driving motor");
            return;
        }

        if (navigationActive_)
            stopNavigation("Navigation stopped by manual motor command");

        const int channelA = message["channel_a_percent"] | 0;
        const int channelB = message["channel_b_percent"] | 0;
        if (channelA < -100 || channelA > 100 || channelB < -100 || channelB > 100)
        {
            link_.error("Both 203 DC motor speeds must be from -100 to 100 percent");
            return;
        }

        dcMotor203_.setPercent(static_cast<int16_t>(channelA), static_cast<int16_t>(channelB));
        dcMotor203Active_ = channelA != 0 || channelB != 0;
        link_.log(dcMotor203Active_ ? "WARNING" : "INFO",
                  dcMotor203Active_ ? "203 DC motor command applied" : "203 DC motor stopped");
    }
    else if (strcmp(action, "dc_motor_203_stop") == 0)
    {
        dcMotor203_.stop();
        dcMotor203Active_ = false;
        link_.log("INFO", "203 DC motor stopped at neutral pulse");
    }
    else if (strcmp(action, "dc_motor_203_second_speed") == 0)
    {
        if (stopped_)
        {
            link_.error("Robot is stopped; press Run Robot before driving motor");
            return;
        }

        if (navigationActive_)
            stopNavigation("Navigation stopped by manual motor command");

        const int channelA = message["channel_a_percent"] | 0;
        const int channelB = message["channel_b_percent"] | 0;
        if (channelA < -100 || channelA > 100 || channelB < -100 || channelB > 100)
        {
            link_.error("Both second 203 DC motor speeds must be from -100 to 100 percent");
            return;
        }

        const bool changed = channelA != dcMotor203Second_.channelAPercent() ||
                             channelB != dcMotor203Second_.channelBPercent();
        dcMotor203Second_.setPercent(static_cast<int16_t>(channelA), static_cast<int16_t>(channelB));
        dcMotor203SecondActive_ = channelA != 0 || channelB != 0;
        dcMotor203SecondDeadman_ = message["deadman"] | false;
        lastDcMotor203SecondCommandMs_ = millis();
        if (changed)
            link_.log(dcMotor203SecondActive_ ? "WARNING" : "INFO",
                      dcMotor203SecondActive_ ? "Second 203 DC motor command applied" : "Second 203 DC motor stopped");
    }
    else if (strcmp(action, "dc_motor_203_second_stop") == 0)
    {
        stopNavigation();
        dcMotor203Second_.stop();
        dcMotor203SecondActive_ = false;
        dcMotor203SecondDeadman_ = false;
        link_.log("INFO", "Second 203 DC motor stopped at neutral pulse");
    }
    else if (strcmp(action, "navigation_toggle") == 0)
    {
        if (!message["enabled"].is<bool>())
        {
            link_.error("Navigation command requires enabled=true or false");
            return;
        }
        if (!message["enabled"].as<bool>())
        {
            stopNavigation("Autonomous navigation stopped");
            return;
        }
        if (!debugMode_)
        {
            link_.error("Autonomous navigation requires Debug Mode");
            return;
        }
        if (navigationStrategy_ == 3 && !missionPlanValid_)
        {
            link_.error("Pre-laid mission strategy requires a planned route from Arena View");
            return;
        }
        navigationActive_ = true;
        navigationState_ = navigationStrategy_ == 3 ? NAV_MISSION_TRACK : NAV_SEEK_WALL;
        navigationMotionStartedMs_ = millis();
        navigationHeadingReferenceDeg_ = imu_.headingDeg();
        navigationTargetHeadingDeg_ = navigationHeadingReferenceDeg_;
        navigationLaneIndex_ = 0;
        navigationSweepTurnRight_ = true;
        navigationSweepLeftReferenceValid_ = false;
        navigationSweepRightReferenceValid_ = false;
        navigationSweepLateralErrorMm_ = 0;
        navigationSweepProgressMm_ = 0.0f;
        navigationExpectedSweepLengthMm_ = 0.0f;
        navigationExpectedSweepLengthValid_ = false;
        navigationSweepProgressLastEncoder1_ = encoders_.firstCount();
        navigationSweepProgressLastEncoder2_ = encoders_.secondCount();
        navigationDetourRight_ = true;
        navigationDetourOriginalHeadingDeg_ = navigationHeadingReferenceDeg_;
        navigationDetourOffsetMm_ = 0.0f;
        navigationDetourEdgeCleared_ = false;
        navigationDetourObstacleSeen_ = false;
        navigationDetourClearSamples_ = 0;
        navigationDetourLastTriggerCount_ = 0;
        navigationObstacleCount_ = 0;
        navigationRecoveryCount_ = 0;
        navigationRecoveryAttemptCount_ = 0;
        navigationRecoveryTotalReverseMm_ = 0.0f;
        navigationRecoveryForceHalfTurn_ = false;
        navigationClearanceTurnCount_ = 0;
        navigationMotionConsistent_ = true;
        missionWaypointIndex_ = 0;
        missionTargetsVisited_ = 0;
        missionPoseInitialised_ = false;
        missionBlockedSinceMs_ = 0;
        if (navigationStrategy_ == 3)
        {
            missionPoseXmm_ = missionStartXmm_;
            missionPoseYmm_ = missionStartYmm_;
            missionArenaHeadingDeg_ = missionStartArenaHeadingDeg_;
            // Alignment is captured only after updateNavigation sees a valid
            // BNO055 sample. Until then the mission holds the motors stopped.
            missionPoseInitialised_ = false;
        }
        stopped_ = false;
        dcMotor203SecondDeadman_ = false;
        sendState();
        link_.log("WARNING", "Autonomous navigation started");
    }
    else if (strcmp(action, "encoder_zero") == 0)
    {
        encoders_.zero();
        link_.log("INFO", "Both encoder counts zeroed");
    }
    else if (strcmp(action, "hx12k_angles") == 0)
    {
        if (stopped_)
        {
            link_.error("Robot is stopped; press Run Robot before moving HX12K servos");
            return;
        }

        const bool selectA = message["select_a"] | false;
        const bool selectB = message["select_b"] | false;
        const bool selectC = message["select_c"] | false;
        const bool selectD = message["select_d"] | false;
        if (!selectA && !selectB && !selectC && !selectD)
        {
            link_.error("Select at least one HX12K servo");
            return;
        }

        const float angleA = message["angle_a_deg"] | 67.5f;
        const float angleB = message["angle_b_deg"] | 67.5f;
        const float angleC = message["angle_c_deg"] | 67.5f;
        const float angleD = message["angle_d_deg"] | 67.5f;
        if (angleA < 0.0f || angleA > 135.0f ||
            angleB < 0.0f || angleB > 135.0f ||
            angleC < 0.0f || angleC > 135.0f ||
            angleD < 0.0f || angleD > 135.0f)
        {
            link_.error("Every HX12K angle must be from 0 to 135 degrees");
            return;
        }

        if (selectA) hx12kA_.setAngle(angleA);
        if (selectB) hx12kB_.setAngle(angleB);
        if (selectC) hx12kC_.setAngle(angleC);
        if (selectD) hx12kD_.setAngle(angleD);
        link_.log("INFO", "Selected HX12K angles applied");
        sendState();
    }
    else if (strcmp(action, "hx12k_disable") == 0)
    {
        hx12kA_.disable();
        hx12kB_.disable();
        hx12kC_.disable();
        hx12kD_.disable();
        link_.log("INFO", "All HX12K pulse outputs disabled");
    }
    else if (strcmp(action, "hx12k_bumpers") == 0)
    {
        if (stopped_)
        {
            link_.error("Robot is stopped; press Run Robot before moving bumper servos");
            return;
        }
        if (navigationActive_)
            stopNavigation("Navigation stopped by manual drive");
        if (!message["enabled"].is<bool>())
        {
            link_.error("Bumper servo command requires enabled=true or false");
            return;
        }

        const bool enabled = message["enabled"].as<bool>();
        hx12kC_.setAngle(enabled ? 0.0f : 130.0f);
        hx12kD_.setAngle(enabled ? 130.0f : 0.0f);
        link_.log("INFO", enabled ? "Bumper servos ON: C=0, D=130" :
                                  "Bumper servos OFF: C=130, D=0");
        sendState();
    }
    else if (strcmp(action, "tof_read_all") == 0)
    {
        lastTelemetryMs_ = millis() - telemetryIntervalMs_;
        sendTelemetry();
        link_.log("INFO", "Configured TOF sensors read");
    }
    else
    {
        link_.error("Command action has no firmware handler");
    }
}

void BluetoothDebugWorkflow::handleParameter(JsonDocument& message)
{
    const char* name = message["name"] | "";
    JsonObject parameter = findParameter(name);
    if (parameter.isNull() || message["value"].isNull())
    {
        link_.error("Unknown parameter or missing value");
        return;
    }

    const char* datatype = parameter["datatype"] | "float";
    if (strcmp(datatype, "int") == 0)
    {
        if (!message["value"].is<int>())
        {
            link_.error("Parameter requires an integer");
            return;
        }
        int value = message["value"].as<int>();
        value = constrain(value, parameter["min"] | value, parameter["max"] | value);
        parameter["value"] = value;
    }
    else
    {
        if (!message["value"].is<float>() && !message["value"].is<int>())
        {
            link_.error("Parameter requires a number");
            return;
        }
        float value = message["value"].as<float>();
        value = constrain(value, parameter["min"] | value, parameter["max"] | value);
        parameter["value"] = value;
    }

    if (strcmp(name, "debug.telemetry_interval_ms") == 0)
    {
        const uint32_t requested = parameter["value"].as<uint32_t>();
        telemetryIntervalMs_ = requested < 750U ? 750U : requested;
    }
    else if (strcmp(name, "navigation.strategy") == 0)
        navigationStrategy_ = parameter["value"].as<uint8_t>();
    else if (strcmp(name, "navigation.front_avoid_mm") == 0)
        navigationFrontAvoidMm_ = parameter["value"].as<uint16_t>();
    else if (strcmp(name, "navigation.side_avoid_mm") == 0)
        navigationSideAvoidMm_ = parameter["value"].as<uint16_t>();
    else if (strcmp(name, "navigation.wall_follow_mm") == 0)
        navigationWallFollowMm_ = parameter["value"].as<uint16_t>();
    else if (strcmp(name, "navigation.lane_spacing_mm") == 0)
        navigationLaneSpacingMm_ = parameter["value"].as<uint16_t>();
    else if (strcmp(name, "navigation.robot_width_mm") == 0)
        navigationRobotWidthMm_ = parameter["value"].as<uint16_t>();
    else if (strcmp(name, "navigation.gap_margin_mm") == 0)
        navigationGapMarginMm_ = parameter["value"].as<uint16_t>();
    else if (strcmp(name, "navigation.gap_depth_mm") == 0)
        navigationGapDepthMm_ = parameter["value"].as<uint16_t>();
    else if (strcmp(name, "navigation.matrix_floor_rows") == 0)
        navigationMatrixFloorRows_ = parameter["value"].as<uint8_t>();
    else if (strcmp(name, "navigation.matrix_fov_deg") == 0)
        navigationMatrixFovDeg_ = parameter["value"].as<uint8_t>();
    else if (strcmp(name, "navigation.gap_confirm_frames") == 0)
        navigationGapConfirmFrames_ = parameter["value"].as<uint8_t>();

    link_.parameterValue(name, parameter["value"].as<float>());
}

void BluetoothDebugWorkflow::sendDefinitions()
{
    for (JsonObject parameter : config_["parameters"].as<JsonArray>())
    {
        JsonDocument outgoing;
        outgoing.set(parameter);
        outgoing["type"] = "parameter_definition";
        link_.send(outgoing);
    }

    for (JsonObject command : config_["commands"].as<JsonArray>())
    {
        JsonDocument outgoing;
        outgoing.set(command);
        outgoing.remove("action");
        outgoing.remove("debug_mode_required");
        outgoing["type"] = "command_definition";
        link_.send(outgoing);
    }
    lastDefinitionsMs_ = millis();
}

void BluetoothDebugWorkflow::sendState()
{
    link_.state(debugMode_, stopped_, trackingFault_);
}

void BluetoothDebugWorkflow::updateAutomaticServoRead()
{
    constexpr uint32_t READ_INTERVAL_MS = 200;
    const uint32_t now = millis();
    if (!autoReadEnabled_ || now - lastServoReadMs_ < READ_INTERVAL_MS)
        return;
    lastServoReadMs_ = now;

    const float absoluteAngle = servos_.readAngle(autoReadServoId_, 30);
    if (isnan(absoluteAngle))
    {
        if (consecutiveReadFailures_ < 255)
            ++consecutiveReadFailures_;
        if (consecutiveReadFailures_ >= 3)
        {
            const bool wasResponding = servoResponding_;
            servoResponding_ = false;
            if (wasResponding)
            {
                trackingFault_ = true;
                sendState();
                link_.error("Herkulex position monitoring lost servo response");
            }
        }
        return;
    }

    servoResponding_ = true;
    consecutiveReadFailures_ = 0;
    measuredServoAngleDeg_ = servoZeroed_[autoReadServoId_]
        ? absoluteAngle - servoZeroOffsetsDeg_[autoReadServoId_]
        : absoluteAngle;
    updateTrackingState(absoluteAngle);
}

void BluetoothDebugWorkflow::updateTrackingState(float absoluteAngle)
{
    if (continuousVelocityActive_)
        return;
    if (autoReadServoId_ != lastServoId_ || !servoZeroed_[autoReadServoId_])
        return;

    const float measuredRelative = absoluteAngle - servoZeroOffsetsDeg_[autoReadServoId_];
    servoPositionErrorDeg_ = lastServoAngleDeg_ - measuredRelative;

    // Do not judge tracking while the requested movement should still be in progress.
    if (millis() - lastMoveStartMs_ <= static_cast<uint32_t>(lastMoveDurationMs_) + 300U)
        return;

    constexpr float MAX_TRACKING_ERROR_DEG = 8.0f;
    if (!stopped_ && fabsf(servoPositionErrorDeg_) > MAX_TRACKING_ERROR_DEG)
    {
        if (consecutiveTrackingErrors_ < 255)
            ++consecutiveTrackingErrors_;
    }
    else
    {
        consecutiveTrackingErrors_ = 0;
    }

    const bool newFault = consecutiveTrackingErrors_ >= 3;
    if (newFault != trackingFault_)
    {
        trackingFault_ = newFault;
        sendState();
        link_.log(newFault ? "ERROR" : "INFO",
                  newFault ? "Herkulex tracking error exceeds 8 degrees" : "Herkulex tracking recovered");
    }
}

void BluetoothDebugWorkflow::sendTelemetry()
{
    const uint32_t now = millis();
    if (now - lastTelemetryMs_ < telemetryIntervalMs_)
        return;
    lastTelemetryMs_ = now;

    JsonDocument message;
    message["type"] = "telemetry";
    message["time"] = now;
    JsonObject data = message["data"].to<JsonObject>();
    data["system.uptime_s"] = now / 1000.0f;
    data["system.debug_mode"] = debugMode_;
    data["system.stopped"] = stopped_;
    data["system.navigation_controller_version"] = 9;
    data["system.max_loop_gap_ms"] = maxUpdateGapUs_ / 1000.0f;
    data["bluetooth.messages_received"] = receivedMessages_;
    data["bluetooth.tx_dropped_messages"] = link_.droppedMessages();
    data["bluetooth.tx_dropped_bytes"] = link_.droppedBytes();
    data["servo.last_id"] = lastServoId_;
    data["servo.commanded_angle_deg"] = lastServoAngleDeg_;
    data["servo.zeroed"] = servoZeroed_[lastServoId_];
    data["servo.auto_read_enabled"] = autoReadEnabled_;
    data["servo.responding"] = servoResponding_;
    data["servo.tracking_fault"] = trackingFault_;
    data["servo.continuous_velocity_active"] = continuousVelocityActive_;
    data["servo.commanded_velocity"] = commandedVelocity_;
    data["dc_motor_203.active"] = dcMotor203Active_;
    data["dc_motor_203.channel_a_percent"] = dcMotor203_.channelAPercent();
    data["dc_motor_203.channel_b_percent"] = dcMotor203_.channelBPercent();
    data["dc_motor_203.channel_a_pulse_us"] = dcMotor203_.channelAPulseUs();
    data["dc_motor_203.channel_b_pulse_us"] = dcMotor203_.channelBPulseUs();
    data["dc_motor_203_second.active"] = dcMotor203SecondActive_;
    data["dc_motor_203_second.channel_a_percent"] = dcMotor203Second_.channelAPercent();
    data["dc_motor_203_second.channel_b_percent"] = dcMotor203Second_.channelBPercent();
    data["dc_motor_203_second.channel_a_pulse_us"] = dcMotor203Second_.channelAPulseUs();
    data["dc_motor_203_second.channel_b_pulse_us"] = dcMotor203Second_.channelBPulseUs();
    data["navigation.active"] = navigationActive_;
    data["navigation.state"] = navigationState_;
    data["navigation.front_mm"] = navigationFrontMm_;
    data["navigation.left_mm"] = navigationLeftMm_;
    data["navigation.right_mm"] = navigationRightMm_;
    const char* navigationPhase = "idle";
    switch (navigationState_)
    {
        case NAV_SEEK_WALL: navigationPhase = "seek_wall"; break;
        case NAV_INITIAL_TURN: navigationPhase = "initial_turn"; break;
        case NAV_FOLLOW_WALL: navigationPhase = "follow_wall"; break;
        case NAV_CORNER_TURN: navigationPhase = "corner_turn"; break;
        case NAV_SWEEP: navigationPhase = "sweep"; break;
        case NAV_LANE_TURN_OUT: navigationPhase = "lane_turn_out"; break;
        case NAV_LANE_SHIFT: navigationPhase = "lane_shift"; break;
        case NAV_LANE_TURN_IN: navigationPhase = "lane_turn_in"; break;
        case NAV_COMPLETE: navigationPhase = "complete"; break;
        case NAV_OBSTACLE_TURN_OUT: navigationPhase = "obstacle_turn_out"; break;
        case NAV_OBSTACLE_OFFSET: navigationPhase = "obstacle_offset"; break;
        case NAV_OBSTACLE_TURN_FORWARD: navigationPhase = "obstacle_turn_forward"; break;
        case NAV_OBSTACLE_PASS: navigationPhase = "obstacle_pass"; break;
        case NAV_OBSTACLE_TURN_BACK: navigationPhase = "obstacle_turn_back"; break;
        case NAV_OBSTACLE_RETURN: navigationPhase = "obstacle_return"; break;
        case NAV_OBSTACLE_TURN_IN: navigationPhase = "obstacle_turn_in"; break;
        case NAV_ESCAPE_REVERSE: navigationPhase = "escape_reverse"; break;
        case NAV_ESCAPE_TURN: navigationPhase = "escape_turn"; break;
        case NAV_CLEARANCE_TURN: navigationPhase = "clearance_turn"; break;
        case NAV_RECOVERY_REVERSE: navigationPhase = "recovery_reverse"; break;
        case NAV_RECOVERY_TURN: navigationPhase = "recovery_turn"; break;
        case NAV_MISSION_TRACK: navigationPhase = "mission_track"; break;
        default: break;
    }
    data["navigation.phase"] = navigationPhase;
    data["navigation.lane"] = navigationLaneIndex_;
    data["navigation.target_heading_deg"] = navigationTargetHeadingDeg_;
    data["navigation.motion_consistent"] = navigationMotionConsistent_;
    data["navigation.matrix_close_zones"] = navigationMatrixCloseZones_;
    data["navigation.matrix_usable_zones"] = navigationMatrixUsableZones_;
    data["navigation.matrix_broad_wall"] = navigationMatrixBroadWall_;
    if (navigationState_ == NAV_SWEEP)
    {
        const bool remainingSideValid = navigationSweepTurnRight_
            ? navigationSweepRightReferenceValid_
            : navigationSweepLeftReferenceValid_;
        if (remainingSideValid)
            data["navigation.remaining_width_mm"] = navigationSweepTurnRight_
                ? navigationSweepRightReferenceMm_
                : navigationSweepLeftReferenceMm_;
    }
    data["navigation.next_shift_mm"] = navigationSweepLateralErrorMm_ > 0
        ? navigationSweepLateralErrorMm_ : 0;
    data["navigation.sweep_progress_mm"] = navigationSweepProgressMm_;
    data["navigation.expected_sweep_length_mm"] =
        navigationExpectedSweepLengthValid_ ? navigationExpectedSweepLengthMm_ : 0.0f;
    const bool detourActive = navigationState_ >= NAV_OBSTACLE_TURN_OUT &&
        navigationState_ <= NAV_OBSTACLE_TURN_IN;
    data["navigation.detour_active"] = detourActive;
    data["navigation.detour_side"] = detourActive ?
        (navigationDetourRight_ ? 1 : -1) : 0;
    data["navigation.obstacle_count"] = navigationObstacleCount_;
    data["navigation.detour_offset_mm"] = navigationDetourOffsetMm_;
    data["navigation.recovery_count"] = navigationRecoveryCount_;
    data["navigation.recovery_logic_version"] = 2;
    data["navigation.recovery_attempt"] = navigationRecoveryAttemptCount_;
    data["navigation.recovery_total_reverse_mm"] = navigationRecoveryTotalReverseMm_;
    data["navigation.recovery_force_half_turn"] = navigationRecoveryForceHalfTurn_;
    data["navigation.clearance_turn_count"] = navigationClearanceTurnCount_;
    data["navigation.strategy"] = navigationStrategy_;
    data["navigation.gap_passable"] = navigationGapPassable_;
    data["navigation.gap_width_mm"] = navigationGapWidthMm_;
    data["navigation.gap_centre_column_x2"] = navigationGapCentreColumnX2_;
    data["navigation.matrix_floor_rows"] = navigationMatrixFloorRows_;
    data["mission.plan_valid"] = missionPlanValid_;
    data["mission.active"] = navigationActive_ && navigationStrategy_ == 3;
    data["mission.waypoint_index"] = missionWaypointIndex_;
    data["mission.waypoint_count"] = missionWaypointCount_;
    data["mission.targets_visited"] = missionTargetsVisited_;
    data["mission.target_count"] = missionTargetCount_;
    data["mission.pose_x_mm"] = missionPoseXmm_;
    data["mission.pose_y_mm"] = missionPoseYmm_;
    data["mission.heading_deg"] = missionArenaHeadingDeg_;
    data["mission.blocked"] = missionBlockedSinceMs_ != 0;
    data["encoder.1.count"] = encoders_.firstCount();
    data["encoder.1.delta"] = encoders_.firstDelta();
    data["encoder.1.counts_per_s"] = encoders_.firstCountsPerSecond();
    data["encoder.2.count"] = encoders_.secondCount();
    data["encoder.2.delta"] = encoders_.secondDelta();
    data["encoder.2.counts_per_s"] = encoders_.secondCountsPerSecond();
    for (uint8_t i = 0; i < digitalInputCount_; ++i)
    {
        char key[64];
        snprintf(key, sizeof(key), "digital.%s.detected", digitalInputNames_[i]);
        data[key] = digitalInputs_[i].detected();
        snprintf(key, sizeof(key), "digital.%s.raw_high", digitalInputNames_[i]);
        data[key] = digitalInputs_[i].rawHigh();
        snprintf(key, sizeof(key), "digital.%s.transitions", digitalInputNames_[i]);
        data[key] = digitalInputs_[i].transitionCount();
    }
    for (uint8_t i = 0; i < ultrasoundSensorCount_; ++i)
    {
        char key[64];
        snprintf(key, sizeof(key), "ultrasound.%s.valid", ultrasoundSensorNames_[i]);
        data[key] = ultrasoundSensors_[i].valid();
        snprintf(key, sizeof(key), "ultrasound.%s.timed_out", ultrasoundSensorNames_[i]);
        data[key] = ultrasoundSensors_[i].timedOut();
        snprintf(key, sizeof(key), "ultrasound.%s.echo_us", ultrasoundSensorNames_[i]);
        data[key] = ultrasoundSensors_[i].echoUs();
        if (ultrasoundSensors_[i].valid())
        {
            snprintf(key, sizeof(key), "ultrasound.%s.distance_mm", ultrasoundSensorNames_[i]);
            data[key] = ultrasoundSensors_[i].distanceMm();
        }
    }
    data["imu.available"] = imu_.available();
    data["imu.valid"] = lastImuSampleValid_;
    data["imu.motion_active"] = imuMotionActive;
    data["imu.motion_event_count"] = imuMotionEventCount;
    if (imu_.available()) {
        data["imu.bus"] = imu_.bus();
        data["imu.address"] = imu_.address();
        data["imu.calibration.system"] = imu_.systemCalibration();
        data["imu.calibration.gyro"] = imu_.gyroCalibration();
        data["imu.calibration.accel"] = imu_.accelCalibration();
        data["imu.calibration.mag"] = imu_.magCalibration();
        data["imu.system_status"] = imu_.systemStatus();
        data["imu.system_error"] = imu_.systemError();
        data["imu.system_error_active"] = imu_.systemErrorActive();
        data["imu.operation_mode"] = imu_.operationMode();
        data["imu.fusion_running"] = imu_.fusionRunning();
        data["imu.self_test_result"] = imu_.selfTestResult();
        data["imu.self_test_passed"] = imu_.selfTestPassed();
        if (lastImuSampleValid_) {
            data["imu.heading_deg"] = imu_.headingDeg();
            data["imu.roll_deg"] = imu_.rollDeg();
            data["imu.pitch_deg"] = imu_.pitchDeg();
            data["imu.quaternion.w"] = imu_.quaternionW();
            data["imu.quaternion.x"] = imu_.quaternionX();
            data["imu.quaternion.y"] = imu_.quaternionY();
            data["imu.quaternion.z"] = imu_.quaternionZ();
            data["imu.linear_accel.x_mps2"] = imu_.linearAccelX();
            data["imu.linear_accel.y_mps2"] = imu_.linearAccelY();
            data["imu.linear_accel.z_mps2"] = imu_.linearAccelZ();
            data["imu.gravity.x_mps2"] = imu_.gravityX();
            data["imu.gravity.y_mps2"] = imu_.gravityY();
            data["imu.gravity.z_mps2"] = imu_.gravityZ();
            data["imu.temperature_c"] = imu_.temperatureC();
        }
    }
    data["hx12k.a.enabled"] = hx12kA_.enabled();
    data["hx12k.a.angle_deg"] = hx12kA_.commandedAngle();
    data["hx12k.b.enabled"] = hx12kB_.enabled();
    data["hx12k.b.angle_deg"] = hx12kB_.commandedAngle();
    data["hx12k.c.enabled"] = hx12kC_.enabled();
    data["hx12k.c.angle_deg"] = hx12kC_.commandedAngle();
    data["hx12k.d.enabled"] = hx12kD_.enabled();
    data["hx12k.d.angle_deg"] = hx12kD_.commandedAngle();
    for (uint8_t i = 0; i < tofSensorCount_; ++i)
    {
        char key[56];
        snprintf(key, sizeof(key), "tof.%s.available", tofSensorNames_[i]);
        data[key] = tofAvailable_[i];
        snprintf(key, sizeof(key), "tof.%s.timed_out", tofSensorNames_[i]);
        data[key] = tofTimedOut_[i];
        if (tofAvailable_[i] && !tofTimedOut_[i] && tofDistanceMm_[i] >= 0)
        {
            snprintf(key, sizeof(key), "tof.%s.distance_mm", tofSensorNames_[i]);
            data[key] = tofDistanceMm_[i];
        }
    }
    if (servoZeroed_[lastServoId_])
        data["servo.zero_offset_deg"] = servoZeroOffsetsDeg_[lastServoId_];
    if (!isnan(measuredServoAngleDeg_))
        data["servo.measured_angle_deg"] = measuredServoAngleDeg_;
    if (!isnan(servoPositionErrorDeg_))
        data["servo.position_error_deg"] = servoPositionErrorDeg_;
    link_.send(message, 2048);
    maxUpdateGapUs_ = 0;

    // Keep electrical diagnostics in a second, slower packet. These raw pin
    // values change far less often than motion state and sending them at the
    // full telemetry rate needlessly saturates the 115200-baud radio link.
    if (now - lastElectricalDiagnosticsMs_ < 1000U)
        return;
    lastElectricalDiagnosticsMs_ = now;
    JsonDocument diagnosticMessage;
    diagnosticMessage["type"] = "telemetry";
    diagnosticMessage["time"] = now;
    JsonObject diagnostics = diagnosticMessage["data"].to<JsonObject>();
    for (uint8_t i = 0; i < ultrasoundSensorCount_; ++i)
    {
        char key[64];
        snprintf(key, sizeof(key), "ultrasound.%s.echo_high", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].echoHigh();
        snprintf(key, sizeof(key), "ultrasound.%s.trigger_count", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].triggerCount();
        snprintf(key, sizeof(key), "ultrasound.%s.rise_count", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].riseCount();
        snprintf(key, sizeof(key), "ultrasound.%s.fall_count", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].fallCount();
        snprintf(key, sizeof(key), "ultrasound.%s.echo_adc", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].currentEchoAdc();
        snprintf(key, sizeof(key), "ultrasound.%s.echo_voltage_v", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].currentEchoAdc() * (3.3f / 4095.0f);
        snprintf(key, sizeof(key), "ultrasound.%s.ping_min_adc", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].lastMinEchoAdc();
        snprintf(key, sizeof(key), "ultrasound.%s.ping_max_adc", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].lastMaxEchoAdc();
        snprintf(key, sizeof(key), "ultrasound.%s.ping_min_voltage_v", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].lastMinEchoAdc() * (3.3f / 4095.0f);
        snprintf(key, sizeof(key), "ultrasound.%s.ping_max_voltage_v", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].lastMaxEchoAdc() * (3.3f / 4095.0f);
        snprintf(key, sizeof(key), "ultrasound.%s.trigger_low_adc", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].triggerLowAdc();
        snprintf(key, sizeof(key), "ultrasound.%s.trigger_high_adc", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].triggerHighAdc();
        snprintf(key, sizeof(key), "ultrasound.%s.trigger_high_voltage_v", ultrasoundSensorNames_[i]);
        diagnostics[key] = ultrasoundSensors_[i].triggerHighAdc() * (3.3f / 4095.0f);
    }
    link_.send(diagnosticMessage, 2048);
}

void BluetoothDebugWorkflow::initialiseDigitalInputs()
{
    for (JsonObject input : config_["digital_inputs"].as<JsonArray>())
    {
        if (digitalInputCount_ >= MAX_DIGITAL_INPUTS)
        {
            link_.error("Maximum of eight digital inputs exceeded");
            break;
        }

        const char* name = input["name"] | "";
        const int pin = input["pin"] | -1;
        bool duplicate = false;
        for (uint8_t i = 0; i < digitalInputCount_; ++i)
            duplicate = duplicate || strcmp(name, digitalInputNames_[i]) == 0 ||
                        pin == digitalInputs_[i].pin();

        if (name[0] == '\0' || strlen(name) >= sizeof(digitalInputNames_[0]) ||
            pin < 0 || pin > 41 || duplicate)
        {
            link_.error("Invalid or duplicate digital input JSON entry");
            continue;
        }

        strncpy(digitalInputNames_[digitalInputCount_], name,
                sizeof(digitalInputNames_[0]) - 1);
        const bool activeLow = input["active_low"] | true;
        const bool pullup = input["pullup"] | true;
        const uint16_t debounceMs = constrain(input["debounce_ms"] | 20, 0, 1000);
        digitalInputs_[digitalInputCount_].begin(
            static_cast<uint8_t>(pin), activeLow, pullup, debounceMs);
        ++digitalInputCount_;
    }
}

void BluetoothDebugWorkflow::initialiseUltrasoundSensors()
{
    for (JsonObject sensor : config_["ultrasound_sensors"].as<JsonArray>())
    {
        if (ultrasoundSensorCount_ >= MAX_ULTRASOUND_SENSORS)
        {
            link_.error("Maximum of two ultrasound sensors exceeded");
            break;
        }

        const char* name = sensor["name"] | "";
        const int triggerPin = sensor["trigger_pin"] | -1;
        const int echoPin = sensor["echo_pin"] | -1;
        bool duplicate = triggerPin == echoPin;
        for (uint8_t i = 0; i < ultrasoundSensorCount_; ++i)
            duplicate = duplicate || strcmp(name, ultrasoundSensorNames_[i]) == 0 ||
                        triggerPin == ultrasoundSensors_[i].triggerPin() ||
                        triggerPin == ultrasoundSensors_[i].echoPin() ||
                        echoPin == ultrasoundSensors_[i].triggerPin() ||
                        echoPin == ultrasoundSensors_[i].echoPin();

        if (name[0] == '\0' || strlen(name) >= sizeof(ultrasoundSensorNames_[0]) ||
            triggerPin < 0 || triggerPin > 41 || echoPin < 0 || echoPin > 41 || duplicate)
        {
            link_.error("Invalid or duplicate ultrasound JSON entry");
            continue;
        }

        strncpy(ultrasoundSensorNames_[ultrasoundSensorCount_], name,
                sizeof(ultrasoundSensorNames_[0]) - 1);
        const uint32_t intervalMs = constrain(sensor["interval_ms"] | 100UL, 50UL, 2000UL);
        const uint32_t timeoutUs = constrain(sensor["timeout_us"] | 30000UL, 1000UL, 50000UL);
        ultrasoundSensors_[ultrasoundSensorCount_].begin(
            static_cast<uint8_t>(triggerPin), static_cast<uint8_t>(echoPin),
            intervalMs, timeoutUs);
        ++ultrasoundSensorCount_;
    }
}

void BluetoothDebugWorkflow::updateUltrasoundSensors()
{
    if (ultrasoundSensorCount_ == 0)
        return;

    const uint32_t nowUs = micros();
    for (uint8_t i = 0; i < ultrasoundSensorCount_; ++i)
        ultrasoundSensors_[i].sampleEchoLevel();
    if (activeUltrasoundIndex_ >= 0)
    {
        UltrasoundSensor& active = ultrasoundSensors_[activeUltrasoundIndex_];
        active.update(nowUs);
        if (active.busy())
            return;
        nextUltrasoundIndex_ = (static_cast<uint8_t>(activeUltrasoundIndex_) + 1) %
                               ultrasoundSensorCount_;
        activeUltrasoundIndex_ = -1;
    }

    // Only one transducer may transmit/listen at a time. This avoids channel A
    // receiving channel B's ping on the two-socket interface board.
    for (uint8_t offset = 0; offset < ultrasoundSensorCount_; ++offset)
    {
        const uint8_t index = (nextUltrasoundIndex_ + offset) % ultrasoundSensorCount_;
        if (!ultrasoundSensors_[index].readyToTrigger(nowUs))
            continue;
        ultrasoundSensors_[index].trigger(nowUs);
        activeUltrasoundIndex_ = index;
        return;
    }
}

void BluetoothDebugWorkflow::initialiseTofSensors()
{
    Tof::SensorDefinition definitions[MAX_TOF_SENSORS];
    JsonArray configured = config_["tof_sensors"].as<JsonArray>();

    for (JsonObject sensor : configured)
    {
        if (tofSensorCount_ >= MAX_TOF_SENSORS)
        {
            link_.error("Maximum of eight TOF sensors exceeded");
            break;
        }

        const char* name = sensor["name"] | "";
        const char* type = sensor["type"] | "";
        int port = -1;
        if (sensor["port"].is<int>())
            port = sensor["port"].as<int>();
        else
        {
            const char* portText = sensor["port"] | "";
            if ((strncmp(portText, "XSHUT", 5) == 0 || strncmp(portText, "xshut", 5) == 0) &&
                portText[5] >= '0' && portText[5] <= '7' && portText[6] == '\0')
                port = portText[5] - '0';
        }

        const bool validType = strcmp(type, "long") == 0 || strcmp(type, "short") == 0;
        bool duplicate = false;
        for (uint8_t i = 0; i < tofSensorCount_; ++i)
            duplicate = duplicate || strcmp(name, tofSensorNames_[i]) == 0 ||
                        definitions[i].xshutPin == port;

        if (name[0] == '\0' || strlen(name) >= sizeof(tofSensorNames_[0]) ||
            !validType || port < 0 || port > 7 || duplicate)
        {
            link_.error("Invalid or duplicate TOF JSON entry");
            continue;
        }

        strncpy(tofSensorNames_[tofSensorCount_], name, sizeof(tofSensorNames_[0]) - 1);
        definitions[tofSensorCount_].type = strcmp(type, "long") == 0
            ? Tof::SensorType::Long : Tof::SensorType::Short;
        definitions[tofSensorCount_].xshutPin = static_cast<uint8_t>(port);
        definitions[tofSensorCount_].address = TofConfig::FIRST_ASSIGNED_ADDRESS + port;
        ++tofSensorCount_;
    }

    Tof::begin(definitions, tofSensorCount_);
    for (uint8_t i = 0; i < tofSensorCount_; ++i)
    {
        tofAvailable_[i] = Tof::available(i);
        tofTimedOut_[i] = !tofAvailable_[i];
        tofDistanceMm_[i] = -1;
    }

    link_.log("INFO", "JSON-configured TOF sensor setup complete");
}

void BluetoothDebugWorkflow::readTofSensors()
{
    for (uint8_t i = 0; i < tofSensorCount_; ++i)
    {
        tofAvailable_[i] = Tof::available(i);
        if (!tofAvailable_[i])
        {
            tofTimedOut_[i] = true;
            tofDistanceMm_[i] = -1;
            continue;
        }

        tofDistanceMm_[i] = Tof::read(i);
        tofTimedOut_[i] = Tof::timedOut(i);
    }
}

void BluetoothDebugWorkflow::updateMotionAndRangeSensors()
{
    const uint32_t now = millis();
    if (now - lastMotionSampleMs_ >= 50U)
    {
        lastMotionSampleMs_ = now;
        encoders_.sample(now);
        lastImuSampleValid_ = imu_.update();
        if (lastImuSampleValid_)
        {
            const float ax = imu_.linearAccelX();
            const float ay = imu_.linearAccelY();
            const float az = imu_.linearAccelZ();
            const float motionAccel = sqrtf(ax * ax + ay * ay + az * az);
            constexpr float IMU_MOTION_START_MPS2 = 0.22f;
            constexpr float IMU_MOTION_STOP_MPS2 = 0.08f;

            if (!imuMotionActive)
            {
                imuMotionLowSamples = 0;
                if (motionAccel >= IMU_MOTION_START_MPS2)
                {
                    if (imuMotionHighSamples < 255)
                        ++imuMotionHighSamples;
                    if (imuMotionHighSamples >= 2)
                    {
                        imuMotionActive = true;
                        imuMotionHighSamples = 0;
                        ++imuMotionEventCount;
                    }
                }
                else
                    imuMotionHighSamples = 0;
            }
            else
            {
                imuMotionHighSamples = 0;
                if (motionAccel <= IMU_MOTION_STOP_MPS2)
                {
                    if (imuMotionLowSamples < 255)
                        ++imuMotionLowSamples;
                    if (imuMotionLowSamples >= 3)
                    {
                        imuMotionActive = false;
                        imuMotionLowSamples = 0;
                    }
                }
                else
                    imuMotionLowSamples = 0;
            }
        }
        else
        {
            imuMotionActive = false;
            imuMotionHighSamples = 0;
            imuMotionLowSamples = 0;
        }
    }
    if (now - lastRangePollMs_ >= 100U)
    {
        lastRangePollMs_ = now;
        readTofSensors();
    }
}

void BluetoothDebugWorkflow::stopNavigation(const char* reason)
{
    navigationActive_ = false;
    navigationState_ = NAV_IDLE;
    dcMotor203Second_.stop();
    dcMotor203SecondActive_ = false;
    dcMotor203SecondDeadman_ = false;
    navigationSweepLeftReferenceValid_ = false;
    navigationSweepRightReferenceValid_ = false;
    navigationSweepLateralErrorMm_ = 0;
    navigationSweepProgressMm_ = 0.0f;
    navigationDetourOffsetMm_ = 0.0f;
    navigationDetourEdgeCleared_ = false;
    navigationDetourObstacleSeen_ = false;
    navigationDetourClearSamples_ = 0;
    navigationClearanceTurnCount_ = 0;
    if (reason != nullptr)
        link_.log("INFO", reason);
}

void BluetoothDebugWorkflow::updateNavigation()
{
    if (!navigationActive_)
        return;
    const uint32_t now = millis();
    if (!debugMode_ || stopped_)
    {
        stopNavigation();
        return;
    }

    // Default retained for recording/test compatibility: FRONT_AVOID_MM = 300.
    const uint16_t FRONT_AVOID_MM = navigationFrontAvoidMm_;
    // At the observed ~1.2 s matrix frame cadence the robot can travel over
    // 200 mm between frames. A genuinely broad wall therefore needs an
    // earlier threshold than a point return. Requiring three quarters of the
    // centre zones prevents one or two bad pixels from causing a turn.
    constexpr uint16_t MATRIX_BROAD_WALL_MM = 550;
    const uint16_t SIDE_AVOID_MM = navigationSideAvoidMm_;
    // Default: WALL_FOLLOW_TARGET_MM = 200.
    const uint16_t WALL_FOLLOW_TARGET_MM = navigationWallFollowMm_;
    // 200 mm is now only the nominal spacing. The opposite side ultrasound
    // determines how much arena width remains, so there is no fixed lane count.
    // Default: LANE_SPACING_MM = 200.0f.
    const float LANE_SPACING_MM = navigationLaneSpacingMm_;
    constexpr uint16_t SWEEP_EDGE_TARGET_MM = 250;
    constexpr uint16_t SWEEP_EDGE_TOLERANCE_MM = 75;

    // Localised forward obstacles are bypassed with a rectangular detour. A
    // true arena end wall should block the full 8x8 horizontal view, while an
    // internal wall/obstacle leaves at least one outer flank visibly open.
    const uint16_t OBSTACLE_FORWARD_GAP_MM = navigationGapDepthMm_;
    constexpr uint16_t OBSTACLE_MIN_SIDE_ROOM_MM = 450;
    constexpr float OBSTACLE_EARLY_WALL_MARGIN_MM = 450.0f;
    constexpr uint16_t OBSTACLE_SIDE_TRACK_MAX_MM = 650;
    constexpr uint16_t OBSTACLE_SIDE_RELEASE_MM = 800;
    constexpr float OBSTACLE_CLEAR_MARGIN_MM = 220.0f;
    constexpr float OBSTACLE_MAX_OFFSET_MM = 1200.0f;
    constexpr float OBSTACLE_MAX_PASS_MM = 2500.0f;
    constexpr float RECOVERY_REVERSE_MM = 300.0f;
    constexpr float RECOVERY_MAX_REVERSE_MM = 450.0f;
    constexpr float RECOVERY_TOTAL_REVERSE_LIMIT_MM = 1200.0f;
    constexpr uint8_t RECOVERY_MAX_ATTEMPTS = 3;
    constexpr uint16_t RECOVERY_SIDE_OPEN_MM = 350;
    constexpr uint32_t RECOVERY_SIDE_FILTER_FRESH_MS = 800U;

    constexpr float ENCODER_1_MM_PER_COUNT = 0.09094f;
    constexpr float ENCODER_2_MM_PER_COUNT = 0.09592f;

    static float wallFollowFilteredLeftMm = NAN;
    static uint32_t wallFollowLastFallCount = 0;
    static uint32_t wallFollowLastGoodMs = 0;
    static bool wallFollowFilterArmed = false;
    static float laneShiftTargetMm = LANE_SPACING_MM;
    static uint32_t sweepLeftLastFallCount = 0;
    static uint32_t sweepRightLastFallCount = 0;
    static bool sweepReferenceFilterArmed = false;

    // Recovery decisions use a fresh median instead of one raw ultrasound
    // return. The failed run contained an isolated ~36 mm side reading among
    // neighbouring readings hundreds of millimetres away.
    static uint16_t recoverySideSamples[2][3] = {};
    static uint8_t recoverySideSampleCount[2] = {};
    static uint8_t recoverySideWriteIndex[2] = {};
    static uint32_t recoverySideLastFallCount[2] = {};
    static uint32_t recoverySideLastGoodMs[2] = {};

    uint16_t front = 0xFFFF, left = 0xFFFF, right = 0xFFFF;
    auto includeMinimum = [](uint16_t& target, int value, int maximum = 3500) {
        if (value >= 30 && value <= maximum)
            target = min(target, static_cast<uint16_t>(value));
    };

    // Navigation deliberately ignores every bottom point TOF. Those sensors
    // can see one another/chassis hardware and are reserved for weight sensing
    // and mapping. Only the two top point TOFs feed forward navigation.
    uint16_t frontCandidates[MAX_TOF_SENSORS + 1] = {};
    uint8_t frontCandidateCount = 0;
    for (uint8_t i = 0; i < tofSensorCount_; ++i)
    {
        if (!tofAvailable_[i] || tofTimedOut_[i])
            continue;
        if (strncmp(tofSensorNames_[i], "top_", 4) != 0)
            continue;
        const int value = tofDistanceMm_[i];
        if (value >= 30 && value <= 3500)
            frontCandidates[frontCandidateCount++] = static_cast<uint16_t>(value);
    }
    for (uint8_t i = 0; i < ultrasoundSensorCount_; ++i)
    {
        if (!ultrasoundSensors_[i].valid() || ultrasoundSensors_[i].timedOut())
            continue;
        // Ultrasound A faces robot-left and B faces robot-right.
        if (i == 0) includeMinimum(left, ultrasoundSensors_[i].distanceMm(), 5000);
        else includeMinimum(right, ultrasoundSensors_[i].distanceMm(), 5000);
    }

    // Update recovery-side filtering only when a new echo has actually
    // completed. Re-reading the same echo in the fast navigation loop must not
    // make a single sample look like three independent measurements.
    for (uint8_t i = 0; i < ultrasoundSensorCount_ && i < 2; ++i)
    {
        const uint32_t fallCount = ultrasoundSensors_[i].fallCount();
        if (fallCount == recoverySideLastFallCount[i])
            continue;
        recoverySideLastFallCount[i] = fallCount;
        if (!ultrasoundSensors_[i].valid() || ultrasoundSensors_[i].timedOut())
            continue;
        const uint16_t sample = ultrasoundSensors_[i].distanceMm();
        if (sample < 30 || sample > 5000)
            continue;
        recoverySideSamples[i][recoverySideWriteIndex[i]] = sample;
        recoverySideWriteIndex[i] =
            static_cast<uint8_t>((recoverySideWriteIndex[i] + 1U) % 3U);
        if (recoverySideSampleCount[i] < 3U)
            ++recoverySideSampleCount[i];
        recoverySideLastGoodMs[i] = now;
    }
    auto recoverySideMedian = [&](uint8_t index) -> uint16_t {
        if (index >= 2U || recoverySideSampleCount[index] < 2U ||
            now - recoverySideLastGoodMs[index] > RECOVERY_SIDE_FILTER_FRESH_MS)
            return 0xFFFF;
        uint16_t values[3] = {};
        const uint8_t count = recoverySideSampleCount[index];
        for (uint8_t i = 0; i < count; ++i)
            values[i] = recoverySideSamples[index][i];
        for (uint8_t i = 1; i < count; ++i)
        {
            const uint16_t value = values[i];
            uint8_t j = i;
            while (j > 0 && values[j - 1] > value)
            {
                values[j] = values[j - 1];
                --j;
            }
            values[j] = value;
        }
        if (count == 2U)
            return static_cast<uint16_t>(
                (static_cast<uint32_t>(values[0]) + values[1]) / 2U);
        return values[1];
    };
    const uint16_t recoveryLeft = recoverySideMedian(0);
    const uint16_t recoveryRight = recoverySideMedian(1);
    uint8_t matrixLeftOpenColumns = 0;
    uint8_t matrixRightOpenColumns = 0;
    uint16_t matrixLeftGapMm = 0;
    uint16_t matrixRightGapMm = 0;
    uint16_t matrixBroadValues[32] = {};
    uint8_t matrixBroadValueCount = 0;
    uint8_t matrixUsableCentreCount = 0;
    bool matrixGapPassableNow = false;
    int8_t matrixGapCentreColumnX2 = 0;
    uint16_t matrixGapWidthMm = 0;
    if (tof8x8_.available() && tof8x8_.lastReadSucceeded())
    {
        uint16_t centreValues[32];
        uint8_t centreCount = 0;
        uint16_t columnValues[8][8] = {};
        uint8_t columnCounts[8] = {};

        const uint8_t ignoredRows = navigationMatrixFloorRows_ > 4U
            ? 4U : navigationMatrixFloorRows_;
        const uint8_t matrixRowsUsed = 8U - ignoredRows;
        for (uint8_t row = 0; row < matrixRowsUsed; ++row)
            for (uint8_t col = 0; col < 8; ++col)
            {
                const uint16_t value = tof8x8_.distanceMm(row, col);
                // Zero is an invalid return, but a coherent 10-29 mm field is
                // a real near-contact wall. The 14:57 run produced 12-21 mm
                // across almost every zone while the chassis pushed into it.
                if (col >= 2 && col <= 5 && value >= 10 && value <= 3500)
                {
                    ++matrixUsableCentreCount;
                    if (value < MATRIX_BROAD_WALL_MM)
                        matrixBroadValues[matrixBroadValueCount++] = value;
                }
                // Preserve the established 8x8 rejection of chassis/noise
                // returns below 200 mm for ordinary range estimation. The
                // separate broad-wall consensus above deliberately retains a
                // coherent close surface, which must not disappear merely
                // because every zone crossed below 200 mm together.
                if (value < 200 || value > 3500)
                    continue;
                columnValues[col][columnCounts[col]++] = value;
                if (col >= 2 && col <= 5)
                    centreValues[centreCount++] = value;
            }

        auto sortSmall = [](uint16_t* values, uint8_t count) {
            for (uint8_t i = 1; i < count; ++i)
            {
                const uint16_t value = values[i];
                uint8_t j = i;
                while (j > 0 && values[j - 1] > value)
                {
                    values[j] = values[j - 1];
                    --j;
                }
                values[j] = value;
            }
        };
        auto lowerQuartile = [&](uint16_t* values, uint8_t count) -> uint16_t {
            if (count == 0) return 0xFFFF;
            sortSmall(values, count);
            return values[(count - 1) / 4];
        };

        const uint16_t matrixForward = lowerQuartile(centreValues, centreCount);
        if (matrixForward != 0xFFFF)
            frontCandidates[frontCandidateCount++] = matrixForward;

        const bool broadCloseWall = matrixUsableCentreCount >= 20 &&
            static_cast<uint16_t>(matrixBroadValueCount) * 3U >=
                static_cast<uint16_t>(matrixUsableCentreCount) * 2U;
        if (broadCloseWall)
        {
            sortSmall(matrixBroadValues, matrixBroadValueCount);
            const uint16_t broadMedian =
                matrixBroadValues[matrixBroadValueCount / 2];
            // This is independent evidence from dozens of zones, not one
            // candidate in the later three-sensor median. Make it authoritative.
            front = min(front, broadMedian);
        }
        navigationMatrixBroadWall_ = broadCloseWall;

        // Treat an outer flank as genuinely open only when at least two of its
        // three horizontal columns contain a stable median return farther than
        // the detour-gap threshold. A broad arena wall therefore does not look
        // like a bypassable obstacle merely because one pixel is noisy.
        for (uint8_t col = 0; col < 8; ++col)
        {
            if (columnCounts[col] < 3)
                continue;
            sortSmall(columnValues[col], columnCounts[col]);
            const uint16_t columnMedian =
                columnValues[col][columnCounts[col] / 2];
            if (columnMedian <= OBSTACLE_FORWARD_GAP_MM)
                continue;
            if (col <= 2)
            {
                ++matrixLeftOpenColumns;
                matrixLeftGapMm = max(matrixLeftGapMm, columnMedian);
            }
            if (col >= 5)
            {
                ++matrixRightOpenColumns;
                matrixRightGapMm = max(matrixRightGapMm, columnMedian);
            }
        }

        // Find the widest contiguous opening anywhere in the horizontal view,
        // rather than only accepting gaps on the two outer flanks. Width is
        // projected at the nearest open-column depth, making the decision a
        // physical chassis-width check instead of a fixed pixel-count guess.
        uint8_t runStart = 0;
        uint8_t runLength = 0;
        uint16_t runDepth = 3500;
        auto evaluateGap = [&](uint8_t start, uint8_t length, uint16_t depth) {
            if (length == 0 || depth == 0 || depth == 0xFFFF) return;
            const float HORIZONTAL_FOV_DEG = navigationMatrixFovDeg_;
            const float leftAngle = (-0.5f + static_cast<float>(start) / 8.0f) *
                                    HORIZONTAL_FOV_DEG * DEG_TO_RAD;
            const float rightAngle = (-0.5f + static_cast<float>(start + length) / 8.0f) *
                                     HORIZONTAL_FOV_DEG * DEG_TO_RAD;
            const uint16_t width = static_cast<uint16_t>(constrain(
                lroundf(depth * (tanf(rightAngle) - tanf(leftAngle))), 0L, 5000L));
            if (width > matrixGapWidthMm)
            {
                matrixGapWidthMm = width;
                matrixGapCentreColumnX2 = static_cast<int8_t>(2 * start + length - 8);
            }
        };
        for (uint8_t col = 0; col <= 8; ++col)
        {
            bool open = false;
            uint16_t depth = 0xFFFF;
            if (col < 8 && columnCounts[col] >= 3)
            {
                sortSmall(columnValues[col], columnCounts[col]);
                depth = columnValues[col][columnCounts[col] / 2];
                open = depth >= OBSTACLE_FORWARD_GAP_MM;
            }
            if (open)
            {
                if (runLength == 0) { runStart = col; runDepth = depth; }
                else runDepth = min(runDepth, depth);
                ++runLength;
            }
            else if (runLength != 0)
            {
                evaluateGap(runStart, runLength, runDepth);
                runLength = 0;
                runDepth = 3500;
            }
        }
        matrixGapPassableNow = matrixGapWidthMm >=
            navigationRobotWidthMm_ + navigationGapMarginMm_;
    }

    if (matrixGapPassableNow)
    {
        if (navigationGapSeenFrames_ < 10) ++navigationGapSeenFrames_;
    }
    else
        navigationGapSeenFrames_ = 0;
    navigationGapPassable_ = matrixGapPassableNow &&
        navigationGapSeenFrames_ >= navigationGapConfirmFrames_;
    navigationGapCentreColumnX2_ = matrixGapCentreColumnX2;
    navigationGapWidthMm_ = matrixGapWidthMm;

    // A median across the top-left, top-right and matrix forward estimates
    // rejects one isolated short return without hiding a broad wall.
    for (uint8_t i = 1; i < frontCandidateCount; ++i)
    {
        const uint16_t value = frontCandidates[i];
        uint8_t j = i;
        while (j > 0 && frontCandidates[j - 1] > value)
        {
            frontCandidates[j] = frontCandidates[j - 1];
            --j;
        }
        frontCandidates[j] = value;
    }
    if (frontCandidateCount > 0)
    {
        const uint16_t candidateMedian =
            frontCandidates[frontCandidateCount / 2];
        front = min(front, candidateMedian);
    }

    navigationMatrixCloseZones_ = matrixBroadValueCount;
    navigationMatrixUsableZones_ = matrixUsableCentreCount;
    if (!tof8x8_.available() || !tof8x8_.lastReadSucceeded())
        navigationMatrixBroadWall_ = false;

    if (navigationState_ != NAV_FOLLOW_WALL)
    {
        wallFollowFilteredLeftMm = NAN;
        wallFollowLastGoodMs = 0;
        wallFollowFilterArmed = false;
        if (ultrasoundSensorCount_ > 0)
            wallFollowLastFallCount = ultrasoundSensors_[0].fallCount();
    }
    else if (ultrasoundSensorCount_ > 0)
    {
        const uint32_t fallCount = ultrasoundSensors_[0].fallCount();
        if (!wallFollowFilterArmed)
        {
            wallFollowLastFallCount = fallCount;
            wallFollowFilterArmed = true;
        }
        else if (fallCount != wallFollowLastFallCount)
        {
            wallFollowLastFallCount = fallCount;
            if (ultrasoundSensors_[0].valid() && !ultrasoundSensors_[0].timedOut())
            {
                const float sampleMm = ultrasoundSensors_[0].distanceMm();
                if (isnan(wallFollowFilteredLeftMm))
                    wallFollowFilteredLeftMm = sampleMm;
                else
                {
                    float stepMm = sampleMm - wallFollowFilteredLeftMm;
                    stepMm = constrain(stepMm, -120.0f, 120.0f);
                    wallFollowFilteredLeftMm += 0.35f * stepMm;
                }
                wallFollowLastGoodMs = now;
            }
        }
    }

    navigationFrontMm_ = front == 0xFFFF ? 0 : front;
    navigationLeftMm_ = left == 0xFFFF ? 0 : left;
    navigationRightMm_ = right == 0xFFFF ? 0 : right;
    const bool rawFrontBlocked = navigationMatrixBroadWall_ ||
        (front != 0xFFFF && front < FRONT_AVOID_MM);
    // Gap Explorer trusts a confirmed physical-width opening. Balanced mode
    // also accepts it when centred; Conservative mode never overrides a wall.
    const bool gapMayOverrideWall = navigationGapPassable_ &&
        (navigationStrategy_ == 1 ||
         (navigationStrategy_ == 0 && abs(navigationGapCentreColumnX2_) <= 2));
    const bool frontBlocked = rawFrontBlocked && !gapMayOverrideWall;
    const bool leftBlocked = left != 0xFFFF && left < SIDE_AVOID_MM;
    const bool rightBlocked = right != 0xFFFF && right < SIDE_AVOID_MM;

    auto updateSweepReference = [](uint16_t sample, uint16_t& filtered, bool& valid) {
        if (sample == 0xFFFF)
            return;
        if (!valid)
        {
            filtered = sample;
            valid = true;
            return;
        }
        int32_t delta = static_cast<int32_t>(sample) - filtered;
        delta = constrain(delta, -200L, 200L);
        int32_t adjustment = delta / 3;
        if (adjustment == 0 && delta != 0)
            adjustment = delta > 0 ? 1 : -1;
        filtered = static_cast<uint16_t>(constrain(
            static_cast<int32_t>(filtered) + adjustment, 30L, 5000L));
    };

    // The main loop runs much faster than an ultrasound ping. Update each
    // sweep-width filter only when that sensor has completed a new echo,
    // otherwise repeatedly feeding the same stale sample would make the
    // filter appear far more confident than the physical measurement rate.
    if (navigationState_ != NAV_SWEEP)
    {
        sweepReferenceFilterArmed = false;
    }
    else if (!sweepReferenceFilterArmed)
    {
        if (ultrasoundSensorCount_ > 0)
            sweepLeftLastFallCount = ultrasoundSensors_[0].fallCount();
        if (ultrasoundSensorCount_ > 1)
            sweepRightLastFallCount = ultrasoundSensors_[1].fallCount();
        sweepReferenceFilterArmed = true;
    }
    else
    {
        if (ultrasoundSensorCount_ > 0)
        {
            const uint32_t fallCount = ultrasoundSensors_[0].fallCount();
            if (fallCount != sweepLeftLastFallCount)
            {
                sweepLeftLastFallCount = fallCount;
                updateSweepReference(left, navigationSweepLeftReferenceMm_,
                                     navigationSweepLeftReferenceValid_);
            }
        }
        if (ultrasoundSensorCount_ > 1)
        {
            const uint32_t fallCount = ultrasoundSensors_[1].fallCount();
            if (fallCount != sweepRightLastFallCount)
            {
                sweepRightLastFallCount = fallCount;
                updateSweepReference(right, navigationSweepRightReferenceMm_,
                                     navigationSweepRightReferenceValid_);
            }
        }
    }

    auto normaliseHeading = [](float heading) {
        while (heading >= 360.0f) heading -= 360.0f;
        while (heading < 0.0f) heading += 360.0f;
        return heading;
    };
    auto headingDelta = [](float from, float to) {
        float delta = to - from;
        while (delta > 180.0f) delta -= 360.0f;
        while (delta < -180.0f) delta += 360.0f;
        return delta;
    };

    auto setDrive = [&](int16_t channelA, int16_t channelB) {
        dcMotor203Second_.setPercent(channelA, channelB);
        dcMotor203SecondActive_ = channelA != 0 || channelB != 0;
        dcMotor203SecondDeadman_ = false;
    };
    auto beginHeadingTurn = [&](float targetHeading, NavigationState state) {
        navigationTargetHeadingDeg_ = normaliseHeading(targetHeading);
        navigationState_ = state;
        navigationMotionStartedMs_ = now;
        navigationTurnSettledSinceMs_ = 0;
        navigationTurnPulseStartedMs_ = now;
        navigationTurnCoastUntilMs_ = now;
        navigationTurnDirection_ = 0;
        navigationMotionConsistent_ = true;
    };
    auto beginRightAngleTurn = [&](bool rightTurn, NavigationState state) {
        // Normal coverage turns stay on the exact heading lattice so small
        // turn errors cannot accumulate from lane to lane.
        beginHeadingTurn(
            navigationHeadingReferenceDeg_ + (rightTurn ? 90.0f : -90.0f), state);
    };
    auto beginRelativeTurnFromCurrent = [&](bool rightTurn, float angleDeg,
                                            NavigationState state) {
        // Recovery must not use navigationHeadingReferenceDeg_. The failed run
        // repeatedly produced a target almost equal to the heading already
        // reached, causing reverse -> near-zero turn -> reverse loops.
        const float currentHeading = lastImuSampleValid_
            ? imu_.headingDeg() : navigationTargetHeadingDeg_;
        beginHeadingTurn(
            currentHeading + (rightTurn ? angleDeg : -angleDeg), state);
    };
    auto runHeadingTurn = [&]() {
        const float error = headingDelta(imu_.headingDeg(), navigationTargetHeadingDeg_);
        if (!lastImuSampleValid_)
        {
            setDrive(0, 0);
            return false;
        }

        const float absoluteError = fabsf(error);
        if (absoluteError <= 4.0f)
        {
            setDrive(0, 0);
            if (navigationTurnSettledSinceMs_ == 0)
                navigationTurnSettledSinceMs_ = now;
            // Do not start the next forward leg while rotational inertia is
            // still carrying the chassis through the target heading.
            return now - navigationTurnSettledSinceMs_ >= 300U;
        }
        navigationTurnSettledSinceMs_ = 0;

        const int8_t direction = error >= 0.0f ? 1 : -1;
        if (direction != navigationTurnDirection_)
        {
            // The recordings showed full-power direction reversals producing
            // a 20+ degree ping-pong. Coast before applying reverse torque.
            navigationTurnDirection_ = direction;
            navigationTurnCoastUntilMs_ = now + 180U;
            navigationTurnPulseStartedMs_ = navigationTurnCoastUntilMs_;
            setDrive(0, 0);
            return false;
        }
        if (now < navigationTurnCoastUntilMs_)
        {
            setDrive(0, 0);
            return false;
        }

        const bool turnRight = direction > 0;
        if (absoluteError > 55.0f)
        {
            setDrive(turnRight ? -100 : 100, turnRight ? 100 : -100);
            navigationTurnPulseStartedMs_ = now;
            return false;
        }

        // The drivetrain does not move reliably below 75%, so reduce angular
        // speed with duty cycle rather than an unusably small PWM command.
        const uint32_t pulsePhase = (now - navigationTurnPulseStartedMs_) % 300U;
        if (pulsePhase < 90U)
            setDrive(turnRight ? -100 : 100, turnRight ? 100 : -100);
        else
            setDrive(0, 0);
        return false;
    };
    auto driveOnHeading = [&](float targetHeading, int16_t basePower) {
        if (!lastImuSampleValid_)
        {
            setDrive(-basePower, -basePower);
            return;
        }
        const float error = headingDelta(imu_.headingDeg(), targetHeading);
        float correction = constrain(error * 0.65f, -5.0f, 5.0f);
        if (fabsf(error) < 0.75f)
            correction = 0.0f;
        const int16_t channelAPower = constrain(
            static_cast<int16_t>(lroundf(basePower + correction)), 75, 100);
        const int16_t channelBPower = constrain(
            static_cast<int16_t>(lroundf(basePower - correction)), 75, 100);
        setDrive(-channelAPower, -channelBPower);
    };
    auto beginForwardLeg = [&](float exactHeading) {
        navigationHeadingReferenceDeg_ = normaliseHeading(exactHeading);
        navigationMotionStartedMs_ = now;
        navigationMotionConsistent_ = true;
    };
    auto encoderDistanceFrom = [&](int32_t firstStart, int32_t secondStart) {
        const float firstDistance = fabsf(
            (encoders_.firstCount() - firstStart) * ENCODER_1_MM_PER_COUNT);
        const float secondDistance = fabsf(
            (encoders_.secondCount() - secondStart) * ENCODER_2_MM_PER_COUNT);
        return (firstDistance + secondDistance) * 0.5f;
    };
    auto detourSensorIndex = [&]() -> int8_t {
        // Turning right places the obstacle on robot-left (ultrasound A).
        // Turning left places it on robot-right (ultrasound B).
        return navigationDetourRight_ ? 0 : 1;
    };
    auto armDetourSideTracking = [&]() {
        navigationDetourObstacleSeen_ = false;
        navigationDetourClearSamples_ = 0;
        navigationDetourEdgeCleared_ = false;
        const int8_t index = detourSensorIndex();
        navigationDetourLastTriggerCount_ =
            index >= 0 && index < ultrasoundSensorCount_
                ? ultrasoundSensors_[index].triggerCount()
                : 0;
    };
    auto detourSideJustCleared = [&]() {
        const int8_t index = detourSensorIndex();
        if (index < 0 || index >= ultrasoundSensorCount_)
            return false;
        const uint32_t triggerCount = ultrasoundSensors_[index].triggerCount();
        if (triggerCount == navigationDetourLastTriggerCount_)
            return false;
        navigationDetourLastTriggerCount_ = triggerCount;

        const bool valid = ultrasoundSensors_[index].valid() &&
                           !ultrasoundSensors_[index].timedOut();
        const uint16_t distance = valid ? ultrasoundSensors_[index].distanceMm() : 0;
        if (valid && distance <= OBSTACLE_SIDE_TRACK_MAX_MM)
        {
            navigationDetourObstacleSeen_ = true;
            navigationDetourClearSamples_ = 0;
            return false;
        }

        const bool clear = ultrasoundSensors_[index].timedOut() ||
            (valid && distance >= OBSTACLE_SIDE_RELEASE_MM);
        if (navigationDetourObstacleSeen_ && clear)
        {
            if (navigationDetourClearSamples_ < 3)
                ++navigationDetourClearSamples_;
            if (navigationDetourClearSamples_ >= 2)
            {
                navigationDetourClearSamples_ = 0;
                return true;
            }
        }
        else if (valid)
            navigationDetourClearSamples_ = 0;
        return false;
    };
    auto startRecovery = [&](const char* reason) {
        setDrive(0, 0);
        ++navigationRecoveryCount_;
        navigationRecoveryAttemptCount_ = 0;
        navigationRecoveryTotalReverseMm_ = 0.0f;
        navigationRecoveryForceHalfTurn_ = false;
        navigationClearanceTurnCount_ = 0;
        navigationRecoveryStartEncoder1_ = encoders_.firstCount();
        navigationRecoveryStartEncoder2_ = encoders_.secondCount();

        // Choose the escape side once from FILTERED ultrasound and lock it for
        // this attempt. NAV_RECOVERY_REVERSE is not allowed to overwrite the
        // decision with whichever raw sample happens to arrive before turning.
        const bool leftKnown = recoveryLeft != 0xFFFF;
        const bool rightKnown = recoveryRight != 0xFFFF;
        if (leftKnown || rightKnown)
            navigationRecoveryTurnRight_ = rightKnown &&
                (!leftKnown || recoveryRight >= recoveryLeft);
        else
            navigationRecoveryTurnRight_ =
                (navigationRecoveryCount_ & 1U) != 0U;

        navigationState_ = NAV_RECOVERY_REVERSE;
        navigationMotionStartedMs_ = now;
        navigationMotionConsistent_ = true;
        navigationDetourOffsetMm_ = 0.0f;
        navigationDetourEdgeCleared_ = false;
        navigationDetourObstacleSeen_ = false;
        navigationDetourClearSamples_ = 0;
        navigationSweepLateralErrorMm_ = 0;
        navigationSweepLeftReferenceValid_ = false;
        navigationSweepRightReferenceValid_ = false;
        navigationExpectedSweepLengthMm_ = 0.0f;
        navigationExpectedSweepLengthValid_ = false;

        setDrive(100, 100);
        if (reason != nullptr)
            link_.log("WARNING", reason);
    };

    auto avoidBlockedTurnExit = [&]() -> bool {
        if (!frontBlocked)
        {
            navigationClearanceTurnCount_ = 0;
            return false;
        }

        const bool leftClose =
            recoveryLeft != 0xFFFF && recoveryLeft < RECOVERY_SIDE_OPEN_MM;
        const bool rightClose =
            recoveryRight != 0xFFFF && recoveryRight < RECOVERY_SIDE_OPEN_MM;
        if (leftClose && rightClose)
        {
            navigationClearanceTurnCount_ = 0;
            navigationState_ = NAV_ESCAPE_REVERSE;
            navigationEscapeStartEncoder1_ = encoders_.firstCount();
            navigationEscapeStartEncoder2_ = encoders_.secondCount();
            navigationEscapeTurnRight_ =
                recoveryRight != 0xFFFF &&
                (recoveryLeft == 0xFFFF || recoveryRight >= recoveryLeft);
            navigationMotionStartedMs_ = now;
            navigationMotionConsistent_ = true;
            setDrive(100, 100);
            link_.log("WARNING", "Turn ended inside a U-shaped enclosure; backing out");
            return true;
        }

        if (navigationClearanceTurnCount_ >= 2)
        {
            startRecovery(
                "Repeated blocked turn exits; backing away and replanning");
            return true;
        }

        ++navigationClearanceTurnCount_;
        const bool turnRight = recoveryRight != 0xFFFF &&
            (recoveryLeft == 0xFFFF || recoveryRight >= recoveryLeft);
        setDrive(0, 0);
        beginRelativeTurnFromCurrent(turnRight, 90.0f, NAV_CLEARANCE_TURN);
        link_.log(
            "WARNING",
            "Turn ended facing a wall; trying a bounded clearance turn");
        return true;
    };

    // Learn the arena's long dimension from completed wall-to-wall sweep legs.
    // Only encoder travel while actually in NAV_SWEEP contributes here; all
    // sideways/forward detour motion is excluded so going around an obstacle
    // cannot make the learned arena length grow.
    if (navigationState_ == NAV_SWEEP)
    {
        const float firstStepMm = fabsf(
            (encoders_.firstCount() - navigationSweepProgressLastEncoder1_) *
            ENCODER_1_MM_PER_COUNT);
        const float secondStepMm = fabsf(
            (encoders_.secondCount() - navigationSweepProgressLastEncoder2_) *
            ENCODER_2_MM_PER_COUNT);
        navigationSweepProgressMm_ += (firstStepMm + secondStepMm) * 0.5f;
        navigationSweepProgressLastEncoder1_ = encoders_.firstCount();
        navigationSweepProgressLastEncoder2_ = encoders_.secondCount();
    }

    // Encoder/IMU agreement is recorded for diagnosis only. It never cancels
    // the coverage program.
    if (now - navigationMotionStartedMs_ >= 500U)
    {
        const float e1 = encoders_.firstCountsPerSecond();
        const float e2 = encoders_.secondCountsPerSecond();
        const bool forwardState = navigationState_ == NAV_SEEK_WALL ||
            navigationState_ == NAV_FOLLOW_WALL ||
            navigationState_ == NAV_SWEEP ||
            navigationState_ == NAV_LANE_SHIFT ||
            navigationState_ == NAV_MISSION_TRACK ||
            navigationState_ == NAV_OBSTACLE_OFFSET ||
            navigationState_ == NAV_OBSTACLE_PASS ||
            navigationState_ == NAV_OBSTACLE_RETURN;
        if (forwardState)
            navigationMotionConsistent_ = lastImuSampleValid_ &&
                e1 < -10.0f && e2 > 10.0f;
        else if (navigationState_ != NAV_IDLE && navigationState_ != NAV_COMPLETE)
            navigationMotionConsistent_ = lastImuSampleValid_ &&
                (fabsf(e1) > 10.0f || fabsf(e2) > 10.0f);
    }

    // A simultaneous close front, left and right return is a U-shaped trap,
    // not an ordinary end wall. Back out before attempting another turn so the
    // chassis has physical clearance to rotate. Bottom TOFs remain excluded.
    const bool forwardNavigationState = navigationState_ == NAV_SEEK_WALL ||
        navigationState_ == NAV_FOLLOW_WALL || navigationState_ == NAV_SWEEP;
    const bool uTrapLeftClose =
        recoveryLeft != 0xFFFF && recoveryLeft < RECOVERY_SIDE_OPEN_MM;
    const bool uTrapRightClose =
        recoveryRight != 0xFFFF && recoveryRight < RECOVERY_SIDE_OPEN_MM;
    if (forwardNavigationState && frontBlocked &&
        uTrapLeftClose && uTrapRightClose)
    {
        navigationState_ = NAV_ESCAPE_REVERSE;
        navigationEscapeStartEncoder1_ = encoders_.firstCount();
        navigationEscapeStartEncoder2_ = encoders_.secondCount();
        navigationEscapeTurnRight_ =
            recoveryRight != 0xFFFF &&
            (recoveryLeft == 0xFFFF || recoveryRight >= recoveryLeft);
        navigationMotionStartedMs_ = now;
        navigationMotionConsistent_ = true;
        setDrive(100, 100);
        link_.log("WARNING", "U-shaped enclosure detected; reversing to find turning clearance");
    }

    if (navigationState_ == NAV_MISSION_TRACK && navigationStrategy_ == 3)
    {
        if (!lastImuSampleValid_)
        {
            setDrive(0, 0);
            return;
        }
        // Mission coordinates use screen-style arena axes: +X right/east,
        // +Y down/south and positive headings clockwise. BNO055 heading is
        // clockwise too, so the arena/BNO alignment is a fixed start offset.
        if (!missionPoseInitialised_)
        {
            missionPoseXmm_ = missionStartXmm_;
            missionPoseYmm_ = missionStartYmm_;
            missionArenaHeadingDeg_ = missionStartArenaHeadingDeg_;
            missionStartImuHeadingDeg_ = imu_.headingDeg();
            missionLastEncoder1_ = encoders_.firstCount();
            missionLastEncoder2_ = encoders_.secondCount();
            missionPoseInitialised_ = true;
        }

        const int32_t currentEncoder1 = encoders_.firstCount();
        const int32_t currentEncoder2 = encoders_.secondCount();
        const int32_t deltaEncoder1 = currentEncoder1 - missionLastEncoder1_;
        const int32_t deltaEncoder2 = currentEncoder2 - missionLastEncoder2_;
        missionLastEncoder1_ = currentEncoder1;
        missionLastEncoder2_ = currentEncoder2;

        if (lastImuSampleValid_)
        {
            missionArenaHeadingDeg_ = normaliseHeading(
                missionStartArenaHeadingDeg_ +
                headingDelta(missionStartImuHeadingDeg_, imu_.headingDeg()));

            // Powered straight-run calibration has encoder 1 reversed and
            // encoder 2 forward in physical robot coordinates.
            const float leftMm = -deltaEncoder1 * ENCODER_1_MM_PER_COUNT;
            const float rightMm = deltaEncoder2 * ENCODER_2_MM_PER_COUNT;
            const float distanceMm = (leftMm + rightMm) * 0.5f;
            if (fabsf(distanceMm) <= 350.0f)
            {
                const float angleRad = missionArenaHeadingDeg_ * DEG_TO_RAD;
                missionPoseXmm_ += distanceMm * cosf(angleRad);
                missionPoseYmm_ += distanceMm * sinf(angleRad);
            }
        }

        if (missionWaypointIndex_ >= missionWaypointCount_ ||
            missionTargetsVisited_ >= missionTargetCount_)
        {
            setDrive(0, 0);
            navigationStrategy_ = missionFallbackStrategy_;
            navigationState_ = NAV_SEEK_WALL;
            navigationHeadingReferenceDeg_ = imu_.headingDeg();
            navigationTargetHeadingDeg_ = navigationHeadingReferenceDeg_;
            navigationLaneIndex_ = 0;
            navigationSweepLeftReferenceValid_ = false;
            navigationSweepRightReferenceValid_ = false;
            navigationSweepProgressMm_ = 0.0f;
            navigationExpectedSweepLengthValid_ = false;
            missionBlockedSinceMs_ = 0;
            beginForwardLeg(navigationHeadingReferenceDeg_);
            link_.log("INFO", "All pre-laid weight targets visited; switching to roaming strategy");
            return;
        }

        const MissionWaypoint& waypoint = missionWaypoints_[missionWaypointIndex_];
        const float dx = static_cast<float>(waypoint.xMm) - missionPoseXmm_;
        const float dy = static_cast<float>(waypoint.yMm) - missionPoseYmm_;
        const float distanceToWaypoint = sqrtf(dx * dx + dy * dy);
        const float arrivalMm = (waypoint.flags & 0x01) != 0 ? 100.0f : 120.0f;

        if (distanceToWaypoint <= arrivalMm)
        {
            setDrive(0, 0);
            if ((waypoint.flags & 0x01) != 0 &&
                missionTargetsVisited_ < missionTargetCount_)
            {
                ++missionTargetsVisited_;
                link_.log("INFO", "Pre-laid mission weight location visited");
            }
            ++missionWaypointIndex_;
            missionBlockedSinceMs_ = 0;
            return;
        }

        // Known walls/ramps/tubes/dummy weights are already inflated into the
        // desktop A* route. Live ranging remains a safety layer. If reality
        // disagrees with the pre-laid map, hold position without aborting the
        // mission; once the path clears, course-to-waypoint resumes and pulls
        // the robot back onto the polyline automatically.
        if (frontBlocked && !navigationGapPassable_)
        {
            setDrive(0, 0);
            if (missionBlockedSinceMs_ == 0)
            {
                missionBlockedSinceMs_ = now;
                link_.log("WARNING", "Pre-laid mission path blocked by live ranging; holding for clearance");
            }
            return;
        }
        missionBlockedSinceMs_ = 0;

        float desiredArenaHeadingDeg = atan2f(dy, dx) * RAD_TO_DEG;
        desiredArenaHeadingDeg = normaliseHeading(desiredArenaHeadingDeg);
        const float targetBnoHeadingDeg = normaliseHeading(
            missionStartImuHeadingDeg_ +
            headingDelta(missionStartArenaHeadingDeg_, desiredArenaHeadingDeg));
        navigationTargetHeadingDeg_ = targetBnoHeadingDeg;

        if (!lastImuSampleValid_)
        {
            setDrive(0, 0);
            return;
        }

        const float headingError = headingDelta(imu_.headingDeg(), targetBnoHeadingDeg);
        if (fabsf(headingError) > 18.0f)
        {
            // Point-turn until the requested path is reasonably aligned, then
            // proportional straight-line heading correction takes over.
            const bool turnRight = headingError > 0.0f;
            const int16_t turnPower = fabsf(headingError) > 55.0f ? 90 : 80;
            setDrive(turnRight ? -turnPower : turnPower,
                     turnRight ? turnPower : -turnPower);
        }
        else
        {
            const int16_t basePower = distanceToWaypoint < 350.0f ? 78 : 88;
            driveOnHeading(targetBnoHeadingDeg, basePower);
        }
        return;
    }

    switch (navigationState_)
    {
        case NAV_SEEK_WALL:
            laneShiftTargetMm = LANE_SPACING_MM;
            if (frontBlocked)
            {
                setDrive(0, 0);
                beginRightAngleTurn(true, NAV_INITIAL_TURN);
                link_.log("INFO", "Navigation reached first wall; turning right");
            }
            else if (gapMayOverrideWall && rawFrontBlocked)
                driveOnHeading(normaliseHeading(navigationHeadingReferenceDeg_ +
                    constrain(navigationGapCentreColumnX2_ *
                        (navigationMatrixFovDeg_ / 16.0f), -20.0f, 20.0f)), 100);
            else if (front == 0xFFFF)
                setDrive(-100, 100);  // rotate until forward ranging is recovered
            else
                driveOnHeading(navigationHeadingReferenceDeg_, 100);
            break;

        case NAV_INITIAL_TURN:
            if (runHeadingTurn())
            {
                if (avoidBlockedTurnExit())
                    break;
                navigationState_ = NAV_FOLLOW_WALL;
                beginForwardLeg(navigationTargetHeadingDeg_);
                link_.log("INFO", "Navigation following first wall to corner");
            }
            break;

        case NAV_FOLLOW_WALL:
            if (frontBlocked)
            {
                setDrive(0, 0);
                beginRightAngleTurn(true, NAV_CORNER_TURN);
                link_.log("INFO", "Navigation reached corner; entering sweep");
            }
            else if (gapMayOverrideWall && rawFrontBlocked)
                driveOnHeading(normaliseHeading(navigationHeadingReferenceDeg_ +
                    constrain(navigationGapCentreColumnX2_ *
                        (navigationMatrixFovDeg_ / 16.0f), -20.0f, 20.0f)), 100);
            else if (!isnan(wallFollowFilteredLeftMm) &&
                     now - wallFollowLastGoodMs <= 650U)
            {
                float wallErrorMm =
                    static_cast<float>(WALL_FOLLOW_TARGET_MM) -
                    wallFollowFilteredLeftMm;
                if (fabsf(wallErrorMm) < 15.0f)
                    wallErrorMm = 0.0f;
                const float headingOffsetDeg =
                    constrain(wallErrorMm * 0.06f, -10.0f, 10.0f);
                const float wallTargetHeading = normaliseHeading(
                    navigationHeadingReferenceDeg_ + headingOffsetDeg);
                driveOnHeading(wallTargetHeading, 100);
            }
            else
                driveOnHeading(navigationHeadingReferenceDeg_, 100);
            break;

        case NAV_CORNER_TURN:
            if (runHeadingTurn())
            {
                if (avoidBlockedTurnExit())
                    break;
                navigationState_ = NAV_SWEEP;
                navigationLaneIndex_ = 0;
                navigationSweepTurnRight_ = true;
                navigationSweepLeftReferenceValid_ = false;
                navigationSweepRightReferenceValid_ = false;
                navigationSweepLateralErrorMm_ = 0;
                navigationSweepProgressMm_ = 0.0f;
                navigationSweepProgressLastEncoder1_ = encoders_.firstCount();
                navigationSweepProgressLastEncoder2_ = encoders_.secondCount();
                beginForwardLeg(navigationTargetHeadingDeg_);
                link_.log("INFO", "Arena sweep lane 1 started");
            }
            break;

        case NAV_SWEEP:
            if (frontBlocked)
            {
                // A real arena end wall should occupy the broad horizontal
                // forward field. An internal wall/obstacle is considered
                // bypassable only when the 8x8 sees a clearly open outer
                // flank and the corresponding side ultrasound says there is
                // enough lateral room to start the manoeuvre.
                const bool leftGapOpen = matrixLeftOpenColumns >= 2;
                const bool rightGapOpen = matrixRightOpenColumns >= 2;
                const bool blockedMuchEarlierThanExpected =
                    navigationExpectedSweepLengthValid_ &&
                    navigationSweepProgressMm_ + OBSTACLE_EARLY_WALL_MARGIN_MM <
                        navigationExpectedSweepLengthMm_;
                const bool unexpectedObstacle = blockedMuchEarlierThanExpected ||
                    leftGapOpen || rightGapOpen;
                const bool canDetourLeft = unexpectedObstacle &&
                    left != 0xFFFF && left >= OBSTACLE_MIN_SIDE_ROOM_MM &&
                    (blockedMuchEarlierThanExpected || leftGapOpen);
                const bool canDetourRight = unexpectedObstacle &&
                    right != 0xFFFF && right >= OBSTACLE_MIN_SIDE_ROOM_MM &&
                    (blockedMuchEarlierThanExpected || rightGapOpen);

                if (canDetourLeft || canDetourRight)
                {
                    if (canDetourLeft && canDetourRight)
                    {
                        // Prefer the side with more verified lateral room. The
                        // 8x8 flank distance is used only as a tie-breaker.
                        if (right != left)
                            navigationDetourRight_ = right > left;
                        else
                            navigationDetourRight_ = matrixRightGapMm >= matrixLeftGapMm;
                    }
                    else
                        navigationDetourRight_ = canDetourRight;

                    navigationDetourOriginalHeadingDeg_ =
                        navigationHeadingReferenceDeg_;
                    navigationDetourOffsetMm_ = 0.0f;
                    navigationDetourEdgeCleared_ = false;
                    navigationDetourObstacleSeen_ = false;
                    navigationDetourClearSamples_ = 0;
                    ++navigationObstacleCount_;
                    setDrive(0, 0);
                    beginRightAngleTurn(
                        navigationDetourRight_, NAV_OBSTACLE_TURN_OUT);
                    link_.log(
                        "WARNING",
                        navigationDetourRight_
                            ? "Unexpected obstacle detected; detouring right"
                            : "Unexpected obstacle detected; detouring left");
                    break;
                }

                if (unexpectedObstacle)
                {
                    setDrive(0, 0);
                    startRecovery(
                    "Unexpected obstacle has no safe immediate detour; backing away to recover");
                    break;
                }

                // No unexpected-obstacle evidence exists, so handle the broad
                // return as the normal arena end wall. Completed long passes
                // teach a filtered expected arena length for later lanes.
                if (!navigationExpectedSweepLengthValid_)
                {
                    navigationExpectedSweepLengthMm_ = navigationSweepProgressMm_;
                    navigationExpectedSweepLengthValid_ =
                        navigationSweepProgressMm_ >= 600.0f;
                }
                else if (navigationSweepProgressMm_ >= 600.0f)
                {
                    navigationExpectedSweepLengthMm_ =
                        navigationExpectedSweepLengthMm_ * 0.75f +
                        navigationSweepProgressMm_ * 0.25f;
                }

                const bool remainingValid = navigationSweepTurnRight_
                    ? navigationSweepRightReferenceValid_
                    : navigationSweepLeftReferenceValid_;
                const uint16_t remainingMm = navigationSweepTurnRight_
                    ? navigationSweepRightReferenceMm_
                    : navigationSweepLeftReferenceMm_;

                if (!remainingValid)
                {
                    // Do not blindly step sideways when the boundary sensor is
                    // unavailable. Sit at the end wall until a usable side
                    // range is recovered.
                    navigationSweepLateralErrorMm_ = 0;
                    setDrive(0, 0);
                }
                else if (remainingMm <=
                         SWEEP_EDGE_TARGET_MM + SWEEP_EDGE_TOLERANCE_MM)
                {
                    navigationState_ = NAV_COMPLETE;
                    navigationSweepLateralErrorMm_ = 0;
                    setDrive(0, 0);
                    link_.log("INFO", "Arena sweep complete at opposite side wall");
                }
                else
                {
                    // Normally move 200 mm. On the last transition, shorten
                    // the lateral step so the next pass lands close to the
                    // opposite wall instead of overshooting it.
                    const float remainingShiftMm =
                        static_cast<float>(remainingMm - SWEEP_EDGE_TARGET_MM);
                    laneShiftTargetMm = min(LANE_SPACING_MM, remainingShiftMm);
                    navigationSweepLateralErrorMm_ = static_cast<int16_t>(
                        constrain(lroundf(laneShiftTargetMm), 0L, 5000L));
                    setDrive(0, 0);
                    beginRightAngleTurn(navigationSweepTurnRight_, NAV_LANE_TURN_OUT);
                }
            }
            else if (gapMayOverrideWall && rawFrontBlocked)
            {
                // One half-column is 2.5 degrees with the configured 40-degree
                // field. Aim through the centre while retaining IMU control.
                const float gapOffsetDeg = constrain(
                    navigationGapCentreColumnX2_ *
                        (navigationMatrixFovDeg_ / 16.0f), -20.0f, 20.0f);
                driveOnHeading(normaliseHeading(
                    navigationHeadingReferenceDeg_ + gapOffsetDeg), 100);
            }
            else if (leftBlocked && !rightBlocked)
                driveOnHeading(
                    normaliseHeading(navigationHeadingReferenceDeg_ + 7.0f),
                    100);
            else if (rightBlocked && !leftBlocked)
                driveOnHeading(
                    normaliseHeading(navigationHeadingReferenceDeg_ - 7.0f),
                    100);
            else
                driveOnHeading(navigationHeadingReferenceDeg_, 100);
            break;

        case NAV_ESCAPE_REVERSE:
        {
            const float reversedMm = encoderDistanceFrom(
                navigationEscapeStartEncoder1_, navigationEscapeStartEncoder2_);
            const bool leftOpen =
                recoveryLeft != 0xFFFF && recoveryLeft >= RECOVERY_SIDE_OPEN_MM;
            const bool rightOpen =
                recoveryRight != 0xFFFF && recoveryRight >= RECOVERY_SIDE_OPEN_MM;
            if ((reversedMm >= 350.0f && (leftOpen || rightOpen)) ||
                reversedMm >= 900.0f)
            {
                setDrive(0, 0);
                if (leftOpen || rightOpen)
                    navigationEscapeTurnRight_ = rightOpen &&
                        (!leftOpen || right >= left);
                beginRelativeTurnFromCurrent(
                    navigationEscapeTurnRight_, 90.0f, NAV_ESCAPE_TURN);
            }
            else
                setDrive(100, 100);
            break;
        }

        case NAV_ESCAPE_TURN:
            if (runHeadingTurn())
            {
                if (avoidBlockedTurnExit())
                    break;
                navigationState_ = NAV_SEEK_WALL;
                navigationLaneIndex_ = 0;
                navigationSweepProgressMm_ = 0.0f;
                navigationExpectedSweepLengthMm_ = 0.0f;
                navigationExpectedSweepLengthValid_ = false;
                navigationSweepLeftReferenceValid_ = false;
                navigationSweepRightReferenceValid_ = false;
                beginForwardLeg(navigationTargetHeadingDeg_);
                link_.log("INFO", "U-shaped enclosure cleared; seeking wall on new heading");
            }
            break;

        case NAV_OBSTACLE_TURN_OUT:
            if (runHeadingTurn())
            {
                if (avoidBlockedTurnExit())
                    break;
                navigationState_ = NAV_OBSTACLE_OFFSET;
                navigationDetourStartEncoder1_ = encoders_.firstCount();
                navigationDetourStartEncoder2_ = encoders_.secondCount();
                navigationDetourPhaseStartEncoder1_ = navigationDetourStartEncoder1_;
                navigationDetourPhaseStartEncoder2_ = navigationDetourStartEncoder2_;
                armDetourSideTracking();
                beginForwardLeg(navigationTargetHeadingDeg_);
            }
            break;

        case NAV_OBSTACLE_OFFSET:
        {
            const float totalOffsetMm = encoderDistanceFrom(
                navigationDetourStartEncoder1_, navigationDetourStartEncoder2_);
            if (frontBlocked && !navigationDetourEdgeCleared_)
            {
                startRecovery(
                    "Detour lateral route blocked; backing away to recover");
                break;
            }

            if (!navigationDetourEdgeCleared_ && detourSideJustCleared())
            {
                navigationDetourEdgeCleared_ = true;
                navigationDetourPhaseStartEncoder1_ = encoders_.firstCount();
                navigationDetourPhaseStartEncoder2_ = encoders_.secondCount();
            }

            if (totalOffsetMm >= OBSTACLE_MAX_OFFSET_MM &&
                !navigationDetourEdgeCleared_)
            {
                startRecovery(
                    "Detour edge not found within lateral limit; backing away to recover");
                break;
            }

            const float clearMarginMm = navigationDetourEdgeCleared_
                ? encoderDistanceFrom(navigationDetourPhaseStartEncoder1_,
                                      navigationDetourPhaseStartEncoder2_)
                : 0.0f;
            if (navigationDetourEdgeCleared_ &&
                (clearMarginMm >= OBSTACLE_CLEAR_MARGIN_MM || frontBlocked))
            {
                navigationDetourOffsetMm_ = totalOffsetMm;
                setDrive(0, 0);
                beginHeadingTurn(
                    navigationDetourOriginalHeadingDeg_,
                    NAV_OBSTACLE_TURN_FORWARD);
            }
            else
                driveOnHeading(navigationHeadingReferenceDeg_, 100);
            break;
        }

        case NAV_OBSTACLE_TURN_FORWARD:
            if (runHeadingTurn())
            {
                if (avoidBlockedTurnExit())
                    break;
                navigationState_ = NAV_OBSTACLE_PASS;
                navigationDetourPhaseStartEncoder1_ = encoders_.firstCount();
                navigationDetourPhaseStartEncoder2_ = encoders_.secondCount();
                armDetourSideTracking();
                beginForwardLeg(navigationDetourOriginalHeadingDeg_);
            }
            break;

        case NAV_OBSTACLE_PASS:
        {
            const float passDistanceMm = encoderDistanceFrom(
                navigationDetourPhaseStartEncoder1_,
                navigationDetourPhaseStartEncoder2_);
            if (frontBlocked)
            {
                startRecovery(
                    "Detour forward path blocked; backing away to recover");
                break;
            }

            if (!navigationDetourEdgeCleared_ && detourSideJustCleared())
            {
                navigationDetourEdgeCleared_ = true;
                navigationDetourPhaseStartEncoder1_ = encoders_.firstCount();
                navigationDetourPhaseStartEncoder2_ = encoders_.secondCount();
            }

            if (passDistanceMm >= OBSTACLE_MAX_PASS_MM &&
                !navigationDetourEdgeCleared_)
            {
                startRecovery(
                    "Obstacle extends beyond detour limit; backing away to recover");
                break;
            }

            const float clearMarginMm = navigationDetourEdgeCleared_
                ? encoderDistanceFrom(navigationDetourPhaseStartEncoder1_,
                                      navigationDetourPhaseStartEncoder2_)
                : 0.0f;
            if (navigationDetourEdgeCleared_ &&
                clearMarginMm >= OBSTACLE_CLEAR_MARGIN_MM)
            {
                setDrive(0, 0);
                const float returnHeading = normaliseHeading(
                    navigationDetourOriginalHeadingDeg_ +
                    (navigationDetourRight_ ? -90.0f : 90.0f));
                beginHeadingTurn(returnHeading, NAV_OBSTACLE_TURN_BACK);
            }
            else
                driveOnHeading(navigationHeadingReferenceDeg_, 100);
            break;
        }

        case NAV_OBSTACLE_TURN_BACK:
            if (runHeadingTurn())
            {
                if (avoidBlockedTurnExit())
                    break;
                navigationState_ = NAV_OBSTACLE_RETURN;
                navigationDetourPhaseStartEncoder1_ = encoders_.firstCount();
                navigationDetourPhaseStartEncoder2_ = encoders_.secondCount();
                beginForwardLeg(navigationTargetHeadingDeg_);
            }
            break;

        case NAV_OBSTACLE_RETURN:
        {
            const float returnedMm = encoderDistanceFrom(
                navigationDetourPhaseStartEncoder1_,
                navigationDetourPhaseStartEncoder2_);
            if (frontBlocked && returnedMm + 40.0f < navigationDetourOffsetMm_)
            {
                startRecovery(
                    "Detour return path blocked; backing away to recover");
                break;
            }
            if (returnedMm >= navigationDetourOffsetMm_)
            {
                setDrive(0, 0);
                beginHeadingTurn(
                    navigationDetourOriginalHeadingDeg_,
                    NAV_OBSTACLE_TURN_IN);
            }
            else
                driveOnHeading(navigationHeadingReferenceDeg_, 100);
            break;
        }

        case NAV_OBSTACLE_TURN_IN:
            if (runHeadingTurn())
            {
                if (avoidBlockedTurnExit())
                    break;
                navigationState_ = NAV_SWEEP;
                navigationSweepLeftReferenceValid_ = false;
                navigationSweepRightReferenceValid_ = false;
                navigationSweepLateralErrorMm_ = 0;
                navigationDetourEdgeCleared_ = false;
                navigationDetourObstacleSeen_ = false;
                navigationDetourClearSamples_ = 0;
                navigationSweepProgressLastEncoder1_ = encoders_.firstCount();
                navigationSweepProgressLastEncoder2_ = encoders_.secondCount();
                beginForwardLeg(navigationDetourOriginalHeadingDeg_);
                link_.log("INFO", "Obstacle cleared; returned to original sweep path");
            }
            break;

        case NAV_LANE_TURN_OUT:
            if (runHeadingTurn())
            {
                if (avoidBlockedTurnExit())
                    break;
                navigationState_ = NAV_LANE_SHIFT;
                navigationShiftStartEncoder1_ = encoders_.firstCount();
                navigationShiftStartEncoder2_ = encoders_.secondCount();
                beginForwardLeg(navigationTargetHeadingDeg_);
            }
            break;

        case NAV_LANE_SHIFT:
        {
            const float firstDistance = fabsf(
                (encoders_.firstCount() - navigationShiftStartEncoder1_) *
                ENCODER_1_MM_PER_COUNT);
            const float secondDistance = fabsf(
                (encoders_.secondCount() - navigationShiftStartEncoder2_) *
                ENCODER_2_MM_PER_COUNT);
            const float shiftedMm = (firstDistance + secondDistance) * 0.5f;
            if (shiftedMm >= laneShiftTargetMm || frontBlocked)
            {
                setDrive(0, 0);
                beginRightAngleTurn(navigationSweepTurnRight_, NAV_LANE_TURN_IN);
            }
            else
                driveOnHeading(navigationHeadingReferenceDeg_, 100);
            break;
        }

        case NAV_LANE_TURN_IN:
            if (runHeadingTurn())
            {
                if (avoidBlockedTurnExit())
                    break;
                ++navigationLaneIndex_;
                navigationSweepTurnRight_ = !navigationSweepTurnRight_;
                navigationState_ = NAV_SWEEP;
                navigationSweepLeftReferenceValid_ = false;
                navigationSweepRightReferenceValid_ = false;
                navigationSweepLateralErrorMm_ = 0;
                navigationSweepProgressMm_ = 0.0f;
                navigationSweepProgressLastEncoder1_ = encoders_.firstCount();
                navigationSweepProgressLastEncoder2_ = encoders_.secondCount();
                beginForwardLeg(navigationTargetHeadingDeg_);
                link_.log("INFO", "Next arena sweep lane started");
            }
            break;

        case NAV_RECOVERY_REVERSE:
        {
            const float reversedMm = encoderDistanceFrom(
                navigationRecoveryStartEncoder1_,
                navigationRecoveryStartEncoder2_);
            const bool leftOpen =
                recoveryLeft != 0xFFFF && recoveryLeft >= RECOVERY_SIDE_OPEN_MM;
            const bool rightOpen =
                recoveryRight != 0xFFFF && recoveryRight >= RECOVERY_SIDE_OPEN_MM;
            const bool sideOpen = leftOpen || rightOpen;

            if ((reversedMm >= RECOVERY_REVERSE_MM && sideOpen) ||
                reversedMm >= RECOVERY_MAX_REVERSE_MM)
            {
                setDrive(0, 0);
                navigationRecoveryTotalReverseMm_ += reversedMm;
                if (navigationRecoveryAttemptCount_ < 255U)
                    ++navigationRecoveryAttemptCount_;

                navigationRecoveryForceHalfTurn_ =
                    navigationRecoveryAttemptCount_ >= RECOVERY_MAX_ATTEMPTS ||
                    navigationRecoveryTotalReverseMm_ >=
                        RECOVERY_TOTAL_REVERSE_LIMIT_MM;

                if (navigationRecoveryForceHalfTurn_)
                {
                    // A bounded hard exit points the chassis toward a region it
                    // has already physically occupied, rather than accumulating
                    // arbitrarily more reverse distance.
                    beginRelativeTurnFromCurrent(
                        true, 180.0f, NAV_RECOVERY_TURN);
                    link_.log(
                        "WARNING",
                        "Recovery budget reached; forcing current-heading 180-degree replan");
                }
                else
                {
                    // The side choice was made before this reverse leg and is
                    // intentionally not recalculated here.
                    beginRelativeTurnFromCurrent(
                        navigationRecoveryTurnRight_, 90.0f, NAV_RECOVERY_TURN);
                }
            }
            else
                setDrive(100, 100);
            break;
        }

        case NAV_RECOVERY_TURN:
            if (runHeadingTurn())
            {
                if (frontBlocked)
                {
                    if (navigationRecoveryForceHalfTurn_)
                    {
                        // The hard exit has consumed the allowed recovery budget.
                        // Do not begin another long reverse/turn loop; restart
                        // wall acquisition from this materially different heading.
                        navigationState_ = NAV_SEEK_WALL;
                        navigationLaneIndex_ = 0;
                        navigationSweepProgressMm_ = 0.0f;
                        navigationExpectedSweepLengthMm_ = 0.0f;
                        navigationExpectedSweepLengthValid_ = false;
                        navigationSweepLeftReferenceValid_ = false;
                        navigationSweepRightReferenceValid_ = false;
                        navigationRecoveryAttemptCount_ = 0;
                        navigationRecoveryTotalReverseMm_ = 0.0f;
                        navigationRecoveryForceHalfTurn_ = false;
                        beginForwardLeg(
                            lastImuSampleValid_
                                ? imu_.headingDeg()
                                : navigationTargetHeadingDeg_);
                        link_.log(
                            "WARNING",
                            "Forced recovery heading still sees a wall; replanning from new heading without more reverse");
                        break;
                    }

                    navigationRecoveryStartEncoder1_ = encoders_.firstCount();
                    navigationRecoveryStartEncoder2_ = encoders_.secondCount();
                    // A failed 90-degree route forces the next attempt to the
                    // opposite side. The reverse state cannot overwrite this.
                    navigationRecoveryTurnRight_ = !navigationRecoveryTurnRight_;
                    navigationState_ = NAV_RECOVERY_REVERSE;
                    navigationMotionStartedMs_ = now;
                    navigationMotionConsistent_ = true;
                    setDrive(100, 100);
                    link_.log(
                        "WARNING",
                        "Recovery heading blocked; bounded back-off before opposite current-heading turn");
                    break;
                }

                navigationClearanceTurnCount_ = 0;
                navigationRecoveryAttemptCount_ = 0;
                navigationRecoveryTotalReverseMm_ = 0.0f;
                navigationRecoveryForceHalfTurn_ = false;
                navigationState_ = NAV_SEEK_WALL;
                navigationLaneIndex_ = 0;
                navigationSweepProgressMm_ = 0.0f;
                navigationExpectedSweepLengthMm_ = 0.0f;
                navigationExpectedSweepLengthValid_ = false;
                navigationSweepLeftReferenceValid_ = false;
                navigationSweepRightReferenceValid_ = false;
                beginForwardLeg(navigationTargetHeadingDeg_);
                link_.log(
                    "INFO",
                    "Recovery found a clear current-heading exit; reacquiring arena coverage");
            }
            break;

        case NAV_CLEARANCE_TURN:
            if (runHeadingTurn())
            {
                if (avoidBlockedTurnExit())
                    break;
                navigationClearanceTurnCount_ = 0;
                navigationState_ = NAV_SEEK_WALL;
                navigationLaneIndex_ = 0;
                navigationSweepProgressMm_ = 0.0f;
                navigationExpectedSweepLengthMm_ = 0.0f;
                navigationExpectedSweepLengthValid_ = false;
                navigationSweepLeftReferenceValid_ = false;
                navigationSweepRightReferenceValid_ = false;
                beginForwardLeg(navigationTargetHeadingDeg_);
                link_.log("INFO", "Front path clear after additional turn; reacquiring wall");
            }
            break;

        case NAV_COMPLETE:
            setDrive(0, 0);
            break;

        default:
            navigationState_ = NAV_SEEK_WALL;
            beginForwardLeg(imu_.headingDeg());
            break;
    }
}

void BluetoothDebugWorkflow::updateTof8x8()
{
    const uint32_t now = millis();
    if (now - lastTof8x8FrameMs_ < Tof8x8Config::FRAME_INTERVAL_MS)
        return;
    lastTof8x8FrameMs_ = now;

    if (tof8x8_.available())
        tof8x8_.read();
    // Navigation consumes every fresh sensor read, but the low-bandwidth BLE
    // debug link only needs one visualisation frame per second.
    if (now - lastTof8x8TransmitMs_ >= 1000U)
    {
        lastTof8x8TransmitMs_ = now;
        sendTof8x8Frame();
    }
}

void BluetoothDebugWorkflow::sendTof8x8Frame()
{
    JsonDocument message;
    message["type"] = "tof_8x8";
    message["time"] = millis();
    message["name"] = "matrix";
    message["available"] = tof8x8_.available();
    message["valid"] = tof8x8_.lastReadSucceeded();
    message["frame"] = tof8x8_.frameNumber();
    message["address"] = tof8x8_.detectedAddress();
    message["address_ack_mask"] = tof8x8_.addressAckMask();
    message["address_52_ack"] = tof8x8_.address52Acknowledged();
    message["bus"] = "I2C1";
    JsonArray i2cAddresses = message["i2c_addresses"].to<JsonArray>();
    for (uint8_t address = 0x08; address <= 0x77; ++address)
        if (tof8x8_.addressAcknowledged(address))
            i2cAddresses.add(address);

    JsonArray zones = message["data"].to<JsonArray>();
    if (tof8x8_.lastReadSucceeded())
        for (uint8_t i = 0; i < Tof8x8::ZONE_COUNT; ++i)
            zones.add(tof8x8_.data()[i]);

    link_.send(message, 2048);
}

JsonObject BluetoothDebugWorkflow::findParameter(const char* name)
{
    for (JsonObject parameter : config_["parameters"].as<JsonArray>())
        if (strcmp(parameter["name"] | "", name) == 0)
            return parameter;
    return JsonObject();
}

JsonObject BluetoothDebugWorkflow::findCommand(const char* name)
{
    for (JsonObject command : config_["commands"].as<JsonArray>())
        if (strcmp(command["name"] | "", name) == 0)
            return command;
    return JsonObject();
}


