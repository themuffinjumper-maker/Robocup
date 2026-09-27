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

// Range-to-map checks use the actual 600 × 130 mm wall, not its axis-aligned
// upload envelope (which older firmware can still treat conservatively).
float rayRotatedWall(float x, float y, float ux, float uy,
                     float cx, float cy, uint8_t angleDeg)
{
    const float angle = angleDeg * DEG_TO_RAD;
    const float cosine = cosf(angle), sine = sinf(angle);
    const float dx = x - cx, dy = y - cy;
    const float lx = dx * cosine + dy * sine;
    const float ly = -dx * sine + dy * cosine;
    const float lu = ux * cosine + uy * sine;
    const float lv = -ux * sine + uy * cosine;
    float nearT = -5000.0f, farT = 5000.0f;
    for (uint8_t axis = 0; axis < 2; ++axis)
    {
        const float origin = axis == 0 ? lx : ly;
        const float direction = axis == 0 ? lu : lv;
        const float half = axis == 0 ? 300.0f : 65.0f;
        if (fabsf(direction) < 0.0001f)
        {
            if (fabsf(origin) > half) return 5000.0f;
            continue;
        }
        const float a = (-half - origin) / direction;
        const float b = (half - origin) / direction;
        nearT = max(nearT, min(a, b));
        farT = min(farT, max(a, b));
    }
    const float hit = nearT > 0.0f ? nearT : farT;
    return farT >= nearT && hit > 0.0f ? hit : 5000.0f;
}
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
    updateArmSorting(now);
    if (sortingGateTorqueOffAtMs_ != 0 &&
        static_cast<int32_t>(now - sortingGateTorqueOffAtMs_) >= 0)
    {
        servos_.torqueOff(4);
        sortingGateTorqueOffAtMs_ = 0;
    }
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
    recoverTofSensorsIfNeeded();
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
            disarmArmSorting(false);
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
        missionOdometryRejectedSteps_ = 0;
        missionFeatureCount_ = 0;
        missionBottomSensorGeometryValid_ = false;
        missionUltrasoundGeometryValid_ = false;
        missionReturnHome_ = false;
        missionAutoReturnHome_ = false;
        missionDockTurnStartedMs_ = 0;
        missionDockAligned_ = false;
        missionUnloadPhase_ = 0;
        missionHomeValid_ = false;
        missionSearchWaypoint_ = 255;
        missionSearchStep_ = 0;
        missionVerifyPhase_ = 0;
        missionVerifyRetries_ = 0;
        link_.log("INFO", "Pre-laid mission plan cleared");
        return;
    }
    if (strcmp(commandName, "mission_map_set") == 0)
    {
        if (!debugMode_ || navigationActive_)
        {
            link_.error("Stop navigation and enter Debug Mode before replacing mission features");
            return;
        }
        if (!message["features"].is<JsonArray>())
        {
            link_.error("Mission map requires flat x0,y0,x1,y1,kind features");
            return;
        }
        JsonArray values = message["features"].as<JsonArray>();
        const int dummyExtraMm = message["dummy_extra_mm"] | 150;
        if (dummyExtraMm < 0 || dummyExtraMm > 500)
        {
            link_.error("Dummy extra berth must be 0-500 mm");
            return;
        }
        if (values.size() % 5 != 0 || values.size() / 5 > MAX_MISSION_FEATURES)
        {
            link_.error("Mission map exceeds 24 features or has an incomplete feature");
            return;
        }
        MissionFeature parsed[MAX_MISSION_FEATURES] = {};
        uint8_t count = 0;
        for (size_t i = 0; i < values.size(); i += 5)
        {
            const int x0 = values[i].as<int>();
            const int y0 = values[i + 1].as<int>();
            const int x1 = values[i + 2].as<int>();
            const int y1 = values[i + 3].as<int>();
            const int kind = values[i + 4].as<int>();
            if (x0 < 0 || y0 < 0 || x1 > 4900 || y1 > 2400 ||
                x0 >= x1 || y0 >= y1 || (kind < 0 || kind > 2))
            {
                link_.error("Invalid mission map rectangle");
                return;
            }
            parsed[count].x0 = x0;
            parsed[count].y0 = y0;
            parsed[count].x1 = x1;
            parsed[count].y1 = y1;
            parsed[count].kind = kind;
            ++count;
        }
        if (message["rotated_walls"].is<JsonArray>())
        {
            JsonArray walls = message["rotated_walls"].as<JsonArray>();
            if (walls.size() % 4 != 0 || walls.size() / 4 > count)
            {
                link_.error("Rotated walls require index,x,y,angle quadruples");
                return;
            }
            for (size_t i = 0; i < walls.size(); i += 4)
            {
                const int index = walls[i].as<int>();
                const int x = walls[i + 1].as<int>();
                const int y = walls[i + 2].as<int>();
                const int angle = walls[i + 3].as<int>();
                if (index < 0 || index >= count || parsed[index].kind != 1 ||
                    parsed[index].rotatedWall || x < 0 || x > 4900 ||
                    y < 0 || y > 2400 || angle < 0 || angle >= 180)
                {
                    link_.error("Invalid rotated wall feature");
                    return;
                }
                parsed[index].rotatedWall = true;
                parsed[index].wallX = static_cast<int16_t>(x);
                parsed[index].wallY = static_cast<int16_t>(y);
                parsed[index].rotationDeg = static_cast<uint8_t>(angle);
            }
        }
        MissionBottomSensorPose sensorPoses[4] = {};
        bool sensorGeometryValid = false;
        int16_t ultrasoundLateral[2] = {};
        int16_t ultrasoundForward[2] = {};
        bool ultrasoundGeometryValid = false;
        if (message["bottom_sensors"].is<JsonArray>())
        {
            JsonArray sensors = message["bottom_sensors"].as<JsonArray>();
            if (sensors.size() != 12U)
            {
                link_.error("Mission bottom sensor geometry requires four x,y,angle triples");
                return;
            }
            for (uint8_t i = 0; i < 4; ++i)
            {
                const int lateral = sensors[3U * i].as<int>();
                const int forward = sensors[3U * i + 1U].as<int>();
                const int angle = sensors[3U * i + 2U].as<int>();
                if (abs(lateral) > 600 || abs(forward) > 600 ||
                    abs(angle) > 180)
                {
                    link_.error("Mission bottom sensor geometry is outside robot bounds");
                    return;
                }
                sensorPoses[i].lateralMm = lateral;
                sensorPoses[i].forwardMm = forward;
                sensorPoses[i].angleDeg = angle;
            }
            sensorGeometryValid = true;
        }
        if (message["ultrasound_offsets"].is<JsonArray>())
        {
            JsonArray offsets = message["ultrasound_offsets"].as<JsonArray>();
            if (offsets.size() != 4U)
            {
                link_.error("Mission ultrasound offsets require two x,y pairs");
                return;
            }
            for (uint8_t i = 0; i < 2; ++i)
            {
                const int lateral = offsets[2U * i].as<int>();
                const int forward = offsets[2U * i + 1U].as<int>();
                if (abs(lateral) > 600 || abs(forward) > 600)
                {
                    link_.error("Mission ultrasound offset is outside robot bounds");
                    return;
                }
                ultrasoundLateral[i] = lateral;
                ultrasoundForward[i] = forward;
            }
            ultrasoundGeometryValid = true;
        }
        for (uint8_t i = 0; i < count; ++i) missionFeatures_[i] = parsed[i];
        missionFeatureCount_ = count;
        missionDummyExtraMm_ = static_cast<uint16_t>(dummyExtraMm);
        for (uint8_t i = 0; i < 4; ++i)
            missionBottomSensors_[i] = sensorPoses[i];
        missionBottomSensorGeometryValid_ = sensorGeometryValid;
        for (uint8_t i = 0; i < 2; ++i)
        {
            missionUltrasoundLateralMm_[i] = ultrasoundLateral[i];
            missionUltrasoundForwardMm_[i] = ultrasoundForward[i];
        }
        missionUltrasoundGeometryValid_ = ultrasoundGeometryValid;
        link_.log("INFO", "Pre-laid mission map features loaded");
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
        const int homeX = message["home_x_mm"] | -1;
        const int homeY = message["home_y_mm"] | -1;
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
        missionReturnHome_ = message["return_home"] | false;
        missionHomeValid_ = !missionReturnHome_ && homeX >= 0 &&
            homeX <= 4900 && homeY >= 0 && homeY <= 2400;
        missionHomeXmm_ = static_cast<int16_t>(missionHomeValid_ ? homeX : 0);
        missionHomeYmm_ = static_cast<int16_t>(missionHomeValid_ ? homeY : 0);
        missionAutoReturnHome_ = false;
        missionUnloadPhase_ = 0;
        missionTurnDirection_ = 0;
        missionPointTurnActive_ = false;
        missionTurnFinishing_ = false;
        missionTurnLastSampleMs_ = 0;
        missionTurnYawRateDegPerSec_ = 0.0f;
        missionTurnPulseUntilMs_ = 0;
        missionLastPointTurnMs_ = 0;
        missionFrontBlockedLatched_ = false;
        missionFrontClearSinceMs_ = 0;
        missionPlanValid_ = count > 0 && targets > 0;
        missionPoseInitialised_ = false;
        missionTargetAlignSinceMs_ = 0;
        missionTargetAligned_ = false;
        missionTargetAlignWaypoint_ = 255;
        missionWeightVectorActive_ = false;
        missionCollectionFollowThroughUntilMs_ = 0;
        missionHandledSortingSerial_ = sortingConfirmedSerial_;
        missionPendingPickupWaypoint_ = 255;
        missionPostPickupStraightUntilMs_ = 0;
        missionWeightVectorWaypoint_ = 255;
        missionWeightVectorCompletedWaypoint_ = 255;
        missionWeightVectorOvershootMm_ = 0;
        missionInnerWeightLocked_ = false;
        missionInnerAlignStartedMs_ = 0;
        missionInnerAlignFailedWaypoint_ = 255;
        missionBlockedSinceMs_ = 0;
        missionLastRerouteMs_ = 0;
        missionReroutePoseValid_ = false;
        missionTofRecoveryHold_ = false;
        missionRerouteCount_ = 0;
        missionSkippedWaypoints_ = 0;
        weightBearingWaypoint_ = 255;
        weightBearingSector_ = 255;
        missionSearchWaypoint_ = 255;
        missionSearchStep_ = 0;
        missionSearchSitesChecked_ = 0;
        missionSearchSitesIncomplete_ = 0;
        missionVerifyPhase_ = 0;
        missionVerifyWaypoint_ = 255;
        missionVerifyRetries_ = 0;
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
        disarmArmSorting(false);
        stopNavigation();
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
        // Stop propulsion and release all servo torque first. If sorting was
        // zeroed at reject, restore only gate 4 long enough to park it, then
        // release its torque in update() after the 500 ms position command.
        servos_.torqueOff(0xFE);
        if (sortingGateReferenceValid_)
        {
            setSortingGate(false, true);
            sortingGateTorqueOffAtMs_ = millis() + 550U;
        }
        stopped_ = true;
        sendState();
        link_.log("WARNING", "STOP received; servo torque disabled");
    }
    else if (strcmp(action, "run") == 0)
    {
        sortingGateTorqueOffAtMs_ = 0;
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
        if (armSortingEnabled_ && id == 4)
        {
            link_.error("Disarm sorting before commanding Herkulex 4 manually");
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
        if (armSortingEnabled_ && id == 4)
        {
            link_.error("Disarm sorting before re-zeroing Herkulex 4");
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
        if (id == 4)
            sortingGateReferenceValid_ = false;
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
        if (armSortingEnabled_ && id == 4)
        {
            link_.error("Disarm sorting before turning Herkulex 4 manually");
            return;
        }

        lastServoId_ = static_cast<uint8_t>(id);
        commandedVelocity_ = static_cast<int16_t>(speed);
        if (id == 4 && speed != 0)
            sortingGateReferenceValid_ = false;
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
        if (armSortingEnabled_ && id == 4)
        {
            link_.error("Disarm sorting before stopping Herkulex 4 manually");
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
            disarmArmSorting(false);
            if (sortingGateReferenceValid_)
            {
                setSortingBumpers(false);
                setSortingGate(false, true);
            }
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
        if (navigationStrategy_ == 3 &&
            (tofSensorCount_ == 0 || tofReadyCount_ != tofSensorCount_))
        {
            link_.error("Point TOFs unavailable; wait for automatic I2C0 recovery before starting mission");
            return;
        }
        navigationActive_ = true;
        // B is the storage-release gate: keep it shut throughout travel.
        hx12kB_.setAngle(50.0f);
        missionUnloadPhase_ = 0;
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
        missionTargetAlignSinceMs_ = 0;
        missionTargetAligned_ = false;
        missionTargetAlignWaypoint_ = 255;
        missionWeightVectorActive_ = false;
        missionWeightVectorWaypoint_ = 255;
        missionWeightVectorCompletedWaypoint_ = 255;
        missionWeightVectorOvershootMm_ = 0;
        missionInnerWeightLocked_ = false;
        missionInnerAlignStartedMs_ = 0;
        missionInnerAlignFailedWaypoint_ = 255;
        missionBlockedSinceMs_ = 0;
        missionLastRerouteMs_ = 0;
        missionRerouteCount_ = 0;
        missionSkippedWaypoints_ = 0;
        weightBearingWaypoint_ = 255;
        weightBearingSector_ = 255;
        missionSearchWaypoint_ = 255;
        missionSearchStep_ = 0;
        missionSearchSitesChecked_ = 0;
        missionSearchSitesIncomplete_ = 0;
        missionLandmarkRecoveryMatches_ = 0;
        missionDockTurnStartedMs_ = 0;
        missionDockAligned_ = false;
        missionUnloadPhase_ = 0;
        missionPendingPickupWaypoint_ = 255;
        missionPostPickupStraightUntilMs_ = 0;
        missionLandmarkRecoveryCount_ = 0;
        missionLandmarkRecoveryResidualMm_ = 0;
        missionOdometryRejectedSteps_ = 0;
        missionAvoidPhase_ = 0;
        missionAvoidAttempts_ = 0;
        missionWallReverseActive_ = false;
        missionWallReverseFailed_ = false;
        missionWallReverseNoProgress_ = false;
        missionWallReverseWaypoint_ = 255;
        missionWallReverseRepeatCount_ = 0;
        missionOuterCueWaypoint_ = 255;
        missionOuterCueTriedMask_ = 0;
        missionOuterCueActive_ = false;
        missionOuterCueAlignedSinceMs_ = 0;
        missionForwardProgressStartedMs_ = 0;
        missionForwardStalled_ = false;
        missionTurnProgressStartedMs_ = 0;
        missionLandmarkCorrections_ = 0;
        missionLastLandmarkId_ = -1;
        for (uint8_t i = 0; i < 2; ++i)
        {
            missionLastLandmarkEcho_[i] = ultrasoundSensors_[i].fallCount();
            missionLastLandmarkCorrectionMs_[i] = 0;
            missionLandmarkMatches_[i] = 0;
            missionLandmarkIds_[i] = -1;
        }
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
        if (navigationActive_ && selectB)
        {
            link_.error("Stop navigation before moving storage gate B manually");
            return;
        }
        if (armSortingEnabled_ && (selectC || selectD))
        {
            link_.error("Disarm sorting before moving HX12K C/D manually");
            return;
        }
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
        if (armSortingEnabled_ || navigationActive_)
        {
            link_.error("Stop navigation and disarm sorting before disabling HX12K outputs");
            return;
        }
        hx12kA_.disable();
        hx12kB_.disable();
        hx12kC_.disable();
        hx12kD_.disable();
        link_.log("INFO", "All HX12K pulse outputs disabled");
    }
    else if (strcmp(action, "hx12k_storage_gate") == 0)
    {
        if (stopped_ || navigationActive_)
        {
            link_.error("Run Robot and stop navigation before moving storage gate manually");
            return;
        }
        const bool open = message["open"] | false;
        hx12kB_.setAngle(open ? 10.0f : 50.0f);
        link_.log("INFO", open ? "Storage gate OPEN (HX12K B 10 deg)" :
                                   "Storage gate CLOSED (HX12K B 50 deg)");
        sendState();
    }
    else if (strcmp(action, "hx12k_bumpers") == 0)
    {
        if (armSortingEnabled_)
        {
            link_.error("Disarm sorting before moving bumpers manually");
            return;
        }
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
    else if (strcmp(action, "arm_sorting_toggle") == 0)
    {
        if (!message["enabled"].is<bool>())
        {
            link_.error("Arm sorting requires enabled=true or false");
            return;
        }
        if (!message["enabled"].as<bool>())
        {
            disarmArmSorting(!stopped_);
            link_.log("INFO", "Arm sorting OFF");
            sendState();
            return;
        }
        if (armSortingEnabled_)
        {
            link_.log("INFO", "Arm sorting is already ON");
            return;
        }
        if (stopped_)
        {
            link_.error("Press Run Robot before arming sorting");
            return;
        }
        if (continuousVelocityActive_ && lastServoId_ == 4)
        {
            link_.error("Stop Herkulex 4 continuous rotation before arming sorting");
            return;
        }
        int inputIndex = -1;
        for (uint8_t i = 0; i < digitalInputCount_; ++i)
            if (strcmp(digitalInputNames_[i], "inductive_proximity") == 0)
                inputIndex = i;
        if (inputIndex < 0)
        {
            link_.error("Arm sorting needs the inductive_proximity input");
            return;
        }
        if (sortingGateReferenceValid_)
        {
            // Reuse the previous reject reference to park before taking a
            // fresh zero. The first arm after startup needs the gate placed
            // at reject physically because no absolute reference exists yet.
            setSortingGate(false, true);
            delay(520);
        }
        const float zero = servos_.readAngle(4, 30);
        if (isnan(zero) ||
            zero < HerkulexConfig::MIN_ANGLE_DEG ||
            zero + 60.0f > HerkulexConfig::MAX_ANGLE_DEG)
        {
            link_.error("Cannot arm sorting: place gate at reject, then check Herkulex 4 and its +60 range");
            return;
        }
        // The gate must physically be at reject when sorting is armed.
        // Capture that position as relative zero for this run.
        servoZeroOffsetsDeg_[4] = zero;
        servoZeroed_[4] = true;
        sortingGateReferenceValid_ = true;
        sortingGateTorqueOffAtMs_ = 0;
        lastServoId_ = 4;
        lastServoAngleDeg_ = 0.0f;
        measuredServoAngleDeg_ = 0.0f;
        sortingInputIndex_ = static_cast<uint8_t>(inputIndex);
        sortingDetectionConfirmed_ = false;
        sortingDetectedSinceMs_ = 0;
        sortingClearedSinceMs_ = 0;
        sortingIdlePulseActive_ = false;
        sortingNextPulseMs_ = millis() + 20000U;
        armSortingEnabled_ = true;
        // Send initial states unconditionally, even if a previous session
        // ended at the same commanded angles.
        sortingBumpersOn_ = true;
        setSortingBumpers(false);
        sortingGatePositive_ = true;
        setSortingGate(false);
        link_.log("INFO", "Arm sorting ON: reject gate zeroed; bumpers OFF; gate 0");
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
    else if (strcmp(name, "navigation.matrix_wall_mm") == 0)
        navigationMatrixWallMm_ = parameter["value"].as<uint16_t>();
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

void BluetoothDebugWorkflow::setSortingBumpers(bool enabled)
{
    if (sortingBumpersOn_ == enabled) return;
    hx12kC_.setAngle(enabled ? 0.0f : 130.0f);
    hx12kD_.setAngle(enabled ? 130.0f : 0.0f);
    sortingBumpersOn_ = enabled;
}

void BluetoothDebugWorkflow::setSortingGate(bool positive, bool force)
{
    if (!sortingGateReferenceValid_ || (!force && sortingGatePositive_ == positive)) return;
    const float relativeAngle = positive ? 60.0f : 0.0f;
    const float absoluteAngle = servoZeroOffsetsDeg_[4] + relativeAngle;
    constexpr uint16_t MOVE_MS = 500;
    // Match the proven Herkulex position-mode sequence used by the manual
    // relative-angle command. The first JOG selects position mode; the second
    // executes after torque is restored.
    servos_.torqueOff(4);
    delay(5);
    servos_.clearError(4);
    servos_.moveAngle(4, absoluteAngle, MOVE_MS, HerkulexTeensy::LED_BLUE);
    delay(5);
    servos_.torqueOn(4);
    delay(5);
    servos_.moveAngle(4, absoluteAngle, MOVE_MS, HerkulexTeensy::LED_BLUE);
    lastServoId_ = 4;
    lastServoAngleDeg_ = relativeAngle;
    lastMoveStartMs_ = millis();
    lastMoveDurationMs_ = MOVE_MS;
    sortingGatePositive_ = positive;
}

void BluetoothDebugWorkflow::disarmArmSorting(bool restoreOutputs)
{
    if (!armSortingEnabled_) return;
    armSortingEnabled_ = false;
    sortingDetectionConfirmed_ = false;
    sortingDetectedSinceMs_ = 0;
    sortingClearedSinceMs_ = 0;
    sortingIdlePulseActive_ = false;
    sortingNextPulseMs_ = 0;
    sortingPulseEndMs_ = 0;
    if (restoreOutputs)
    {
        setSortingBumpers(false);
        setSortingGate(false);
    }
}

void BluetoothDebugWorkflow::updateArmSorting(uint32_t now)
{
    if (!armSortingEnabled_) return;
    if (stopped_ || !debugMode_)
    {
        disarmArmSorting(false);
        return;
    }
    const bool detected = digitalInputs_[sortingInputIndex_].detected();
    if (detected)
    {
        sortingClearedSinceMs_ = 0;
        if (sortingDetectedSinceMs_ == 0) sortingDetectedSinceMs_ = now;
        if (sortingDetectionConfirmed_)
            setSortingBumpers(true);
        if (!sortingDetectionConfirmed_ &&
            now - sortingDetectedSinceMs_ >= 500U)
        {
            sortingDetectionConfirmed_ = true;
            ++sortingConfirmedSerial_;
            sortingIdlePulseActive_ = false;
            setSortingBumpers(true);
            setSortingGate(true);
            link_.log("INFO", "Inductive detection confirmed: bumpers ON, gate +60");
        }
        return;
    }

    sortingDetectedSinceMs_ = 0;
    if (sortingDetectionConfirmed_)
    {
        if (sortingClearedSinceMs_ == 0) sortingClearedSinceMs_ = now;
        if (now - sortingClearedSinceMs_ >= 2000U)
            setSortingBumpers(false);
        if (now - sortingClearedSinceMs_ >= 5000U)
        {
            setSortingGate(false);
            sortingDetectionConfirmed_ = false;
            sortingClearedSinceMs_ = 0;
            sortingNextPulseMs_ = now + 20000U;
            link_.log("INFO", "Inductive target cleared: gate 0; idle pulse timer restarted");
        }
        return;
    }

    if (sortingIdlePulseActive_)
    {
        if (static_cast<int32_t>(now - sortingPulseEndMs_) >= 0)
        {
            setSortingBumpers(false);
            sortingIdlePulseActive_ = false;
        }
    }
    else if (static_cast<int32_t>(now - sortingNextPulseMs_) >= 0)
    {
        setSortingBumpers(true);
        sortingIdlePulseActive_ = true;
        sortingPulseEndMs_ = now + 1000U;
        sortingNextPulseMs_ = now + 20000U;
    }
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
    data["system.navigation_controller_version"] = 21;
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
    data["sorting.armed"] = armSortingEnabled_;
    data["sorting.detected"] = armSortingEnabled_ &&
        digitalInputs_[sortingInputIndex_].detected();
    data["sorting.confirmed"] = sortingDetectionConfirmed_;
    data["sorting.bumpers_on"] = armSortingEnabled_ && sortingBumpersOn_;
    data["sorting.gate_angle_deg"] = sortingGatePositive_ ? 60 : 0;
    data["sorting.idle_pulse"] = sortingIdlePulseActive_;
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
    data["navigation.matrix_wall_mm"] = navigationMatrixWallMm_;
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
    const char* missionPhase = !navigationActive_ ? "idle" :
        navigationStrategy_ != 3 ? "exploring" :
        missionUnloadPhase_ != 0 ? "unloading_at_home" :
        missionVerifyPhase_ == 5 ? "reversing_for_weight_retry" :
        missionVerifyPhase_ != 0 ? "checking_pickup" :
        missionTofRecoveryHold_ ? "waiting_for_point_tofs" :
        missionWallReverseActive_ ? "reversing_from_wall" :
        missionReturnHome_ && missionWaypointIndex_ < missionWaypointCount_ &&
            (missionWaypoints_[missionWaypointIndex_].flags & 0x04) != 0
            ? "backing_into_home" :
        missionReturnHome_ ? "returning_home" :
        missionOuterCueActive_ ? "checking_outer_weight_cue" :
        missionInnerAlignStartedMs_ != 0 ? "centering_weight" :
        missionWeightVectorActive_ ?
            (missionTargetAligned_ ? "collecting_weight" : "aligning_to_weight") :
        missionSearchWaypoint_ == missionWaypointIndex_ ? "searching_weight" :
        missionAvoidPhase_ != 0 ? "avoiding_obstacle" :
        missionBlockedSinceMs_ != 0 ? "blocked" : "following_route";
    data["mission.phase"] = missionPhase;
    data["mission.waypoint_index"] = missionWaypointIndex_;
    data["mission.waypoint_count"] = missionWaypointCount_;
    if (missionWaypointIndex_ < missionWaypointCount_)
    {
        data["mission.current_waypoint_x_mm"] = missionWaypoints_[missionWaypointIndex_].xMm;
        data["mission.current_waypoint_y_mm"] = missionWaypoints_[missionWaypointIndex_].yMm;
    }
    data["mission.targets_visited"] = missionTargetsVisited_;
    data["mission.returning_home"] = missionReturnHome_ && navigationActive_;
    data["mission.unload_phase"] = missionUnloadPhase_;
    data["mission.target_count"] = missionTargetCount_;
    data["mission.pose_x_mm"] = missionPoseXmm_;
    data["mission.pose_y_mm"] = missionPoseYmm_;
    data["mission.heading_deg"] = missionArenaHeadingDeg_;
    data["mission.pose_live"] = missionPoseInitialised_ && lastImuSampleValid_;
    data["mission.target_aligned"] = missionTargetAligned_;
    data["weight.vector_active"] = missionWeightVectorActive_;
    data["weight.inner_locked"] = missionInnerWeightLocked_;
    data["weight.outer_cue_active"] = missionOuterCueActive_;
    if (weightInnerLeftMm_ != 0xFFFF)
        data["weight.inner_left_mm"] = weightInnerLeftMm_;
    if (weightInnerRightMm_ != 0xFFFF)
        data["weight.inner_right_mm"] = weightInnerRightMm_;
    if (weightInnerLeftMm_ != 0xFFFF && weightInnerRightMm_ != 0xFFFF)
        data["weight.inner_delta_mm"] =
            static_cast<int16_t>(weightInnerLeftMm_) - weightInnerRightMm_;
    data["weight.vector_overshoot_mm"] = missionWeightVectorOvershootMm_;
    if (missionWeightVectorWaypoint_ != 255)
    {
        data["weight.vector_hit_x_mm"] = missionWeightVectorHitXmm_;
        data["weight.vector_hit_y_mm"] = missionWeightVectorHitYmm_;
        data["weight.vector_end_x_mm"] = missionWeightVectorEndXmm_;
        data["weight.vector_end_y_mm"] = missionWeightVectorEndYmm_;
        data["weight.vector_heading_deg"] = missionWeightVectorHeadingDeg_;
        data["weight.vector_progress_mm"] =
            (missionPoseXmm_ - missionWeightVectorStartXmm_) *
                cosf(missionWeightVectorHeadingDeg_ * DEG_TO_RAD) +
            (missionPoseYmm_ - missionWeightVectorStartYmm_) *
                sinf(missionWeightVectorHeadingDeg_ * DEG_TO_RAD);
    }
    data["mission.odometry_rejected_steps"] = missionOdometryRejectedSteps_;
    data["mission.blocked"] = missionBlockedSinceMs_ != 0;
    data["mission.wall_reverse_active"] = missionWallReverseActive_;
    data["mission.wall_reverse_failed"] = missionWallReverseFailed_;
    data["mission.forward_stalled"] = missionForwardStalled_;
    data["mission.avoid_phase"] = missionAvoidPhase_;
    data["mission.reroute_count"] = missionRerouteCount_;
    data["mission.skipped_route_points"] = missionSkippedWaypoints_;
    data["mission.search_active"] = missionSearchWaypoint_ == missionWaypointIndex_;
    data["mission.search_step"] = missionSearchStep_;
    data["mission.search_samples"] = missionSearchSamples_;
    data["mission.search_valid_range_samples"] = missionSearchCoverageSamples_;
    data["mission.search_sites_checked"] = missionSearchSitesChecked_;
    data["mission.search_sites_incomplete"] = missionSearchSitesIncomplete_;
    data["mission.landmark_recovery_count"] = missionLandmarkRecoveryCount_;
    data["mission.landmark_recovery_residual_mm"] = missionLandmarkRecoveryResidualMm_;
    data["mission.landmark_corrections"] = missionLandmarkCorrections_;
    data["mission.last_landmark_id"] = missionLastLandmarkId_;
    data["weight.detected"] = weightSectorMask_ != 0;
    data["weight.sector_mask"] = weightSectorMask_;
    data["weight.geometry_valid"] = missionBottomSensorGeometryValid_;
    data["weight.valid_mask"] = weightValidMask_;
    data["weight.gap_mask"] = weightGapMask_;
    data["weight.target_mask"] = weightTargetMask_;
    data["weight.map_wall_mask"] = weightMapWallMask_;
    data["weight.nearest_mm"] = weightNearestMm_;
    data["weight.direction"] = weightDirection_;
    const bool weightBearingFresh = weightBearingWaypoint_ == missionWaypointIndex_ &&
        (now - weightBearingSeenMs_) <= 2500U;
    data["weight.bearing_fresh"] = weightBearingFresh;
    data["weight.bearing_waypoint_index"] = weightBearingWaypoint_;
    data["weight.bearing_active"] = weightBearingSteeringActive_;
    data["weight.bearing_sector"] = weightBearingSector_;
    data["weight.vector_waypoint_index"] = missionWeightVectorWaypoint_;
    data["weight.front_target_visible"] = missionFrontTargetVisible_;
    data["weight.target_commit_active"] = missionTargetCommitActive_;
    data["weight.front_target_range_error_mm"] = missionFrontTargetRangeErrorMm_;
    data["weight.front_target_mapped_wall_mm"] = missionFrontTargetMappedWallMm_;
    if (weightBearingFresh)
    {
        data["weight.bearing_hit_x_mm"] = weightBearingHitXmm_;
        data["weight.bearing_hit_y_mm"] = weightBearingHitYmm_;
        data["weight.bearing_error_mm"] = weightBearingErrorMm_;
    }
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
    data["tof.bank_ready_count"] = tofReadyCount_;
    data["tof.bank_configured_count"] = tofSensorCount_;
    data["tof.expander_available"] = Tof::expanderAvailable();
    data["tof.recovery_attempts"] = tofRecoveryAttempts_;
    data["tof.last_recovery_success_ms"] = lastTofRecoverySuccessMs_;
    data["tof.navigation_hold"] = missionTofRecoveryHold_;
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
    // A complete motion frame reached 6,137 bytes in the recorded run. The
    // next weight-vector fields pushed it over RobotDebug's 6,144-byte line
    // buffer, so the *entire* primary frame vanished while the smaller
    // ultrasound diagnostic packet continued. Send bounded chunks with the
    // same timestamp; the GUI merges those into one sensor frame.
    JsonDocument chunk;
    chunk["type"] = "telemetry";
    chunk["time"] = now;
    JsonObject chunkData = chunk["data"].to<JsonObject>();
    for (JsonPair field : data)
    {
        const char* key = field.key().c_str();
        chunkData[key] = field.value();
        if (measureJson(chunk) > 3000U)
        {
            chunkData.remove(key);
            if (!chunkData.isNull() && chunkData.size() > 0)
                link_.send(chunk, 768);
            chunk.clear();
            chunk["type"] = "telemetry";
            chunk["time"] = now;
            chunkData = chunk["data"].to<JsonObject>();
            chunkData[key] = field.value();
        }
    }
    if (chunkData.size() > 0)
        link_.send(chunk, 768);
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
    link_.send(diagnosticMessage, 768);
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
    tofReadyCount_ = 0;
    for (uint8_t i = 0; i < tofSensorCount_; ++i)
    {
        tofAvailable_[i] = Tof::available(i);
        tofTimedOut_[i] = !tofAvailable_[i];
        tofDistanceMm_[i] = -1;
        if (tofAvailable_[i])
            ++tofReadyCount_;
    }
    tofBadPolls_ = tofReadyCount_ == tofSensorCount_ ? 0 : 10;
    lastTofRecoveryAttemptMs_ = millis();
    if (tofReadyCount_ == tofSensorCount_)
        lastTofRecoverySuccessMs_ = millis();

    link_.log("INFO", "JSON-configured TOF sensor setup complete");
}

void BluetoothDebugWorkflow::readTofSensors()
{
    tofReadyCount_ = 0;
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
        tofTimedOut_[i] = Tof::timedOut(i) || tofDistanceMm_[i] < 0;
        if (!tofTimedOut_[i] && tofDistanceMm_[i] >= 0)
            ++tofReadyCount_;
    }
    if (tofReadyCount_ == tofSensorCount_)
    {
        tofBadPolls_ = 0;
        missionTofRecoveryHold_ = false;
    }
    else if (tofBadPolls_ < 255)
        ++tofBadPolls_;
    updateWeightDetection();
}

void BluetoothDebugWorkflow::recoverTofSensorsIfNeeded()
{
    if (tofSensorCount_ == 0 || tofBadPolls_ < 10)
        return;
    // Preserve the mission state while the bus resets. The robot must not
    // drive blind, but a transient TOF outage should not drop it into idle.
    if (navigationActive_ && navigationStrategy_ == 3)
    {
        dcMotor203Second_.stop();
        dcMotor203SecondActive_ = false;
        if (!missionTofRecoveryHold_)
            link_.log("WARNING", "Point TOF bank offline; holding navigation for automatic recovery");
        missionTofRecoveryHold_ = true;
    }

    const uint32_t now = millis();
    if (now - lastTofRecoveryAttemptMs_ < 10000U)
        return;
    lastTofRecoveryAttemptMs_ = now;
    if (tofRecoveryAttempts_ < 65535U)
        ++tofRecoveryAttempts_;
    const bool allReady = Tof::retry();
    tofReadyCount_ = 0;
    for (uint8_t i = 0; i < tofSensorCount_; ++i)
    {
        tofAvailable_[i] = Tof::available(i);
        tofTimedOut_[i] = !tofAvailable_[i];
        tofDistanceMm_[i] = -1;
        if (tofAvailable_[i])
            ++tofReadyCount_;
    }
    if (allReady && tofReadyCount_ == tofSensorCount_)
    {
        tofBadPolls_ = 1; // Resume only after an actual post-reset range poll.
        lastTofRecoverySuccessMs_ = millis();
        link_.log("INFO", "Point TOF bank recovered; all XSHUT sensors readdressed");
    }
    else
        link_.log("WARNING", Tof::expanderAvailable()
            ? "Point TOF retry incomplete; checking XSHUT sensors again in 10 s"
            : "I2C0 SX1509 expander not responding; retrying point TOF bank in 10 s");
}

void BluetoothDebugWorkflow::updateWeightDetection()
{
    static const char* BOTTOM_NAMES[4] = {
        "bottom_left_left", "bottom_mid_left",
        "bottom_mid_right", "bottom_right_right"
    };
    static const char* TOP_NAMES[4] = {
        "top_left_left", "top_mid_left", "top_mid_right", "top_right_right"
    };
    static const int8_t DIRECTIONS[4] = {-2, -1, 1, 2};
    constexpr uint16_t MAX_WEIGHT_RANGE_MM = 1200;
    constexpr uint16_t MIN_TOP_BOTTOM_GAP_MM = 150;
    constexpr uint8_t CONFIRM_POLLS = 3;
    constexpr float TARGET_RADIUS_MM = 300.0f;

    weightSectorMask_ = 0;
    weightValidMask_ = 0;
    weightGapMask_ = 0;
    weightTargetMask_ = 0;
    weightMapWallMask_ = 0;
    weightNearestMm_ = 0;
    weightDirection_ = 0;
    weightInnerLeftMm_ = 0xFFFF;
    weightInnerRightMm_ = 0xFFFF;
    // The broad 300 mm visibility gate is not accurate enough for steering.
    // 170 mm plus the 130 mm arrival radius still keeps a sensed approach
    // within the requested 300 mm of the planned target.
    const bool postCollectionLook = missionVerifyPhase_ != 0 ||
        missionWeightVectorCompletedWaypoint_ == missionWaypointIndex_;
    float bestBearingErrorMm = postCollectionLook ? 321.0f : 301.0f;
    bool bestBearingInner = false;
    float innerHitX[2] = {};
    float innerHitY[2] = {};
    auto expectedMappedWallRange = [&](uint8_t sector) -> float {
        if (!missionBottomSensorGeometryValid_ || !missionPoseInitialised_)
            return 5000.0f;
        const MissionBottomSensorPose& sensor = missionBottomSensors_[sector];
        const float heading = missionArenaHeadingDeg_ * DEG_TO_RAD;
        // The Arena View uploads x as robot-right-positive. Its world
        // projection is +sin(heading)*x, -cos(heading)*x; keep the Teensy
        // projection identical so the inward-facing inner beams intersect.
        const float originX = missionPoseXmm_ +
            cosf(heading) * sensor.forwardMm + sinf(heading) * sensor.lateralMm;
        const float originY = missionPoseYmm_ +
            sinf(heading) * sensor.forwardMm - cosf(heading) * sensor.lateralMm;
        const float ray = (missionArenaHeadingDeg_ + sensor.angleDeg) * DEG_TO_RAD;
        const float ux = cosf(ray), uy = sinf(ray);
        float nearest = 5000.0f;
        // Slab intersection with physical walls; no-go home zones are not
        // physical surfaces. Include the four real arena boundaries.
        auto intersect = [&](float x0, float y0, float x1, float y1) {
            float enter = -5000.0f, leave = 5000.0f;
            if (fabsf(ux) < 0.0001f)
            {
                if (originX < x0 || originX > x1) return;
            }
            else
            {
                const float a = (x0 - originX) / ux;
                const float b = (x1 - originX) / ux;
                enter = max(enter, min(a, b));
                leave = min(leave, max(a, b));
            }
            if (fabsf(uy) < 0.0001f)
            {
                if (originY < y0 || originY > y1) return;
            }
            else
            {
                const float a = (y0 - originY) / uy;
                const float b = (y1 - originY) / uy;
                enter = max(enter, min(a, b));
                leave = min(leave, max(a, b));
            }
            const float hit = enter > 0.0f ? enter : leave;
            if (leave >= enter && hit > 0.0f)
                nearest = min(nearest, hit);
        };
        intersect(-1.0f, -1.0f, 0.0f, 2401.0f);
        intersect(4900.0f, -1.0f, 4901.0f, 2401.0f);
        intersect(0.0f, -1.0f, 4900.0f, 0.0f);
        intersect(0.0f, 2400.0f, 4900.0f, 2401.0f);
        for (uint8_t i = 0; i < missionFeatureCount_; ++i)
        {
            const MissionFeature& feature = missionFeatures_[i];
            if (feature.kind != 0)
            {
                if (feature.rotatedWall)
                    nearest = min(nearest, rayRotatedWall(originX, originY,
                        ux, uy, feature.wallX, feature.wallY,
                        feature.rotationDeg));
                else
                    intersect(feature.x0, feature.y0, feature.x1, feature.y1);
            }
        }
        return nearest;
    };
    for (uint8_t sector = 0; sector < 4; ++sector)
    {
        int8_t bottomIndex = -1;
        int8_t topIndex = -1;
        for (uint8_t i = 0; i < tofSensorCount_; ++i)
        {
            if (strcmp(tofSensorNames_[i], BOTTOM_NAMES[sector]) == 0)
                bottomIndex = static_cast<int8_t>(i);
            if (strcmp(tofSensorNames_[i], TOP_NAMES[sector]) == 0)
                topIndex = static_cast<int8_t>(i);
        }

        bool validPair = false;
        bool heightGap = false;
        uint16_t bottomMm = 0;
        if (sector >= 1 && sector <= 2 && bottomIndex >= 0 &&
            tofAvailable_[bottomIndex] && !tofTimedOut_[bottomIndex] &&
            tofDistanceMm_[bottomIndex] >= 30 &&
            tofDistanceMm_[bottomIndex] <= MAX_WEIGHT_RANGE_MM)
        {
            if (sector == 1)
                weightInnerLeftMm_ = tofDistanceMm_[bottomIndex];
            else
                weightInnerRightMm_ = tofDistanceMm_[bottomIndex];
        }
        if (bottomIndex >= 0 && topIndex >= 0 &&
            tofAvailable_[bottomIndex] && !tofTimedOut_[bottomIndex] &&
            tofAvailable_[topIndex] && !tofTimedOut_[topIndex])
        {
            const int bottom = tofDistanceMm_[bottomIndex];
            const int top = tofDistanceMm_[topIndex];
            if (bottom >= 30 && bottom <= MAX_WEIGHT_RANGE_MM && top >= 30)
            {
                bottomMm = static_cast<uint16_t>(bottom);
                validPair = true;
                heightGap = top >= bottom + MIN_TOP_BOTTOM_GAP_MM;
            }
        }
        const uint8_t bit = static_cast<uint8_t>(1U << sector);
        if (validPair) weightValidMask_ |= bit;
        if (heightGap) weightGapMask_ |= bit;

        // Compare the measured hit point, not the robot centre or the ray
        // angle from that centre, with the pre-laid weight location. These
        // sensors are mounted up to 305 mm forward and far out to the sides.
        bool nearPlannedWeight = false;
        float hitX = 0.0f;
        float hitY = 0.0f;
        float currentTargetErrorMm = 5000.0f;
        if (validPair && missionPoseInitialised_ && missionPlanValid_ &&
            missionBottomSensorGeometryValid_ && !missionReturnHome_)
        {
            const MissionBottomSensorPose& sensor = missionBottomSensors_[sector];
            const float heading = missionArenaHeadingDeg_ * DEG_TO_RAD;
            const float originX = missionPoseXmm_ +
                cosf(heading) * sensor.forwardMm + sinf(heading) * sensor.lateralMm;
            const float originY = missionPoseYmm_ +
                sinf(heading) * sensor.forwardMm - cosf(heading) * sensor.lateralMm;
            const float ray = (missionArenaHeadingDeg_ + sensor.angleDeg) * DEG_TO_RAD;
            hitX = originX + bottomMm * cosf(ray);
            hitY = originY + bottomMm * sinf(ray);
            for (uint8_t waypoint = 0; waypoint < missionWaypointCount_; ++waypoint)
            {
                if ((missionWaypoints_[waypoint].flags & 0x01) == 0)
                    continue;
                const float dx = missionWaypoints_[waypoint].xMm - hitX;
                const float dy = missionWaypoints_[waypoint].yMm - hitY;
                const float errorMm = sqrtf(dx * dx + dy * dy);
                const bool outerPostPass = (sector == 0 || sector == 3) &&
                    postCollectionLook && waypoint == missionWaypointIndex_ &&
                    bottomMm <= 450U && errorMm <= 450.0f;
                if (errorMm <= TARGET_RADIUS_MM || outerPostPass)
                {
                    nearPlannedWeight = true;
                    if (waypoint == missionWaypointIndex_)
                        currentTargetErrorMm = errorMm;
                }
            }
        }
        if (nearPlannedWeight) weightTargetMask_ |= bit;

        bool lowObject = validPair && heightGap && nearPlannedWeight;

        if (lowObject)
        {
            const float wallMm = expectedMappedWallRange(sector);
            if (wallMm <= MAX_WEIGHT_RANGE_MM &&
                fabsf(wallMm - bottomMm) <= 120.0f)
            {
                weightMapWallMask_ |= static_cast<uint8_t>(1U << sector);
                lowObject = false;
            }
        }

        if (lowObject)
        {
            if (weightEvidence_[sector] < CONFIRM_POLLS)
                ++weightEvidence_[sector];
        }
        else
            weightEvidence_[sector] = 0;

        // An outer sensor can see a passing weight for only a few polls.
        // Two qualified, mapped-to-target returns are enough for recovery;
        // ordinary wall/floor echoes still require three confirmations.
        const uint8_t requiredPolls = (sector == 0 || sector == 3) &&
            nearPlannedWeight && bottomMm <= 450U ? 2U : CONFIRM_POLLS;
        if (weightEvidence_[sector] >= requiredPolls)
        {
            weightSectorMask_ |= static_cast<uint8_t>(1U << sector);
            if (sector == 1 || sector == 2)
            {
                innerHitX[sector - 1] = hitX;
                innerHitY[sector - 1] = hitY;
            }
            // Only a well-localised return associated with the active target
            // steers the route. The persistent far-left wall/floor echo in
            // recorded runs passed the broad 300 mm visibility gate but lay
            // 244-284 mm from the actual targets; it must not redirect drive.
            const bool inner = sector == 1 || sector == 2;
            const bool outerTargetHit = !inner && bottomMm <= 450U &&
                currentTargetErrorMm <= (postCollectionLook ? 320.0f : 300.0f);
            if ((inner ? currentTargetErrorMm <= 220.0f : outerTargetHit) &&
                ((inner && !bestBearingInner) ||
                 (inner == bestBearingInner &&
                  currentTargetErrorMm < bestBearingErrorMm)))
            {
                bestBearingErrorMm = currentTargetErrorMm;
                bestBearingInner = inner;
                weightBearingHitXmm_ = hitX;
                weightBearingHitYmm_ = hitY;
                weightBearingErrorMm_ = currentTargetErrorMm;
                weightBearingWaypoint_ = missionWaypointIndex_;
                weightBearingSector_ = sector;
                weightBearingSeenMs_ = millis();
            }
            if (weightNearestMm_ == 0 || bottomMm < weightNearestMm_)
            {
                weightNearestMm_ = bottomMm;
                weightDirection_ = DIRECTIONS[sector];
            }
        }
    }
    if ((weightSectorMask_ & 0x06U) == 0x06U &&
        missionWaypointIndex_ < missionWaypointCount_)
    {
        // These two inner sensors point inward. Still compare their
        // projected hit locations before treating equal ranges as one
        // centred weight, since each beam may see a different surface.
        const float pairSeparation = hypotf(innerHitX[0] - innerHitX[1],
                                            innerHitY[0] - innerHitY[1]);
        const float hitX = 0.5f * (innerHitX[0] + innerHitX[1]);
        const float hitY = 0.5f * (innerHitY[0] + innerHitY[1]);
        const float error = hypotf(hitX - missionWaypoints_[missionWaypointIndex_].xMm,
                                   hitY - missionWaypoints_[missionWaypointIndex_].yMm);
        if (pairSeparation <= 220.0f && error <= 200.0f)
        {
            weightBearingHitXmm_ = hitX;
            weightBearingHitYmm_ = hitY;
            weightBearingErrorMm_ = error;
            weightBearingWaypoint_ = missionWaypointIndex_;
            weightBearingSector_ = 4;  // fused inner pair
            weightBearingSeenMs_ = millis();
        }
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
    // Update position before converting a fresh bottom-TOF return into a
    // robot-relative weight vector. This also runs outside mission mode.
    updateMissionPose(now);
    if (now - lastRangePollMs_ >= 100U)
    {
        lastRangePollMs_ = now;
        readTofSensors();
    }
}

void BluetoothDebugWorkflow::stopNavigation(const char* reason)
{
    navigationActive_ = false;
    missionTofRecoveryHold_ = false;
    missionWeightVectorActive_ = false;
    missionUnloadPhase_ = 0;
    missionCollectionFollowThroughUntilMs_ = 0;
    missionVerifyPhase_ = 0;
    missionInnerWeightLocked_ = false;
    missionTargetAligned_ = false;
    missionPointTurnActive_ = false;
    missionTurnFinishing_ = false;
    missionTurnLastSampleMs_ = 0;
    missionTurnYawRateDegPerSec_ = 0.0f;
    missionTurnPulseUntilMs_ = 0;
    missionLastPointTurnMs_ = 0;
    missionFrontBlockedLatched_ = false;
    missionFrontClearSinceMs_ = 0;
    navigationState_ = NAV_IDLE;
    missionAvoidPhase_ = 0;
    missionWallReverseActive_ = false;
    missionWallReverseFailed_ = false;
    missionWallReverseNoProgress_ = false;
    missionWallReverseWaypoint_ = 255;
    missionWallReverseRepeatCount_ = 0;
    missionOuterCueWaypoint_ = 255;
    missionOuterCueTriedMask_ = 0;
    missionOuterCueActive_ = false;
    missionOuterCueAlignedSinceMs_ = 0;
    missionForwardProgressStartedMs_ = 0;
    missionForwardStalled_ = false;
    missionTurnProgressStartedMs_ = 0;
    missionBlockedSinceMs_ = 0;
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

void BluetoothDebugWorkflow::updateMissionPose(uint32_t now)
{
    if (!missionPlanValid_ || !lastImuSampleValid_)
        return;
    if (!missionPoseInitialised_)
    {
        if (!navigationActive_ || navigationStrategy_ != 3)
            return;
        missionPoseXmm_ = missionStartXmm_;
        missionPoseYmm_ = missionStartYmm_;
        missionArenaHeadingDeg_ = missionStartArenaHeadingDeg_;
        missionStartImuHeadingDeg_ = imu_.headingDeg();
        missionLastEncoder1_ = encoders_.firstCount();
        missionLastEncoder2_ = encoders_.secondCount();
        missionLastPoseUpdateMs_ = now;
        missionPoseInitialised_ = true;
        return;
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
    constexpr float ENCODER_1_MM_PER_COUNT = 0.08437374f;
    constexpr float ENCODER_2_MM_PER_COUNT = 0.08461399f;
    const int32_t currentEncoder1 = encoders_.firstCount();
    const int32_t currentEncoder2 = encoders_.secondCount();
    const int32_t deltaEncoder1 = currentEncoder1 - missionLastEncoder1_;
    const int32_t deltaEncoder2 = currentEncoder2 - missionLastEncoder2_;
    missionLastEncoder1_ = currentEncoder1;
    missionLastEncoder2_ = currentEncoder2;
    const uint32_t poseElapsedMs = now - missionLastPoseUpdateMs_;
    missionLastPoseUpdateMs_ = now;

    const float previousHeadingDeg = missionArenaHeadingDeg_;
    missionArenaHeadingDeg_ = normaliseHeading(
        missionStartArenaHeadingDeg_ +
        headingDelta(missionStartImuHeadingDeg_, imu_.headingDeg()));
    const float leftMm = -deltaEncoder1 * ENCODER_1_MM_PER_COUNT;
    const float rightMm = deltaEncoder2 * ENCODER_2_MM_PER_COUNT;
    // During an intentional point turn, unequal wheel slip is not useful
    // evidence of forward travel. The two recorded searches fabricated
    // hundreds of millimetres of translation while spinning on the spot.
    const int16_t motorA = dcMotor203Second_.channelAPercent();
    const int16_t motorB = dcMotor203Second_.channelBPercent();
    const bool commandedPointTurn = motorA * motorB < 0 &&
        abs(motorA) >= 75 && abs(motorB) >= 75;
    if (commandedPointTurn)
        missionLastPointTurnMs_ = now;
    // The driver is often at zero between trim pulses while the high-geared
    // wheels and chassis are still rotating. Counting those unequal encoder
    // ticks as translation slowly moves the map while the robot stays put.
    const bool turningCoast = !commandedPointTurn &&
        motorA == 0 && motorB == 0 && missionLastPointTurnMs_ != 0 &&
        now - missionLastPointTurnMs_ < 500U;
    const float distanceMm = (commandedPointTurn || turningCoast) ? 0.0f :
        (leftMm + rightMm) * 0.5f;
    // Allow genuine long sample gaps, but reject impossible encoder jumps.
    const float plausibleWheelMm = 100.0f + 2.0f * poseElapsedMs;
    if (fabsf(leftMm) <= plausibleWheelMm &&
        fabsf(rightMm) <= plausibleWheelMm)
    {
        const float middleHeadingDeg = normaliseHeading(
            previousHeadingDeg + 0.5f * headingDelta(
                previousHeadingDeg, missionArenaHeadingDeg_));
        const float angleRad = middleHeadingDeg * DEG_TO_RAD;
        missionPoseXmm_ += distanceMm * cosf(angleRad);
        missionPoseYmm_ += distanceMm * sinf(angleRad);
    }
    else if (missionOdometryRejectedSteps_ < 65535U)
        ++missionOdometryRejectedSteps_;
}

void BluetoothDebugWorkflow::updateNavigation()
{
    weightBearingSteeringActive_ = false;
    missionFrontTargetVisible_ = false;
    missionTargetCommitActive_ = false;
    missionFrontTargetRangeErrorMm_ = 0;
    missionFrontTargetMappedWallMm_ = 0;
    if (!navigationActive_)
        return;
    if (missionTofRecoveryHold_)
    {
        dcMotor203Second_.stop();
        dcMotor203SecondActive_ = false;
        return;
    }
    const uint32_t now = millis();
    if (!debugMode_ || stopped_)
    {
        stopNavigation();
        return;
    }

    // Default retained for recording/test compatibility: FRONT_AVOID_MM = 300.
    const uint16_t FRONT_AVOID_MM = navigationFrontAvoidMm_;
    // At the observed ~1.2 s matrix frame cadence the robot can travel over
    // 200 mm between frames. This threshold recognises a coherent surface;
    // actual stopping still uses FRONT_AVOID_MM. Requiring broad consensus
    // prevents one or two bad pixels from causing a turn.
    const uint16_t MATRIX_BROAD_WALL_MM = navigationMatrixWallMm_;
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

    // 2026-09-23 powered straight-run calibration. Combined across two measured
    // runs: 4190 mm over 49,660 counts (encoder 1) and 49,519 counts
    // (encoder 2). Sign/direction remains handled separately from scale.
    constexpr float ENCODER_1_MM_PER_COUNT = 0.08437374f;
    constexpr float ENCODER_2_MM_PER_COUNT = 0.08461399f;

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
    // and mapping. The top point TOFs feed forward navigation.
    uint16_t frontCandidates[MAX_TOF_SENSORS + 1] = {};
    uint8_t frontCandidateCount = 0;
    uint16_t outerTopFallback = 0xFFFF;
    for (uint8_t i = 0; i < tofSensorCount_; ++i)
    {
        if (!tofAvailable_[i] || tofTimedOut_[i])
            continue;
        if (strncmp(tofSensorNames_[i], "top_", 4) != 0)
            continue;
        const int value = tofDistanceMm_[i];
        if (value < 30 || value > 3500)
            continue;
        // An outer-facing sensor sees side walls at shallow angles. Prefer
        // the forward pair, but retain the outer sensors as a fallback when
        // the forward pair is offline (as it was in the recorded run).
        if (strstr(tofSensorNames_[i], "mid_left") != nullptr ||
            strstr(tofSensorNames_[i], "mid_right") != nullptr)
            frontCandidates[frontCandidateCount++] = static_cast<uint16_t>(value);
        else
            outerTopFallback = min(outerTopFallback, static_cast<uint16_t>(value));
    }
    if (frontCandidateCount == 0 && outerTopFallback != 0xFFFF)
        frontCandidates[frontCandidateCount++] = outerTopFallback;
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
    float matrixHeadingChange = tof8x8FrameHeadingDeg_ - imu_.headingDeg();
    while (matrixHeadingChange > 180.0f) matrixHeadingChange -= 360.0f;
    while (matrixHeadingChange < -180.0f) matrixHeadingChange += 360.0f;
    const float matrixLeftTravelMm =
        -(encoders_.firstCount() - tof8x8FrameEncoder1_) * ENCODER_1_MM_PER_COUNT;
    const float matrixRightTravelMm =
        (encoders_.secondCount() - tof8x8FrameEncoder2_) * ENCODER_2_MM_PER_COUNT;
    const float matrixForwardTravelMm =
        0.5f * (matrixLeftTravelMm + matrixRightTravelMm);
    const bool matrixFresh = tof8x8_.available() && tof8x8_.lastReadSucceeded() &&
        tof8x8_.frameNumber() > 0 && now - tof8x8_.lastFrameMs() <= 2000U &&
        (!lastImuSampleValid_ || fabsf(matrixHeadingChange) <= 25.0f) &&
        matrixForwardTravelMm >= 0.0f && matrixForwardTravelMm <= 1500.0f &&
        fabsf(matrixLeftTravelMm - matrixRightTravelMm) <=
            max(100.0f, 0.4f * matrixForwardTravelMm);
    if (matrixFresh)
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
                const uint16_t rawValue = tof8x8_.distanceMm(row, col);
                // The matrix updates slowly. Advance a valid, straight-ahead
                // measurement by the distance driven since that exact frame;
                // never let a stale frame describe the robot's current view.
                const uint16_t value = rawValue >= 10 && rawValue <= 3500
                    ? static_cast<uint16_t>(max(10.0f,
                        rawValue - matrixForwardTravelMm)) : rawValue;
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

    // A median across the available top point and matrix forward estimates
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
    if (!matrixFresh)
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
    const bool rawFrontBlocked = front != 0xFFFF && front < FRONT_AVOID_MM;
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
        if (absoluteError <= 8.0f)
        {
            setDrive(0, 0);
            if (navigationTurnSettledSinceMs_ == 0)
                navigationTurnSettledSinceMs_ = now;
            // Do not start the next forward leg while rotational inertia is
            // still carrying the chassis through the target heading.
            return now - navigationTurnSettledSinceMs_ >= 180U;
        }
        navigationTurnSettledSinceMs_ = 0;

        const int8_t direction = error >= 0.0f ? 1 : -1;
        if (direction != navigationTurnDirection_)
        {
            // The recordings showed full-power direction reversals producing
            // a 20+ degree ping-pong. Coast before applying reverse torque.
            navigationTurnDirection_ = direction;
            navigationTurnCoastUntilMs_ = now + 100U;
            setDrive(0, 0);
            return false;
        }
        if (now < navigationTurnCoastUntilMs_)
        {
            setDrive(0, 0);
            return false;
        }

        const bool turnRight = direction > 0;
        // Both wheels remain powered throughout the turn. Below 75% this
        // drivetrain stalls, so use 75% for the final approach and let the
        // IMU settle once, instead of a repeated on/off pulse train.
        const int16_t power = absoluteError > 55.0f ? 100 : 85;
        setDrive(turnRight ? -power : power,
                 turnRight ? power : -power);
        return false;
    };
    auto driveOnHeading = [&](float targetHeading, int16_t basePower) {
        if (basePower <= 65)
        {
            // Keep the early weight-site approach slow, yet retain a modest
            // heading correction. Once the inner TOFs have locked the actual
            // pickup vector, hold both wheels at exactly 65% for the intake.
            if (!lastImuSampleValid_ ||
                (missionWeightVectorActive_ && missionTargetAligned_))
            {
                setDrive(-65, -65);
                return;
            }
            const float error = headingDelta(imu_.headingDeg(), targetHeading);
            const int16_t faster = static_cast<int16_t>(constrain(
                65.0f + fabsf(error) * 0.8f, 65.0f, 75.0f));
            setDrive(error > 1.0f ? -faster : -65,
                     error < -1.0f ? -faster : -65);
            return;
        }
        if (!lastImuSampleValid_)
        {
            setDrive(-basePower, -basePower);
            return;
        }
        const float error = headingDelta(imu_.headingDeg(), targetHeading);
        float correction = constrain(error * 0.9f, -12.0f, 12.0f);
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
        // Associate a NEW side-ultrasound echo with the nearest uploaded
        // physical rectangle or arena boundary. Three independent echoes must
        // match the same surface before a small position correction is made.
        // This is deliberately not an object-ID claim: ambiguous/noisy echoes
        // cannot teleport the pose to a similarly shaped object elsewhere.
        auto rayRect = [](float x, float y, float ux, float uy,
                          float x0, float y0, float x1, float y1) -> float {
            float nearT = 0.0f, farT = 5000.0f;
            if (fabsf(ux) < 0.0001f)
            {
                if (x < x0 || x > x1) return 5000.0f;
            }
            else
            {
                const float a = (x0 - x) / ux, b = (x1 - x) / ux;
                nearT = max(nearT, min(a, b));
                farT = min(farT, max(a, b));
            }
            if (fabsf(uy) < 0.0001f)
            {
                if (y < y0 || y > y1) return 5000.0f;
            }
            else
            {
                const float a = (y0 - y) / uy, b = (y1 - y) / uy;
                nearT = max(nearT, min(a, b));
                farT = min(farT, max(a, b));
            }
            return farT >= nearT && nearT > 1.0f ? nearT : 5000.0f;
        };
        for (uint8_t side = 0; side < ultrasoundSensorCount_ && side < 2; ++side)
        {
            const uint32_t echo = ultrasoundSensors_[side].fallCount();
            if (echo == missionLastLandmarkEcho_[side]) continue;
            missionLastLandmarkEcho_[side] = echo;
            // A sweeping side beam may briefly match an unrelated wall or
            // obstacle edge. Require straight travel for landmark consensus.
            if (missionLastPointTurnMs_ != 0 &&
                now - missionLastPointTurnMs_ < 500U)
            {
                missionLandmarkMatches_[side] = 0;
                continue;
            }
            if (!ultrasoundSensors_[side].valid() || ultrasoundSensors_[side].timedOut())
            {
                missionLandmarkMatches_[side] = 0;
                continue;
            }
            const float observed = ultrasoundSensors_[side].distanceMm();
            if (observed > 2400.0f) continue;
            const float rayAngle = (missionArenaHeadingDeg_ +
                                    (side == 0 ? -90.0f : 90.0f)) * DEG_TO_RAD;
            const float ux = cosf(rayAngle), uy = sinf(rayAngle);
            const float bodyHeading = missionArenaHeadingDeg_ * DEG_TO_RAD;
            const float lateral = missionUltrasoundGeometryValid_
                ? missionUltrasoundLateralMm_[side] : 0.0f;
            const float forward = missionUltrasoundGeometryValid_
                ? missionUltrasoundForwardMm_[side] : 0.0f;
            const float sensorX = missionPoseXmm_ +
                cosf(bodyHeading) * forward - sinf(bodyHeading) * lateral;
            const float sensorY = missionPoseYmm_ +
                sinf(bodyHeading) * forward + cosf(bodyHeading) * lateral;
            float expected = 5000.0f;
            int8_t landmark = -1;
            // The arena perimeter is an identified, persistent landmark.
            if (ux > 0.001f) expected = min(expected, (4900.0f - sensorX) / ux);
            if (ux < -0.001f) expected = min(expected, -sensorX / ux);
            if (uy > 0.001f) expected = min(expected, (2400.0f - sensorY) / uy);
            if (uy < -0.001f) expected = min(expected, -sensorY / uy);
            for (uint8_t i = 0; i < missionFeatureCount_; ++i)
            {
                const MissionFeature& feature = missionFeatures_[i];
                if (feature.kind == 0) continue;
                const float hit = feature.rotatedWall
                    ? rayRotatedWall(sensorX, sensorY, ux, uy,
                        feature.wallX, feature.wallY, feature.rotationDeg)
                    : rayRect(sensorX, sensorY, ux, uy,
                        feature.x0, feature.y0, feature.x1, feature.y1);
                if (hit < expected) { expected = hit; landmark = i; }
            }
            const float residual = expected - observed;
            // Valid side-ultrasound echoes are reliable range evidence even
            // very close to a wall. Their map identity still needs repeated
            // agreement with a physical perimeter ray; a nearby top TOF
            // shortens the confirmation but is not mandatory.
            bool nearPerimeterRecovery = false;
            bool topSideAgrees = false;
            if (landmark == -1 && observed >= 25.0f && observed <= 180.0f &&
                expected >= 250.0f && expected <= 900.0f &&
                residual >= 120.0f && residual <= 650.0f &&
                fabsf(sinf(2.0f * missionArenaHeadingDeg_ * DEG_TO_RAD)) < 0.35f)
            {
                nearPerimeterRecovery = true;
                const char* corroboratingName = side == 0
                    ? "top_left_left" : "top_right_right";
                for (uint8_t i = 0; i < tofSensorCount_; ++i)
                {
                    if (strcmp(tofSensorNames_[i], corroboratingName) == 0 &&
                        tofAvailable_[i] && !tofTimedOut_[i] &&
                        tofDistanceMm_[i] >= 50 && tofDistanceMm_[i] <= 280)
                    {
                        topSideAgrees = true;
                        break;
                    }
                }
            }
            if (nearPerimeterRecovery)
            {
                if (missionLandmarkIds_[side] != -2)
                {
                    missionLandmarkIds_[side] = -2;
                    missionLandmarkMatches_[side] = 1;
                }
                else if (missionLandmarkMatches_[side] < 6)
                    ++missionLandmarkMatches_[side];
                if (missionLandmarkMatches_[side] >= (topSideAgrees ? 3U : 6U) &&
                    now - missionLastLandmarkCorrectionMs_[side] >= 250U)
                {
                    const float correction = constrain(residual * 0.35f,
                                                       0.0f, 100.0f);
                    missionPoseXmm_ = constrain(missionPoseXmm_ + correction * ux,
                                                0.0f, 4900.0f);
                    missionPoseYmm_ = constrain(missionPoseYmm_ + correction * uy,
                                                0.0f, 2400.0f);
                    missionLastLandmarkCorrectionMs_[side] = now;
                    ++missionLandmarkRecoveryCount_;
                    missionLandmarkRecoveryResidualMm_ =
                        static_cast<uint16_t>(residual);
                    missionLastLandmarkId_ = -2;
                }
                continue;
            }
            if (expected < 250.0f || expected > 2400.0f ||
                fabsf(residual) > 110.0f)
            {
                missionLandmarkMatches_[side] = 0;
                continue;
            }
            if (missionLandmarkIds_[side] != landmark)
            {
                missionLandmarkIds_[side] = landmark;
                missionLandmarkMatches_[side] = 1;
            }
            else if (missionLandmarkMatches_[side] < 3)
                ++missionLandmarkMatches_[side];
            if (missionLandmarkMatches_[side] >= 3 &&
                now - missionLastLandmarkCorrectionMs_[side] >= 500U)
            {
                // An incorrect match must not drag the estimated pose by
                // hundreds of millimetres while following one long wall.
                const float correction = constrain(residual * 0.05f,
                                                   -3.0f, 3.0f);
                missionPoseXmm_ = constrain(missionPoseXmm_ + correction * ux,
                                            0.0f, 4900.0f);
                missionPoseYmm_ = constrain(missionPoseYmm_ + correction * uy,
                                            0.0f, 2400.0f);
                missionLastLandmarkCorrectionMs_[side] = now;
                ++missionLandmarkCorrections_;
                missionLastLandmarkId_ = landmark;
            }
        }

        auto pointFreeAt = [&](float x, float y, float clearance) -> bool {
            if (x < clearance || y < clearance ||
                x > 4900.0f - clearance || y > 2400.0f - clearance)
                return false;
            for (uint8_t i = 0; i < missionFeatureCount_; ++i)
            {
                const MissionFeature& feature = missionFeatures_[i];
                const float featureClearance = feature.kind == 2
                    ? max(clearance, 0.5f * navigationRobotWidthMm_ +
                          max(40.0f, static_cast<float>(navigationGapMarginMm_))) +
                      missionDummyExtraMm_ : clearance;
                if (feature.rotatedWall)
                {
                    const float angle = feature.rotationDeg * DEG_TO_RAD;
                    const float dx = x - feature.wallX;
                    const float dy = y - feature.wallY;
                    const float localX = dx * cosf(angle) + dy * sinf(angle);
                    const float localY = -dx * sinf(angle) + dy * cosf(angle);
                    if (fabsf(localX) <= 300.0f + featureClearance &&
                        fabsf(localY) <= 65.0f + featureClearance)
                        return false;
                }
                else if (x >= feature.x0 - featureClearance &&
                         x <= feature.x1 + featureClearance &&
                         y >= feature.y0 - featureClearance &&
                         y <= feature.y1 + featureClearance)
                    return false;
            }
            return true;
        };
        // Match the desktop planner's robot-radius plus gap-margin model;
        // a fixed 305 mm rejected otherwise valid staging points near the
        // arena boundary and repeatedly replanned the weight approach.
        const float mapClearanceMm = missionReturnHome_
            ? 0.5f * static_cast<float>(navigationRobotWidthMm_) + 10.0f
            : 0.5f * static_cast<float>(navigationRobotWidthMm_) +
              max(40.0f, static_cast<float>(navigationGapMarginMm_));
        auto pointFree = [&](float x, float y) -> bool {
            return pointFreeAt(x, y, mapClearanceMm);
        };
        auto segmentFreeAt = [&](float x0, float y0, float x1, float y1,
                                 float clearance) -> bool {
            const float length = hypotf(x1 - x0, y1 - y0);
            const int samples = max(1, static_cast<int>(ceilf(length / 50.0f)));
            for (int i = 1; i <= samples; ++i)
            {
                const float t = static_cast<float>(i) / samples;
                if (!pointFreeAt(x0 + (x1 - x0) * t,
                                 y0 + (y1 - y0) * t, clearance))
                    return false;
            }
            return true;
        };
        auto segmentFree = [&](float x0, float y0, float x1, float y1) -> bool {
            return segmentFreeAt(x0, y0, x1, y1, mapClearanceMm);
        };

        // The sorter confirms a pickup after the weight has travelled through
        // the mechanism. Let it finish moving straight, but only while the
        // forward sensor and mapped corridor still allow that motion.
        if (missionPostPickupStraightUntilMs_ != 0)
        {
            const float headingRad = missionArenaHeadingDeg_ * DEG_TO_RAD;
            const float aheadX = missionPoseXmm_ + 160.0f * cosf(headingRad);
            const float aheadY = missionPoseYmm_ + 160.0f * sinf(headingRad);
            if (now < missionPostPickupStraightUntilMs_ &&
                front != 0xFFFF && front >= 400U &&
                segmentFreeAt(missionPoseXmm_, missionPoseYmm_, aheadX,
                              aheadY, 0.5f * navigationRobotWidthMm_ + 10.0f))
            {
                driveOnHeading(missionPostPickupHeadingDeg_, 95);
                return;
            }
            missionPostPickupStraightUntilMs_ = 0;
            setDrive(0, 0);
        }

        if (missionWallReverseActive_)
        {
            const float reversedLeft = max(0.0f,
                (encoders_.firstCount() - missionWallReverseEncoder1_) * 0.08437374f);
            const float reversedRight = max(0.0f,
                (missionWallReverseEncoder2_ - encoders_.secondCount()) * 0.08461399f);
            const float reversedMm = 0.5f * (reversedLeft + reversedRight);
            if (reversedMm < 350.0f && now - missionWallReverseStartedMs_ < 2400U &&
                !(reversedMm >= 120.0f && front >= 300U && front != 0xFFFF))
            {
                setDrive(75, 75);
                return;
            }
            setDrive(0, 0);
            missionWallReverseActive_ = false;
            missionWallReverseNoProgress_ = reversedMm < 80.0f;
            missionWallReverseFailed_ = missionWallReverseNoProgress_ ||
                front == 0xFFFF || front < 220U;
            missionAvoidPhase_ = 0;
            missionBlockedSinceMs_ = 0;
            link_.log(missionWallReverseFailed_ ? "WARNING" : "INFO",
                missionWallReverseNoProgress_
                    ? "Reverse produced no encoder progress; motors held stopped"
                    : missionWallReverseFailed_
                    ? "Wall still too close after bounded reverse; holding motors stopped"
                    : "Backed away from frontal wall; replanning before turning");
            return;
        }
        if (missionWallReverseFailed_)
        {
            if (missionWallReverseNoProgress_ || front == 0xFFFF || front < 220U)
            {
                setDrive(0, 0);
                return;
            }
            missionWallReverseFailed_ = false;
        }
        const int16_t commandedA = dcMotor203Second_.channelAPercent();
        const int16_t commandedB = dcMotor203Second_.channelBPercent();
        if (commandedA <= -75 && commandedB <= -75)
        {
            if (missionForwardProgressStartedMs_ == 0)
            {
                missionForwardProgressStartedMs_ = now;
                missionForwardProgressEncoder1_ = encoders_.firstCount();
                missionForwardProgressEncoder2_ = encoders_.secondCount();
            }
            else if (now - missionForwardProgressStartedMs_ >= 1000U)
            {
                const float leftTravel = fabsf(
                    (encoders_.firstCount() - missionForwardProgressEncoder1_) *
                    0.08437374f);
                const float rightTravel = fabsf(
                    (encoders_.secondCount() - missionForwardProgressEncoder2_) *
                    0.08461399f);
                missionForwardStalled_ = leftTravel < 35.0f && rightTravel < 35.0f;
                missionForwardProgressStartedMs_ = now;
                missionForwardProgressEncoder1_ = encoders_.firstCount();
                missionForwardProgressEncoder2_ = encoders_.secondCount();
            }
        }
        else
        {
            missionForwardProgressStartedMs_ = 0;
            missionForwardStalled_ = false;
        }
        auto beginMissionWallReverse = [&](const char* reason) {
            if (missionWallReverseWaypoint_ == missionWaypointIndex_ &&
                now - missionWallReverseLastStartMs_ <= 30000U)
                missionWallReverseRepeatCount_ = static_cast<uint8_t>(min(
                    static_cast<unsigned int>(missionWallReverseRepeatCount_) + 1U,
                    255U));
            else
                missionWallReverseRepeatCount_ = 1;
            missionWallReverseWaypoint_ = missionWaypointIndex_;
            missionWallReverseLastStartMs_ = now;
            missionWallReverseActive_ = true;
            missionWallReverseStartedMs_ = now;
            missionWallReverseEncoder1_ = encoders_.firstCount();
            missionWallReverseEncoder2_ = encoders_.secondCount();
            missionForwardStalled_ = false;
            missionForwardProgressStartedMs_ = 0;
            missionTurnProgressStartedMs_ = 0;
            missionWeightVectorActive_ = false;
            missionInnerWeightLocked_ = false;
            missionAvoidPhase_ = 0;
            setDrive(75, 75);
            link_.log("WARNING", reason);
        };

        // Large odometry errors cannot be repaired by the ordinary 3 mm
        // side-wall trim above. During a blocked return-home leg, use two
        // independent known-wall ranges (a coherent 8x8 front face and one
        // side ultrasound) to search for a consistent nearby map pose.
        // Neither observation alone is allowed to teleport the robot.
        if (missionReturnHome_ && lastImuSampleValid_ && matrixFresh &&
            missionBlockedSinceMs_ != 0 &&
            now - missionBlockedSinceMs_ >= 1500U &&
            now - missionLandmarkRecoveryLastMs_ >= 500U &&
            dcMotor203Second_.channelAPercent() == 0 &&
            dcMotor203Second_.channelBPercent() == 0 &&
            front >= 200U && front <= 1600U &&
            navigationMatrixCloseZones_ >= 10U &&
            navigationMatrixUsableZones_ >= 14U &&
            ((left >= 250U && left <= 2000U) ||
             (right >= 250U && right <= 2000U)))
        {
            missionLandmarkRecoveryLastMs_ = now;
            const uint8_t side = right >= 250U && right <= 2000U ? 1U : 0U;
            const float observedSide = side == 1U ? right : left;
            const float headingRad = missionArenaHeadingDeg_ * DEG_TO_RAD;
            const float fx = cosf(headingRad), fy = sinf(headingRad);
            const float sx = side == 1U ? -fy : fy;
            const float sy = side == 1U ? fx : -fx;
            auto expectedWall = [&](float originX, float originY,
                                    float ux, float uy) -> float {
                float distance = 5000.0f;
                if (ux > 0.001f) distance = min(distance, (4900.0f - originX) / ux);
                if (ux < -0.001f) distance = min(distance, -originX / ux);
                if (uy > 0.001f) distance = min(distance, (2400.0f - originY) / uy);
                if (uy < -0.001f) distance = min(distance, -originY / uy);
                for (uint8_t i = 0; i < missionFeatureCount_; ++i)
                {
                    const MissionFeature& feature = missionFeatures_[i];
                    if (feature.kind == 0) continue;
                    distance = min(distance, feature.rotatedWall
                        ? rayRotatedWall(originX, originY, ux, uy,
                            feature.wallX, feature.wallY, feature.rotationDeg)
                        : rayRect(originX, originY, ux, uy, feature.x0,
                            feature.y0, feature.x1, feature.y1));
                }
                return distance;
            };
            const float lateral = missionUltrasoundGeometryValid_
                ? missionUltrasoundLateralMm_[side] : 0.0f;
            const float forward = missionUltrasoundGeometryValid_
                ? missionUltrasoundForwardMm_[side] : 0.0f;
            const float clearance =
                0.5f * static_cast<float>(navigationRobotWidthMm_) + 10.0f;
            float bestScore = 1.0e9f, bestX = missionPoseXmm_;
            float bestY = missionPoseYmm_, bestResidual = 5000.0f;
            for (int ox = -800; ox <= 800; ox += 40)
                for (int oy = -800; oy <= 800; oy += 40)
                {
                    const float x = missionPoseXmm_ + ox;
                    const float y = missionPoseYmm_ + oy;
                    if (!pointFreeAt(x, y, clearance)) continue;
                    const float frontExpected = expectedWall(
                        x + 150.0f * fx, y + 150.0f * fy, fx, fy);
                    const float sideExpected = expectedWall(
                        x + fx * forward - fy * lateral,
                        y + fy * forward + fx * lateral, sx, sy);
                    const float frontError = frontExpected - front;
                    const float sideError = sideExpected - observedSide;
                    if (fabsf(frontError) > 110.0f ||
                        fabsf(sideError) > 110.0f) continue;
                    const float score = frontError * frontError +
                        sideError * sideError + 0.01f * (ox * ox + oy * oy);
                    if (score < bestScore)
                    {
                        bestScore = score;
                        bestX = x;
                        bestY = y;
                        bestResidual = sqrtf(0.5f * (frontError * frontError +
                                                      sideError * sideError));
                    }
                }
            if (bestScore < 1.0e9f &&
                hypotf(bestX - missionPoseXmm_,
                       bestY - missionPoseYmm_) >= 120.0f)
            {
                if (missionLandmarkRecoveryMatches_ > 0 &&
                    hypotf(bestX - missionLandmarkRecoveryXmm_,
                           bestY - missionLandmarkRecoveryYmm_) <= 100.0f)
                    missionLandmarkRecoveryMatches_ = min(
                        static_cast<uint8_t>(3U),
                        static_cast<uint8_t>(missionLandmarkRecoveryMatches_ + 1U));
                else
                    missionLandmarkRecoveryMatches_ = 1;
                missionLandmarkRecoveryXmm_ = bestX;
                missionLandmarkRecoveryYmm_ = bestY;
                missionLandmarkRecoveryResidualMm_ =
                    static_cast<uint16_t>(bestResidual);
                if (missionLandmarkRecoveryMatches_ >= 3U)
                {
                    missionPoseXmm_ = bestX;
                    missionPoseYmm_ = bestY;
                    if (missionLandmarkRecoveryCount_ < 65535U)
                        ++missionLandmarkRecoveryCount_;
                    missionLandmarkRecoveryMatches_ = 0;
                    link_.log("INFO", "Return-home pose matched to front and side landmarks");
                }
            }
            else
                missionLandmarkRecoveryMatches_ = 0;
        }

        // Replan around uploaded rectangles from the *current* pose. The GUI
        // route can become unusable after localisation correction, and a
        // fixed 500 mm side-step cannot fit a narrow but genuinely clear
        // corridor. A small visibility graph over inflated obstacle corners
        // gives a bounded, map-checked replacement for that dead end.
        auto rerouteAroundMap = [&]() -> bool {
            const float CLEARANCE = mapClearanceMm + 20.0f;
            constexpr uint8_t MAX_NODES = 2 + 4 * MAX_MISSION_FEATURES;
            // In the 15:16 run, forty identical corner sets were inserted
            // while odometry stayed at one pose. A new route requires actual
            // translation before another insertion from the same location.
            if (missionReroutePoseValid_ &&
                hypotf(missionPoseXmm_ - missionReroutePoseXmm_,
                       missionPoseYmm_ - missionReroutePoseYmm_) < 100.0f)
                return false;
            if (missionWaypointIndex_ >= missionWaypointCount_ ||
                !pointFree(missionPoseXmm_, missionPoseYmm_))
                return false;
            const MissionWaypoint goal = missionWaypoints_[missionWaypointIndex_];
            if (!pointFree(goal.xMm, goal.yMm)) return false;
            float nx[MAX_NODES] = {missionPoseXmm_, static_cast<float>(goal.xMm)};
            float ny[MAX_NODES] = {missionPoseYmm_, static_cast<float>(goal.yMm)};
            uint8_t nodes = 2;
            for (uint8_t i = 0; i < missionFeatureCount_; ++i)
            {
                const MissionFeature& feature = missionFeatures_[i];
                const float featureClearance = feature.kind == 2
                    ? max(CLEARANCE, 0.5f * navigationRobotWidthMm_ +
                          max(40.0f, static_cast<float>(navigationGapMarginMm_)) +
                          missionDummyExtraMm_ + 20.0f) : CLEARANCE;
                const float xs[2] = {feature.x0 - featureClearance,
                                     feature.x1 + featureClearance};
                const float ys[2] = {feature.y0 - featureClearance,
                                     feature.y1 + featureClearance};
                for (uint8_t xi = 0; xi < 2; ++xi)
                    for (uint8_t yi = 0; yi < 2; ++yi)
                        if (pointFree(xs[xi], ys[yi]) && nodes < MAX_NODES)
                        {
                            nx[nodes] = xs[xi];
                            ny[nodes] = ys[yi];
                            ++nodes;
                        }
            }
            float distance[MAX_NODES];
            int8_t parent[MAX_NODES];
            bool visited[MAX_NODES] = {};
            for (uint8_t i = 0; i < nodes; ++i)
            {
                distance[i] = 1.0e9f;
                parent[i] = -1;
            }
            distance[0] = 0.0f;
            for (uint8_t iteration = 0; iteration < nodes; ++iteration)
            {
                int8_t current = -1;
                for (uint8_t i = 0; i < nodes; ++i)
                    if (!visited[i] &&
                        (current < 0 || distance[i] < distance[current]))
                        current = static_cast<int8_t>(i);
                if (current < 0 || distance[current] >= 1.0e9f) break;
                if (current == 1) break;
                visited[current] = true;
                for (uint8_t next = 1; next < nodes; ++next)
                {
                    if (visited[next]) continue;
                    const float length = hypotf(nx[next] - nx[current],
                                                ny[next] - ny[current]);
                    if (distance[current] + length >= distance[next] ||
                        !segmentFree(nx[current], ny[current], nx[next], ny[next]))
                        continue;
                    distance[next] = distance[current] + length;
                    parent[next] = current;
                }
            }
            if (parent[1] < 0) return false;
            uint8_t reversed[MAX_NODES];
            uint8_t pathNodes = 0;
            int8_t cursor = 1;
            while (cursor >= 0 && pathNodes < nodes)
            {
                reversed[pathNodes++] = static_cast<uint8_t>(cursor);
                cursor = parent[cursor];
            }
            if (cursor >= 0 || pathNodes < 3 ||
                reversed[pathNodes - 1] != 0) return false;
            const uint8_t intermediate = pathNodes - 2;
            if (missionWaypointCount_ + intermediate > MAX_MISSION_WAYPOINTS)
                return false;
            for (int i = missionWaypointCount_ - 1;
                 i >= missionWaypointIndex_; --i)
                missionWaypoints_[i + intermediate] = missionWaypoints_[i];
            for (uint8_t i = 0; i < intermediate; ++i)
            {
                const uint8_t node = reversed[pathNodes - 2 - i];
                missionWaypoints_[missionWaypointIndex_ + i].xMm =
                    static_cast<int16_t>(roundf(nx[node]));
                missionWaypoints_[missionWaypointIndex_ + i].yMm =
                    static_cast<int16_t>(roundf(ny[node]));
                missionWaypoints_[missionWaypointIndex_ + i].flags = 0;
            }
            missionWaypointCount_ += intermediate;
            ++missionRerouteCount_;
            missionReroutePoseXmm_ = missionPoseXmm_;
            missionReroutePoseYmm_ = missionPoseYmm_;
            missionReroutePoseValid_ = true;
            missionAvoidAttempts_ = 0;
            weightBearingWaypoint_ = 255;
            link_.log("INFO", "Mission map reroute inserted around blocked route");
            return true;
        };

        if (missionUnloadPhase_ != 0)
        {
            // The chassis is facing out of home. Shake only along the same
            // mapped-clear line used for reverse docking; never turn here.
            if (!lastImuSampleValid_ || fabsf(headingDelta(
                    imu_.headingDeg(), missionUnloadImuHeadingDeg_)) > 12.0f ||
                hypotf(missionPoseXmm_ - missionUnloadHomeXmm_,
                       missionPoseYmm_ - missionUnloadHomeYmm_) > 260.0f)
            {
                stopNavigation("Home unload aborted: pose or heading changed");
                return;
            }
            if (missionUnloadPhase_ == 1)
            {
                setDrive(0, 0);
                if (now - missionUnloadPhaseStartedMs_ >= 650U)
                {
                    missionUnloadPhase_ = 2;
                    missionUnloadPhaseStartedMs_ = now;
                    missionUnloadStartEncoder1_ = encoders_.firstCount();
                    missionUnloadStartEncoder2_ = encoders_.secondCount();
                }
                return;
            }
            const bool outward = (missionUnloadPhase_ & 1U) == 0;
            const float leftMm = fabsf((encoders_.firstCount() -
                missionUnloadStartEncoder1_) * ENCODER_1_MM_PER_COUNT);
            const float rightMm = fabsf((encoders_.secondCount() -
                missionUnloadStartEncoder2_) * ENCODER_2_MM_PER_COUNT);
            const float travelledMm = min(leftMm, rightMm);
            const uint32_t elapsedMs = now - missionUnloadPhaseStartedMs_;
            // Give the motor driver a short neutral interval before each
            // direction change instead of slamming straight into reverse.
            if (elapsedMs < 150U)
            {
                setDrive(0, 0);
                return;
            }
            if (leftMm > 145.0f || rightMm > 145.0f ||
                (elapsedMs > 1800U && travelledMm < 35.0f))
            {
                stopNavigation("Home unload aborted: shake travel did not track encoders");
                return;
            }
            if (travelledMm >= 90.0f || elapsedMs > 1800U)
            {
                setDrive(0, 0);
                if (missionUnloadPhase_ == 5)
                {
                    missionUnloadPhase_ = 0;
                    link_.log("INFO", "Home unload shake complete; storage gate open");
                }
                else
                {
                    ++missionUnloadPhase_;
                    missionUnloadPhaseStartedMs_ = now;
                    missionUnloadStartEncoder1_ = encoders_.firstCount();
                    missionUnloadStartEncoder2_ = encoders_.secondCount();
                }
                return;
            }
            const float outwardRad = missionUnloadArenaHeadingDeg_ * DEG_TO_RAD;
            const float targetX = outward
                ? missionPoseXmm_ + 120.0f * cosf(outwardRad)
                : missionUnloadHomeXmm_;
            const float targetY = outward
                ? missionPoseYmm_ + 120.0f * sinf(outwardRad)
                : missionUnloadHomeYmm_;
            if (!segmentFreeAt(missionPoseXmm_, missionPoseYmm_, targetX,
                               targetY, mapClearanceMm) ||
                (outward && front != 0xFFFF && front < 180U))
            {
                stopNavigation("Home unload aborted: shake corridor blocked");
                return;
            }
            setDrive(outward ? -75 : 75, outward ? -75 : 75);
            return;
        }

        if (!missionReturnHome_ && missionTargetsVisited_ >= missionTargetCount_ &&
            missionHomeValid_)
        {
            if (missionWaypointIndex_ + 1 >= MAX_MISSION_WAYPOINTS)
            {
                // Past waypoints are irrelevant once all sites are checked.
                // Compact the route so a long detour history cannot remove
                // the two slots needed for home staging and reverse docking.
                missionWaypointIndex_ = 0;
                missionWaypointCount_ = 0;
            }
            // Find a turn-around point with more clearance than a travel
            // waypoint, and a straight mapped-clear corridor back into home.
            bool stageFound = false;
            const float inward = missionHomeXmm_ < 2450 ? 1.0f : -1.0f;
            for (const int distance : {750, 950, 1150, 1350, 1550, 1750})
            {
                if (stageFound) break;
                for (const int offset : {0, 100, -100, 200, -200})
                {
                    const float stageX = missionHomeXmm_ + inward * distance;
                    const float stageY = missionHomeYmm_ + offset;
                    if (!pointFreeAt(stageX, stageY, mapClearanceMm + 70.0f) ||
                        !segmentFreeAt(stageX, stageY, missionHomeXmm_,
                                       missionHomeYmm_, mapClearanceMm))
                        continue;
                    missionWaypoints_[missionWaypointIndex_].xMm =
                        static_cast<int16_t>(stageX);
                    missionWaypoints_[missionWaypointIndex_].yMm =
                        static_cast<int16_t>(stageY);
                    missionWaypoints_[missionWaypointIndex_].flags = 0;
                    stageFound = true;
                    break;
                }
            }
            if (!stageFound)
            {
                for (const int distance : {750, 950, 1150, 1350, 1550, 1750})
                {
                    if (stageFound) break;
                    for (const int angleDeg : {20, -20, 35, -35, 50, -50,
                                               65, -65, 80, -80})
                    {
                        const float angle = angleDeg * DEG_TO_RAD;
                        const float stageX = missionHomeXmm_ +
                            inward * distance * cosf(angle);
                        const float stageY = missionHomeYmm_ +
                            distance * sinf(angle);
                        if (!pointFreeAt(stageX, stageY, mapClearanceMm + 70.0f) ||
                            !segmentFreeAt(stageX, stageY, missionHomeXmm_,
                                           missionHomeYmm_, mapClearanceMm))
                            continue;
                        missionWaypoints_[missionWaypointIndex_].xMm =
                            static_cast<int16_t>(roundf(stageX));
                        missionWaypoints_[missionWaypointIndex_].yMm =
                            static_cast<int16_t>(roundf(stageY));
                        missionWaypoints_[missionWaypointIndex_].flags = 0;
                        stageFound = true;
                        break;
                    }
                }
            }
            if (!stageFound)
            {
                setDrive(0, 0);
                if (missionBlockedSinceMs_ == 0)
                {
                    missionBlockedSinceMs_ = now;
                    link_.log("WARNING", "Home corridor not yet verified; navigation remains active and will retry");
                }
                return;
            }
            missionBlockedSinceMs_ = 0;
            missionWaypoints_[missionWaypointIndex_ + 1].xMm = missionHomeXmm_;
            missionWaypoints_[missionWaypointIndex_ + 1].yMm = missionHomeYmm_;
            missionWaypoints_[missionWaypointIndex_ + 1].flags = 4;
            missionWaypointCount_ = missionWaypointIndex_ + 2;
            missionReturnHome_ = true;
            missionAutoReturnHome_ = true;
            missionSearchWaypoint_ = 255;
            missionAvoidPhase_ = 0;
            missionLastRerouteMs_ = 0;
            setDrive(0, 0);
            link_.log("INFO", "All weight sites checked; returning home before exploring");
            return;
        }
        if (missionWaypointIndex_ >= missionWaypointCount_ ||
            (!missionReturnHome_ && missionTargetsVisited_ >= missionTargetCount_))
        {
            setDrive(0, 0);
            if (missionReturnHome_)
            {
                if (missionAutoReturnHome_)
                {
                    // The existing mission continues exploring after this
                    // intermediate home visit, so latch the gate again.
                    hx12kB_.setAngle(50.0f);
                    missionAutoReturnHome_ = false;
                    missionReturnHome_ = false;
                    navigationStrategy_ = missionFallbackStrategy_;
                    const float inwardArenaHeading = normaliseHeading(atan2f(
                        1200.0f - missionPoseYmm_, 2450.0f - missionPoseXmm_) * RAD_TO_DEG);
                    const float inwardImuHeading = normaliseHeading(
                        missionStartImuHeadingDeg_ + headingDelta(
                            missionStartArenaHeadingDeg_, inwardArenaHeading));
                    beginHeadingTurn(inwardImuHeading, NAV_INITIAL_TURN);
                    navigationHeadingReferenceDeg_ = inwardImuHeading;
                    navigationLaneIndex_ = 0;
                    navigationSweepLeftReferenceValid_ = false;
                    navigationSweepRightReferenceValid_ = false;
                    navigationSweepProgressMm_ = 0.0f;
                    navigationExpectedSweepLengthValid_ = false;
                    missionBlockedSinceMs_ = 0;
                    link_.log("INFO", "Returned home; turning into arena before exploration");
                    return;
                }
                stopNavigation("Return-home route complete; motors stopped");
                missionReturnHome_ = false;
                return;
            }
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

        // A routing corner can be clear in a differently tuned desktop plan
        // yet fall inside the firmware's actual footprint. It is not a weight
        // target, so skip it and replan to the next reachable waypoint.
        while (missionWaypointIndex_ + 1 < missionWaypointCount_ &&
               (missionWaypoints_[missionWaypointIndex_].flags & 0x01) == 0 &&
               !pointFree(missionWaypoints_[missionWaypointIndex_].xMm,
                          missionWaypoints_[missionWaypointIndex_].yMm))
        {
            if ((missionWaypoints_[missionWaypointIndex_ + 1].flags & 0x01) != 0)
            {
                stopNavigation("Weight staging point is not map-clear; replan route");
                return;
            }
            if (missionReturnHome_ &&
                (missionWaypoints_[missionWaypointIndex_ + 1].flags & 0x04) != 0)
            {
                stopNavigation("Home turnaround point is no longer map-clear");
                return;
            }
            ++missionWaypointIndex_;
            ++missionSkippedWaypoints_;
            missionAvoidPhase_ = 0;
            weightBearingWaypoint_ = 255;
            link_.log("WARNING", "Skipped routing corner inside map clearance");
        }

        const MissionWaypoint& routeWaypoint = missionWaypoints_[missionWaypointIndex_];
        const float routeRangeMm = hypotf(routeWaypoint.xMm - missionPoseXmm_,
                                          routeWaypoint.yMm - missionPoseYmm_);
        // A transit point is a corridor guide, not a place to double back to
        // after driving past it. The two 27 Sep runs overshot such a point by
        // ~300 mm and then spun in place trying to enter its 120 mm arrival
        // circle, despite a clear route through the wall gap ahead.
        if (!missionReturnHome_ && missionAvoidPhase_ == 0 &&
            !missionWeightVectorActive_ &&
            (routeWaypoint.flags & 0x01) == 0 &&
            missionWaypointIndex_ + 1 < missionWaypointCount_ &&
            (missionWaypoints_[missionWaypointIndex_ + 1].flags & 0x01) == 0)
        {
            const MissionWaypoint& next = missionWaypoints_[missionWaypointIndex_ + 1];
            const float legX = next.xMm - routeWaypoint.xMm;
            const float legY = next.yMm - routeWaypoint.yMm;
            const float legMm = hypotf(legX, legY);
            if (legMm > 1.0f)
            {
                const float fromX = missionPoseXmm_ - routeWaypoint.xMm;
                const float fromY = missionPoseYmm_ - routeWaypoint.yMm;
                const float alongMm = (fromX * legX + fromY * legY) / legMm;
                const float lateralMm = fabsf(fromX * legY - fromY * legX) / legMm;
                if (alongMm > 130.0f && lateralMm < 240.0f &&
                    pointFreeAt(missionPoseXmm_, missionPoseYmm_, mapClearanceMm) &&
                    segmentFree(missionPoseXmm_, missionPoseYmm_,
                                next.xMm, next.yMm))
                {
                    ++missionWaypointIndex_;
                    ++missionSkippedWaypoints_;
                    missionBlockedSinceMs_ = 0;
                    weightBearingWaypoint_ = 255;
                    setDrive(0, 0);
                    link_.log("INFO", "Passed transit waypoint; following clear next leg");
                    return;
                }
            }
        }
        if (missionReturnHome_ && (routeWaypoint.flags & 0x04) != 0)
        {
            // The robot has no rear range sensor. Reverse only on the
            // pre-verified straight corridor, with valid IMU/odometry and a
            // bounded turn at the staging point. Do not let ordinary live
            // front-obstacle avoidance point the chassis back into a wall.
            if (missionWaypointIndex_ == 0 || !lastImuSampleValid_ ||
                !segmentFreeAt(missionPoseXmm_, missionPoseYmm_,
                               routeWaypoint.xMm, routeWaypoint.yMm,
                               mapClearanceMm))
            {
                stopNavigation("Reverse home corridor or IMU no longer valid");
                return;
            }
            const MissionWaypoint& stage = missionWaypoints_[missionWaypointIndex_ - 1];
            if (missionDockTurnStartedMs_ == 0 &&
                hypotf(stage.xMm - missionPoseXmm_,
                       stage.yMm - missionPoseYmm_) > 200.0f)
            {
                stopNavigation("Robot is not at the clear home turnaround point");
                return;
            }
            const float outwardArena = normaliseHeading(atan2f(
                stage.yMm - routeWaypoint.yMm,
                stage.xMm - routeWaypoint.xMm) * RAD_TO_DEG);
            const float outwardImu = normaliseHeading(missionStartImuHeadingDeg_ +
                headingDelta(missionStartArenaHeadingDeg_, outwardArena));
            if (missionDockTurnStartedMs_ == 0)
            {
                missionDockTurnStartedMs_ = now;
                navigationTargetHeadingDeg_ = outwardImu;
                navigationTurnSettledSinceMs_ = 0;
                navigationTurnDirection_ = 0;
                navigationTurnCoastUntilMs_ = now;
                navigationTurnPulseStartedMs_ = now;
                missionDockAligned_ = false;
                link_.log("INFO", "Turning in clear staging area to reverse into home");
            }
            if (now - missionDockTurnStartedMs_ > 18000U)
            {
                stopNavigation("Reverse home docking timed out");
                return;
            }
            if (!missionDockAligned_)
            {
                if (!runHeadingTurn()) return;
                missionDockAligned_ = true;
            }
            if (routeRangeMm <= 110.0f)
            {
                setDrive(0, 0);
                ++missionWaypointIndex_;
                missionDockTurnStartedMs_ = 0;
                missionDockAligned_ = false;
                missionUnloadHomeXmm_ = routeWaypoint.xMm;
                missionUnloadHomeYmm_ = routeWaypoint.yMm;
                missionUnloadArenaHeadingDeg_ = outwardArena;
                missionUnloadImuHeadingDeg_ = outwardImu;
                missionUnloadPhase_ = 1;
                missionUnloadPhaseStartedMs_ = now;
                hx12kB_.setAngle(10.0f);
                link_.log("INFO", "Home reached; storage gate opening for two shake cycles");
                return;
            }
            const float error = headingDelta(imu_.headingDeg(), outwardImu);
            if (fabsf(error) > 12.0f)
            {
                stopNavigation("Reverse docking heading drifted outside tolerance");
                return;
            }
            const int16_t correction = static_cast<int16_t>(constrain(
                error * 0.4f, -5.0f, 5.0f));
            setDrive(75 - correction, 75 + correction);
            return;
        }
        // The final approach can fit closer to a wall than the general
        // route-clearance envelope. Keep half the configured chassis width
        // plus 10 mm from features, only on a known target leg.
        const float targetClearanceMm =
            0.5f * static_cast<float>(navigationRobotWidthMm_) + 10.0f;
        const bool targetCorridorClear = !missionReturnHome_ &&
            (routeWaypoint.flags & 0x01) != 0 && routeRangeMm <= 700.0f &&
            segmentFreeAt(missionPoseXmm_, missionPoseYmm_,
                          routeWaypoint.xMm, routeWaypoint.yMm, targetClearanceMm);
        if (missionVerifyPhase_ == 0 && missionAvoidPhase_ == 0 &&
            now - missionLastRerouteMs_ >= 1000U &&
            !targetCorridorClear &&
            !segmentFree(missionPoseXmm_, missionPoseYmm_,
                         routeWaypoint.xMm, routeWaypoint.yMm))
        {
            missionLastRerouteMs_ = now;
            if (rerouteAroundMap())
            {
                setDrive(0, 0);
                return;
            }
        }

        const MissionWaypoint& waypoint = missionWaypoints_[missionWaypointIndex_];
        const float routeDx = static_cast<float>(waypoint.xMm) - missionPoseXmm_;
        const float routeDy = static_cast<float>(waypoint.yMm) - missionPoseYmm_;
        const bool searchActive = missionSearchWaypoint_ == missionWaypointIndex_ &&
            !missionReturnHome_ && (waypoint.flags & 0x01) != 0;
        auto beginSiteSearch = [&]() {
            missionSearchWaypoint_ = missionWaypointIndex_;
            missionSearchStep_ = 0;
            missionSearchPasses_ = 0;
            missionSearchSamples_ = 0;
            missionSearchCoverageSamples_ = 0;
            missionSearchStartedMs_ = now;
            missionSearchDwellStartedMs_ = 0;
            missionSearchLastEvidenceMs_ = 0;
            missionSearchApproachHeadingDeg_ =
                missionWeightVectorCompletedWaypoint_ == missionWaypointIndex_
                ? missionWeightVectorHeadingDeg_ :
                (hypotf(routeDx, routeDy) >= 100.0f
                    ? normaliseHeading(atan2f(routeDy, routeDx) * RAD_TO_DEG)
                    : missionArenaHeadingDeg_);
            // A previous pass was not an independently confirmed pickup.
            // Allow a new sensor lock during the search instead of treating
            // this waypoint as permanently collected.
            missionWeightVectorCompletedWaypoint_ = 255;
            missionInnerAlignFailedWaypoint_ = 255;
            link_.log("INFO", "Searching around planned weight site before moving on");
        };
        auto finishSiteSearch = [&](bool incomplete) {
            if (incomplete)
            {
                missionPendingPickupWaypoint_ = missionWaypointIndex_;
                missionPendingPickupSinceMs_ = now;
            }
            if (missionTargetsVisited_ < missionTargetCount_)
                ++missionTargetsVisited_;
            if (missionSearchSitesChecked_ < 255U)
                ++missionSearchSitesChecked_;
            if (incomplete && missionSearchSitesIncomplete_ < 255U)
                ++missionSearchSitesIncomplete_;
            link_.log(incomplete ? "WARNING" : "INFO", incomplete
                ? "Weight-site search incomplete; pickup not verified"
                : "Weight site swept; pickup not independently verified");
            ++missionWaypointIndex_;
            missionSearchWaypoint_ = 255;
            missionSearchStep_ = 0;
            missionVerifyRetries_ = 0;
            weightBearingWaypoint_ = 255;
            missionBlockedSinceMs_ = 0;
            missionAvoidAttempts_ = 0;
        };
        if (missionHandledSortingSerial_ != sortingConfirmedSerial_)
        {
            missionHandledSortingSerial_ = sortingConfirmedSerial_;
            // A weight already passed through the intake can reach the
            // inductor while travelling to the next site. Never credit that
            // delayed confirmation to a site we have not searched yet.
            if (missionPendingPickupWaypoint_ != 255 &&
                now - missionPendingPickupSinceMs_ <= 30000U &&
                !missionWeightVectorActive_ && missionVerifyPhase_ == 0)
            {
                missionPendingPickupWaypoint_ = 255;
                if (missionSearchSitesIncomplete_ > 0)
                    --missionSearchSitesIncomplete_;
                missionPostPickupHeadingDeg_ = imu_.headingDeg();
                missionPostPickupStraightUntilMs_ = now + 1200U;
                link_.log("INFO", "Delayed inductive pickup credited to previous site; next site remains scheduled");
                return;
            }
            missionPendingPickupWaypoint_ = 255;
            if (!missionReturnHome_ && (waypoint.flags & 0x01) != 0 &&
                (searchActive || missionWeightVectorActive_ ||
                 routeRangeMm <= 220.0f))
            {
                // The inductor is direct evidence that a weight entered the
                // mechanism. Do not keep searching the already-collected site.
                setDrive(0, 0);
                missionWeightVectorActive_ = false;
                missionCollectionFollowThroughUntilMs_ = 0;
                missionInnerWeightLocked_ = false;
                missionVerifyPhase_ = 0;
                missionSearchWaypoint_ = 255;
                missionSearchStep_ = 0;
                missionVerifyRetries_ = 0;
                weightBearingWaypoint_ = 255;
                if (missionTargetsVisited_ < missionTargetCount_)
                    ++missionTargetsVisited_;
                if (missionSearchSitesChecked_ < 255U)
                    ++missionSearchSitesChecked_;
                ++missionWaypointIndex_;
                missionBlockedSinceMs_ = 0;
                missionPostPickupHeadingDeg_ = imu_.headingDeg();
                missionPostPickupStraightUntilMs_ = now + 1200U;
                link_.log("INFO", "Inductive pickup confirmed; moving to next weight site");
                return;
            }
            link_.log("WARNING", "Inductive pickup confirmed without a matching active weight site");
        }
        if (missionVerifyPhase_ != 0 &&
            missionVerifyWaypoint_ == missionWaypointIndex_)
        {
            if (missionVerifyPhase_ == 5)
            {
                const float reversedLeft = max(0.0f,
                    (encoders_.firstCount() - missionRetryReverseEncoder1_) *
                        ENCODER_1_MM_PER_COUNT);
                const float reversedRight = max(0.0f,
                    (missionRetryReverseEncoder2_ - encoders_.secondCount()) *
                        ENCODER_2_MM_PER_COUNT);
                const float reversedMm = 0.5f * (reversedLeft + reversedRight);
                if (reversedMm + 25.0f < missionRetryReverseDistanceMm_ &&
                    now - missionRetryReverseStartedMs_ < 4500U)
                {
                    setDrive(75, 75);
                    return;
                }
                setDrive(0, 0);
                missionVerifyPhase_ = 0;
                ++missionVerifyRetries_;
                beginSiteSearch();
                // We are back on the original approach line. Look toward
                // the target first rather than immediately traversing both
                // lateral search points and losing the weight again.
                missionSearchStep_ = 2;
                weightBearingWaypoint_ = 255;
                link_.log(reversedMm >= missionRetryReverseDistanceMm_ * 0.5f
                    ? "INFO" : "WARNING",
                    reversedMm >= missionRetryReverseDistanceMm_ * 0.5f
                    ? "Missed pickup; reversed behind weight for a centred retry"
                    : "Pickup retry reverse made little progress; searching site");
                return;
            }
            // Wait for the inductive sorting confirmation before deciding
            // that the drive-through missed. A weight behind the robot is
            // not visible in a forward-facing in-place scan.
            setDrive(0, 0);
            if (missionVerifyPhase_ == 1)
            {
                if (now - missionVerifyStartedMs_ < 1000U)
                    return;
                if (missionVerifyRetries_ < 2U)
                {
                    const float heading = missionWeightVectorHeadingDeg_ * DEG_TO_RAD;
                    const float ux = cosf(heading), uy = sinf(heading);
                    for (const float standOffMm : {240.0f, 180.0f, 120.0f})
                    {
                        const float backX = missionWeightVectorHitXmm_ - standOffMm * ux;
                        const float backY = missionWeightVectorHitYmm_ - standOffMm * uy;
                        const float reverseMm =
                            (missionPoseXmm_ - backX) * ux +
                            (missionPoseYmm_ - backY) * uy;
                        if (reverseMm < 120.0f || reverseMm > 600.0f ||
                            !pointFreeAt(backX, backY, targetClearanceMm) ||
                            !segmentFreeAt(missionPoseXmm_, missionPoseYmm_,
                                           backX, backY, targetClearanceMm))
                            continue;
                        missionRetryReverseDistanceMm_ = reverseMm;
                        missionRetryReverseEncoder1_ = encoders_.firstCount();
                        missionRetryReverseEncoder2_ = encoders_.secondCount();
                        missionRetryReverseStartedMs_ = now;
                        missionVerifyPhase_ = 5;
                        link_.log("WARNING", "Pickup not confirmed; reversing behind the sensed weight");
                        return;
                    }
                }
                missionVerifyPhase_ = 2;
                link_.log("INFO", "No clear reverse corridor; scanning for weight after pass");
            }
            // Only an inner-bottom hit can justify another collection pass.
            // Outer sensors face away from the intake and must never be
            // promoted into a synthetic inner bearing during verification.
            if ((weightBearingSector_ == 1U || weightBearingSector_ == 2U ||
                 weightBearingSector_ == 4U) &&
                weightBearingWaypoint_ == missionWaypointIndex_ &&
                weightBearingSeenMs_ >= missionVerifyStartedMs_ + 1000U &&
                now - weightBearingSeenMs_ <= 350U)
            {
                missionVerifySawWeight_ = true;
                missionVerifyHitXmm_ = weightBearingHitXmm_;
                missionVerifyHitYmm_ = weightBearingHitYmm_;
            }
            const float scanOffsets[3] = {30.0f, -30.0f, 0.0f};
            const float scanTarget = normaliseHeading(
                missionVerifyHeadingDeg_ + scanOffsets[missionVerifyPhase_ - 2]);
            const float scanError = headingDelta(imu_.headingDeg(), scanTarget);
            if (fabsf(scanError) <= 8.0f)
                ++missionVerifyPhase_;
            else
                setDrive(scanError > 0 ? -85 : 85,
                         scanError > 0 ? 85 : -85);
            if (missionVerifyPhase_ <= 4 &&
                now - missionVerifyStartedMs_ < 14000U)
                return;
            missionVerifyPhase_ = 0;
            if (missionVerifySawWeight_ && missionVerifyRetries_ < 2U &&
                pointFreeAt(missionVerifyHitXmm_, missionVerifyHitYmm_,
                            targetClearanceMm) &&
                segmentFreeAt(missionPoseXmm_, missionPoseYmm_,
                              missionVerifyHitXmm_, missionVerifyHitYmm_,
                              targetClearanceMm))
            {
                ++missionVerifyRetries_;
                missionWeightVectorCompletedWaypoint_ = 255;
                weightBearingWaypoint_ = missionWaypointIndex_;
                weightBearingSector_ = 2;
                weightBearingHitXmm_ = missionVerifyHitXmm_;
                weightBearingHitYmm_ = missionVerifyHitYmm_;
                weightBearingSeenMs_ = now;
                link_.log("INFO", "Weight still visible after scan; retrying collection");
                return;
            }
            if (missionVerifySawWeight_)
            {
                beginSiteSearch();
                link_.log("WARNING", "Weight still visible but retry is not clear; searching site");
                return;
            }
            if (missionVerifyRetries_ < 2U)
            {
                ++missionVerifyRetries_;
                beginSiteSearch();
                link_.log("WARNING", "Pickup not confirmed; searching weight site again");
            }
            else
            {
                finishSiteSearch(true);
                link_.log("WARNING", "Pickup still not confirmed after repeated passes");
            }
            return;
        }
        if (searchActive && weightBearingWaypoint_ == missionWaypointIndex_ &&
            now - weightBearingSeenMs_ <= 800U)
            missionSearchLastEvidenceMs_ = now;
        if (searchActive && !missionWeightVectorActive_ &&
            now - missionSearchStartedMs_ >= 28000U)
        {
            setDrive(0, 0);
            finishSiteSearch(true);
            return;
        }
        if (searchActive && missionSearchStep_ >= 3U)
        {
            if (weightBearingWaypoint_ == missionWaypointIndex_ &&
                now - weightBearingSeenMs_ <= 800U &&
                missionWeightVectorCompletedWaypoint_ != missionWaypointIndex_)
            {
                // A confirmed hit takes precedence over finishing a blind
                // sweep, including on the final search pass.
                missionSearchStep_ = 2;
                missionSearchDwellStartedMs_ = 0;
            }
            else
            {
                const bool recentlySeen = missionSearchLastEvidenceMs_ != 0 &&
                    now - missionSearchLastEvidenceMs_ <= 2500U;
                const bool needsAnotherLook = recentlySeen || missionSearchSamples_ < 2U ||
                    missionSearchCoverageSamples_ < 2U;
                if (needsAnotherLook && missionSearchPasses_ < 1U &&
                    now - missionSearchStartedMs_ < 22000U)
                {
                    missionSearchStep_ = 0;
                    ++missionSearchPasses_;
                    missionSearchDwellStartedMs_ = 0;
                }
                else
                {
                    setDrive(0, 0);
                    finishSiteSearch(recentlySeen || missionSearchSamples_ < 2U ||
                                     missionSearchCoverageSamples_ < 2U);
                    return;
                }
            }
        }
        float searchX = waypoint.xMm, searchY = waypoint.yMm;
        if (searchActive && missionSearchStep_ < 2U)
        {
            const float angle = missionSearchApproachHeadingDeg_ * DEG_TO_RAD;
            const float backward = missionSearchPasses_ == 0 ? 180.0f : 130.0f;
            const float lateral = (missionSearchStep_ == 0 ? -1.0f : 1.0f) *
                (missionSearchPasses_ == 0 ? 150.0f : 100.0f);
            searchX -= backward * cosf(angle) + lateral * sinf(angle);
            searchY -= backward * sinf(angle) - lateral * cosf(angle);
            if (!pointFreeAt(searchX, searchY, targetClearanceMm) ||
                !segmentFreeAt(missionPoseXmm_, missionPoseYmm_,
                               searchX, searchY, targetClearanceMm))
            {
                ++missionSearchStep_;
                missionSearchDwellStartedMs_ = 0;
                setDrive(0, 0);
                return;
            }
        }
        const bool innerPairNear = !missionReturnHome_ &&
            missionBottomSensorGeometryValid_ &&
            (waypoint.flags & 0x01) != 0 && missionAvoidPhase_ == 0 &&
            (weightSectorMask_ & 0x06U) == 0x06U &&
            weightBearingSector_ == 4U &&
            weightBearingWaypoint_ == missionWaypointIndex_ &&
            now - weightBearingSeenMs_ <= 350U &&
            (routeRangeMm <= 350.0f ||
             (routeRangeMm <= 500.0f &&
              (weightSectorMask_ != 0 ||
               (weightBearingWaypoint_ == missionWaypointIndex_ &&
                now - weightBearingSeenMs_ <= 2500U)))) &&
            weightInnerLeftMm_ <= 300U && weightInnerRightMm_ <= 300U;
        const int16_t innerDeltaMm = static_cast<int16_t>(weightInnerLeftMm_) -
            static_cast<int16_t>(weightInnerRightMm_);
        if (innerPairNear && !missionInnerWeightLocked_ &&
            missionInnerAlignFailedWaypoint_ != missionWaypointIndex_ &&
            missionWeightVectorCompletedWaypoint_ != missionWaypointIndex_ &&
            front != 0xFFFF && front >= 100U)
        {
            if (abs(innerDeltaMm) > 80)
            {
                if (missionInnerAlignStartedMs_ == 0)
                    missionInnerAlignStartedMs_ = now;
                if (now - missionInnerAlignStartedMs_ >= 2500U)
                {
                    missionInnerAlignFailedWaypoint_ = missionWaypointIndex_;
                    setDrive(0, 0);
                    if (!searchActive)
                        beginSiteSearch();
                    else
                        ++missionSearchStep_;
                    link_.log("WARNING", "Inner TOF centring did not converge; continuing site search");
                    return;
                }
                // Steer toward the nearer inner return with both motors
                // powered continuously. The drivetrain stalls below 75%,
                // so the inner wheel runs at 75% and the outer at 100%.
                missionTargetAligned_ = false;
                missionTargetAlignSinceMs_ = 0;
                setDrive(innerDeltaMm > 0 ? -100 : -75,
                         innerDeltaMm > 0 ? -75 : -100);
                return;
            }
            missionInnerAlignStartedMs_ = 0;
            const MissionBottomSensorPose& midLeft = missionBottomSensors_[1];
            const MissionBottomSensorPose& midRight = missionBottomSensors_[2];
            const float hitForwardMm = 0.5f * (
                midLeft.forwardMm + weightInnerLeftMm_ *
                    cosf(midLeft.angleDeg * DEG_TO_RAD) +
                midRight.forwardMm + weightInnerRightMm_ *
                    cosf(midRight.angleDeg * DEG_TO_RAD));
            const float headingRad = missionArenaHeadingDeg_ * DEG_TO_RAD;
            const float ux = cosf(headingRad), uy = sinf(headingRad);
            const float hitX = missionPoseXmm_ + hitForwardMm * ux;
            const float hitY = missionPoseYmm_ + hitForwardMm * uy;
            if (pointFreeAt(hitX, hitY, targetClearanceMm) &&
                segmentFreeAt(missionPoseXmm_, missionPoseYmm_,
                              hitX, hitY, targetClearanceMm))
            {
                uint16_t overshoot = 0;
                for (uint16_t candidate : {240U, 200U, 160U, 100U})
                {
                    const float endX = hitX + candidate * ux;
                    const float endY = hitY + candidate * uy;
                    if (pointFreeAt(endX, endY, targetClearanceMm) &&
                        segmentFreeAt(missionPoseXmm_, missionPoseYmm_,
                                      endX, endY, targetClearanceMm))
                    {
                        overshoot = candidate;
                        break;
                    }
                }
                missionWeightVectorActive_ = true;
                missionCollectionFollowThroughUntilMs_ = 0;
                missionInnerWeightLocked_ = true;
                missionWeightVectorWaypoint_ = missionWaypointIndex_;
                missionWeightVectorStartXmm_ = missionPoseXmm_;
                missionWeightVectorStartYmm_ = missionPoseYmm_;
                missionWeightVectorHitXmm_ = hitX;
                missionWeightVectorHitYmm_ = hitY;
                missionWeightVectorHeadingDeg_ = missionArenaHeadingDeg_;
                missionWeightVectorOvershootMm_ = overshoot;
                missionWeightVectorTravelMm_ = hitForwardMm + overshoot;
                missionWeightVectorEndXmm_ = hitX + overshoot * ux;
                missionWeightVectorEndYmm_ = hitY + overshoot * uy;
                missionTargetAligned_ = true;
                link_.log("INFO", "Inner bottom TOFs centred; sending straight through weight");
            }
        }
        else if (!innerPairNear)
            missionInnerAlignStartedMs_ = 0;
        if (missionOuterCueWaypoint_ != missionWaypointIndex_)
        {
            missionOuterCueWaypoint_ = missionWaypointIndex_;
            missionOuterCueTriedMask_ = 0;
            missionOuterCueActive_ = false;
        }
        const bool freshFusedInnerBearing = weightBearingSector_ == 4U &&
            weightBearingWaypoint_ == missionWaypointIndex_ &&
            now - weightBearingSeenMs_ <= 350U;
        if (freshFusedInnerBearing || missionWeightVectorActive_)
            missionOuterCueActive_ = false;
        // One inner return is still a directional hint, not evidence that
        // the weight is centred in the intake. Treat it like an outer cue.
        const bool freshDirectionCue = !missionReturnHome_ &&
            (waypoint.flags & 0x01) != 0 && missionAvoidPhase_ == 0 &&
            missionWeightVectorCompletedWaypoint_ != missionWaypointIndex_ &&
            routeRangeMm <= 450.0f &&
            weightBearingSector_ < 4U &&
            weightBearingWaypoint_ == missionWaypointIndex_ &&
            now - weightBearingSeenMs_ <= 350U &&
            (weightSectorMask_ & (1U << weightBearingSector_)) != 0 &&
            pointFreeAt(missionPoseXmm_, missionPoseYmm_, targetClearanceMm) &&
            pointFreeAt(weightBearingHitXmm_, weightBearingHitYmm_,
                        targetClearanceMm);
        if (!missionOuterCueActive_ && freshDirectionCue &&
            (missionOuterCueTriedMask_ & (1U << weightBearingSector_)) == 0)
        {
            missionOuterCueActive_ = true;
            missionOuterCueTriedMask_ |= 1U << weightBearingSector_;
            missionOuterCueStartedMs_ = now;
            missionOuterCueAlignedSinceMs_ = 0;
            missionOuterCueHeadingDeg_ = normaliseHeading(atan2f(
                weightBearingHitYmm_ - missionPoseYmm_,
                weightBearingHitXmm_ - missionPoseXmm_) * RAD_TO_DEG);
            link_.log("INFO", "Bottom TOF suggests weight direction; turning for paired inner confirmation");
        }
        if (missionOuterCueActive_)
        {
            if (now - missionOuterCueStartedMs_ >= 3500U)
            {
                missionOuterCueActive_ = false;
                setDrive(0, 0);
                return;
            }
            const float cueError = headingDelta(missionArenaHeadingDeg_,
                                                missionOuterCueHeadingDeg_);
            if (fabsf(cueError) > 10.0f)
            {
                missionOuterCueAlignedSinceMs_ = 0;
                setDrive(cueError > 0 ? -85 : 85,
                         cueError > 0 ? 85 : -85);
                return;
            }
            setDrive(0, 0);
            if (missionOuterCueAlignedSinceMs_ == 0)
                missionOuterCueAlignedSinceMs_ = now;
            if (now - missionOuterCueAlignedSinceMs_ < 400U)
                return;
            missionOuterCueActive_ = false;
        }
        // Neither an outer cue nor a single inner ray establishes the
        // intake centreline. In the latest run a lone right-inner return
        // froze a vector 165 mm from the planned weight and drove past it.
        // Only a spatially corroborated pair may start the collection pass.
        const bool followWeightBearing = missionAvoidPhase_ == 0 &&
            (waypoint.flags & 0x01) != 0 &&
            missionWeightVectorCompletedWaypoint_ != missionWaypointIndex_ &&
            weightBearingSector_ == 4U &&
            weightBearingWaypoint_ == missionWaypointIndex_ &&
            now - weightBearingSeenMs_ <= 800U &&
            (!searchActive || hypotf(weightBearingHitXmm_ - missionPoseXmm_,
                                     weightBearingHitYmm_ - missionPoseYmm_) > 120.0f) &&
            pointFreeAt(weightBearingHitXmm_, weightBearingHitYmm_, targetClearanceMm) &&
            segmentFreeAt(missionPoseXmm_, missionPoseYmm_,
                          weightBearingHitXmm_, weightBearingHitYmm_, targetClearanceMm);
        if (missionWeightVectorActive_ &&
            (missionWeightVectorWaypoint_ != missionWaypointIndex_ ||
             missionAvoidPhase_ != 0))
        {
            missionWeightVectorActive_ = false;
            missionCollectionFollowThroughUntilMs_ = 0;
            missionInnerWeightLocked_ = false;
        }
        // A single inner beam may still rescue a weight that the inward-
        // angled pair cannot see together. It must first agree closely with
        // the independently uploaded site, be far enough ahead to line up,
        // and have finished its direction-only cue. The latest miss was a
        // lone 85 mm right-inner echo whose projected hit was 165 mm away
        // from the site; that must never become a collection vector.
        const bool mappedSingleInner = !missionWeightVectorActive_ &&
            !missionOuterCueActive_ && missionAvoidPhase_ == 0 &&
            (waypoint.flags & 0x01) != 0 &&
            missionWeightVectorCompletedWaypoint_ != missionWaypointIndex_ &&
            (weightBearingSector_ == 1U || weightBearingSector_ == 2U) &&
            weightBearingWaypoint_ == missionWaypointIndex_ &&
            now - weightBearingSeenMs_ <= 350U &&
            weightBearingErrorMm_ <= 70.0f;
        if (mappedSingleInner)
        {
            const float vx = weightBearingHitXmm_ - missionPoseXmm_;
            const float vy = weightBearingHitYmm_ - missionPoseYmm_;
            const float hitDistance = hypotf(vx, vy);
            if (hitDistance >= 250.0f && hitDistance <= 600.0f)
            {
                const float ux = vx / hitDistance, uy = vy / hitDistance;
                uint16_t overshoot = 0;
                for (uint16_t candidate : {200U, 160U, 100U})
                {
                    const float endX = weightBearingHitXmm_ + candidate * ux;
                    const float endY = weightBearingHitYmm_ + candidate * uy;
                    if (pointFreeAt(endX, endY, targetClearanceMm) &&
                        segmentFreeAt(missionPoseXmm_, missionPoseYmm_,
                                      endX, endY, targetClearanceMm))
                    {
                        overshoot = candidate;
                        break;
                    }
                }
                if (overshoot > 0)
                {
                    missionWeightVectorActive_ = true;
                    missionCollectionFollowThroughUntilMs_ = 0;
                    missionInnerWeightLocked_ = false;
                    missionWeightVectorWaypoint_ = missionWaypointIndex_;
                    missionWeightVectorStartXmm_ = missionPoseXmm_;
                    missionWeightVectorStartYmm_ = missionPoseYmm_;
                    missionWeightVectorHitXmm_ = weightBearingHitXmm_;
                    missionWeightVectorHitYmm_ = weightBearingHitYmm_;
                    missionWeightVectorHeadingDeg_ = normaliseHeading(
                        atan2f(vy, vx) * RAD_TO_DEG);
                    missionWeightVectorOvershootMm_ = overshoot;
                    missionWeightVectorTravelMm_ = hitDistance + overshoot;
                    missionWeightVectorEndXmm_ = weightBearingHitXmm_ + overshoot * ux;
                    missionWeightVectorEndYmm_ = weightBearingHitYmm_ + overshoot * uy;
                    missionTargetAligned_ = false;
                    missionTargetAlignSinceMs_ = 0;
                    link_.log("INFO", "Map-matched inner TOF; aligning for collection without paired lock");
                }
            }
        }
        weightBearingSteeringActive_ = missionWeightVectorActive_ || followWeightBearing;
        const float targetX = missionAvoidPhase_ == 1 ? missionAvoidXmm_ :
                              missionAvoidPhase_ == 2 ? missionAvoidForwardXmm_ :
                              missionWeightVectorActive_ ? missionWeightVectorEndXmm_ :
                              followWeightBearing ? weightBearingHitXmm_ :
                              searchActive ? searchX : static_cast<float>(waypoint.xMm);
        const float targetY = missionAvoidPhase_ == 1 ? missionAvoidYmm_ :
                              missionAvoidPhase_ == 2 ? missionAvoidForwardYmm_ :
                              missionWeightVectorActive_ ? missionWeightVectorEndYmm_ :
                              followWeightBearing ? weightBearingHitYmm_ :
                              searchActive ? searchY : static_cast<float>(waypoint.yMm);
        const float dx = targetX - missionPoseXmm_;
        const float dy = targetY - missionPoseYmm_;
        const float distanceToWaypoint = sqrtf(dx * dx + dy * dy);
        const float arrivalMm = missionAvoidPhase_ != 0 ? 130.0f :
                                searchActive ? 85.0f :
                                (waypoint.flags & 0x01) != 0 ? 70.0f : 120.0f;

        const float vectorProgressMm = missionWeightVectorActive_ ?
            (missionPoseXmm_ - missionWeightVectorStartXmm_) *
                cosf(missionWeightVectorHeadingDeg_ * DEG_TO_RAD) +
            (missionPoseYmm_ - missionWeightVectorStartYmm_) *
                sinf(missionWeightVectorHeadingDeg_ * DEG_TO_RAD) : 0.0f;
        if (missionWeightVectorActive_ &&
            vectorProgressMm >= missionWeightVectorTravelMm_)
        {
            if (missionInnerWeightLocked_)
            {
                if (missionCollectionFollowThroughUntilMs_ == 0)
                {
                    missionCollectionFollowThroughUntilMs_ = now + 2500U;
                    link_.log("INFO", "Centred pickup reached; slow collection follow-through");
                }
                const float headingRad = missionWeightVectorHeadingDeg_ * DEG_TO_RAD;
                const float aheadX = missionPoseXmm_ + 100.0f * cosf(headingRad);
                const float aheadY = missionPoseYmm_ + 100.0f * sinf(headingRad);
                if (static_cast<int32_t>(now - missionCollectionFollowThroughUntilMs_) < 0 &&
                    front != 0xFFFF && front >= 150U &&
                    segmentFreeAt(missionPoseXmm_, missionPoseYmm_, aheadX, aheadY,
                                  targetClearanceMm))
                {
                    const float frozenBnoHeading = normaliseHeading(
                        missionStartImuHeadingDeg_ + headingDelta(
                            missionStartArenaHeadingDeg_, missionWeightVectorHeadingDeg_));
                    driveOnHeading(frozenBnoHeading, 65);
                    return;
                }
            }
            setDrive(0, 0);
            missionWeightVectorActive_ = false;
            missionCollectionFollowThroughUntilMs_ = 0;
            missionInnerWeightLocked_ = false;
            missionWeightVectorCompletedWaypoint_ = missionWaypointIndex_;
            missionSearchWaypoint_ = 255;
            missionVerifyWaypoint_ = missionWaypointIndex_;
            missionVerifyPhase_ = 1;
            missionVerifyStartedMs_ = now;
            missionVerifyHeadingDeg_ = imu_.headingDeg();
            missionVerifySawWeight_ = false;
            return;
        }
        if (!missionWeightVectorActive_ && distanceToWaypoint <= arrivalMm)
        {
            setDrive(0, 0);
            if (missionAvoidPhase_ == 1)
            {
                missionAvoidPhase_ = 2;
                missionBlockedSinceMs_ = 0;
                return;
            }
            if (missionAvoidPhase_ == 2)
            {
                missionAvoidPhase_ = 0;
                missionBlockedSinceMs_ = 0;
                // A detour can carry us past a non-target routing corner.
                // Never skip a real-weight visit point.
                if ((waypoint.flags & 0x01) == 0 &&
                    missionWaypointIndex_ + 1 < missionWaypointCount_ &&
                    (missionWaypoints_[missionWaypointIndex_ + 1].flags & 0x01) == 0)
                {
                    const MissionWaypoint& next = missionWaypoints_[missionWaypointIndex_ + 1];
                    const float nextDistance = hypotf(next.xMm - missionPoseXmm_,
                                                      next.yMm - missionPoseYmm_);
                    if (nextDistance + 150.0f < hypotf(routeDx, routeDy))
                        ++missionWaypointIndex_;
                }
                return;
            }
            if (searchActive)
            {
                if (missionSearchDwellStartedMs_ == 0)
                    missionSearchDwellStartedMs_ = now;
                if (now - missionSearchDwellStartedMs_ >= 800U)
                {
                    if (missionSearchSamples_ < 255U)
                        ++missionSearchSamples_;
                    if (weightValidMask_ != 0 &&
                        missionSearchCoverageSamples_ < 255U)
                        ++missionSearchCoverageSamples_;
                    ++missionSearchStep_;
                    missionSearchDwellStartedMs_ = 0;
                }
                return;
            }
            if ((waypoint.flags & 0x01) != 0 && !missionReturnHome_)
            {
                beginSiteSearch();
                return;
            }
            if ((waypoint.flags & 0x01) != 0 &&
                missionTargetsVisited_ < missionTargetCount_)
            {
                ++missionTargetsVisited_;
                link_.log("INFO", missionReturnHome_
                    ? "Return-home destination reached"
                    : "Pre-laid mission weight location visited");
            }
            ++missionWaypointIndex_;
            weightBearingWaypoint_ = 255;
            missionBlockedSinceMs_ = 0;
            missionAvoidAttempts_ = 0;
            return;
        }

        float steeringDx = dx, steeringDy = dy;
        if (!searchActive && !missionWeightVectorActive_ &&
            missionAvoidPhase_ == 0 && (waypoint.flags & 0x01) == 0 &&
            missionWaypointIndex_ + 1 < missionWaypointCount_ &&
            distanceToWaypoint < 450.0f)
        {
            float previewX = waypoint.xMm, previewY = waypoint.yMm;
            float remainingMm = 300.0f;
            // Several planned corners can be only 100 mm apart. Preview
            // across them, but stop before a weight, docking leg or wall.
            for (uint8_t next = missionWaypointIndex_ + 1;
                 next < missionWaypointCount_ &&
                 next <= missionWaypointIndex_ + 4; ++next)
            {
                const MissionWaypoint& following = missionWaypoints_[next];
                if (following.flags != 0) break;
                const float legX = following.xMm - previewX;
                const float legY = following.yMm - previewY;
                const float legMm = hypotf(legX, legY);
                if (legMm <= 1.0f) continue;
                const float stepMm = min(remainingMm, legMm);
                const float candidateX = previewX + stepMm * legX / legMm;
                const float candidateY = previewY + stepMm * legY / legMm;
                // Do not cut a corner across an inflated mapped obstacle.
                if (!segmentFree(missionPoseXmm_, missionPoseYmm_,
                                 candidateX, candidateY))
                    break;
                steeringDx = candidateX - missionPoseXmm_;
                steeringDy = candidateY - missionPoseYmm_;
                remainingMm -= stepMm;
                if (remainingMm <= 1.0f) break;
                previewX = following.xMm;
                previewY = following.yMm;
                if (stepMm < legMm) break;
            }
        }
        float desiredArenaHeadingDeg = atan2f(steeringDy, steeringDx) * RAD_TO_DEG;
        desiredArenaHeadingDeg = normaliseHeading(desiredArenaHeadingDeg);
        if (missionWeightVectorActive_)
        {
            const float trackHeading = missionWeightVectorHeadingDeg_ * DEG_TO_RAD;
            const float sidewaysMm =
                (missionPoseXmm_ - missionWeightVectorStartXmm_) * -sinf(trackHeading) +
                (missionPoseYmm_ - missionWeightVectorStartYmm_) * cosf(trackHeading);
            // A frozen bearing alone let the last pickup pass drift 70-80 mm
            // sideways even while its reported heading was "aligned". Aim
            // gently back at the original line, then stop and re-align if
            // the resulting heading error exceeds the collection tolerance.
            desiredArenaHeadingDeg = normaliseHeading(
                missionWeightVectorHeadingDeg_ -
                constrain(sidewaysMm * 0.12f, -12.0f, 12.0f));
        }
        // A real weight can be directly ahead even when none of the four
        // outward-facing bottom TOFs intersects it. Match the forward return
        // to the planned weight, and verify a mapped physical wall is behind
        // the target before reducing the forward stop distance. A short echo
        // far in front of the weight remains an obstacle, not a target.
        float mappedWallMm = 5000.0f;
        const bool targetLeg = !missionReturnHome_ &&
            missionAvoidPhase_ == 0 && (waypoint.flags & 0x01) != 0 &&
            (missionWeightVectorActive_ ||
             ((!searchActive || missionSearchStep_ >= 2U || followWeightBearing) &&
              targetCorridorClear && distanceToWaypoint <= 700.0f));
        if (!targetLeg || missionTargetAlignWaypoint_ != missionWaypointIndex_)
        {
            missionTargetAligned_ = false;
            missionTargetAlignSinceMs_ = 0;
            missionTargetAlignWaypoint_ = targetLeg ? missionWaypointIndex_ : 255;
        }
        if (targetLeg)
        {
            const float headingRad = missionArenaHeadingDeg_ * DEG_TO_RAD;
            const float ux = cosf(headingRad), uy = sinf(headingRad);
            const float sensorX = missionPoseXmm_ + 150.0f * ux;
            const float sensorY = missionPoseYmm_ + 150.0f * uy;
            if (ux > 0.001f) mappedWallMm = min(mappedWallMm, (4900.0f - sensorX) / ux);
            if (ux < -0.001f) mappedWallMm = min(mappedWallMm, -sensorX / ux);
            if (uy > 0.001f) mappedWallMm = min(mappedWallMm, (2400.0f - sensorY) / uy);
            if (uy < -0.001f) mappedWallMm = min(mappedWallMm, -sensorY / uy);
            for (uint8_t i = 0; i < missionFeatureCount_; ++i)
            {
                const MissionFeature& feature = missionFeatures_[i];
                if (feature.kind == 0) continue;
                mappedWallMm = min(mappedWallMm, feature.rotatedWall
                    ? rayRotatedWall(sensorX, sensorY, ux, uy,
                        feature.wallX, feature.wallY, feature.rotationDeg)
                    : rayRect(sensorX, sensorY, ux, uy, feature.x0,
                        feature.y0, feature.x1, feature.y1));
            }
            const float targetHitDistanceMm = missionWeightVectorActive_ ?
                hypotf(missionWeightVectorHitXmm_ - missionPoseXmm_,
                       missionWeightVectorHitYmm_ - missionPoseYmm_) :
                distanceToWaypoint;
            const float expectedTargetMm = max(0.0f, targetHitDistanceMm - 150.0f);
            const float rangeErrorMm = front == 0xFFFF ? 5000.0f :
                fabsf(static_cast<float>(front) - expectedTargetMm);
            missionFrontTargetRangeErrorMm_ = static_cast<uint16_t>(min(rangeErrorMm, 5000.0f));
            missionFrontTargetMappedWallMm_ = static_cast<uint16_t>(min(mappedWallMm, 5000.0f));
            missionFrontTargetVisible_ =
                front != 0xFFFF && front >= 30U && rangeErrorMm <= 170.0f &&
                mappedWallMm >= targetHitDistanceMm + 90.0f &&
                mappedWallMm >= static_cast<float>(front) +
                    (targetHitDistanceMm <= 220.0f ? 50.0f : 120.0f) &&
                fabsf(headingDelta(missionArenaHeadingDeg_,
                                   desiredArenaHeadingDeg)) <= 22.0f;
        }
        // A frontal return matters only when the robot is pointing along the
        // commanded leg; otherwise it may be the wall we are turning away from.
        // Mission legs are fast. The previous 150 mm user tuning first
        // blocked at ~160 mm and the chassis continued to an 87 mm reading.
        // Reserve 400 mm for braking/turning even when roaming is tuned closer.
        // Once the target occupies the frontal beam, the return can switch
        // from weight to the wall behind it. Keep the close approach only
        // while that wall agrees with the uploaded map and remains at least
        // 100 mm in front of the sensor; an unmatched early return still
        // uses the normal 400 mm stop.
        const bool mappedWallEcho = targetLeg && front != 0xFFFF &&
            (missionWeightVectorActive_ ? mappedWallMm >= 100.0f :
             mappedWallMm >= distanceToWaypoint + 90.0f) &&
            fabsf(mappedWallMm - static_cast<float>(front)) <=
                (missionWeightVectorActive_ ? 180.0f : 100.0f) &&
            fabsf(headingDelta(missionArenaHeadingDeg_,
                               desiredArenaHeadingDeg)) <= 22.0f;
        const bool sensedWeightCorridor = targetLeg && front >= 100U &&
            front != 0xFFFF && routeRangeMm <= 500.0f &&
            ((weightSectorMask_ & 0x06U) != 0 ||
             (weightBearingSector_ >= 1U && weightBearingSector_ <= 2U &&
              weightBearingWaypoint_ == missionWaypointIndex_ &&
              now - weightBearingSeenMs_ <= 2500U)) &&
            mappedWallMm >= static_cast<float>(front) + 100.0f &&
            fabsf(mappedWallMm - static_cast<float>(front)) <= 200.0f;
        missionTargetCommitActive_ = missionInnerWeightLocked_ ||
            missionFrontTargetVisible_ || mappedWallEcho || sensedWeightCorridor;
        const uint16_t missionStopMm = max(FRONT_AVOID_MM,
                                           static_cast<uint16_t>(400U));
        // A noisy frontal range near 400 mm made the last run alternate
        // STOP/GO and repeatedly restart bypasses. Release only after a
        // stable, clearly larger reading; keep the 100 mm hard wall limit.
        if (missionTargetCommitActive_)
        {
            missionFrontBlockedLatched_ = false;
            missionFrontClearSinceMs_ = 0;
        }
        else if (front != 0xFFFF && front < missionStopMm)
        {
            missionFrontBlockedLatched_ = true;
            missionFrontClearSinceMs_ = 0;
        }
        else if (missionFrontBlockedLatched_ && front != 0xFFFF &&
                 front >= missionStopMm + 80U)
        {
            if (missionFrontClearSinceMs_ == 0)
                missionFrontClearSinceMs_ = now;
            if (now - missionFrontClearSinceMs_ >= 300U)
                missionFrontBlockedLatched_ = false;
        }
        else
            missionFrontClearSinceMs_ = 0;
        const bool closeWallBlocked = missionTargetCommitActive_ &&
            (mappedWallMm < 100.0f ||
             (front < 100U && !missionFrontTargetVisible_));
        const bool hardFrontWall = front != 0xFFFF &&
            front < (missionTargetCommitActive_ ? 100U : 180U);
        // A 240-260 mm return is not grounds for reversing a pivot away
        // from a wall; that threshold created the repeated reverse/advance
        // loop at the first weight site. A genuinely close wall still backs out.
        if (hardFrontWall || missionForwardStalled_)
        {
            if (missionWallReverseWaypoint_ == missionWaypointIndex_ &&
                missionWallReverseRepeatCount_ >= 3U &&
                now - missionWallReverseLastStartMs_ <= 30000U)
            {
                setDrive(0, 0);
                if (searchActive && (waypoint.flags & 0x01) != 0)
                {
                    finishSiteSearch(true);
                    link_.log("WARNING", "Repeated wall retreat ended weight-site search");
                }
                else if (!rerouteAroundMap())
                    beginMissionWallReverse(
                        "Repeated wall retreat; backing out again for a new mapped approach");
                return;
            }
            beginMissionWallReverse(missionForwardStalled_
                ? "Forward drive stalled; reversing before replanning"
                : "Frontal wall too close to turn; reversing before replanning");
            return;
        }
        const bool blockedOnLeg = (front != 0xFFFF || missionFrontBlockedLatched_) &&
            (missionTargetCommitActive_ ? closeWallBlocked :
             missionFrontBlockedLatched_) &&
            (missionTargetCommitActive_ || !navigationGapPassable_ || front < 300U) &&
            // A sizeable new bearing needs a point turn, not a forward drive.
            // Allow that turn, then recheck the wall before moving ahead.
            fabsf(headingDelta(missionArenaHeadingDeg_, desiredArenaHeadingDeg)) <= 18.0f;
        if (blockedOnLeg && !searchActive && !missionReturnHome_ &&
            missionAvoidPhase_ == 0 && (waypoint.flags & 0x01) != 0 &&
            routeRangeMm <= 450.0f)
        {
            setDrive(0, 0);
            if (missionBlockedSinceMs_ == 0)
                missionBlockedSinceMs_ = now;
            if (now - missionBlockedSinceMs_ >= 2500U)
            {
                if (missionWeightVectorActive_)
                {
                    missionWeightVectorActive_ = false;
                    missionInnerWeightLocked_ = false;
                    missionWeightVectorCompletedWaypoint_ = missionWaypointIndex_;
                }
                beginSiteSearch();
            }
            return;
        }
        if (blockedOnLeg && searchActive && missionAvoidPhase_ == 0)
        {
            setDrive(0, 0);
            if (missionBlockedSinceMs_ == 0)
                missionBlockedSinceMs_ = now;
            if (now - missionBlockedSinceMs_ >= 1200U)
            {
                // The final search approach is blocked: do not wait at zero
                // motor command for the entire site-search timeout.
                missionSearchStep_ = 3;
                missionBlockedSinceMs_ = 0;
                link_.log("WARNING", "Search approach blocked; trying alternate sweep");
            }
            return;
        }
        if (blockedOnLeg && missionReturnHome_ && missionAvoidPhase_ == 0 &&
            missionBlockedSinceMs_ == 0)
        {
            missionBlockedSinceMs_ = now;
            setDrive(0, 0);
            return;
        }
        if (blockedOnLeg && missionReturnHome_ && missionAvoidPhase_ == 0 &&
            now - missionBlockedSinceMs_ < 3500U)
        {
            // Give the independent front/side landmark check time to reject
            // a drifting home pose before choosing a detour from that pose.
            setDrive(0, 0);
            return;
        }
        if (blockedOnLeg && missionAvoidPhase_ == 0 &&
            missionAvoidAttempts_ < (missionReturnHome_ ? 6U : 3U))
        {
            const float headingRad = missionArenaHeadingDeg_ * DEG_TO_RAD;
            const float forwardX = cosf(headingRad), forwardY = sinf(headingRad);
            int8_t selectedSide = 0;
            float bestScore = -1000000.0f;
            for (int8_t side = -1; side <= 1; side += 2)
            {
                const uint16_t sideRange = side < 0 ? left : right;
                if (sideRange == 0xFFFF) continue;
                // A fixed 500 mm sidestep plus 650 mm forward leg was
                // impossible near the arena edge in the return-home run,
                // even though a shorter right-side route was map-clear.
                for (const float lateralMm : {250.0f, 350.0f, 500.0f})
                {
                    if (sideRange < lateralMm +
                        0.5f * navigationRobotWidthMm_ + 50.0f) continue;
                    const float x = missionPoseXmm_ - forwardY * side * lateralMm;
                    const float y = missionPoseYmm_ + forwardX * side * lateralMm;
                    if (!segmentFree(missionPoseXmm_, missionPoseYmm_, x, y)) continue;
                    for (const float forwardMm : {250.0f, 400.0f, 650.0f})
                    {
                        const float passX = x + forwardX * forwardMm;
                        const float passY = y + forwardY * forwardMm;
                        if (!segmentFree(x, y, passX, passY)) continue;
                        const float score = static_cast<float>(sideRange) -
                            0.2f * hypotf(waypoint.xMm - passX,
                                         waypoint.yMm - passY) -
                            0.05f * (lateralMm + forwardMm) -
                            (side == missionAvoidSide_ ? 100.0f : 0.0f);
                        if (score > bestScore)
                        {
                            bestScore = score;
                            selectedSide = side;
                            missionAvoidXmm_ = x;
                            missionAvoidYmm_ = y;
                            missionAvoidForwardXmm_ = passX;
                            missionAvoidForwardYmm_ = passY;
                        }
                    }
                }
            }
            if (selectedSide != 0)
            {
                missionAvoidSide_ = selectedSide;
                missionAvoidPhase_ = 1;
                ++missionAvoidAttempts_;
                missionBlockedSinceMs_ = 0;
                link_.log("INFO", "Mission route blocked; taking map-checked live detour");
                setDrive(0, 0);
                return;
            }
        }
        if (blockedOnLeg)
        {
            setDrive(0, 0);
            if (missionBlockedSinceMs_ == 0)
            {
                missionBlockedSinceMs_ = now;
                link_.log("WARNING", "Mission leg blocked; no verified clear bypass yet");
            }
            // A live obstacle may also cut off the detour itself. Retry the
            // original route from the present pose, with bounded attempts.
            if (missionAvoidPhase_ != 0 && now - missionBlockedSinceMs_ > 1200U)
            {
                missionAvoidPhase_ = 0;
                missionBlockedSinceMs_ = 0;
            }
            else if (missionAvoidPhase_ == 0 && front != 0xFFFF &&
                     front < 400U && now - missionBlockedSinceMs_ > 3000U)
            {
                missionBlockedSinceMs_ = 0;
                beginMissionWallReverse("Blocked mission leg; backing out to seek a new approach");
            }
            return;
        }
        missionBlockedSinceMs_ = 0;
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
        bool slowWeightApproach = !missionReturnHome_ &&
            (searchActive || missionWeightVectorActive_ ||
             ((waypoint.flags & 0x01) != 0 && routeRangeMm <= 700.0f));
        if (!slowWeightApproach && !missionReturnHome_ &&
            missionWaypointIndex_ + 1 < missionWaypointCount_ &&
            (missionWaypoints_[missionWaypointIndex_ + 1].flags & 0x01) != 0)
        {
            const MissionWaypoint& site = missionWaypoints_[missionWaypointIndex_ + 1];
            slowWeightApproach = hypotf(site.xMm - missionPoseXmm_,
                                        site.yMm - missionPoseYmm_) <= 700.0f;
        }
        if (fabsf(headingError) > 16.0f &&
            (commandedA * commandedB < 0 || missionPointTurnActive_ ||
             (targetLeg && !missionTargetAligned_)))
        {
            if (missionTurnProgressStartedMs_ == 0)
            {
                missionTurnProgressStartedMs_ = now;
                missionTurnProgressHeadingDeg_ = imu_.headingDeg();
            }
            else if (now - missionTurnProgressStartedMs_ >= 3000U)
            {
                const float turnedDeg = fabsf(headingDelta(
                    missionTurnProgressHeadingDeg_, imu_.headingDeg()));
                missionTurnProgressStartedMs_ = now;
                missionTurnProgressHeadingDeg_ = imu_.headingDeg();
                if (turnedDeg < 5.0f)
                {
                    beginMissionWallReverse(
                        "Point turn made no IMU progress; reversing before another turn");
                    return;
                }
            }
        }
        else
            missionTurnProgressStartedMs_ = 0;
        // Estimate actual yaw speed from the IMU, not the changing waypoint
        // error: a search/replan can move the goal without moving the robot.
        if (missionTurnLastSampleMs_ == 0)
        {
            missionTurnLastHeadingDeg_ = imu_.headingDeg();
            missionTurnLastSampleMs_ = now;
            missionTurnYawRateDegPerSec_ = 0.0f;
        }
        else if (now - missionTurnLastSampleMs_ >= 40U)
        {
            const float dt = (now - missionTurnLastSampleMs_) * 0.001f;
            const float rate = headingDelta(missionTurnLastHeadingDeg_,
                                            imu_.headingDeg()) / dt;
            missionTurnYawRateDegPerSec_ =
                0.65f * missionTurnYawRateDegPerSec_ +
                0.35f * constrain(rate, -360.0f, 360.0f);
            missionTurnLastHeadingDeg_ = imu_.headingDeg();
            missionTurnLastSampleMs_ = now;
        }
        auto trimTurn = [&](int8_t direction, float absoluteError,
                            float toleranceDeg, uint32_t /*initialCoastMs*/) {
            // No duty-cycle turning: use both wheels continuously until the
            // measured heading is inside tolerance. Coast only if momentum
            // is already carrying the chassis into the target heading.
            if (absoluteError <= toleranceDeg + 3.0f &&
                fabsf(missionTurnYawRateDegPerSec_) >= 24.0f)
            {
                setDrive(0, 0);
                return;
            }
            setDrive(direction > 0 ? -85 : 85,
                     direction > 0 ? 85 : -85);
        };
        if (targetLeg)
        {
            // Before committing to a mapped weight location, point the robot
            // down the actual robot-to-target bearing and allow yaw to settle.
            // Do not use the wide 18-degree transit tolerance for collection.
            const float alignmentToleranceDeg = missionWeightVectorActive_
                ? (missionTargetAligned_ ? 5.0f : 4.0f)
                : (missionTargetAligned_ ? 10.0f : 8.0f);
            if (fabsf(headingError) > alignmentToleranceDeg)
            {
                missionTargetAligned_ = false;
                missionTargetAlignSinceMs_ = 0;
                if (fabsf(headingError) <= 45.0f &&
                    fabsf(headingError) >= 12.0f &&
                    distanceToWaypoint > 350.0f &&
                    front != 0xFFFF && front >= 400U)
                {
                    // Approach the target on a powered arc while still well
                    // away from it; reserve in-place alignment for the final
                    // short collection corridor.
                    setDrive(headingError > 0.0f ? -100 : -75,
                             headingError > 0.0f ? -75 : -100);
                    return;
                }
                const bool turnRight = headingError > 0.0f;
                const int8_t direction = turnRight ? 1 : -1;
                if (direction != missionTurnDirection_)
                {
                    missionTurnDirection_ = direction;
                    missionTurnFinishing_ = false;
                    missionTurnPulseUntilMs_ = 0;
                    missionTurnCoastUntilMs_ = now + 100U;
                }
                if (now < missionTurnCoastUntilMs_)
                {
                    setDrive(0, 0);
                    return;
                }
                const float brakingAngle = constrain(
                    10.0f + fabsf(missionTurnYawRateDegPerSec_) * 0.30f,
                    14.0f, 32.0f);
                if (fabsf(headingError) > brakingAngle)
                {
                    missionTurnFinishing_ = false;
                    missionTurnPulseUntilMs_ = 0;
                    setDrive(turnRight ? -85 : 85, turnRight ? 85 : -85);
                }
                else
                    trimTurn(direction, fabsf(headingError),
                             alignmentToleranceDeg, 180U);
                return;
            }
            missionTurnFinishing_ = false;
            missionTurnPulseUntilMs_ = 0;
            if (fabsf(missionTurnYawRateDegPerSec_) >= 18.0f)
            {
                setDrive(0, 0);
                return;
            }
            if (!missionTargetAligned_)
            {
                setDrive(0, 0);
                if (missionTargetAlignSinceMs_ == 0)
                    missionTargetAlignSinceMs_ = now;
                if (now - missionTargetAlignSinceMs_ < 100U)
                    return;
                missionTargetAligned_ = true;
            }
            // A locked collection bearing should continue forward, not fall
            // into the generic point-turn state retained from a route leg.
            missionPointTurnActive_ = false;
            missionTurnDirection_ = 0;
            missionTurnFinishing_ = false;
            driveOnHeading(targetBnoHeadingDeg, 65);
            return;
        }
        // Hysteresis avoids alternating between a stationary point turn and
        // forward drive at the same 18-degree boundary each sensor update.
        if (!missionPointTurnActive_ &&
            fabsf(headingError) > (slowWeightApproach ? 25.0f : 70.0f))
            missionPointTurnActive_ = true;
        else if (missionPointTurnActive_ &&
                 fabsf(headingError) <= (slowWeightApproach ? 10.0f : 35.0f))
            missionPointTurnActive_ = false;
        if (missionPointTurnActive_)
        {
            // Point-turn until the requested path is reasonably aligned, then
            // proportional straight-line heading correction takes over. A
            // target exactly behind the robot has two equally short turns;
            // keep the chosen direction through that 180-degree boundary.
            int8_t direction = headingError > 0.0f ? 1 : -1;
            if (fabsf(headingError) >= 160.0f && missionTurnDirection_ != 0)
                direction = missionTurnDirection_;
            if (direction != missionTurnDirection_)
            {
                missionTurnDirection_ = direction;
                missionTurnFinishing_ = false;
                missionTurnPulseUntilMs_ = 0;
                missionTurnCoastUntilMs_ = now + 100U;
            }
            if (now < missionTurnCoastUntilMs_)
            {
                setDrive(0, 0);
                return;
            }
            const float absoluteError = fabsf(headingError);
            const int16_t turnPower = absoluteError > 90.0f ? 95 : 85;
            setDrive(direction > 0 ? -turnPower : turnPower,
                     direction > 0 ? turnPower : -turnPower);
        }
        else
        {
            missionTurnDirection_ = 0;
            missionTurnFinishing_ = false;
            missionTurnPulseUntilMs_ = 0;
            const int16_t basePower = slowWeightApproach ? 65 :
                (distanceToWaypoint < 350.0f ? 90 : 100);
            if (fabsf(headingError) > 12.0f && !slowWeightApproach)
            {
                const float differential = constrain(
                    fabsf(headingError) * 0.65f, 8.0f, 25.0f);
                const int16_t outer = static_cast<int16_t>(constrain(
                    basePower + differential * 0.5f, 75.0f, 100.0f));
                const int16_t inner = static_cast<int16_t>(constrain(
                    basePower - differential * 0.5f, 75.0f, 100.0f));
                setDrive(headingError > 0.0f ? -outer : -inner,
                         headingError > 0.0f ? -inner : -outer);
            }
            else
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
    if (now - lastTof8x8FrameMs_ < 10U)
        return;
    lastTof8x8FrameMs_ = now;

    bool freshFrame = false;
    if (tof8x8_.available())
        freshFrame = tof8x8_.read();
    if (freshFrame)
    {
        tof8x8FrameHeadingDeg_ = imu_.headingDeg();
        tof8x8FrameEncoder1_ = encoders_.firstCount();
        tof8x8FrameEncoder2_ = encoders_.secondCount();
    }
    // Navigation consumes every fresh sensor read, but the low-bandwidth BLE
    // debug link only needs one visualisation frame per second.
    if (now - lastTof8x8TransmitMs_ >= 1000U &&
        (freshFrame || !tof8x8_.lastReadSucceeded()))
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


