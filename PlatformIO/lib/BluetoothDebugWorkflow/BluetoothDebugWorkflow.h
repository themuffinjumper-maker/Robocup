#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Bno055Imu.h>
#include <DcMotor203.h>
#include <DigitalInputSensor.h>
#include <DualEncoder.h>
#include <HerkulexTeensy.h>
#include <Hx12kServo.h>
#include <RobotDebug.h>
#include <Tof.h>
#include <Tof8x8.h>
#include <UltrasoundSensor.h>

class BluetoothDebugWorkflow {
public:
    BluetoothDebugWorkflow(HardwareSerialIMXRT& bluetoothPort, HardwareSerial& herkulexPort);

    void begin();
    void update();

private:
    static constexpr uint8_t MAX_TOF_SENSORS = 8;
    static constexpr uint8_t MAX_DIGITAL_INPUTS = 8;
    static constexpr uint8_t MAX_ULTRASOUND_SENSORS = 2;
    HardwareSerialIMXRT& bluetoothPort_;
    uint8_t bluetoothRxBuffer_[2048] = {};
    // 8 KiB is large enough for one complete telemetry frame plus the 2 KiB
    // command-response reserve.  Keep this below 32 KiB: Teensy's UART ring
    // head/tail counters are 16-bit and the larger field test buffer prevented
    // Serial1 from producing any bytes after boot on this board.
    uint8_t bluetoothTxBuffer_[8192] = {};
    HerkulexTeensy servos_;
    DcMotor203 dcMotor203_;
    DcMotor203 dcMotor203Second_;
    DualEncoder encoders_;
    Bno055Imu imu_;
    Hx12kServo hx12kA_;
    Hx12kServo hx12kB_;
    Hx12kServo hx12kC_;
    Hx12kServo hx12kD_;
    RobotDebug link_;
    Tof8x8 tof8x8_;
    JsonDocument config_;

    bool debugMode_ = false;
    bool stopped_ = true;
    uint8_t lastServoId_ = 1;
    float lastServoAngleDeg_ = 0.0f;
    float measuredServoAngleDeg_ = NAN;
    float servoZeroOffsetsDeg_[254] = {};
    bool servoZeroed_[254] = {};
    bool autoReadEnabled_ = false;
    uint8_t autoReadServoId_ = 1;
    bool servoResponding_ = false;
    bool trackingFault_ = false;
    uint8_t consecutiveReadFailures_ = 0;
    uint8_t consecutiveTrackingErrors_ = 0;
    float servoPositionErrorDeg_ = NAN;
    uint32_t lastServoReadMs_ = 0;
    uint32_t lastMoveStartMs_ = 0;
    uint16_t lastMoveDurationMs_ = 0;
    bool continuousVelocityActive_ = false;
    int16_t commandedVelocity_ = 0;
    bool dcMotor203Active_ = false;
    bool dcMotor203SecondActive_ = false;
    bool dcMotor203SecondDeadman_ = false;
    uint32_t lastDcMotor203SecondCommandMs_ = 0;
    static constexpr uint32_t KEYBOARD_DRIVE_TIMEOUT_MS = 500;
    uint32_t telemetryIntervalMs_ = 200;
    uint32_t lastTelemetryMs_ = 0;
    uint32_t lastElectricalDiagnosticsMs_ = 0;
    uint32_t lastDefinitionsMs_ = 0;
    uint32_t receivedMessages_ = 0;
    uint32_t lastUpdateStartedUs_ = 0;
    uint32_t maxUpdateGapUs_ = 0;
    uint8_t tofSensorCount_ = 0;
    char tofSensorNames_[MAX_TOF_SENSORS][25] = {};
    bool tofAvailable_[MAX_TOF_SENSORS] = {};
    bool tofTimedOut_[MAX_TOF_SENSORS] = {};
    int16_t tofDistanceMm_[MAX_TOF_SENSORS] = {};
    uint32_t lastTof8x8FrameMs_ = 0;
    uint32_t lastTof8x8TransmitMs_ = 0;
    uint8_t digitalInputCount_ = 0;
    char digitalInputNames_[MAX_DIGITAL_INPUTS][25] = {};
    DigitalInputSensor digitalInputs_[MAX_DIGITAL_INPUTS];
    uint8_t ultrasoundSensorCount_ = 0;
    char ultrasoundSensorNames_[MAX_ULTRASOUND_SENSORS][25] = {};
    UltrasoundSensor ultrasoundSensors_[MAX_ULTRASOUND_SENSORS];
    int8_t activeUltrasoundIndex_ = -1;
    uint8_t nextUltrasoundIndex_ = 0;
    enum NavigationState : uint8_t {
        NAV_IDLE = 0,
        NAV_SEEK_WALL = 1,
        NAV_INITIAL_TURN = 2,
        NAV_FOLLOW_WALL = 3,
        NAV_CORNER_TURN = 4,
        NAV_SWEEP = 5,
        NAV_LANE_TURN_OUT = 6,
        NAV_LANE_SHIFT = 7,
        NAV_LANE_TURN_IN = 8,
        NAV_COMPLETE = 9,
        NAV_OBSTACLE_TURN_OUT = 10,
        NAV_OBSTACLE_OFFSET = 11,
        NAV_OBSTACLE_TURN_FORWARD = 12,
        NAV_OBSTACLE_PASS = 13,
        NAV_OBSTACLE_TURN_BACK = 14,
        NAV_OBSTACLE_RETURN = 15,
        NAV_OBSTACLE_TURN_IN = 16,
        NAV_ESCAPE_REVERSE = 17,
        NAV_ESCAPE_TURN = 18,
        NAV_CLEARANCE_TURN = 19,
        NAV_RECOVERY_REVERSE = 20,
        NAV_RECOVERY_TURN = 21,
        NAV_MISSION_TRACK = 22
    };
    bool navigationActive_ = false;
    uint8_t navigationState_ = 0;
    uint32_t lastRangePollMs_ = 0;
    uint32_t lastMotionSampleMs_ = 0;
    bool lastImuSampleValid_ = false;
    uint16_t navigationFrontMm_ = 0;
    uint16_t navigationLeftMm_ = 0;
    uint16_t navigationRightMm_ = 0;
    uint32_t navigationMotionStartedMs_ = 0;
    float navigationHeadingReferenceDeg_ = 0.0f;
    float navigationTargetHeadingDeg_ = 0.0f;
    uint32_t navigationTurnSettledSinceMs_ = 0;
    uint32_t navigationTurnPulseStartedMs_ = 0;
    uint32_t navigationTurnCoastUntilMs_ = 0;
    int8_t navigationTurnDirection_ = 0;
    uint8_t navigationMatrixCloseZones_ = 0;
    uint8_t navigationMatrixUsableZones_ = 0;
    bool navigationMatrixBroadWall_ = false;
    int32_t navigationShiftStartEncoder1_ = 0;
    int32_t navigationShiftStartEncoder2_ = 0;
    uint8_t navigationLaneIndex_ = 0;
    bool navigationSweepTurnRight_ = true;
    uint16_t navigationSweepLeftReferenceMm_ = 0;
    uint16_t navigationSweepRightReferenceMm_ = 0;
    bool navigationSweepLeftReferenceValid_ = false;
    bool navigationSweepRightReferenceValid_ = false;
    int16_t navigationSweepLateralErrorMm_ = 0;
    float navigationSweepProgressMm_ = 0.0f;
    float navigationExpectedSweepLengthMm_ = 0.0f;
    bool navigationExpectedSweepLengthValid_ = false;
    int32_t navigationSweepProgressLastEncoder1_ = 0;
    int32_t navigationSweepProgressLastEncoder2_ = 0;

    // Temporary obstacle-bypass state. A detour leaves the active sweep lane,
    // tracks around one localised obstacle, then uses encoder distance and the
    // exact BNO055 heading lattice to return to the same original path.
    bool navigationDetourRight_ = true;
    float navigationDetourOriginalHeadingDeg_ = 0.0f;
    float navigationDetourOffsetMm_ = 0.0f;
    int32_t navigationDetourStartEncoder1_ = 0;
    int32_t navigationDetourStartEncoder2_ = 0;
    int32_t navigationDetourPhaseStartEncoder1_ = 0;
    int32_t navigationDetourPhaseStartEncoder2_ = 0;
    bool navigationDetourEdgeCleared_ = false;
    bool navigationDetourObstacleSeen_ = false;
    uint8_t navigationDetourClearSamples_ = 0;
    uint32_t navigationDetourLastTriggerCount_ = 0;
    uint16_t navigationObstacleCount_ = 0;
    int32_t navigationEscapeStartEncoder1_ = 0;
    int32_t navigationEscapeStartEncoder2_ = 0;
    bool navigationEscapeTurnRight_ = true;

    // Generic non-aborting recovery. Failed local plans back away before
    // choosing a new heading, so navigation keeps exploring instead of
    // disabling itself or chaining endless in-place quarter-turns.
    int32_t navigationRecoveryStartEncoder1_ = 0;
    int32_t navigationRecoveryStartEncoder2_ = 0;
    bool navigationRecoveryTurnRight_ = true;
    uint16_t navigationRecoveryCount_ = 0;
    uint8_t navigationRecoveryAttemptCount_ = 0;
    float navigationRecoveryTotalReverseMm_ = 0.0f;
    bool navigationRecoveryForceHalfTurn_ = false;
    uint8_t navigationClearanceTurnCount_ = 0;
    uint8_t navigationStrategy_ = 0;
    uint16_t navigationFrontAvoidMm_ = 300;
    uint16_t navigationSideAvoidMm_ = 200;
    uint16_t navigationWallFollowMm_ = 200;
    uint16_t navigationLaneSpacingMm_ = 200;
    uint16_t navigationRobotWidthMm_ = 430;
    uint16_t navigationGapMarginMm_ = 80;
    uint16_t navigationGapDepthMm_ = 550;
    uint8_t navigationMatrixFloorRows_ = 2;
    uint8_t navigationMatrixFovDeg_ = 40;
    uint8_t navigationGapConfirmFrames_ = 2;
    uint8_t navigationGapSeenFrames_ = 0;
    bool navigationGapPassable_ = false;
    int8_t navigationGapCentreColumnX2_ = 0;
    uint16_t navigationGapWidthMm_ = 0;

    // Pre-laid mission strategy (navigation.strategy == 3). The desktop GUI
    // does the expensive grid/A* planning and uploads a compact polyline.
    // Firmware continuously re-aims at the active waypoint using BNO055 yaw
    // and encoder distance so small path errors are corrected on the robot.
    static constexpr uint8_t MAX_MISSION_WAYPOINTS = 64;
    struct MissionWaypoint {
        int16_t xMm = 0;
        int16_t yMm = 0;
        uint8_t flags = 0;  // bit 0 = real-weight visit point
    };
    MissionWaypoint missionWaypoints_[MAX_MISSION_WAYPOINTS] = {};
    uint8_t missionWaypointCount_ = 0;
    uint8_t missionWaypointIndex_ = 0;
    uint8_t missionFallbackStrategy_ = 0;
    uint8_t missionTargetCount_ = 0;
    uint8_t missionTargetsVisited_ = 0;
    bool missionPlanValid_ = false;
    bool missionPoseInitialised_ = false;
    float missionStartXmm_ = 0.0f;
    float missionStartYmm_ = 0.0f;
    float missionStartArenaHeadingDeg_ = 0.0f;
    float missionStartImuHeadingDeg_ = 0.0f;
    float missionPoseXmm_ = 0.0f;
    float missionPoseYmm_ = 0.0f;
    float missionArenaHeadingDeg_ = 0.0f;
    int32_t missionLastEncoder1_ = 0;
    int32_t missionLastEncoder2_ = 0;
    uint32_t missionBlockedSinceMs_ = 0;

    // Live low-object/weight visibility from matched bottom/top point-TOF pairs.
    // Four sectors run far-left, mid-left, mid-right, far-right. A sector is
    // confirmed only after three fresh 100 ms polls agree that the bottom TOF
    // sees an object at least 150 mm closer than its matched top TOF.
    uint8_t weightEvidence_[4] = {};
    uint8_t weightSectorMask_ = 0;
    uint16_t weightNearestMm_ = 0;
    int8_t weightDirection_ = 0;

    bool navigationMotionConsistent_ = true;

    static void dispatch(JsonDocument& message, void* context);
    void handleMessage(JsonDocument& message);
    void handleCommand(JsonDocument& message);
    void handleParameter(JsonDocument& message);
    void sendDefinitions();
    void sendState();
    void sendTelemetry();
    void initialiseTofSensors();
    void initialiseDigitalInputs();
    void initialiseUltrasoundSensors();
    void updateUltrasoundSensors();
    void updateMotionAndRangeSensors();
    void updateNavigation();
    void stopNavigation(const char* reason = nullptr);
    void readTofSensors();
    void updateWeightDetection();
    void updateTof8x8();
    void sendTof8x8Frame();
    void updateAutomaticServoRead();
    void updateTrackingState(float absoluteAngle);
    JsonObject findParameter(const char* name);
    JsonObject findCommand(const char* name);
};
