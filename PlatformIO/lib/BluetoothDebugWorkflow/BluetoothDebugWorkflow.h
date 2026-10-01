#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Bno055Imu.h>
#include <DcMotor203.h>
#include <DigitalInputSensor.h>
#include <DualEncoder.h>
#include <HerkulexTeensy.h>
#include <Hx12kServo.h>
#include <LocalCompetition.h>
#include <RobotDebug.h>
#include <SparkFunSX1509.h>
#include <Tof.h>
#include <Tof8x8.h>
#include <UltrasoundSensor.h>

class BluetoothDebugWorkflow {
public:
    BluetoothDebugWorkflow(HardwareSerialIMXRT& bluetoothPort, HardwareSerial& herkulexPort);

    void begin();
    void update();
    // Optional local-autonomy hook, deliberately called as one line from
    // main.cpp so the whole competition supervisor can be unplugged easily.
    void updateLocalCompetition();

private:
    static constexpr uint8_t MAX_TOF_SENSORS = 8;
    static constexpr uint8_t MAX_DIGITAL_INPUTS = 8;
    static constexpr uint8_t MAX_ULTRASOUND_SENSORS = 2;
    HardwareSerialIMXRT& bluetoothPort_;
    uint8_t bluetoothRxBuffer_[2048] = {};
    // Telemetry and definitions are paced into small complete JSON lines.
    // This ring size has already worked with the Teensy Serial1 driver.
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
    SX1509 aioExpander_;
    JsonDocument config_;
    LocalCompetition localCompetition_;
    bool localRunActive_ = false;
    bool localMissionReturning_ = false;
    bool localBlockTargetValid_ = false;
    bool localBlockTargetEnroute_ = false;
    int16_t localBlockTargetXmm_ = 0;
    int16_t localBlockTargetYmm_ = 0;
    uint8_t localPostUnloadPhase_ = 0; // 0 idle, 1 open-gate dwell, 2 travelling, 3 blocking spin
    uint32_t localPostUnloadStartedMs_ = 0;
    static constexpr uint32_t LOCAL_POST_UNLOAD_WAIT_MS = 5000U;
    uint16_t localCompetitionSortingSerial_ = 0;
    char localCompetitionStatus_[24] = "DISABLED";

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
    bool armSortingEnabled_ = false;
    bool sortingDetectionConfirmed_ = false;
    bool sortingBumpersOn_ = false;
    bool sortingGatePositive_ = false;
    bool sortingGateReferenceValid_ = false;
    uint32_t sortingGateTorqueOffAtMs_ = 0;
    uint16_t sortingConfirmedSerial_ = 0;
    uint16_t missionHandledSortingSerial_ = 0;
    uint8_t missionPendingPickupWaypoint_ = 255;
    uint32_t missionPendingPickupSinceMs_ = 0;
    bool missionPendingPickupWasIncomplete_ = false;
    uint32_t missionPostPickupStraightUntilMs_ = 0;
    float missionPostPickupHeadingDeg_ = 0.0f;
    bool sortingIdlePulseActive_ = false;
    uint8_t sortingInputIndex_ = 0;
    uint32_t sortingDetectedSinceMs_ = 0;
    uint32_t sortingClearedSinceMs_ = 0;
    uint32_t sortingNextPulseMs_ = 0;
    uint32_t sortingPulseEndMs_ = 0;
    bool continuousVelocityActive_ = false;
    int16_t commandedVelocity_ = 0;
    bool dcMotor203Active_ = false;
    bool drumJamBackoffActive_ = false;
    bool drumJamLatched_ = false;
    uint32_t drumJamLowSinceMs_ = 0;
    uint32_t drumJamClearSinceMs_ = 0;
    uint32_t drumJamBackoffUntilMs_ = 0;
    uint32_t drumJamCooldownUntilMs_ = 0;
    uint32_t drumJamMonitorStartedMs_ = 0;
    uint32_t drumJamLastSampleMs_ = 0;
    uint16_t drumJamBackoffTimeMs_ = 500;
    int16_t drumJamBackoffPercent_ = -100;
    uint16_t drumJamDropAdcCounts_ = 40;
    uint16_t drumJamConfirmTimeMs_ = 400;
    float drumJamStartupCurrentSumA_ = 0.0f;
    float drumJamStartupAdcSum_ = 0.0f;
    uint16_t drumJamStartupSampleCount_ = 0;
    int16_t drumJamResumeA_ = 0;
    int16_t drumJamResumeB_ = 0;
    float drumJamBaselineA_ = 10.0f;
    float drumJamBaselineAdc_ = 0.0f;
    bool drumJamBaselineInitialized_ = false;
    uint8_t drumJamAttemptsInEpisode_ = 0;
    uint16_t drumJamRecoveryCount_ = 0;
    bool dcMotor203SecondActive_ = false;
    bool dcMotor203SecondDeadman_ = false;
    uint32_t lastDcMotor203SecondCommandMs_ = 0;
    static constexpr uint32_t KEYBOARD_DRIVE_TIMEOUT_MS = 500;
    uint32_t telemetryIntervalMs_ = 200;
    uint32_t lastTelemetryMs_ = 0;
    uint32_t lastElectricalDiagnosticsMs_ = 0;
    uint32_t lastCurrentSampleMs_ = 0;
    uint16_t currentSensorRawAdc_ = 0;
    float currentSensorFilteredVolts_ = 0.0f;
    bool currentSensorSampleValid_ = false;
    // ACS712-30A at 5 V: 2.5 V zero and 66 mV/A; 12k/12k halves both.
    float currentSensorZeroVolts_ = 1.25f;
    uint32_t lastDefinitionsMs_ = 0;
    uint32_t lastDefinitionSendMs_ = 0;
    uint32_t lastPeerMessageMs_ = 0;
    uint32_t lastLinkProbeMs_ = 0;
    uint32_t lastTelemetryChunkMs_ = 0;
    uint16_t nextTelemetryField_ = 0;
    uint8_t nextDefinition_ = 0;
    bool peerSeen_ = false;
    bool definitionsPending_ = false;
    bool telemetryPending_ = false;
    JsonDocument pendingTelemetry_;
    uint32_t receivedMessages_ = 0;
    uint32_t lastUpdateStartedUs_ = 0;
    uint32_t maxUpdateGapUs_ = 0;
    uint8_t tofSensorCount_ = 0;
    char tofSensorNames_[MAX_TOF_SENSORS][25] = {};
    bool tofAvailable_[MAX_TOF_SENSORS] = {};
    bool tofTimedOut_[MAX_TOF_SENSORS] = {};
    bool tofFresh_[MAX_TOF_SENSORS] = {};
    int16_t tofDistanceMm_[MAX_TOF_SENSORS] = {};
    uint32_t tofLastSampleMs_[MAX_TOF_SENSORS] = {};
    uint8_t tofReadyCount_ = 0;
    uint8_t tofBadPolls_ = 0;
    uint16_t tofRecoveryAttempts_ = 0;
    uint32_t lastTofRecoveryAttemptMs_ = 0;
    uint32_t lastTofRecoverySuccessMs_ = 0;
    uint32_t lastTofRecoveryDurationMs_ = 0;
    uint8_t tofRecoverySensorCursor_ = 0;
    uint8_t tofRecoveryFailures_[MAX_TOF_SENSORS] = {};
    uint32_t tofNextRecoveryAttemptMs_[MAX_TOF_SENSORS] = {};
    bool missionTofRecoveryHold_ = false;
    bool tofDegradedLogged_ = false;
    uint8_t missionOuterCueWaypoint_ = 255;
    uint8_t missionOuterCueTriedMask_ = 0;
    uint8_t missionOuterReverseTriedMask_ = 0;
    uint32_t missionOuterCueStartedMs_ = 0;
    uint32_t missionOuterCueAlignedSinceMs_ = 0;
    float missionOuterCueHeadingDeg_ = 0.0f;
    float missionOuterCueHitXmm_ = 0.0f;
    float missionOuterCueHitYmm_ = 0.0f;
    bool missionOuterCueActive_ = false;
    bool missionOuterReverseActive_ = false;
    uint32_t missionOuterReverseStartedMs_ = 0;
    int32_t missionOuterReverseEncoder1_ = 0;
    int32_t missionOuterReverseEncoder2_ = 0;
    uint32_t lastTof8x8FrameMs_ = 0;
    uint32_t lastTof8x8TransmitMs_ = 0;
    float tof8x8FrameHeadingDeg_ = 0.0f;
    int32_t tof8x8FrameEncoder1_ = 0;
    int32_t tof8x8FrameEncoder2_ = 0;
    uint8_t digitalInputCount_ = 0;
    char digitalInputNames_[MAX_DIGITAL_INPUTS][25] = {};
    DigitalInputSensor digitalInputs_[MAX_DIGITAL_INPUTS];
    bool aioExpanderReady_ = false;
    bool aio9ButtonRawHigh_ = false;
    uint32_t lastAio9ButtonSampleMs_ = 0;
    uint32_t lastAio9ExpanderRetryMs_ = 0;
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
    uint16_t navigationHomeFrontTofMm_ = 0;
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
    // These are physical, user-tunable distances.  The wall clearance is
    // measured from the outside of the chassis, not from its centre.
    uint16_t navigationFrontAvoidMm_ = 150;
    uint16_t navigationMatrixWallMm_ = 400;
    uint16_t navigationSideAvoidMm_ = 200;
    uint16_t navigationWallFollowMm_ = 200;
    uint16_t navigationLaneSpacingMm_ = 200;
    // Width determines whether a forward gap is passable. Length converts
    // centre-to-target distance into a range at the forward sensor face.
    uint16_t navigationRobotWidthMm_ = 290;
    uint16_t navigationRobotLengthMm_ = 430;
    uint16_t navigationWallClearanceMm_ = 30;
    uint16_t navigationRouteLineToleranceMm_ = 100;
    uint16_t navigationGapMarginMm_ = 60;
    uint16_t navigationGapDepthMm_ = 550;
    uint16_t navigationWeightSearchRadiusMm_ = 500;
    uint16_t navigationWeightApproachClearanceMm_ = 180;
    // Either inward-facing intake TOF may authorise collection by itself.
    // This keeps the pickup path usable if its opposite sensor is offline.
    uint16_t navigationWeightInnerLockMm_ = 130;
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
        uint8_t flags = 0;  // bit 0 = target, bit 2 = reverse dock, bit 3 = gap-centre transit
    };
    MissionWaypoint missionWaypoints_[MAX_MISSION_WAYPOINTS] = {};
    MissionWaypoint localBlockRoute_[MAX_MISSION_WAYPOINTS] = {};
    uint8_t localBlockRouteCount_ = 0;
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
    uint8_t missionLaunchPhase_ = 0;  // 1 = align in home, 2 = straight exit
    uint32_t missionLaunchStartedMs_ = 0;
    int32_t missionLaunchEncoder1_ = 0;
    int32_t missionLaunchEncoder2_ = 0;
    uint8_t missionLaunchRealignAttempts_ = 0;
    bool missionLaunchBlocked_ = false;
    float missionPoseXmm_ = 0.0f;
    float missionPoseYmm_ = 0.0f;
    float missionArenaHeadingDeg_ = 0.0f;
    int32_t missionLastEncoder1_ = 0;
    int32_t missionLastEncoder2_ = 0;
    uint32_t missionLastPoseUpdateMs_ = 0;
    uint16_t missionOdometryRejectedSteps_ = 0;
    int8_t missionTurnDirection_ = 0;
    bool missionPointTurnActive_ = false;
    bool missionTurnFinishing_ = false;
    uint32_t missionTurnBrakeUntilMs_ = 0;
    bool missionFrontBlockedLatched_ = false;
    uint32_t missionFrontClearSinceMs_ = 0;
    uint32_t missionTurnCoastUntilMs_ = 0;
    uint32_t missionTurnPulseUntilMs_ = 0;
    uint32_t missionLastPointTurnMs_ = 0;
    uint32_t missionTurnLastSampleMs_ = 0;
    float missionTurnLastHeadingDeg_ = 0.0f;
    float missionTurnYawRateDegPerSec_ = 0.0f;
    uint32_t missionTargetAlignSinceMs_ = 0;
    bool missionTargetAligned_ = false;
    uint8_t missionTargetAlignWaypoint_ = 255;
    bool missionWeightVectorActive_ = false;
    uint32_t missionCollectionFollowThroughUntilMs_ = 0;
    uint8_t missionWeightVectorWaypoint_ = 255;
    uint8_t missionWeightVectorCompletedWaypoint_ = 255;
    float missionWeightVectorStartXmm_ = 0.0f;
    float missionWeightVectorStartYmm_ = 0.0f;
    float missionWeightVectorHitXmm_ = 0.0f;
    float missionWeightVectorHitYmm_ = 0.0f;
    float missionWeightVectorEndXmm_ = 0.0f;
    float missionWeightVectorEndYmm_ = 0.0f;
    float missionWeightVectorHeadingDeg_ = 0.0f;
    float missionWeightVectorTravelMm_ = 0.0f;
    uint16_t missionWeightVectorOvershootMm_ = 0;
    bool missionInnerWeightLocked_ = false;
    uint32_t missionInnerAlignStartedMs_ = 0;
    uint32_t missionInnerCentreStableSinceMs_ = 0;
    uint32_t missionBlockedSinceMs_ = 0;
    uint32_t missionLastRerouteMs_ = 0;
    float missionReroutePoseXmm_ = 0.0f;
    float missionReroutePoseYmm_ = 0.0f;
    bool missionReroutePoseValid_ = false;
    uint8_t missionRerouteCount_ = 0;
    uint8_t missionSkippedWaypoints_ = 0;
    uint8_t missionSearchWaypoint_ = 255;
    uint8_t missionSearchStep_ = 0;
    uint8_t missionSearchSitesChecked_ = 0;
    uint8_t missionSearchSitesIncomplete_ = 0;
    uint32_t missionSearchStartedMs_ = 0;
    uint32_t missionSearchDwellStartedMs_ = 0;
    uint32_t missionSearchLastEvidenceMs_ = 0;
    uint8_t missionSearchPasses_ = 0;
    uint8_t missionSearchSamples_ = 0;
    uint8_t missionSearchCoverageSamples_ = 0;
    float missionSearchApproachHeadingDeg_ = 0.0f;
    float missionSearchCenterXmm_ = 0.0f;
    float missionSearchCenterYmm_ = 0.0f;
    uint8_t missionVerifyPhase_ = 0; // 0 idle, 1 pause, 2-4 scan, 5 reverse for retry
    uint8_t missionVerifyWaypoint_ = 255;
    uint8_t missionVerifyRetries_ = 0;
    uint32_t missionVerifyStartedMs_ = 0;
    float missionVerifyHeadingDeg_ = 0.0f;
    bool missionVerifySawWeight_ = false;
    float missionVerifyHitXmm_ = 0.0f;
    float missionVerifyHitYmm_ = 0.0f;
    int32_t missionRetryReverseEncoder1_ = 0;
    int32_t missionRetryReverseEncoder2_ = 0;
    float missionRetryReverseDistanceMm_ = 0.0f;
    uint32_t missionRetryReverseStartedMs_ = 0;
    uint32_t missionLandmarkRecoveryLastMs_ = 0;
    uint8_t missionLandmarkRecoveryMatches_ = 0;
    float missionLandmarkRecoveryXmm_ = 0.0f;
    float missionLandmarkRecoveryYmm_ = 0.0f;
    uint32_t missionPoseRecoveryHoldUntilMs_ = 0;
    uint16_t missionLandmarkRecoveryCount_ = 0;
    uint16_t missionLandmarkRecoveryResidualMm_ = 0;
    // Arena features uploaded with the route: 0 no-go ramp, 1 wall,
    // 2 dummy weight, 3 traversable go-ramp (centreline is in the route).
    static constexpr uint8_t MAX_MISSION_FEATURES = 24;
    struct MissionFeature {
        int16_t x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        uint8_t kind = 0;
        bool rotatedWall = false;
        int16_t wallX = 0, wallY = 0;
        uint8_t rotationDeg = 0;
    };
    MissionFeature missionFeatures_[MAX_MISSION_FEATURES] = {};
    uint8_t missionFeatureCount_ = 0;
    // Clearance from the outer edge of the chassis to a dummy weight.
    uint16_t missionDummyClearanceMm_ = 100;
    struct MissionBottomSensorPose {
        int16_t lateralMm = 0, forwardMm = 0, angleDeg = 0;
    };
    MissionBottomSensorPose missionBottomSensors_[4] = {};
    bool missionBottomSensorGeometryValid_ = false;
    MissionBottomSensorPose missionTopSensors_[4] = {};
    bool missionTopSensorGeometryValid_ = false;
    uint32_t missionLastTopLandmarkSampleMs_[4] = {};
    uint32_t missionLastTopLandmarkCorrectionMs_[4] = {};
    uint8_t missionTopLandmarkMatches_[4] = {};
    int8_t missionTopLandmarkIds_[4] = {-1, -1, -1, -1};
    uint16_t missionTopLandmarkCorrections_ = 0;
    // A paired inner TOF sighting of the currently planned, uncollected
    // weight is a precise local landmark.  Corrections are deliberately
    // rate-limited and bounded so a single bad pair cannot teleport pose.
    uint32_t missionLastWeightLandmarkCorrectionMs_ = 0;
    uint16_t missionWeightLandmarkCorrections_ = 0;
    uint16_t missionWeightLandmarkResidualMm_ = 0;
    float weightSensorAuthority_ = 0.0f;
    uint8_t missionTopLocationConfirmations_ = 0;
    bool navigationMatrixWallRejectedByTopTof_ = false;
    int16_t missionUltrasoundLateralMm_[2] = {};
    int16_t missionUltrasoundForwardMm_[2] = {};
    bool missionUltrasoundGeometryValid_ = false;
    bool missionReturnHome_ = false;
    bool missionAutoReturnHome_ = false;
    uint8_t missionDockPhase_ = 0; // 0 approach wall, 1 turn, 2 retreat clear of wall
    uint32_t missionDockTurnStartedMs_ = 0;
    uint32_t missionDockPhaseStartedMs_ = 0;
    uint32_t missionDockNearWallSinceMs_ = 0;
    bool missionDockAligned_ = false;
    uint8_t missionUnloadPhase_ = 0; // 0 idle, 1 gate settling, then configured shake legs
    uint32_t missionUnloadPhaseStartedMs_ = 0;
    int32_t missionUnloadStartEncoder1_ = 0;
    int32_t missionUnloadStartEncoder2_ = 0;
    float missionUnloadArenaHeadingDeg_ = 0.0f;
    float missionUnloadImuHeadingDeg_ = 0.0f;
    int16_t missionUnloadHomeXmm_ = 0;
    int16_t missionUnloadHomeYmm_ = 0;
    bool missionHomeValid_ = false;
    int16_t missionHomeXmm_ = 0;
    int16_t missionHomeYmm_ = 0;
    uint8_t missionAvoidPhase_ = 0; // 0 route, 1 lateral, 2 past obstacle
    int8_t missionAvoidSide_ = 0;
    uint8_t missionAvoidAttempts_ = 0;
    bool missionWallReverseActive_ = false;
    bool missionWallReverseFailed_ = false;
    bool missionWallReverseNoProgress_ = false;
    bool missionWallRecoveryDetourRequested_ = false;
    bool missionWallRecoveryMapTried_ = false;
    int8_t missionWallRecoveryPreferredSide_ = 0;
    uint32_t missionWallReverseStartedMs_ = 0;
    uint8_t missionWallReverseWaypoint_ = 255;
    uint8_t missionWallReverseRepeatCount_ = 0;
    uint32_t missionWallReverseLastStartMs_ = 0;
    int32_t missionWallReverseEncoder1_ = 0;
    int32_t missionWallReverseEncoder2_ = 0;
    uint32_t missionForwardProgressStartedMs_ = 0;
    int32_t missionForwardProgressEncoder1_ = 0;
    int32_t missionForwardProgressEncoder2_ = 0;
    uint16_t missionForwardProgressFrontMm_ = 0xFFFF;
    uint32_t missionForwardProgressImuEventCount_ = 0;
    float missionForwardProgressHeadingDeg_ = 0.0f;
    bool missionForwardStalled_ = false;
    bool missionEncoderImuSkid_ = false;
    uint32_t missionTurnProgressStartedMs_ = 0;
    float missionTurnProgressHeadingDeg_ = 0.0f;
    float missionAvoidXmm_ = 0.0f;
    float missionAvoidYmm_ = 0.0f;
    float missionAvoidForwardXmm_ = 0.0f;
    float missionAvoidForwardYmm_ = 0.0f;
    uint32_t missionLastLandmarkEcho_[2] = {};
    uint32_t missionLastLandmarkCorrectionMs_[2] = {};
    uint8_t missionLandmarkMatches_[2] = {};
    int8_t missionLandmarkIds_[2] = {-1, -1};
    uint16_t missionLandmarkCorrections_ = 0;
    int8_t missionLastLandmarkId_ = -1;

    // Live low-object/weight visibility from matched bottom/top point-TOF pairs.
    // Four sectors run far-left, mid-left, mid-right, far-right. Evidence
    // needs fresh polls and a 150 mm bottom/top height gap; confirmation count
    // increases toward the outer edge of the planned 500 mm weight region.
    uint8_t weightEvidence_[4] = {};
    uint32_t weightLastEvidenceMs_[4] = {};
    uint32_t weightLastQualifiedMs_[4] = {};
    uint8_t weightEvidenceWaypoint_[4] = {255, 255, 255, 255};
    uint8_t weightValidMask_ = 0;
    uint8_t weightGapMask_ = 0;
    uint8_t weightFreshMask_ = 0;
    uint8_t weightTargetMask_ = 0;
    float weightTargetConfidence_ = 0.0f;
    uint8_t weightMapWallMask_ = 0;
    uint8_t weightSectorMask_ = 0;
    uint16_t weightNearestMm_ = 0;
    int8_t weightDirection_ = 0;
    float weightBearingHitXmm_ = 0.0f;
    float weightBearingHitYmm_ = 0.0f;
    float weightBearingErrorMm_ = 0.0f;
    uint32_t weightBearingSeenMs_ = 0;
    uint8_t weightBearingWaypoint_ = 255;
    uint8_t weightBearingSector_ = 255;
    uint16_t weightInnerLeftMm_ = 0xFFFF;
    uint16_t weightInnerRightMm_ = 0xFFFF;
    bool weightBearingSteeringActive_ = false;
    bool missionFrontTargetVisible_ = false;
    bool missionTargetCommitActive_ = false;
    uint16_t missionFrontTargetRangeErrorMm_ = 0;
    uint16_t missionFrontTargetMappedWallMm_ = 0;

    bool navigationMotionConsistent_ = true;

    static void dispatch(JsonDocument& message, void* context);
    void handleMessage(JsonDocument& message);
    void handleCommand(JsonDocument& message);
    void handleParameter(JsonDocument& message);
    bool loadLocalMission();
    void requestLocalReturnHome();
    void finishLocalCompetition();
    void sendDefinitions();
    void updateDefinitionBroadcast(uint32_t now);
    bool bluetoothPeerAlive(uint32_t now) const;
    void sendState();
    void sendTelemetry();
    void sendNextTelemetryChunk(uint32_t now);
    void initialiseTofSensors();
    void initialiseDigitalInputs();
    void initialiseUltrasoundSensors();
    void updateUltrasoundSensors();
    void updateMotionAndRangeSensors();
    void updateNavigation();
    void updateMissionPose(uint32_t now);
    float expectedMappedSurfaceRange(float x, float y, float ux, float uy,
                                     int8_t* surfaceId = nullptr) const;
    void stopNavigation(const char* reason = nullptr);
    void readTofSensors();
    void recoverTofSensorsIfNeeded();
    void updateWeightDetection();
    void updateTof8x8();
    void sendTof8x8Frame();
    void updateAutomaticServoRead();
    void updateArmSorting(uint32_t now);
    void updateDrumJamRecovery(uint32_t now);
    void cancelDrumJamRecovery();
    void setDcMotor203Speed(int16_t channelAPercent, int16_t channelBPercent);
    void setSortingBumpers(bool enabled);
    void setSortingGate(bool positive, bool force = false);
    void disarmArmSorting(bool restoreOutputs);
    void updateTrackingState(float absoluteAngle);
    JsonObject findParameter(const char* name);
    JsonObject findCommand(const char* name);
};
