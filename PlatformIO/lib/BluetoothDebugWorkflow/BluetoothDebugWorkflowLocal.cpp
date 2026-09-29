#include "BluetoothDebugWorkflow.h"

#include <config.h>
#include <local_mission.generated.h>

void BluetoothDebugWorkflow::updateLocalCompetition()
{
    if (!LocalMissionConfig::ENABLED)
        return;

    // Count each confirmed inductive event independently of the navigation
    // waypoint attribution. The sorting subsystem already debounces a high
    // detection before incrementing this serial; consume each event once.
    if (localRunActive_)
    {
        while (localCompetitionSortingSerial_ != sortingConfirmedSerial_)
        {
            ++localCompetitionSortingSerial_;
            if (localCompetition_.noteConfirmedWeight())
                link_.log("INFO", "Five induction-confirmed weights reached; returning home");
        }
    }
    else
    {
        localCompetitionSortingSerial_ = sortingConfirmedSerial_;
    }

    const uint32_t now = millis();
    const LocalCompetition::Event event = localCompetition_.update(
        aioExpanderReady_, aio9ButtonRawHigh_, now);
    if (event == LocalCompetition::Event::Start)
    {
        snprintf(localCompetitionStatus_, sizeof(localCompetitionStatus_), "LOADING");
        if (loadLocalMission())
        {
            localRunActive_ = true;
            localMissionReturning_ = false;
            snprintf(localCompetitionStatus_, sizeof(localCompetitionStatus_), "RUNNING");
            link_.log("INFO", "Local mission started by AIO9 rising edge");
        }
        else
        {
            localCompetition_.cancelStart();
            snprintf(localCompetitionStatus_, sizeof(localCompetitionStatus_), "CONFIG_ERROR");
            link_.log("ERROR", "Local mission did not start; robot remains stopped");
        }
    }
    else if (event == LocalCompetition::Event::ReturnHome)
    {
        requestLocalReturnHome();
    }
}

bool BluetoothDebugWorkflow::loadLocalMission()
{
    if (!LocalMissionConfig::ENABLED)
        return false;

    JsonDocument localConfig;
    if (deserializeJson(localConfig, LocalMissionConfig::JSON) ||
        !localConfig["map"].is<JsonObject>() ||
        !localConfig["mission"].is<JsonObject>() ||
        !localConfig["parameters"].is<JsonArray>())
    {
        link_.log("ERROR", "Embedded local mission JSON is invalid or incomplete");
        return false;
    }

    debugMode_ = true;
    stopped_ = true;
    JsonDocument clearPlan;
    clearPlan["command"] = "mission_plan_clear";
    handleCommand(clearPlan);

    JsonDocument mapCommand;
    mapCommand.set(localConfig["map"].as<JsonVariantConst>());
    mapCommand["command"] = "mission_map_set";
    handleCommand(mapCommand);

    JsonDocument planCommand;
    planCommand.set(localConfig["mission"].as<JsonVariantConst>());
    planCommand["command"] = "mission_plan_set";
    handleCommand(planCommand);

    for (JsonVariantConst item : localConfig["parameters"].as<JsonArrayConst>())
    {
        if (!item["name"].is<const char*>() || item["value"].isNull())
            continue;
        JsonDocument parameterCommand;
        parameterCommand["name"] = item["name"].as<const char*>();
        if (item["value"].is<int>())
            parameterCommand["value"] = item["value"].as<int>();
        else if (item["value"].is<float>())
            parameterCommand["value"] = item["value"].as<float>();
        else
            continue;
        handleParameter(parameterCommand);
    }

    if (!missionPlanValid_ || !missionHomeValid_ || missionTargetCount_ == 0)
    {
        link_.log("ERROR", "Local mission requires a valid route, home, and real weight target");
        return false;
    }

    auto runCommand = [this](const char* name, bool hasEnabled, bool enabled) {
        JsonDocument command;
        command["command"] = name;
        if (hasEnabled)
            command["enabled"] = enabled;
        handleCommand(command);
    };

    runCommand("run", false, false);
    runCommand("arm_sorting", true, true);
    if (!armSortingEnabled_)
    {
        JsonDocument stop;
        stop["command"] = "stop";
        handleCommand(stop);
        link_.log("ERROR", "Local mission needs arm sorting to initialise successfully");
        return false;
    }

    JsonDocument drumCommand;
    drumCommand["command"] = "set_dc_motor_203_speed";
    drumCommand["speed_percent"] = 100;
    handleCommand(drumCommand);

    runCommand("autonomous_navigation", true, true);
    if (!navigationActive_)
    {
        JsonDocument stop;
        stop["command"] = "stop";
        handleCommand(stop);
        link_.log("ERROR", "Local mission route could not enter navigation mode");
        return false;
    }
    return true;
}

void BluetoothDebugWorkflow::requestLocalReturnHome()
{
    if (!localRunActive_ || localMissionReturning_)
        return;
    if (!missionPlanValid_ || !missionHomeValid_ || missionTargetCount_ == 0)
    {
        link_.log("ERROR", "Local round timer expired but no valid home route is loaded");
        return;
    }

    // Cancel any active weight approach/search and hand the existing mission
    // tracker directly to its normal map-aware home-and-unload transition.
    missionTargetsVisited_ = missionTargetCount_;
    missionLaunchPhase_ = 0;
    missionLaunchStartedMs_ = 0;
    missionWeightVectorActive_ = false;
    missionWeightVectorCompletedWaypoint_ = 255;
    missionInnerWeightLocked_ = false;
    missionInnerAlignStartedMs_ = 0;
    missionInnerCentreStableSinceMs_ = 0;
    missionTargetAligned_ = false;
    missionTargetAlignSinceMs_ = 0;
    missionVerifyPhase_ = 0;
    missionVerifyWaypoint_ = 255;
    missionVerifyRetries_ = 0;
    missionSearchWaypoint_ = 255;
    missionSearchStep_ = 0;
    missionPendingPickupWaypoint_ = 255;
    missionPendingPickupSinceMs_ = 0;
    missionPendingPickupWasIncomplete_ = false;
    missionPostPickupStraightUntilMs_ = 0;
    missionOuterCueActive_ = false;
    missionOuterReverseActive_ = false;
    missionAvoidPhase_ = 0;
    missionWallReverseActive_ = false;
    missionReturnHome_ = false;
    missionAutoReturnHome_ = false;
    missionUnloadPhase_ = 0;
    missionDockTurnStartedMs_ = 0;
    missionDockAligned_ = false;
    missionReroutePoseValid_ = false;
    missionBlockedSinceMs_ = 0;
    missionFrontBlockedLatched_ = false;
    missionFrontClearSinceMs_ = 0;
    missionWallReverseActive_ = false;
    missionWallReverseFailed_ = false;
    missionWallReverseNoProgress_ = false;
    missionWallRecoveryDetourRequested_ = false;
    missionForwardStalled_ = false;
    missionPointTurnActive_ = false;
    missionTurnFinishing_ = false;
    missionTurnDirection_ = 0;

    // A transient navigation stop must not cancel the hard round deadline.
    // Resume the same mission pose and enter the normal home planner rather
    // than replaying the route from its start or restarting weight collection.
    debugMode_ = true;
    stopped_ = false;
    navigationActive_ = true;
    navigationStrategy_ = 3;
    navigationState_ = NAV_MISSION_TRACK;
    navigationMotionStartedMs_ = millis();
    navigationHeadingReferenceDeg_ = imu_.headingDeg();
    navigationTargetHeadingDeg_ = navigationHeadingReferenceDeg_;
    if (!missionPoseInitialised_ && lastImuSampleValid_)
    {
        missionPoseXmm_ = missionStartXmm_;
        missionPoseYmm_ = missionStartYmm_;
        missionArenaHeadingDeg_ = missionStartArenaHeadingDeg_;
        missionStartImuHeadingDeg_ = imu_.headingDeg();
        missionLastEncoder1_ = encoders_.firstCount();
        missionLastEncoder2_ = encoders_.secondCount();
        missionPoseInitialised_ = true;
    }
    localMissionReturning_ = true;
    localCompetition_.setReturningHome();
    snprintf(localCompetitionStatus_, sizeof(localCompetitionStatus_), "RETURNING_HOME");
    if (localCompetition_.confirmedWeights() >= LocalCompetition::CONFIRMED_WEIGHT_LIMIT)
        link_.log("WARNING", "Five induction-confirmed weights reached; skipping remaining targets and returning home to unload");
    else
        link_.log("WARNING", "90-second local round limit reached; all remaining targets skipped, returning home to unload");
}

void BluetoothDebugWorkflow::finishLocalCompetition()
{
    if (!localRunActive_)
        return;

    // Reuse the existing stop action so motor neutralisation and the protected
    // jam-recovery shutdown path remain exactly the same as manual STOP.
    JsonDocument stop;
    stop["command"] = "stop";
    handleCommand(stop);
    hx12kB_.setAngle(50.0f);
    missionReturnHome_ = false;
    missionAutoReturnHome_ = false;
    localRunActive_ = false;
    localMissionReturning_ = false;
    localCompetition_.markComplete();
    snprintf(localCompetitionStatus_, sizeof(localCompetitionStatus_), "COMPLETE");
    link_.log("INFO", "Local round complete; home unload finished and motors stopped");
}
