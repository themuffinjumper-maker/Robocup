#pragma once

#include <Arduino.h>

class FrontierExplorer {
public:
    static constexpr uint8_t GRID_SIZE = 64;
    static constexpr uint16_t CELL_MM = 100;
    static constexpr uint8_t MAX_PATH = 128;

    void begin(int32_t encoder1, int32_t encoder2, float headingDeg);
    void updatePose(int32_t encoder1, int32_t encoder2, float headingDeg);
    void observeRay(uint16_t distanceMm, float relativeAngleDeg, bool obstacle,
                    float sensorRightMm = 0.0f, float sensorForwardMm = 0.0f);
    bool plan(uint16_t robotWidthMm, uint16_t marginMm);
    bool hasWaypoint() const { return pathIndex_ < pathLength_; }
    bool waypointReached(float radiusMm = 80.0f);
    float waypointHeadingDeg() const;
    float waypointDistanceMm() const;
    void invalidatePath() { pathLength_ = pathIndex_ = 0; }

    float xMm() const { return xMm_; }
    float yMm() const { return yMm_; }
    uint16_t knownCells() const { return knownCells_; }
    uint16_t frontierCount() const { return frontierCount_; }
    uint16_t replanCount() const { return replanCount_; }
    uint8_t pathLength() const { return pathLength_ - pathIndex_; }
    bool complete() const { return complete_; }

private:
    int8_t grid_[GRID_SIZE * GRID_SIZE] = {};
    uint16_t queue_[GRID_SIZE * GRID_SIZE] = {};
    int16_t parent_[GRID_SIZE * GRID_SIZE] = {};
    uint16_t path_[MAX_PATH] = {};
    uint8_t pathLength_ = 0;
    uint8_t pathIndex_ = 0;
    int32_t lastEncoder1_ = 0;
    int32_t lastEncoder2_ = 0;
    float startHeadingDeg_ = 0.0f;
    float headingDeg_ = 0.0f;
    float xMm_ = 0.0f;
    float yMm_ = 0.0f;
    uint16_t knownCells_ = 0;
    uint16_t frontierCount_ = 0;
    uint16_t replanCount_ = 0;
    bool complete_ = false;

    static bool inside(int x, int y) { return x >= 0 && y >= 0 && x < GRID_SIZE && y < GRID_SIZE; }
    static uint16_t index(int x, int y) { return static_cast<uint16_t>(y * GRID_SIZE + x); }
    void worldToCell(float xMm, float yMm, int& x, int& y) const;
    void addEvidence(int x, int y, int delta);
    bool inflatedBlocked(int x, int y, uint8_t radius) const;
};
