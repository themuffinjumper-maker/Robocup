#include "FrontierExplorer.h"

#include <math.h>
#include <string.h>

namespace {
constexpr float E1_MM = 0.09094f;
constexpr float E2_MM = 0.09592f;
float normalise(float value) {
    while (value >= 360.0f) value -= 360.0f;
    while (value < 0.0f) value += 360.0f;
    return value;
}
}

void FrontierExplorer::begin(int32_t encoder1, int32_t encoder2, float headingDeg) {
    memset(grid_, 0, sizeof(grid_));
    lastEncoder1_ = encoder1;
    lastEncoder2_ = encoder2;
    startHeadingDeg_ = headingDeg;
    headingDeg_ = headingDeg;
    xMm_ = yMm_ = 0.0f;
    pathLength_ = pathIndex_ = 0;
    knownCells_ = frontierCount_ = replanCount_ = 0;
    complete_ = false;
    int cx, cy;
    worldToCell(0, 0, cx, cy);
    for (int dy = -2; dy <= 2; ++dy)
        for (int dx = -2; dx <= 2; ++dx)
            if (inside(cx + dx, cy + dy)) addEvidence(cx + dx, cy + dy, -3);
}

void FrontierExplorer::updatePose(int32_t encoder1, int32_t encoder2, float headingDeg) {
    const int32_t d1 = encoder1 - lastEncoder1_;
    const int32_t d2 = encoder2 - lastEncoder2_;
    lastEncoder1_ = encoder1;
    lastEncoder2_ = encoder2;
    headingDeg_ = headingDeg;
    const float distance = ((-d1 * E1_MM) + (d2 * E2_MM)) * 0.5f;
    const float relative = (headingDeg_ - startHeadingDeg_) * DEG_TO_RAD;
    xMm_ += distance * sinf(relative);
    yMm_ += distance * cosf(relative);
}

void FrontierExplorer::worldToCell(float xMm, float yMm, int& x, int& y) const {
    x = GRID_SIZE / 2 + static_cast<int>(lroundf(xMm / CELL_MM));
    y = GRID_SIZE / 2 + static_cast<int>(lroundf(yMm / CELL_MM));
}

void FrontierExplorer::addEvidence(int x, int y, int delta) {
    if (!inside(x, y)) return;
    int8_t& value = grid_[index(x, y)];
    const bool wasKnown = value != 0;
    value = static_cast<int8_t>(constrain(static_cast<int>(value) + delta, -8, 8));
    if (!wasKnown && value != 0) ++knownCells_;
    else if (wasKnown && value == 0 && knownCells_ > 0) --knownCells_;
}

void FrontierExplorer::observeRay(uint16_t distanceMm, float relativeAngleDeg,
                                  bool obstacle, float sensorRightMm,
                                  float sensorForwardMm) {
    if (distanceMm < 30 || distanceMm > 3500) return;
    const float robotWorld = (headingDeg_ - startHeadingDeg_) * DEG_TO_RAD;
    const float originX = xMm_ + sensorRightMm * cosf(robotWorld) +
                          sensorForwardMm * sinf(robotWorld);
    const float originY = yMm_ - sensorRightMm * sinf(robotWorld) +
                          sensorForwardMm * cosf(robotWorld);
    const float world = robotWorld + relativeAngleDeg * DEG_TO_RAD;
    const float freeLimit = max(0.0f, distanceMm - CELL_MM * 0.7f);
    for (float d = 0; d <= freeLimit; d += CELL_MM * 0.5f) {
        int x, y;
        worldToCell(originX + d * sinf(world), originY + d * cosf(world), x, y);
        addEvidence(x, y, -2);
    }
    if (obstacle) {
        int x, y;
        worldToCell(originX + distanceMm * sinf(world),
                    originY + distanceMm * cosf(world), x, y);
        addEvidence(x, y, 3);
    }
}

bool FrontierExplorer::inflatedBlocked(int x, int y, uint8_t radius) const {
    for (int dy = -radius; dy <= radius; ++dy)
        for (int dx = -radius; dx <= radius; ++dx) {
            if (dx * dx + dy * dy > radius * radius) continue;
            if (!inside(x + dx, y + dy) || grid_[index(x + dx, y + dy)] >= 3) return true;
        }
    return false;
}

bool FrontierExplorer::plan(uint16_t robotWidthMm, uint16_t marginMm) {
    ++replanCount_;
    pathLength_ = pathIndex_ = 0;
    complete_ = false;
    memset(parent_, 0xFF, sizeof(parent_));
    int sx, sy;
    worldToCell(xMm_, yMm_, sx, sy);
    if (!inside(sx, sy)) return false;
    const uint8_t inflation = max(
        static_cast<uint8_t>(1),
        static_cast<uint8_t>(ceilf((robotWidthMm * 0.5f + marginMm) / CELL_MM)));
    const uint16_t start = index(sx, sy);
    uint16_t head = 0, tail = 0;
    queue_[tail++] = start;
    parent_[start] = start;
    int bestScore = -32767;
    uint16_t best = start;
    frontierCount_ = 0;
    static const int8_t DX[4] = {1, -1, 0, 0};
    static const int8_t DY[4] = {0, 0, 1, -1};
    while (head < tail) {
        const uint16_t current = queue_[head++];
        const int x = current % GRID_SIZE, y = current / GRID_SIZE;
        uint8_t unknown = 0;
        for (uint8_t n = 0; n < 4; ++n) {
            const int nx = x + DX[n], ny = y + DY[n];
            if (inside(nx, ny) && grid_[index(nx, ny)] == 0) ++unknown;
        }
        if (unknown && current != start) {
            ++frontierCount_;
            const int score = unknown * 12 - static_cast<int>(head / 8);
            if (score > bestScore) { bestScore = score; best = current; }
        }
        for (uint8_t n = 0; n < 4; ++n) {
            const int nx = x + DX[n], ny = y + DY[n];
            if (!inside(nx, ny)) continue;
            const uint16_t ni = index(nx, ny);
            if (parent_[ni] >= 0 || grid_[ni] > -1 || inflatedBlocked(nx, ny, inflation)) continue;
            parent_[ni] = current;
            queue_[tail++] = ni;
        }
    }
    if (best == start) {
        // A small initial scan can temporarily leave no reachable frontier
        // after obstacle inflation.  Rotate and collect more evidence rather
        // than declaring the whole arena searched.
        // No current frontier can also mean that obstacle inflation temporarily
        // surrounds the start or that a scan has not connected its sparse rays.
        // It is not proof that the arena is fully explored.
        complete_ = false;
        return false;
    }
    uint16_t reverse[MAX_PATH];
    uint8_t length = 0;
    for (uint16_t cell = best; cell != start && length < MAX_PATH; cell = parent_[cell])
        reverse[length++] = cell;
    for (uint8_t i = 0; i < length; ++i) path_[i] = reverse[length - 1 - i];
    pathLength_ = length;
    pathIndex_ = 0;
    return length != 0;
}

bool FrontierExplorer::waypointReached(float radiusMm) {
    if (!hasWaypoint()) return true;
    if (waypointDistanceMm() > radiusMm) return false;
    ++pathIndex_;
    return !hasWaypoint();
}

float FrontierExplorer::waypointHeadingDeg() const {
    if (!hasWaypoint()) return headingDeg_;
    const int x = path_[pathIndex_] % GRID_SIZE;
    const int y = path_[pathIndex_] / GRID_SIZE;
    const float wx = (x - GRID_SIZE / 2) * CELL_MM;
    const float wy = (y - GRID_SIZE / 2) * CELL_MM;
    return normalise(startHeadingDeg_ + atan2f(wx - xMm_, wy - yMm_) * RAD_TO_DEG);
}

float FrontierExplorer::waypointDistanceMm() const {
    if (!hasWaypoint()) return 0.0f;
    const int x = path_[pathIndex_] % GRID_SIZE;
    const int y = path_[pathIndex_] / GRID_SIZE;
    const float wx = (x - GRID_SIZE / 2) * CELL_MM;
    const float wy = (y - GRID_SIZE / 2) * CELL_MM;
    return hypotf(wx - xMm_, wy - yMm_);
}
