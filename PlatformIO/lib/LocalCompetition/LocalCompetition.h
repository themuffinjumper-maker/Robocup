#pragma once

#include <Arduino.h>

// Minimal local-run supervisor. It owns only the AIO9 edge/debounce and the
// round clock; navigation, sorting, motor control, and homing remain in the
// existing BluetoothDebugWorkflow implementation.
class LocalCompetition
{
public:
    enum class Event : uint8_t
    {
        None,
        Start,
        ReturnHome
    };

    void begin(bool initialLevelHigh, uint32_t now)
    {
        rawLevelHigh_ = initialLevelHigh;
        stableLevelHigh_ = initialLevelHigh;
        candidateSinceMs_ = now;
        initialized_ = true;
        active_ = false;
        returningHome_ = false;
        completed_ = false;
        startedAtMs_ = 0;
        confirmedWeights_ = 0;
    }

    Event update(bool inputAvailable, bool levelHigh, uint32_t now)
    {
        if (!initialized_)
            begin(levelHigh, now);
        if (inputAvailable)
        {
            if (levelHigh != rawLevelHigh_)
            {
                rawLevelHigh_ = levelHigh;
                candidateSinceMs_ = now;
            }
            if (rawLevelHigh_ != stableLevelHigh_ &&
                now - candidateSinceMs_ >= INPUT_DEBOUNCE_MS)
            {
                const bool rising = !stableLevelHigh_ && rawLevelHigh_;
                stableLevelHigh_ = rawLevelHigh_;
                if (rising && !active_ && !completed_)
                {
                    active_ = true;
                    startedAtMs_ = now;
                    return Event::Start;
                }
            }
        }

        if (active_ && !returningHome_ && now - startedAtMs_ >= ROUND_LIMIT_MS)
        {
            returningHome_ = true;
            return Event::ReturnHome;
        }
        if (active_ && !returningHome_ &&
            confirmedWeights_ >= CONFIRMED_WEIGHT_LIMIT)
        {
            returningHome_ = true;
            return Event::ReturnHome;
        }
        return Event::None;
    }

    bool noteConfirmedWeight()
    {
        if (!active_ || returningHome_ || completed_ ||
            confirmedWeights_ >= CONFIRMED_WEIGHT_LIMIT)
            return false;
        ++confirmedWeights_;
        return confirmedWeights_ >= CONFIRMED_WEIGHT_LIMIT;
    }

    void markComplete()
    {
        active_ = false;
        returningHome_ = false;
        completed_ = true;
    }

    void cancelStart()
    {
        active_ = false;
        returningHome_ = false;
        startedAtMs_ = 0;
    }

    void setReturningHome()
    {
        returningHome_ = true;
    }

    bool active() const { return active_; }
    bool returningHome() const { return returningHome_; }
    bool completed() const { return completed_; }
    uint8_t confirmedWeights() const { return confirmedWeights_; }
    uint32_t elapsedMs(uint32_t now) const
    {
        return startedAtMs_ == 0 ? 0 : now - startedAtMs_;
    }
    uint32_t remainingMs(uint32_t now) const
    {
        const uint32_t elapsed = elapsedMs(now);
        return elapsed >= ROUND_LIMIT_MS ? 0 : ROUND_LIMIT_MS - elapsed;
    }

    static constexpr uint32_t ROUND_LIMIT_MS = 90000U;
    static constexpr uint32_t INPUT_DEBOUNCE_MS = 30U;
    static constexpr uint8_t CONFIRMED_WEIGHT_LIMIT = 3U;

private:
    bool initialized_ = false;
    bool rawLevelHigh_ = false;
    bool stableLevelHigh_ = false;
    bool active_ = false;
    bool returningHome_ = false;
    bool completed_ = false;
    uint8_t confirmedWeights_ = 0;
    uint32_t candidateSinceMs_ = 0;
    uint32_t startedAtMs_ = 0;
};
