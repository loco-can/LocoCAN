#pragma once

#include <cstdint>

namespace loco_sim {

// Deterministic millisecond clock so heartbeat and drive intervals can
// be tested without sleeping or Arduino hardware timers.
class SimClock {
public:
    uint64_t now_ms() const { return now_ms_; }
    void advance(uint64_t ms) { now_ms_ += ms; }
    void reset() { now_ms_ = 0; }

private:
    uint64_t now_ms_ = 0;
};

}  // namespace loco_sim
