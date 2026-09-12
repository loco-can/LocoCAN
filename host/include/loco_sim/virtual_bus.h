#pragma once

#include "loco_sim/can_frame.h"
#include "loco_sim/clock.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace loco_sim {

class VirtualBus;

struct Filter {
    uint16_t mask = 0;
    uint16_t value = 0;
};

// Firmware filter rule from CAN_COM::register_filter:
//   (message.id & mask) == filter
bool filter_match(uint32_t id, uint16_t mask, uint16_t filter);

class Node {
public:
    virtual ~Node() = default;

    virtual std::string name() const = 0;
    virtual uint32_t uuid() const = 0;
    virtual void on_rx(const CanFrame& frame) = 0;
    virtual void tick(uint64_t now_ms) = 0;

    void attach(VirtualBus* bus) { bus_ = bus; }
    VirtualBus* bus() const { return bus_; }

    void add_filter(uint16_t mask, uint16_t value);
    bool accepts(uint32_t id) const;
    const std::vector<Filter>& filters() const { return filters_; }

protected:
    void send(CanFrame frame);

private:
    VirtualBus* bus_ = nullptr;
    std::vector<Filter> filters_;
};

class VirtualBus {
public:
    explicit VirtualBus(SimClock& clock);

    void attach(Node& node);
    void send(Node* source, CanFrame frame);
    void tick_all();
    void advance_and_tick(uint64_t ms);

    const std::vector<CanFrame>& log() const { return log_; }
    void clear_log() { log_.clear(); }

    std::vector<CanFrame> frames_with_id(uint32_t id) const;
    bool saw_id(uint32_t id) const;
    bool saw_id_from(uint32_t id, uint32_t uuid) const;

    SimClock& clock() { return clock_; }
    const SimClock& clock() const { return clock_; }

private:
    SimClock& clock_;
    std::vector<Node*> nodes_;
    std::vector<CanFrame> log_;
};

}  // namespace loco_sim
