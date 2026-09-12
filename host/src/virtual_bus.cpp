#include "loco_sim/virtual_bus.h"

namespace loco_sim {

bool filter_match(uint32_t id, uint16_t mask, uint16_t filter) {
    return (static_cast<uint16_t>(id) & mask) == filter;
}

void Node::add_filter(uint16_t mask, uint16_t value) {
    filters_.push_back(Filter{mask, value});
}

bool Node::accepts(uint32_t id) const {
    if (filters_.empty()) {
        return true;
    }
    for (const Filter& filter : filters_) {
        if (filter_match(id, filter.mask, filter.value)) {
            return true;
        }
    }
    return false;
}

void Node::send(CanFrame frame) {
    if (bus_ != nullptr) {
        bus_->send(this, frame);
    }
}

VirtualBus::VirtualBus(SimClock& clock) : clock_(clock) {}

void VirtualBus::attach(Node& node) {
    node.attach(this);
    nodes_.push_back(&node);
}

void VirtualBus::send(Node* source, CanFrame frame) {
    frame.timestamp_ms = clock_.now_ms();
    if (source != nullptr && frame.uuid == 0) {
        frame.uuid = source->uuid();
    }
    log_.push_back(frame);
    for (Node* node : nodes_) {
        if (node == source) {
            continue;
        }
        if (node->accepts(frame.id)) {
            node->on_rx(frame);
        }
    }
}

void VirtualBus::tick_all() {
    const uint64_t now = clock_.now_ms();
    for (Node* node : nodes_) {
        node->tick(now);
    }
}

void VirtualBus::advance_and_tick(uint64_t ms) {
    // Step in 1 ms increments so interval checks in the modules fire at the
    // same cadence they would on hardware timers.
    for (uint64_t i = 0; i < ms; ++i) {
        clock_.advance(1);
        tick_all();
    }
}

std::vector<CanFrame> VirtualBus::frames_with_id(uint32_t id) const {
    std::vector<CanFrame> out;
    for (const CanFrame& frame : log_) {
        if (frame.id == id) {
            out.push_back(frame);
        }
    }
    return out;
}

bool VirtualBus::saw_id(uint32_t id) const {
    return !frames_with_id(id).empty();
}

bool VirtualBus::saw_id_from(uint32_t id, uint32_t uuid) const {
    for (const CanFrame& frame : log_) {
        if (frame.id == id && frame.uuid == uuid) {
            return true;
        }
    }
    return false;
}

}  // namespace loco_sim
