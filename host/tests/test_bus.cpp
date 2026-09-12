#include "test_support.h"

#include "can_protocol.h"
#include "loco_sim/virtual_bus.h"

#include <string>

namespace {

class CaptureNode : public loco_sim::Node {
public:
    CaptureNode(uint32_t uuid, std::string name) : uuid_(uuid), name_(std::move(name)) {}
    std::string name() const override { return name_; }
    uint32_t uuid() const override { return uuid_; }
    void on_rx(const loco_sim::CanFrame& frame) override { rx.push_back(frame); }
    void tick(uint64_t) override {}
    std::vector<loco_sim::CanFrame> rx;

private:
    uint32_t uuid_;
    std::string name_;
};

}  // namespace

TEST(bus_delivers_to_other_nodes_not_sender) {
    loco_sim::SimClock clock;
    loco_sim::VirtualBus bus(clock);
    CaptureNode a(1, "a");
    CaptureNode b(2, "b");
    bus.attach(a);
    bus.attach(b);

    loco_sim::CanFrame frame;
    frame.id = CAN_ID_DRIVE;
    frame.size = 1;
    frame.data[0] = 0x01;
    bus.send(&a, frame);

    CHECK_EQ(a.rx.size(), 0u);
    CHECK_EQ(b.rx.size(), 1u);
    CHECK_EQ(b.rx[0].uuid, 1u);
    CHECK_EQ(b.rx[0].id, static_cast<uint32_t>(CAN_ID_DRIVE));
    CHECK_EQ(bus.log().size(), 1u);
}

TEST(bus_applies_firmware_filter_rule) {
    loco_sim::SimClock clock;
    loco_sim::VirtualBus bus(clock);
    CaptureNode listener(3, "listener");
    listener.add_filter(CAN_ID_MASK, CAN_ID_DRIVE);
    bus.attach(listener);

    loco_sim::CanFrame drive;
    drive.id = CAN_ID_DRIVE;
    bus.send(nullptr, drive);

    loco_sim::CanFrame light;
    light.id = CAN_ID_LIGHT;
    bus.send(nullptr, light);

    CHECK_EQ(listener.rx.size(), 1u);
    CHECK_EQ(listener.rx[0].id, static_cast<uint32_t>(CAN_ID_DRIVE));
}

TEST(clock_advances_deterministically) {
    loco_sim::SimClock clock;
    loco_sim::VirtualBus bus(clock);
    CHECK_EQ(clock.now_ms(), 0u);
    bus.advance_and_tick(15);
    CHECK_EQ(clock.now_ms(), 15u);
}
