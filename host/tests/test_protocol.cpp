#include "test_support.h"

#include "can_protocol.h"
#include "loco_sim/can_frame.h"
#include "loco_sim/protocol.h"

TEST(extended_id_packing_matches_firmware) {
    const uint32_t ext = loco_sim::pack_extended_id(CAN_ID_DRIVE, 0x12345);
    CHECK_EQ(ext, (CAN_ID_DRIVE << 18) | 0x12345);

    uint32_t id = 0;
    uint32_t uuid = 0;
    loco_sim::unpack_extended_id(ext, id, uuid);
    CHECK_EQ(id, static_cast<uint32_t>(CAN_ID_DRIVE));
    CHECK_EQ(uuid, 0x12345u);

    const uint8_t payload[] = {0x01, 0x02};
    loco_sim::CanFrame frame = loco_sim::CanFrame::from_extended(ext, payload, 2);
    CHECK_EQ(frame.id, static_cast<uint32_t>(CAN_ID_DRIVE));
    CHECK_EQ(frame.uuid, 0x12345u);
    CHECK_EQ(frame.size, 2);
    CHECK_EQ(frame.data[1], 0x02);
}

TEST(uuid_is_masked_to_18_bits) {
    const uint32_t ext = loco_sim::pack_extended_id(0x400, 0xFFFFFFFFu);
    uint32_t id = 0;
    uint32_t uuid = 0;
    loco_sim::unpack_extended_id(ext, id, uuid);
    CHECK_EQ(uuid, 0x3FFFFu);
}

TEST(drive_command_roundtrip) {
    loco_sim::DriveCommand cmd;
    cmd.mains = true;
    cmd.drive = true;
    cmd.dir = true;
    cmd.up = true;
    cmd.throttle = 0x2AB;
    cmd.power = 0x3F0;
    cmd.brake = 0x015;
    cmd.multi_traction = true;
    cmd.master_uuid = 0xABCD;

    loco_sim::CanFrame frame = cmd.to_frame(0x1111);
    CHECK_EQ(frame.id, static_cast<uint32_t>(CAN_ID_DRIVE));
    CHECK_EQ(frame.size, 8);
    CHECK(loco_sim::flag_set(frame.data[0], CONTROL_MAINS_FLAG));
    CHECK(loco_sim::flag_set(frame.data[0], CONTROL_DRIVE_FLAG));
    CHECK(loco_sim::flag_set(frame.data[0], CONTROL_DIR_FLAG));
    CHECK(loco_sim::flag_set(frame.data[0], CONTROL_UP_FLAG));

    loco_sim::DriveCommand decoded = loco_sim::DriveCommand::from_frame(frame);
    CHECK_EQ(decoded.mains, true);
    CHECK_EQ(decoded.drive, true);
    CHECK_EQ(decoded.dir, true);
    CHECK_EQ(decoded.up, true);
    CHECK_EQ(decoded.throttle, 0x2AB);
    CHECK_EQ(decoded.power, 0x3F0);
    CHECK_EQ(decoded.brake, 0x015);
    CHECK_EQ(decoded.multi_traction, true);
    CHECK_EQ(decoded.master_uuid, 0xABCD);
}

TEST(vehicle_status_roundtrip) {
    loco_sim::VehicleStatus st;
    st.ready = true;
    st.moving = true;
    st.mains = true;
    st.drive = true;
    st.uuid = 0xBEEF;

    loco_sim::CanFrame frame = st.to_frame(0xBEEF);
    CHECK_EQ(frame.id, static_cast<uint32_t>(CAN_ID_VEHICLE_STATUS));
    CHECK_EQ(frame.size, 3);

    loco_sim::VehicleStatus decoded = loco_sim::VehicleStatus::from_frame(frame);
    CHECK_EQ(decoded.ready, true);
    CHECK_EQ(decoded.moving, true);
    CHECK_EQ(decoded.mains, true);
    CHECK_EQ(decoded.uuid, 0xBEEF);
}

TEST(light_and_signal_bits) {
    loco_sim::LightCommand lights;
    lights.main = true;
    lights.cab = true;
    lights.posit = true;
    CHECK(loco_sim::flag_set(lights.to_byte(), LIGHT_MAIN));
    CHECK(loco_sim::flag_set(lights.to_byte(), LIGHT_CAB));
    CHECK(loco_sim::flag_set(lights.to_byte(), LIGHT_POSIT));

    loco_sim::LightCommand decoded = loco_sim::LightCommand::from_frame(lights.to_frame(1));
    CHECK_EQ(decoded.main, true);
    CHECK_EQ(decoded.cab, true);
    CHECK_EQ(decoded.posit, true);

    loco_sim::SignalCommand signal;
    signal.low = true;
    CHECK(loco_sim::flag_set(signal.to_byte(), SIGNAL_LOW));
    CHECK_EQ(loco_sim::SignalCommand::from_frame(signal.to_frame(1)).low, true);
}

TEST(loco_setup_roundtrip) {
    loco_sim::LocoSetup setup;
    setup.status = true;
    setup.disable = true;
    setup.target_uuid = 0x1234;
    loco_sim::LocoSetup decoded = loco_sim::LocoSetup::from_frame(setup.to_frame(0x99));
    CHECK_EQ(decoded.status, true);
    CHECK_EQ(decoded.disable, true);
    CHECK_EQ(decoded.target_uuid, 0x1234);
}

TEST(telemetry_u16_roundtrip) {
    loco_sim::CanFrame frame = loco_sim::make_u16_telemetry(CAN_ID_SPEED, 0x10, 1234);
    CHECK_EQ(frame.id, static_cast<uint32_t>(CAN_ID_SPEED));
    CHECK_EQ(loco_sim::read_u16_telemetry(frame), 1234);
}
