#include "test_support.h"

#include "can_protocol.h"
#include "loco_sim/protocol.h"
#include "loco_sim/serialcan.h"

TEST(serialcan_roundtrip_drive_frame) {
    loco_sim::DriveCommand cmd;
    cmd.mains = true;
    cmd.drive = true;
    cmd.throttle = 700;
    cmd.power = 1023;
    loco_sim::CanFrame original = cmd.to_frame(0xA001);

    std::vector<uint8_t> encoded = loco_sim::encode_serialcan(original);
    CHECK(encoded.size() >= 16);
    CHECK_EQ(encoded[0], 0xFF);
    CHECK_EQ(encoded[1], 0xFF);
    CHECK_EQ(encoded[2], 8);

    loco_sim::CanFrame decoded;
    size_t consumed = 0;
    CHECK(loco_sim::decode_serialcan(encoded.data(), encoded.size(), decoded, &consumed));
    CHECK_EQ(consumed, encoded.size());
    CHECK_EQ(decoded.id, original.id);
    CHECK_EQ(decoded.uuid, original.uuid);
    CHECK_EQ(decoded.size, original.size);
    for (uint8_t i = 0; i < original.size; ++i) {
        CHECK_EQ(decoded.data[i], original.data[i]);
    }
}

TEST(serialcan_parser_handles_chunked_bytes) {
    loco_sim::CanFrame hb = loco_sim::make_heartbeat(CAN_ID_DRIVE_HEARTBEAT, 0x22);
    std::vector<uint8_t> encoded = loco_sim::encode_serialcan(hb);

    loco_sim::SerialCanParser parser;
    for (size_t i = 0; i + 1 < encoded.size(); ++i) {
        parser.push(encoded[i]);
        loco_sim::CanFrame tmp;
        CHECK(!parser.pop(tmp));
    }
    parser.push(encoded.back());
    loco_sim::CanFrame out;
    CHECK(parser.pop(out));
    CHECK_EQ(out.id, static_cast<uint32_t>(CAN_ID_DRIVE_HEARTBEAT));
    CHECK_EQ(out.uuid, 0x22u);
    CHECK_EQ(out.size, 0);
}

TEST(serialcan_rejects_bad_checksum) {
    loco_sim::CanFrame frame = loco_sim::make_u16_telemetry(CAN_ID_SPEED, 1, 9);
    std::vector<uint8_t> encoded = loco_sim::encode_serialcan(frame);
    encoded[encoded.size() - 7] ^= 0xFF;  // flip checksum
    loco_sim::CanFrame decoded;
    CHECK(!loco_sim::decode_serialcan(encoded.data(), encoded.size(), decoded, nullptr));
}
