#include "loco_sim/serialcan.h"

#include <cstddef>

namespace loco_sim {
namespace {

constexpr uint8_t kStart = 0xFF;
constexpr int kStartBytes = 2;
const char kTrailer[] = "LOCO\r\n";

uint8_t xor_sum(const uint8_t* data, size_t n) {
    uint8_t sum = 0;
    for (size_t i = 0; i < n; ++i) {
        sum ^= data[i];
    }
    return sum;
}

}  // namespace

std::vector<uint8_t> encode_serialcan(const CanFrame& frame) {
    std::vector<uint8_t> out;
    out.reserve(18 + 6);
    out.push_back(kStart);
    out.push_back(kStart);
    out.push_back(frame.size);
    out.push_back(static_cast<uint8_t>((frame.id >> 24) & 0xFFu));
    out.push_back(static_cast<uint8_t>((frame.id >> 16) & 0xFFu));
    out.push_back(static_cast<uint8_t>((frame.id >> 8) & 0xFFu));
    out.push_back(static_cast<uint8_t>(frame.id & 0xFFu));
    out.push_back(static_cast<uint8_t>((frame.uuid >> 8) & 0xFFu));
    out.push_back(static_cast<uint8_t>(frame.uuid & 0xFFu));
    for (uint8_t i = 0; i < frame.size && i < 8; ++i) {
        out.push_back(frame.data[i]);
    }
    out.push_back(xor_sum(out.data(), out.size()));
    for (const char* p = kTrailer; *p != '\0'; ++p) {
        out.push_back(static_cast<uint8_t>(*p));
    }
    return out;
}

bool decode_serialcan(const uint8_t* bytes, size_t length, CanFrame& out, size_t* consumed) {
    if (bytes == nullptr || length < 9 + 1 + 6) {
        return false;
    }

    size_t start = 0;
    while (start + 1 < length && !(bytes[start] == kStart && bytes[start + 1] == kStart)) {
        ++start;
    }
    if (start + 1 >= length || bytes[start] != kStart || bytes[start + 1] != kStart) {
        return false;
    }

    const uint8_t size = bytes[start + 2];
    if (size > 8) {
        return false;
    }

    const size_t header = 2 + 1 + 4 + 2;  // start, size, id, uuid
    const size_t checksum_index = start + header + size;
    const size_t trailer_index = checksum_index + 1;
    const size_t total = trailer_index + 6;
    if (length < total) {
        return false;
    }

    if (xor_sum(bytes + start, header + size) != bytes[checksum_index]) {
        return false;
    }
    if (bytes[trailer_index] != 'L' || bytes[trailer_index + 1] != 'O' ||
        bytes[trailer_index + 2] != 'C' || bytes[trailer_index + 3] != 'O' ||
        bytes[trailer_index + 4] != '\r' || bytes[trailer_index + 5] != '\n') {
        return false;
    }

    out = CanFrame{};
    out.size = size;
    out.id = (static_cast<uint32_t>(bytes[start + 3]) << 24) |
             (static_cast<uint32_t>(bytes[start + 4]) << 16) |
             (static_cast<uint32_t>(bytes[start + 5]) << 8) | bytes[start + 6];
    out.uuid = (static_cast<uint32_t>(bytes[start + 7]) << 8) | bytes[start + 8];
    for (uint8_t i = 0; i < size; ++i) {
        out.data[i] = bytes[start + 9 + i];
    }
    if (consumed != nullptr) {
        *consumed = total;
    }
    return true;
}

void SerialCanParser::push(uint8_t byte) {
    buf_.push_back(byte);
}

void SerialCanParser::push(const uint8_t* bytes, size_t n) {
    buf_.insert(buf_.end(), bytes, bytes + n);
}

bool SerialCanParser::pop(CanFrame& out) {
    while (!buf_.empty()) {
        size_t consumed = 0;
        if (decode_serialcan(buf_.data(), buf_.size(), out, &consumed)) {
            buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(consumed));
            return true;
        }
        // Drop a leading byte if we have a lot of data and cannot parse yet,
        // but keep a short buffer for a frame still arriving.
        if (buf_.size() > 32 && !(buf_.size() >= 2 && buf_[0] == kStart && buf_[1] == kStart)) {
            buf_.erase(buf_.begin());
            continue;
        }
        if (buf_.size() > 64) {
            buf_.erase(buf_.begin());
            continue;
        }
        return false;
    }
    return false;
}

void SerialCanParser::reset() {
    buf_.clear();
}

}  // namespace loco_sim
