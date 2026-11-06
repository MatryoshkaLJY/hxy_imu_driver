// SPDX-License-Identifier: Apache-2.0
#include <cassert>
#include <limits>
#include <vector>
#include "imu_protocol.hpp"

static std::vector<std::uint8_t> frame(std::uint64_t timestamp, float zacc = -9.80665f,
                                     std::uint16_t fields = 0x003f, std::uint8_t system = 1,
                                     std::uint8_t component = 200, std::uint8_t sensor = 0)
{
    mavlink_message_t message{};
    mavlink_msg_highres_imu_pack_chan(system, component, MAVLINK_COMM_0, &message,
        timestamp, 1, 2, zacc, 0.01f, -0.02f, 0.03f,
        0, 0, 0, 0, 0, 0, 0, fields, sensor);
    std::vector<std::uint8_t> bytes(MAVLINK_MAX_PACKET_LEN);
    bytes.resize(mavlink_msg_to_send_buffer(bytes.data(), &message));
    return bytes;
}

int main()
{
    hxy_imu::ImuDecoder decoder(1, 200, 0);
    std::vector<hxy_imu::Sample> samples;
    auto receive = [&](hxy_imu::Sample sample) { samples.push_back(sample); };
    // Bytewise fragmentation, coalescing, and wire sequence wraparound.
    for (unsigned int i = 0; i < 260; ++i) {
        const auto bytes = frame(1000000 + i * 10000);
        for (const auto byte : bytes) decoder.feed(&byte, 1, receive);
    }
    assert(samples.size() == 260);
    assert(samples.back().time_usec == 3590000);
    assert(std::abs(samples.back().acceleration[2] + 9.80665) < 1e-5);
    auto rotated = samples.back();
    hxy_imu::frd_to_flu(rotated);
    assert(rotated.acceleration[0] == 1 && rotated.acceleration[1] == -2);
    assert(rotated.acceleration[2] > 9.8);
    assert(rotated.angular_velocity[0] > 0 && rotated.angular_velocity[1] > 0 &&
           rotated.angular_velocity[2] < 0);

    auto corrupt = frame(3600000);
    corrupt.back() ^= 0x40;
    const auto good = frame(3610000);
    corrupt.insert(corrupt.end(), good.begin(), good.end());
    decoder.feed(corrupt.data(), corrupt.size(), receive);
    assert(samples.size() == 261 && decoder.stats.crc_errors == 1);

    for (const auto &bytes : {
            frame(3620000, std::numeric_limits<float>::quiet_NaN()),
            frame(3620000, std::numeric_limits<float>::infinity()),
            frame(3620000, -9.8f, 0x001f),
            frame(3620000, -9.8f, 0x003f, 2),
            frame(3620000, -9.8f, 0x003f, 1, 201),
            frame(3620000, -9.8f, 0x003f, 1, 200, 1),
            frame(3610000)}) {
        decoder.feed(bytes.data(), bytes.size(), receive);
    }
    assert(samples.size() == 261);
    assert(decoder.stats.field_errors == 3 && decoder.stats.filtered == 3);
    assert(decoder.stats.duplicates == 1);
    const auto reboot = frame(10000);
    decoder.feed(reboot.data(), reboot.size(), receive);
    assert(samples.size() == 262 && decoder.stats.timestamp_resets == 1);

    // A partial frame must not cross a reconnect boundary.
    const auto next = frame(20000);
    decoder.feed(next.data(), 10, receive);
    decoder.reset_session();
    decoder.feed(next.data(), next.size(), receive);
    assert(samples.size() == 263 && decoder.stats.samples == 263);

    // Separate decoders must not overwrite each other's partial packet state.
    hxy_imu::ImuDecoder other(1, 200, 0);
    decoder.feed(next.data(), 10, receive);
    other.feed(next.data(), next.size(), receive);
    assert(other.stats.samples == 1);
    decoder.feed(next.data() + 10, next.size() - 10, receive);
    assert(decoder.stats.duplicates == 2);
    return 0;
}
