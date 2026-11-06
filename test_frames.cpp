// SPDX-License-Identifier: Apache-2.0
// Test source uses the same official wire encoder as the firmware, without serial I/O.
#include <cstdio>
#include <cstdint>
#include <common/mavlink.h>

int main()
{
    for (unsigned int i = 0; i < 260; ++i) {
        mavlink_message_t message{};
        std::uint8_t frame[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_highres_imu_pack_chan(1, 200, MAVLINK_COMM_0, &message,
            1000000ULL + i * 10000ULL, 0, 0, -9.80665f, 0.01f, -0.02f, 0.03f,
            0, 0, 0, 0, 0, 0, 0, 0x003f, 0);
        const auto size = mavlink_msg_to_send_buffer(frame, &message);
        if (std::fwrite(frame, 1, size, stdout) != size) return 1;
    }
    return 0;
}
