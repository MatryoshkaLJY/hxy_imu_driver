// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <common/mavlink.h>

namespace hxy_imu {

struct Sample {
    std::uint64_t time_usec = 0;
    std::array<double, 3> acceleration{};
    std::array<double, 3> angular_velocity{};
};

inline void frd_to_flu(Sample &sample)
{
    for (std::size_t axis = 1; axis < 3; ++axis) {
        sample.acceleration[axis] *= -1;
        sample.angular_velocity[axis] *= -1;
    }
}

struct ReceiverStats {
    std::uint64_t samples = 0, crc_errors = 0, field_errors = 0;
    std::uint64_t filtered = 0, duplicates = 0, timestamp_resets = 0;
};

// Each instance owns its parser state; no shared MAVLink channel is used.
class ImuDecoder {
public:
    ImuDecoder(std::uint8_t system_id, std::uint8_t component_id, std::uint8_t sensor_id)
        : system_id_(system_id), component_id_(component_id), sensor_id_(sensor_id) {}

    template<typename Callback>
    void feed(const std::uint8_t *data, std::size_t size, Callback &&callback)
    {
        for (std::size_t i = 0; i < size; ++i) {
            mavlink_message_t message{};
            const auto result = mavlink_frame_char_buffer(
                &buffer_, &parser_, data[i], &message, nullptr);
            if (result == MAVLINK_FRAMING_BAD_CRC || result == MAVLINK_FRAMING_BAD_SIGNATURE) {
                ++stats.crc_errors;
                parser_.parse_state = MAVLINK_PARSE_STATE_IDLE;
                if (data[i] == MAVLINK_STX)
                    mavlink_frame_char_buffer(&buffer_, &parser_, data[i], nullptr, nullptr);
            } else if (result == MAVLINK_FRAMING_OK) {
                receive(message, callback);
            }
        }
    }

    void reset_session()
    {
        buffer_ = {};
        parser_ = {};
        have_timestamp_ = false;
    }

    ReceiverStats stats;

private:
    mavlink_message_t buffer_{};
    mavlink_status_t parser_{};
    std::uint8_t system_id_, component_id_, sensor_id_;
    std::uint64_t last_timestamp_ = 0;
    bool have_timestamp_ = false;

    template<typename Callback>
    void receive(const mavlink_message_t &message, Callback &callback)
    {
        if (message.msgid != MAVLINK_MSG_ID_HIGHRES_IMU) return;
        mavlink_highres_imu_t imu{};
        mavlink_msg_highres_imu_decode(&message, &imu);
        if (message.sysid != system_id_ || message.compid != component_id_ ||
            imu.id != sensor_id_) {
            ++stats.filtered;
            return;
        }
        const std::array<float, 6> values{
            imu.xacc, imu.yacc, imu.zacc, imu.xgyro, imu.ygyro, imu.zgyro};
        // This board supplies only the six updated inertial axes. Other fields
        // are placeholders, not measured magnetic/pressure/temperature data.
        if (message.magic != MAVLINK_STX || imu.fields_updated != 0x003f ||
            !std::all_of(values.begin(), values.end(), [](float v) { return std::isfinite(v); }) ||
            imu.xmag != 0 || imu.ymag != 0 || imu.zmag != 0 || imu.abs_pressure != 0 ||
            imu.diff_pressure != 0 || imu.pressure_alt != 0 || imu.temperature != 0) {
            ++stats.field_errors;
            return;
        }
        if (have_timestamp_ && imu.time_usec == last_timestamp_) {
            ++stats.duplicates;
            return;
        }
        // MCU restarts create a new boot-time epoch. ROS stamps use reception
        // time, so a boot-time regression never becomes a backwards ROS stamp.
        if (have_timestamp_ && imu.time_usec < last_timestamp_) ++stats.timestamp_resets;
        last_timestamp_ = imu.time_usec;
        have_timestamp_ = true;
        ++stats.samples;
        callback(Sample{imu.time_usec, {imu.xacc, imu.yacc, imu.zacc},
                        {imu.xgyro, imu.ygyro, imu.zgyro}});
    }
};

}  // namespace hxy_imu
