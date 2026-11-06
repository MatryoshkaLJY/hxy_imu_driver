// SPDX-License-Identifier: Apache-2.0
// Linux USB CDC receiver: standard C++17 + POSIX APIs + vendored MAVLink headers.
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <common/mavlink.h>
#include "serial_port.hpp"

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
static volatile std::sig_atomic_t interrupted = 0;
static void stop(int) { interrupted = 1; }

using hxy_imu::Port;
using hxy_imu::ports;
using hxy_imu::Serial;
using hxy_imu::system_check;

static void list_ports(const std::vector<Port> &available)
{
    if (available.empty()) std::cout << "No USB serial ports found.\n";
    for (const auto &port : available) {
        std::cout << port.path << "  " << port.vid << ':' << port.pid << "  "
                  << port.product << "  serial=" << port.serial
                  << (port.target ? "  [IMU]" : "") << '\n';
    }
}

class Receiver {
public:
    explicit Receiver(std::ostream *csv) : csv_(csv)
    {
        if (csv_) *csv_ << "time_usec,sysid,compid,seq,id,xacc,yacc,zacc,xgyro,ygyro,zgyro\n"
                        << std::setprecision(9);
    }
    void feed(const std::uint8_t *data, std::size_t length)
    {
        bytes += length;
        for (std::size_t i = 0; i < length; ++i) {
            mavlink_message_t message{};
            const auto status = mavlink_frame_char_buffer(&buffer_, &parser_, data[i],
                                                          &message, nullptr);
            if (status == MAVLINK_FRAMING_OK) {
                ++frames;
                if (message.msgid == MAVLINK_MSG_ID_HIGHRES_IMU) receive(message);
                else ++other;
            } else if (status == MAVLINK_FRAMING_BAD_CRC ||
                       status == MAVLINK_FRAMING_BAD_SIGNATURE) {
                ++crc_errors;
                parser_.parse_state = MAVLINK_PARSE_STATE_IDLE;
                // The offending byte may be the first byte of the next frame.
                if (data[i] == MAVLINK_STX)
                    mavlink_frame_char_buffer(&buffer_, &parser_, data[i], nullptr, nullptr);
            }
        }
    }
    void print(bool final, double seconds) const
    {
        const double rate = interval_sum_ ? intervals_ * 1e6 / interval_sum_ : 0;
        std::cout << std::fixed << std::setprecision(3)
                  << (final ? "SUMMARY" : "IMU") << " bytes=" << bytes
                  << " frames=" << frames << " imu=" << samples
                  << " rate_hz=" << rate << " rx_hz=" << (seconds > 0 ? samples / seconds : 0)
                  << " crc_errors=" << crc_errors << " field_errors=" << field_errors
                  << " timestamp_errors=" << timestamp_errors << " seq_gaps=" << seq_gaps
                  << " other=" << other;
        if (intervals_) {
            std::cout << " dt_us=" << interval_min_ << '/' << interval_max_
                      << " timing_9_11ms_pct=" << (100.0 * on_time_ / intervals_);
        }
        if (samples) {
            std::cout << " t_us=" << latest_.time_usec
                      << " acc_mps2=(" << latest_.xacc << ',' << latest_.yacc << ',' << latest_.zacc << ')'
                      << " gyro_rad_s=(" << latest_.xgyro << ',' << latest_.ygyro << ',' << latest_.zgyro << ')';
        }
        std::cout << std::endl;
    }
    bool ok() const { return samples > 0 && !crc_errors && !field_errors && !timestamp_errors; }
    std::uint64_t bytes = 0, frames = 0, samples = 0;
    std::uint64_t crc_errors = 0, field_errors = 0, timestamp_errors = 0, seq_gaps = 0, other = 0;
private:
    mavlink_message_t buffer_{};
    mavlink_status_t parser_{};
    mavlink_highres_imu_t latest_{};
    std::ostream *csv_;
    std::uint64_t interval_sum_ = 0, intervals_ = 0, on_time_ = 0;
    std::uint64_t interval_min_ = std::numeric_limits<std::uint64_t>::max(), interval_max_ = 0;
    std::uint8_t last_sequence_ = 0;
    std::uint32_t sysid_ = 0;
    std::uint8_t compid_ = 0, sensor_id_ = 0;

    void receive(const mavlink_message_t &message)
    {
        mavlink_highres_imu_t imu{};
        mavlink_msg_highres_imu_decode(&message, &imu);
        const std::array<float, 6> axes{imu.xacc, imu.yacc, imu.zacc, imu.xgyro, imu.ygyro, imu.zgyro};
        if (message.magic != MAVLINK_STX || imu.fields_updated != 0x003f ||
            !std::all_of(axes.begin(), axes.end(), [](float v) { return std::isfinite(v); }) ||
            imu.xmag != 0 || imu.ymag != 0 || imu.zmag != 0 || imu.abs_pressure != 0 ||
            imu.diff_pressure != 0 || imu.pressure_alt != 0 || imu.temperature != 0) {
            ++field_errors;
            return;
        }
        if (!samples) {
            sysid_ = message.sysid;
            compid_ = message.compid;
            sensor_id_ = imu.id;
            std::cout << "Connected: MAVLink 2 HIGHRES_IMU sysid=" << sysid_
                      << " compid=" << unsigned(compid_) << " id=" << unsigned(sensor_id_) << '\n';
        } else {
            if (message.sysid != sysid_ || message.compid != compid_ || imu.id != sensor_id_) {
                ++field_errors;
                return;
            }
            seq_gaps += static_cast<std::uint8_t>(message.seq - last_sequence_ - 1);
            if (imu.time_usec <= latest_.time_usec) ++timestamp_errors;
            else {
                const auto dt = imu.time_usec - latest_.time_usec;
                interval_sum_ += dt;
                ++intervals_;
                interval_min_ = std::min(interval_min_, dt);
                interval_max_ = std::max(interval_max_, dt);
                on_time_ += dt >= 9000 && dt <= 11000;
            }
        }
        ++samples;
        latest_ = imu;
        last_sequence_ = message.seq;
        if (csv_) {
            *csv_ << imu.time_usec << ',' << message.sysid << ',' << unsigned(message.compid)
                  << ',' << unsigned(message.seq) << ',' << unsigned(imu.id) << ','
                  << imu.xacc << ',' << imu.yacc << ',' << imu.zacc << ','
                  << imu.xgyro << ',' << imu.ygyro << ',' << imu.zgyro << '\n';
            if (!*csv_) throw std::runtime_error("CSV write failed");
        }
    }
};

static void usage()
{
    std::cout << "Usage: imu_monitor [--port /dev/ttyACM1] [--duration SECONDS] [--csv samples.csv]\n"
                 "       imu_monitor --list\n"
                 "       imu_monitor --file capture.bin [--csv samples.csv]\n"
                 "Without --port, auto-selects an HXY IMU MAVLink USB device.\n"
                 "Default: run until Ctrl+C; display once per second, decode every frame.\n";
}

int main(int argc, char **argv)
{
    try {
        std::string port, input, csv_path;
        double duration = 0;
        bool list = false;
        for (int i = 1; i < argc; ++i) {
            const std::string option = argv[i];
            if (option == "--help" || option == "-h") { usage(); return 0; }
            if (option == "--list") { list = true; continue; }
            if (i + 1 == argc) throw std::runtime_error("missing value for " + option);
            const std::string value = argv[++i];
            if (option == "--port") port = value;
            else if (option == "--file") input = value;
            else if (option == "--csv") csv_path = value;
            else if (option == "--duration") {
                std::size_t used = 0;
                duration = std::stod(value, &used);
                if (used != value.size() || !std::isfinite(duration) || duration <= 0)
                    throw std::runtime_error("--duration must be a positive number");
            } else throw std::runtime_error("unknown option: " + option);
        }
        if (!port.empty() && !input.empty()) throw std::runtime_error("choose --port or --file");
        const auto available = ports();
        if (list) { list_ports(available); return 0; }
        if (input.empty() && port.empty()) {
            for (const auto &candidate : available) if (candidate.target) {
                if (!port.empty()) throw std::runtime_error("multiple IMUs found; specify --port");
                port = candidate.path;
            }
            if (port.empty()) {
                list_ports(available);
                std::cerr << "No HXY IMU CDC found. Check USB firmware IDs and module pin 27 VBUS.\n"
                             "The SEGGER J-Link port is not the board's native USB port.\n";
                return 2;
            }
        }
        if (!port.empty()) {
            std::error_code error;
            const auto resolved = fs::canonical(port, error);
            for (const auto &candidate : available)
                if (!error && resolved == candidate.path && candidate.vid == "1366")
                    throw std::runtime_error(port + " belongs to SEGGER J-Link, not the IMU USB device");
        }
        // Open input before creating CSV, so failed opens do not truncate output.
        std::ifstream file;
        if (!input.empty()) {
            if (!csv_path.empty() && fs::weakly_canonical(input) == fs::weakly_canonical(csv_path))
                throw std::runtime_error("input and CSV output must be different files");
            file.open(input, std::ios::binary);
            if (!file) throw std::runtime_error("cannot open input " + input);
        }
        std::ofstream csv;
        if (!csv_path.empty()) {
            csv.open(csv_path);
            if (!csv) throw std::runtime_error("cannot open CSV " + csv_path);
        }
        Receiver receiver(csv_path.empty() ? nullptr : &csv);
        std::array<std::uint8_t, 4096> data{};
        const auto start = Clock::now();
        auto next_report = start + std::chrono::seconds(1);
        if (!input.empty()) {
            while (file.read(reinterpret_cast<char *>(data.data()), data.size()) || file.gcount())
                receiver.feed(data.data(), static_cast<std::size_t>(file.gcount()));
            if (file.bad()) throw std::runtime_error("input read failed");
        } else {
            Serial serial(port);
            std::cout << "Reading " << port << ", DTR=1, 115200 line coding; Ctrl+C to stop." << std::endl;
            std::signal(SIGINT, stop);
            std::signal(SIGTERM, stop);
            while (!interrupted) {
                const auto now = Clock::now();
                const auto elapsed = std::chrono::duration<double>(now - start).count();
                if (duration > 0 && elapsed >= duration) break;
                pollfd item{serial.fd(), POLLIN, 0};
                const int timeout = duration > 0 ? int(std::clamp((duration - elapsed) * 1000.0, 1.0, 100.0)) : 100;
                const int ready = ::poll(&item, 1, timeout);
                if (ready < 0 && errno == EINTR) continue;
                system_check(ready >= 0, "poll CDC");
                if (item.revents & POLLIN) {
                    const auto count = ::read(serial.fd(), data.data(), data.size());
                    if (count > 0) receiver.feed(data.data(), static_cast<std::size_t>(count));
                    else if (count < 0 && errno != EAGAIN && errno != EINTR)
                        system_check(false, "read CDC");
                }
                if (item.revents & (POLLERR | POLLHUP | POLLNVAL))
                    throw std::runtime_error("USB serial device disconnected");
                if (Clock::now() >= next_report) {
                    receiver.print(false, std::chrono::duration<double>(Clock::now() - start).count());
                    next_report = Clock::now() + std::chrono::seconds(1);
                }
            }
        }
        receiver.print(true, std::chrono::duration<double>(Clock::now() - start).count());
        if (!receiver.samples) std::cerr << "No valid HIGHRES_IMU received. Check firmware initialization and CDC DTR.\n";
        return receiver.ok() ? 0 : 1;
    } catch (const std::exception &error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 2;
    }
}
