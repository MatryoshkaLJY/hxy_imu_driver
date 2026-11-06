// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cerrno>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace hxy_imu {
namespace fs = std::filesystem;

inline void system_check(bool ok, const std::string &action)
{
    if (!ok) throw std::system_error(errno, std::generic_category(), action);
}

inline std::string read_text(const fs::path &path)
{
    std::ifstream file(path);
    std::string text;
    std::getline(file, text);
    return text;
}

struct Port {
    std::string path, vid, pid, product, serial;
    bool target = false;
};

inline std::vector<Port> ports()
{
    std::vector<Port> result;
    for (const auto &entry : fs::directory_iterator("/sys/class/tty")) {
        const auto name = entry.path().filename().string();
        if (name.rfind("ttyACM", 0) != 0 && name.rfind("ttyUSB", 0) != 0) continue;
        Port port;
        port.path = "/dev/" + name;
        std::error_code error;
        auto parent = fs::canonical(entry.path() / "device", error);
        while (!error && parent != parent.root_path()) {
            port.vid = read_text(parent / "idVendor");
            if (!port.vid.empty()) {
                port.pid = read_text(parent / "idProduct");
                port.product = read_text(parent / "product");
                port.serial = read_text(parent / "serial");
                // Shared testing IDs alone are insufficient to identify this board.
                port.target = port.vid != "1366" &&
                              port.product.rfind("HXY IMU MAVLink", 0) == 0;
                break;
            }
            parent = parent.parent_path();
        }
        result.push_back(port);
    }
    std::sort(result.begin(), result.end(), [](const Port &a, const Port &b) {
        return a.path < b.path;
    });
    return result;
}

class Serial {
public:
    explicit Serial(const std::string &path)
    {
        fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        system_check(fd_ >= 0, "open " + path);
        try {
            system_check(tcgetattr(fd_, &saved_) == 0, "tcgetattr");
            saved_valid_ = true;
            auto config = saved_;
            cfmakeraw(&config);
            config.c_iflag &= ~(IXON | IXOFF | IXANY);
            config.c_cflag |= CLOCAL | CREAD;
            config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS | CSIZE);
            config.c_cflag |= CS8 | HUPCL;
            config.c_cc[VMIN] = 0;
            config.c_cc[VTIME] = 0;
            system_check(cfsetispeed(&config, B115200) == 0 &&
                         cfsetospeed(&config, B115200) == 0, "set baud rate");
            system_check(tcsetattr(fd_, TCSANOW, &config) == 0, "tcsetattr");
            system_check(tcflush(fd_, TCIFLUSH) == 0, "clear old input");
            int bits = TIOCM_DTR | TIOCM_RTS;
            if (ioctl(fd_, TIOCMBIS, &bits) < 0) {
                // Linux pseudo terminals do not implement modem control; used in tests.
                // A real CDC device must support DTR for firmware to start sending.
                const int control_error = errno;
                std::error_code resolve_error;
                const auto resolved = fs::canonical(path, resolve_error);
                if (control_error != ENOTTY || resolve_error ||
                    resolved.string().rfind("/dev/pts/", 0) != 0) {
                    errno = control_error;
                    system_check(false, "assert CDC DTR");
                }
            }
        } catch (...) {
            cleanup();
            throw;
        }
    }
    ~Serial() { cleanup(); }
    Serial(const Serial &) = delete;
    Serial &operator=(const Serial &) = delete;
    int fd() const { return fd_; }
private:
    int fd_ = -1;
    termios saved_{};
    bool saved_valid_ = false;
    void cleanup() noexcept
    {
        if (fd_ < 0) return;
        int bits = TIOCM_DTR | TIOCM_RTS;
        ioctl(fd_, TIOCMBIC, &bits);
        if (saved_valid_) tcsetattr(fd_, TCSANOW, &saved_);
        ::close(fd_);
        fd_ = -1;
    }
};

// Resolve stable /dev/serial/by-id links before checking the USB vendor.
inline std::string select_port(const std::string &requested)
{
    const auto available = ports();
    if (!requested.empty()) {
        std::error_code error;
        const auto resolved = fs::canonical(requested, error);
        for (const auto &candidate : available) {
            if (!error && resolved == candidate.path && candidate.vid == "1366")
                throw std::runtime_error(requested + " belongs to SEGGER J-Link, not the IMU");
        }
        return requested;
    }
    std::string selected;
    for (const auto &candidate : available) {
        if (!candidate.target) continue;
        if (!selected.empty())
            throw std::runtime_error("multiple IMUs found; set the port parameter");
        selected = candidate.path;
    }
    if (selected.empty()) throw std::runtime_error("no HXY IMU MAVLink USB CDC device found");
    return selected;
}

}  // namespace hxy_imu
