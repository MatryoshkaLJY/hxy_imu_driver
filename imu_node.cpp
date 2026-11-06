// SPDX-License-Identifier: Apache-2.0
#include <array>
#include <chrono>
#include <cinttypes>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <poll.h>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "imu_protocol.hpp"
#include "serial_port.hpp"

using SteadyClock = std::chrono::steady_clock;

class ImuNode : public rclcpp::Node {
public:
    ImuNode() : Node("hxy_imu")
    {
        port_ = startup_parameter<std::string>("port", "");
        frame_id_ = startup_parameter<std::string>("frame_id", "imu_link");
        const auto topic = startup_parameter<std::string>("topic", "imu/data_raw");
        convert_axes_ = startup_parameter<bool>("convert_frd_to_flu", true);
        const auto system_id = source_id("system_id", 1, 1);
        const auto component_id = source_id("component_id", 200, 1);
        const auto sensor_id = source_id("sensor_id", 0, 0);
        reconnect_interval_ = seconds_parameter("reconnect_interval", 1.0);
        data_timeout_ = seconds_parameter("data_timeout", 2.0);
        if (frame_id_.empty() || frame_id_.front() == '/' || topic.empty())
            throw std::invalid_argument("frame_id must be nonempty without a leading /; topic must be nonempty");
        acceleration_variance_ = variance_parameter("linear_acceleration_variance");
        angular_variance_ = variance_parameter("angular_velocity_variance");
        decoder_ = std::make_unique<hxy_imu::ImuDecoder>(system_id, component_id, sensor_id);
        publisher_ = create_publisher<sensor_msgs::msg::Imu>(topic, rclcpp::SensorDataQoS());
        timer_ = create_wall_timer(std::chrono::milliseconds(2), [this] { read_serial(); });
        report_timer_ = create_wall_timer(std::chrono::seconds(5), [this] { report(); });
        RCLCPP_INFO(get_logger(), "Publishing %s, frame_id=%s, FRD to FLU=%s; stamps use ROS reception time",
                    publisher_->get_topic_name(), frame_id_.c_str(), convert_axes_ ? "true" : "false");
        RCLCPP_WARN(get_logger(), "Firmware mounting axes require physical verification; see README.rst");
    }

private:
    template<typename T>
    T startup_parameter(const std::string &name, const T &value)
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        return declare_parameter<T>(name, value, descriptor);
    }

    std::uint8_t source_id(const std::string &name, int fallback, int minimum)
    {
        const auto value = startup_parameter<std::int64_t>(name, fallback);
        if (value < minimum || value > 255)
            throw std::invalid_argument(name + " is outside the MAVLink ID range");
        return static_cast<std::uint8_t>(value);
    }

    SteadyClock::duration seconds_parameter(const std::string &name, double fallback)
    {
        const auto value = startup_parameter<double>(name, fallback);
        if (!std::isfinite(value) || value < 0.01 || value > 3600)
            throw std::invalid_argument(name + " must be between 0.01 and 3600 seconds");
        return std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<double>(value));
    }

    std::array<double, 3> variance_parameter(const std::string &name)
    {
        const auto values = startup_parameter<std::vector<double>>(name, {0.0, 0.0, 0.0});
        if (values.size() != 3 || !std::all_of(values.begin(), values.end(),
                [](double v) { return std::isfinite(v) && v >= 0; }))
            throw std::invalid_argument(name + " must contain three finite nonnegative variances");
        return {values[0], values[1], values[2]};
    }

    void publish(hxy_imu::Sample sample)
    {
        if (convert_axes_) hxy_imu::frd_to_flu(sample);
        sensor_msgs::msg::Imu message;
        message.header.stamp = now();
        message.header.frame_id = frame_id_;
        message.orientation.w = 1.0;
        message.orientation_covariance[0] = -1.0;
        message.linear_acceleration.x = sample.acceleration[0];
        message.linear_acceleration.y = sample.acceleration[1];
        message.linear_acceleration.z = sample.acceleration[2];
        message.angular_velocity.x = sample.angular_velocity[0];
        message.angular_velocity.y = sample.angular_velocity[1];
        message.angular_velocity.z = sample.angular_velocity[2];
        for (std::size_t axis = 0; axis < 3; ++axis) {
            message.linear_acceleration_covariance[axis * 4] = acceleration_variance_[axis];
            message.angular_velocity_covariance[axis * 4] = angular_variance_[axis];
        }
        publisher_->publish(message);
        last_sample_ = SteadyClock::now();
    }

    void read_serial()
    {
        if (!serial_) {
            if (SteadyClock::now() < next_connect_) return;
            next_connect_ = SteadyClock::now() + reconnect_interval_;
            try {
                const auto selected = hxy_imu::select_port(port_);
                serial_ = std::make_unique<hxy_imu::Serial>(selected);
                decoder_->reset_session();
                last_sample_ = SteadyClock::now();
                RCLCPP_INFO(get_logger(), "Reading %s, DTR=1, 115200 line coding", selected.c_str());
            } catch (const std::exception &error) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for IMU: %s", error.what());
            }
            return;
        }
        try {
            std::array<std::uint8_t, 4096> data{};
            // Bound work in each executor callback even under sustained input.
            for (unsigned int batch = 0; batch < 16; ++batch) {
                pollfd item{serial_->fd(), POLLIN, 0};
                const auto ready = ::poll(&item, 1, 0);
                if (ready < 0 && errno == EINTR) break;
                hxy_imu::system_check(ready >= 0, "poll CDC");
                if (item.revents & (POLLERR | POLLHUP | POLLNVAL))
                    throw std::runtime_error("USB CDC disconnected");
                if (!(item.revents & POLLIN)) break;
                const auto count = ::read(serial_->fd(), data.data(), data.size());
                if (count < 0 && (errno == EAGAIN || errno == EINTR)) break;
                hxy_imu::system_check(count >= 0, "read CDC");
                if (count == 0) break;
                decoder_->feed(data.data(), static_cast<std::size_t>(count),
                               [this](hxy_imu::Sample sample) { publish(sample); });
            }
            if (SteadyClock::now() - last_sample_ > data_timeout_)
                throw std::runtime_error("no valid IMU samples before data_timeout");
        } catch (const std::exception &error) {
            RCLCPP_WARN(get_logger(), "%s; reconnecting", error.what());
            serial_.reset();
            next_connect_ = SteadyClock::now() + reconnect_interval_;
        }
    }

    void report()
    {
        const auto &s = decoder_->stats;
        RCLCPP_INFO(get_logger(), "connected=%s samples=%" PRIu64 " crc_errors=%" PRIu64
                    " field_errors=%" PRIu64 " filtered=%" PRIu64 " duplicates=%" PRIu64
                    " timestamp_resets=%" PRIu64,
                    serial_ ? "true" : "false", s.samples, s.crc_errors, s.field_errors,
                    s.filtered, s.duplicates, s.timestamp_resets);
    }

    std::string port_, frame_id_;
    bool convert_axes_ = true;
    std::array<double, 3> acceleration_variance_{}, angular_variance_{};
    SteadyClock::duration reconnect_interval_{}, data_timeout_{};
    SteadyClock::time_point next_connect_{}, last_sample_{};
    std::unique_ptr<hxy_imu::Serial> serial_;
    std::unique_ptr<hxy_imu::ImuDecoder> decoder_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_, report_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        rclcpp::spin(std::make_shared<ImuNode>());
    } catch (const std::exception &error) {
        if (rclcpp::ok()) {
            RCLCPP_FATAL(rclcpp::get_logger("hxy_imu"), "%s", error.what());
            result = 1;
        }
    }
    rclcpp::shutdown();
    return result;
}
