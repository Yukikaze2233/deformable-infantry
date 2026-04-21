#include <algorithm>
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <string>
#include <span>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

#include "../../librmcs/host/include/librmcs/agent/rmcs_board_lite.hpp"

#include "hardware/device/VT13.hpp"

namespace rmcs_core::hardware {

class Vt13RmcsBoardLite
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::agent::RmcsBoardLite {
public:
    Vt13RmcsBoardLite()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , librmcs::agent::RmcsBoardLite(
              get_parameter("board_serial").as_string(),
              librmcs::agent::AdvancedOptions{.dangerously_skip_version_checks = true})
        , vt13_(*this) {}

    void update() override {
        vt13_.update_status();

        if (uart_packet_count_ == 0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "VT13 UART0 debug: no UART packets received yet");
            return;
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "VT13 UART0 debug: packets=%zu ok=%zu len_err=%zu header_err=%zu crc_err=%zu last_size=%zu expected=%zu idle=%d last_status=%s ",
            uart_packet_count_, ok_packet_count_, length_error_count_, header_error_count_,
            crc_error_count_, last_packet_size_, device::VT13::frame_size_bytes(),
            static_cast<int>(last_idle_delimited_),
            device::VT13::store_status_to_cstr(last_status_));
    }

private:
    static std::string format_bytes(std::span<const std::byte> data) {
        constexpr size_t kMaxBytesToShow = 32;

        std::ostringstream stream;
        stream << std::hex << std::uppercase << std::setfill('0');

        const size_t displayed_size = std::min(data.size(), kMaxBytesToShow);
        for (size_t i = 0; i < displayed_size; ++i) {
            if (i != 0)
                stream << ' ';
            stream << std::setw(2) << static_cast<unsigned>(std::to_integer<uint8_t>(data[i]));
        }

        if (data.size() > displayed_size)
            stream << " ...";

        return stream.str();
    }

    void log_unexpected_uart(
        const char* port_name, const librmcs::data::UartDataView& data) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 500,
            "VT13 debug: received UART data on unexpected %s path: size=%zu idle=%d",
            port_name, data.uart_data.size(), static_cast<int>(data.idle_delimited));
    }

    void uart0_receive_callback(const librmcs::data::UartDataView& data) override {
        ++uart_packet_count_;
        last_packet_size_ = data.uart_data.size();
        last_idle_delimited_ = data.idle_delimited;
        last_packet_hex_ = format_bytes(data.uart_data);

        last_status_ = vt13_.store_status(data.uart_data.data(), data.uart_data.size());
        switch (last_status_) {
        case device::VT13::StoreStatus::kOk:
            ++ok_packet_count_;
            break;
        case device::VT13::StoreStatus::kLengthMismatch:
            ++length_error_count_;
            break;
        case device::VT13::StoreStatus::kHeaderMismatch:
            ++header_error_count_;
            break;
        case device::VT13::StoreStatus::kCrcMismatch:
            ++crc_error_count_;
            break;
        }

        if (last_status_ != device::VT13::StoreStatus::kOk) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 500,
                "VT13 UART0 invalid packet: size=%zu expected=%zu idle=%d status=%s data=%s",
                last_packet_size_, device::VT13::frame_size_bytes(),
                static_cast<int>(last_idle_delimited_),
                device::VT13::store_status_to_cstr(last_status_), last_packet_hex_.c_str());
        }
    }

    void uart1_receive_callback(const librmcs::data::UartDataView& data) override {
        log_unexpected_uart("uart1", data);
    }

    void dbus_receive_callback(const librmcs::data::UartDataView& data) override {
        log_unexpected_uart("dbus", data);
    }

    device::VT13 vt13_;
    size_t uart_packet_count_ = 0;
    size_t ok_packet_count_ = 0;
    size_t length_error_count_ = 0;
    size_t header_error_count_ = 0;
    size_t crc_error_count_ = 0;
    size_t last_packet_size_ = 0;
    bool last_idle_delimited_ = false;
    device::VT13::StoreStatus last_status_ = device::VT13::StoreStatus::kLengthMismatch;
    std::string last_packet_hex_ = "-";
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Vt13RmcsBoardLite, rmcs_executor::Component)
