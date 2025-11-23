#include <memory>

#include <librmcs/client/cboard.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

#include "hardware/device/dr16.hpp"
#include "hardware/device/dji_motor.hpp"

namespace rmcs_core::hardware {

class TestInfantry
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::client::CBoard {
public:
    TestInfantry()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())}
        , logger_(get_logger())
        , infantry_command_(
              create_partner_component<InfantryCommand>(get_component_name() + "_command", *this))
        , transmit_buffer_(*this, 32)
        , event_thread_([this]() { handle_events(); }) {

        for (auto& motor : chassis_lift_motors_)
            motor.configure(device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                                .set_reversed()
                                .set_reduction_ratio(13.)
                                .enable_multi_turn_angle());

        using namespace rmcs_description;

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });

    }

    ~TestInfantry() override {
        stop_handling_events();
        event_thread_.join();
    }

    void update() override {
        update_motors();
        dr16_.update_status();
    }

    void command_update() {
        uint16_t can_commands[4];

        can_commands[0] = chassis_lift_motors_[0].generate_command();
        can_commands[1] = 0.0/*chassis_lift_motors_[1].generate_command()*/;
        can_commands[2] = 0.0/*chassis_lift_motors_[2].generate_command()*/;
        can_commands[3] = 0.0/*chassis_lift_motors_[3].generate_command()*/;

        transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        transmit_buffer_.trigger_transmission();
    }
private:
    void update_motors(){
        using namespace rmcs_description;
        for (auto& motor : chassis_lift_motors_)
            motor.update_status();
    }

    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            logger_, "[chassis calibration] New motor_0 offset: %d",
            chassis_lift_motors_[0].calibrate_zero_point());
    }

protected:
    void can2_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

        if (can_id == 0x201) {
            auto& motor = chassis_lift_motors_[0];
            motor.store_status(can_data);
        } else if (can_id == 0x202) {
            auto& motor = chassis_lift_motors_[1];
            motor.store_status(can_data);
        } else if (can_id == 0x203) {
            auto& motor = chassis_lift_motors_[2];
            motor.store_status(can_data);
        } else if (can_id == 0x204) {
            auto& motor = chassis_lift_motors_[3];
            motor.store_status(can_data);
        }
    }

    void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        dr16_.store_status(uart_data, uart_data_length);
    }


private:
    rclcpp::Logger logger_;

    class InfantryCommand : public rmcs_executor::Component {
    public:
        explicit InfantryCommand(TestInfantry& infantry)
            : infantry_(infantry) {}

        void update() override { infantry_.command_update(); }

        TestInfantry& infantry_;
    };
    std::shared_ptr<InfantryCommand> infantry_command_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;

    device::DjiMotor chassis_lift_motors_[4]{
        {*this, *infantry_command_, "/chassis/lift/left_front_wheel"},
        {*this, *infantry_command_, "/chassis/lift/left_back_wheel"},
        {*this, *infantry_command_, "/chassis/lift/right_front_wheel"},
        {*this, *infantry_command_, "/chassis/lift/right_back_wheel"}
    };
    device::Dr16 dr16_{*this};
    librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
    std::thread event_thread_;

};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::TestInfantry, rmcs_executor::Component)