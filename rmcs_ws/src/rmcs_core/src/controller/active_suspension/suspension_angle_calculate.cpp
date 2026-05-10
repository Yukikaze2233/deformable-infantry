#include <cmath>
#include <limits>
#include <numbers>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::chassis::suspension {

class AngleCalculate
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AngleCalculate()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        {
            pitch_offset_ = get_parameter_or("pitch_offset_deg", 0.0);
            roll_offset_ = get_parameter_or("roll_offset_deg", 0.0);

            register_input("/chassis/imu/pitch", pitch_);
            register_input("/chassis/imu/roll", roll_);

            register_output("/chassis/suspension/pitch_error", pitch_error_, 0.0);
            register_output("/chassis/suspension/roll_error", roll_error_, 0.0);
        }

    void update() override {
       const bool pitch_valid = pitch_.ready() && std::isfinite(*pitch_);
       const bool roll_valid = roll_.ready() && std::isfinite(*roll_);

       *pitch_error_ = pitch_valid ? -(*pitch_ - pitch_offset_) : nan_;
       *roll_error_ = roll_valid ? *roll_ - roll_offset_ : nan_;

       RCLCPP_INFO(get_logger(), "Pitch_error: %f, Roll_error: %f", *pitch_, *roll_);
    }

private:
    static double deg_to_rad(double deg) {
        return deg * std::numbers::pi / 180.0;
    }
    double pitch_offset_ = 0.0;
    double roll_offset_ = 0.0;
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<double> pitch_;
    InputInterface<double> roll_;

    OutputInterface<double> pitch_error_;
    OutputInterface<double> roll_error_;


};

} // namespace rmcs_core::chassis::suspension

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::chassis::suspension::AngleCalculate, rmcs_executor::Component)
