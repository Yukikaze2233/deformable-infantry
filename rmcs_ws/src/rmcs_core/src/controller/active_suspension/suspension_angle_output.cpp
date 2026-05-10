#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace rmcs_core::chassis::suspension {

class AngleOutput
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AngleOutput()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        {
        min_angle_ = deg_to_rad(get_parameter_or("min_angle", 7.0));
        max_angle_ = deg_to_rad(get_parameter_or("max_angle", 58.0));
        register_input("/chassis/suspension/pitch_angle_diff", pitch_angle_diff_);
        register_input("/chassis/suspension/roll_angle_diff", roll_angle_diff_);

        register_input("/chassis/left_front_joint/target_physical_angle", left_front_joint_target_physical_angle_);
        register_input("/chassis/left_back_joint/target_physical_angle", left_back_joint_target_physical_angle_);
        register_input("/chassis/right_back_joint/target_physical_angle", right_back_joint_target_physical_angle_);
        register_input("/chassis/right_front_joint/target_physical_angle", right_front_joint_target_physical_angle_);

        register_output(
            "/chassis/suspension/left_front_joint/target_physical_angle",
            left_front_joint_suspension_target_physical_angle_, 0.0);
        register_output(
            "/chassis/suspension/left_back_joint/target_physical_angle",
            left_back_joint_suspension_target_physical_angle_, 0.0);
        register_output(
            "/chassis/suspension/right_back_joint/target_physical_angle",
            right_back_joint_suspension_target_physical_angle_, 0.0);
        register_output(
            "/chassis/suspension/right_front_joint/target_physical_angle",
            right_front_joint_suspension_target_physical_angle_, 0.0);
    }

    void update() override {
        if (!targets_ready_()) {
            publish_nan_outputs_();
            return;
        }

        const double pitch_angle_diff = finite_or_zero_(pitch_angle_diff_);
        const double roll_angle_diff = finite_or_zero_(roll_angle_diff_);

        *left_back_joint_suspension_target_physical_angle_ =
            *left_back_joint_target_physical_angle_ + pitch_angle_diff + roll_angle_diff;
        *right_back_joint_suspension_target_physical_angle_ =
            *right_back_joint_target_physical_angle_ + pitch_angle_diff - roll_angle_diff;
        *left_front_joint_suspension_target_physical_angle_ =
            *left_front_joint_target_physical_angle_ - pitch_angle_diff + roll_angle_diff;
        *right_front_joint_suspension_target_physical_angle_ =
            *right_front_joint_target_physical_angle_ - pitch_angle_diff - roll_angle_diff;
        
        *left_back_joint_suspension_target_physical_angle_ = std::clamp(*left_back_joint_suspension_target_physical_angle_, min_angle_, max_angle_);
        *right_back_joint_suspension_target_physical_angle_ = std::clamp(*right_back_joint_suspension_target_physical_angle_, min_angle_, max_angle_);
        *left_front_joint_suspension_target_physical_angle_ = std::clamp(*left_front_joint_suspension_target_physical_angle_, min_angle_, max_angle_);
        *right_front_joint_suspension_target_physical_angle_ = std::clamp(*right_front_joint_suspension_target_physical_angle_, min_angle_, max_angle_);
    
    }

private:
    double deg_to_rad(double deg) {
        return deg * std::numbers::pi / 180.0;
    }

    double finite_or_zero_(const InputInterface<double>& input) const {
        return input.ready() && std::isfinite(*input) ? *input : 0.0;
    }

    bool targets_ready_() const {
        return left_front_joint_target_physical_angle_.ready()
            && std::isfinite(*left_front_joint_target_physical_angle_)
            && left_back_joint_target_physical_angle_.ready()
            && std::isfinite(*left_back_joint_target_physical_angle_)
            && right_back_joint_target_physical_angle_.ready()
            && std::isfinite(*right_back_joint_target_physical_angle_)
            && right_front_joint_target_physical_angle_.ready()
            && std::isfinite(*right_front_joint_target_physical_angle_);
    }

    void publish_nan_outputs_() {
        *left_front_joint_suspension_target_physical_angle_ = nan_;
        *left_back_joint_suspension_target_physical_angle_ = nan_;
        *right_back_joint_suspension_target_physical_angle_ = nan_;
        *right_front_joint_suspension_target_physical_angle_ = nan_;
    }

    InputInterface<double> pitch_angle_diff_;
    InputInterface<double> roll_angle_diff_;

    InputInterface<double> left_front_joint_target_physical_angle_;
    InputInterface<double> left_back_joint_target_physical_angle_;
    InputInterface<double> right_back_joint_target_physical_angle_;
    InputInterface<double> right_front_joint_target_physical_angle_;

    OutputInterface<double> left_front_joint_suspension_target_physical_angle_;
    OutputInterface<double> left_back_joint_suspension_target_physical_angle_;
    OutputInterface<double> right_back_joint_suspension_target_physical_angle_;
    OutputInterface<double> right_front_joint_suspension_target_physical_angle_;

    double min_angle_ = 7.0;
    double max_angle_ = 58.0;
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

};

} // namespace rmcs_core::chassis::suspension

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::chassis::suspension::AngleOutput, rmcs_executor::Component)
