#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::active_suspension {

class AdaptiveOmniContactEstimator
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AdaptiveOmniContactEstimator()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , wheel_radius_(get_parameter_or("wheel_radius", 0.055))
        , confidence_alpha_(get_parameter_or("confidence_alpha", 0.15))
        , moving_speed_threshold_(get_parameter_or("moving_speed_threshold", 1.0))
        , slip_ratio_gain_(get_parameter_or("slip_ratio_gain", 1.5))
        , wheel_torque_reference_(get_parameter_or("wheel_torque_reference", 0.8))
        , joint_torque_reference_(get_parameter_or("joint_torque_reference", 5.0))
        , confidence_floor_(get_parameter_or("confidence_floor", 0.15)) {

        register_input("/chassis/control_velocity", chassis_control_velocity_);
        register_input("/chassis/radius", chassis_radius_);

        register_input("/chassis/left_front_wheel/velocity", left_front_wheel_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_wheel_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_wheel_velocity_);
        register_input("/chassis/right_front_wheel/velocity", right_front_wheel_velocity_);

        register_input("/chassis/left_front_wheel/torque", left_front_wheel_torque_);
        register_input("/chassis/left_back_wheel/torque", left_back_wheel_torque_);
        register_input("/chassis/right_back_wheel/torque", right_back_wheel_torque_);
        register_input("/chassis/right_front_wheel/torque", right_front_wheel_torque_);

        register_input("/chassis/left_front_joint/torque", left_front_joint_torque_);
        register_input("/chassis/left_back_joint/torque", left_back_joint_torque_);
        register_input("/chassis/right_back_joint/torque", right_back_joint_torque_);
        register_input("/chassis/right_front_joint/torque", right_front_joint_torque_);

        register_output("/chassis/left_front_contact/confidence", left_front_contact_confidence_, 1.0);
        register_output("/chassis/left_back_contact/confidence", left_back_contact_confidence_, 1.0);
        register_output("/chassis/right_back_contact/confidence", right_back_contact_confidence_, 1.0);
        register_output("/chassis/right_front_contact/confidence", right_front_contact_confidence_, 1.0);

        register_output("/chassis/left_front_contact/residual", left_front_contact_residual_, 0.0);
        register_output("/chassis/left_back_contact/residual", left_back_contact_residual_, 0.0);
        register_output("/chassis/right_back_contact/residual", right_back_contact_residual_, 0.0);
        register_output("/chassis/right_front_contact/residual", right_front_contact_residual_, 0.0);

        register_output("/chassis/contact/confidence_mean", contact_confidence_mean_, 1.0);
    }

    void update() override {
        sanitize_last_confidence_();

        const Eigen::Vector3d control_velocity = chassis_control_velocity_->vector;
        if (!control_velocity.allFinite() || !std::isfinite(*chassis_radius_) || *chassis_radius_ <= 1e-6) {
            publish_neutral_confidence_();
            return;
        }

        const auto wheel_velocities = read_inputs_(
            left_front_wheel_velocity_, left_back_wheel_velocity_, right_back_wheel_velocity_,
            right_front_wheel_velocity_);
        const auto wheel_torques = read_inputs_(
            left_front_wheel_torque_, left_back_wheel_torque_, right_back_wheel_torque_,
            right_front_wheel_torque_);
        const auto joint_torques = read_inputs_(
            left_front_joint_torque_, left_back_joint_torque_, right_back_joint_torque_,
            right_front_joint_torque_);

        const auto expected_wheel_velocities = expected_wheel_velocity_(
            control_velocity, std::max(*chassis_radius_, 1e-6));
        if (!all_finite_(expected_wheel_velocities)) {
            publish_neutral_confidence_();
            return;
        }

        std::array<double, 4> confidence = last_confidence_;
        std::array<double, 4> residual = {0.0, 0.0, 0.0, 0.0};

        const double command_norm = control_velocity.norm();
        for (size_t i = 0; i < 4; ++i) {
            const double expected = expected_wheel_velocities[i];
            const double measured = wheel_velocities[i];
            const double wheel_torque = wheel_torques[i];
            const double joint_torque = joint_torques[i];
            if (!std::isfinite(expected) || !std::isfinite(measured) || !std::isfinite(wheel_torque)
                || !std::isfinite(joint_torque)) {
                confidence[i] = last_confidence_[i];
                residual[i] = 0.0;
                continue;
            }

            const double denom = std::max(std::abs(expected), 1.0);
            residual[i] = std::abs(measured - expected) / denom;

            const double slip_score = 1.0 / (1.0 + slip_ratio_gain_ * residual[i]);
            const double wheel_load_score = std::clamp(
                std::abs(wheel_torque) / std::max(wheel_torque_reference_, 1e-6), 0.0, 1.0);
            const double joint_support_score = std::clamp(
                std::abs(joint_torque) / std::max(joint_torque_reference_, 1e-6), 0.0, 1.0);

            double raw_confidence = 1.0;
            if (command_norm > moving_speed_threshold_) {
                raw_confidence = 0.55 * slip_score + 0.25 * wheel_load_score + 0.20 * joint_support_score;
            } else {
                raw_confidence = 0.60 + 0.40 * joint_support_score;
            }

            const double blended = std::clamp(raw_confidence, confidence_floor_, 1.0);
            const double filtered =
                (1.0 - confidence_alpha_) * last_confidence_[i] + confidence_alpha_ * blended;
            if (!std::isfinite(filtered)) {
                confidence[i] = last_confidence_[i];
                residual[i] = 0.0;
                continue;
            }

            confidence[i] = filtered;
            last_confidence_[i] = filtered;
        }

        publish_confidence(confidence, residual);
    }

private:
    static bool all_finite_(const std::array<double, 4>& values) {
        return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
    }

    template <typename InterfaceT>
    static std::array<double, 4> read_inputs_(
        const InterfaceT& left_front, const InterfaceT& left_back, const InterfaceT& right_back,
        const InterfaceT& right_front) {
        return {*left_front, *left_back, *right_back, *right_front};
    }

    void sanitize_last_confidence_() {
        for (double& confidence : last_confidence_) {
            if (!std::isfinite(confidence))
                confidence = 1.0;
        }
    }

    std::array<double, 4> expected_wheel_velocity_(
        const Eigen::Vector3d& control_velocity, double chassis_radius) const {
        const double x = control_velocity.x();
        const double y = control_velocity.y();
        const double z = control_velocity.z();
        const double a_plus_b = std::numbers::sqrt2 * chassis_radius;

        std::array<double, 4> wheel_control_velocity{
            -x + y + a_plus_b * z,
            -x - y + a_plus_b * z,
            +x - y + a_plus_b * z,
            +x + y + a_plus_b * z,
        };
        for (double& velocity : wheel_control_velocity)
            velocity *= -1.0 / (std::numbers::sqrt2 * wheel_radius_);
        return wheel_control_velocity;
    }

    void publish_confidence(
        const std::array<double, 4>& confidence, const std::array<double, 4>& residual) {
        const auto safe_confidence =
            all_finite_(confidence) ? confidence : std::array<double, 4>{1.0, 1.0, 1.0, 1.0};
        const auto safe_residual =
            all_finite_(residual) ? residual : std::array<double, 4>{0.0, 0.0, 0.0, 0.0};

        *left_front_contact_confidence_ = safe_confidence[0];
        *left_back_contact_confidence_ = safe_confidence[1];
        *right_back_contact_confidence_ = safe_confidence[2];
        *right_front_contact_confidence_ = safe_confidence[3];

        *left_front_contact_residual_ = safe_residual[0];
        *left_back_contact_residual_ = safe_residual[1];
        *right_back_contact_residual_ = safe_residual[2];
        *right_front_contact_residual_ = safe_residual[3];

        *contact_confidence_mean_ =
            (safe_confidence[0] + safe_confidence[1] + safe_confidence[2] + safe_confidence[3]) / 4.0;
    }

    void publish_neutral_confidence_() {
        publish_confidence({1.0, 1.0, 1.0, 1.0}, {0.0, 0.0, 0.0, 0.0});
    }

    const double wheel_radius_;
    const double confidence_alpha_;
    const double moving_speed_threshold_;
    const double slip_ratio_gain_;
    const double wheel_torque_reference_;
    const double joint_torque_reference_;
    const double confidence_floor_;

    std::array<double, 4> last_confidence_ = {1.0, 1.0, 1.0, 1.0};

    InputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
    InputInterface<double> chassis_radius_;

    InputInterface<double> left_front_wheel_velocity_;
    InputInterface<double> left_back_wheel_velocity_;
    InputInterface<double> right_back_wheel_velocity_;
    InputInterface<double> right_front_wheel_velocity_;

    InputInterface<double> left_front_wheel_torque_;
    InputInterface<double> left_back_wheel_torque_;
    InputInterface<double> right_back_wheel_torque_;
    InputInterface<double> right_front_wheel_torque_;

    InputInterface<double> left_front_joint_torque_;
    InputInterface<double> left_back_joint_torque_;
    InputInterface<double> right_back_joint_torque_;
    InputInterface<double> right_front_joint_torque_;

    OutputInterface<double> left_front_contact_confidence_;
    OutputInterface<double> left_back_contact_confidence_;
    OutputInterface<double> right_back_contact_confidence_;
    OutputInterface<double> right_front_contact_confidence_;

    OutputInterface<double> left_front_contact_residual_;
    OutputInterface<double> left_back_contact_residual_;
    OutputInterface<double> right_back_contact_residual_;
    OutputInterface<double> right_front_contact_residual_;

    OutputInterface<double> contact_confidence_mean_;
};

} // namespace rmcs_core::controller::active_suspension

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::active_suspension::AdaptiveOmniContactEstimator,
    rmcs_executor::Component)
