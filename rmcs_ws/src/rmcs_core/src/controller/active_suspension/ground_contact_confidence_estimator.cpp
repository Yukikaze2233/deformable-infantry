#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::chassis::suspension {

class GroundContactConfidenceEstimator
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GroundContactConfidenceEstimator()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , joint_name_(get_parameter_or("joint_name", std::string("unknown_joint")))
        , update_dt_(get_parameter_or("dt", 0.001))
        , z3_weight_(get_parameter_or("z3_weight", 0.40))
        , block_weight_(get_parameter_or("block_weight", 0.30))
        , tracking_weight_(get_parameter_or("tracking_weight", 0.20))
        , torque_weight_(get_parameter_or("torque_weight", 0.10))
        , confidence_rise_rate_(get_parameter_or("confidence_rise_rate", 15.0))
        , confidence_fall_rate_(get_parameter_or("confidence_fall_rate", 6.0))
        , min_command_velocity_(get_parameter_or("min_command_velocity", 0.05))
        , min_command_torque_(get_parameter_or("min_command_torque", 1.0))
        , min_tracking_error_(get_parameter_or("min_tracking_error", 0.01))
        , z3_activation_threshold_(get_parameter_or("z3_activation_threshold", 1.0))
        , torque_activation_threshold_(get_parameter_or("torque_activation_threshold", 1.0)) {
        register_interfaces_();
    }

    void update() override {
        InputSnapshot input;
        if (!read_inputs_(input)) {
            publish_nan_outputs_();
            return;
        }

        update_baselines_(input);
        const FeatureScores scores = compute_scores_(input);
        const double confidence_raw = fuse_scores_(scores);
        const double confidence = filter_confidence_(confidence_raw);

        publish_outputs_(scores, confidence);
    }

private:
    struct InputSnapshot {
        double measurement_angle = std::numeric_limits<double>::quiet_NaN();
        double measurement_velocity = std::numeric_limits<double>::quiet_NaN();
        double setpoint_angle = std::numeric_limits<double>::quiet_NaN();
        double setpoint_velocity = std::numeric_limits<double>::quiet_NaN();
        double control_torque = std::numeric_limits<double>::quiet_NaN();
        double joint_torque = std::numeric_limits<double>::quiet_NaN();
        double eso_z3 = std::numeric_limits<double>::quiet_NaN();
    };

    struct FeatureScores {
        double z3 = std::numeric_limits<double>::quiet_NaN();
        double block = std::numeric_limits<double>::quiet_NaN();
        double tracking = std::numeric_limits<double>::quiet_NaN();
        double torque = std::numeric_limits<double>::quiet_NaN();
    };

    void register_interfaces_() {
        register_input(
            get_parameter_or("measurement_angle", std::string("/chassis/joint/physical_angle")),
            measurement_angle_);
        register_input(
            get_parameter_or("measurement_velocity", std::string("/chassis/joint/physical_velocity")),
            measurement_velocity_);
        register_input(
            get_parameter_or("setpoint_angle", std::string("/chassis/joint/target_physical_angle")),
            setpoint_angle_);
        register_input(
            get_parameter_or(
                "setpoint_velocity", std::string("/chassis/joint/target_physical_velocity")),
            setpoint_velocity_);
        register_input(
            get_parameter_or("control_torque", std::string("/chassis/joint/control_torque")),
            control_torque_);
        register_input(
            get_parameter_or("joint_torque", std::string("/chassis/joint/torque")), joint_torque_);
        register_input(get_parameter_or("eso_z3", std::string("/chassis/joint/eso_z3")), eso_z3_);

        register_output(
            get_parameter_or(
                "ground_contact_confidence",
                std::string("/chassis/joint/ground_contact_confidence")),
            ground_contact_confidence_, nan_);
        register_output(
            get_parameter_or(
                "ground_contact_z3_score",
                std::string("/chassis/joint/ground_contact_z3_score")),
            ground_contact_z3_score_, nan_);
        register_output(
            get_parameter_or(
                "ground_contact_block_score",
                std::string("/chassis/joint/ground_contact_block_score")),
            ground_contact_block_score_, nan_);
        register_output(
            get_parameter_or(
                "ground_contact_tracking_score",
                std::string("/chassis/joint/ground_contact_tracking_score")),
            ground_contact_tracking_score_, nan_);
        register_output(
            get_parameter_or(
                "ground_contact_torque_score",
                std::string("/chassis/joint/ground_contact_torque_score")),
            ground_contact_torque_score_, nan_);
    }

    [[nodiscard]] bool read_inputs_(InputSnapshot& input) const {
        if (!measurement_angle_.ready() || !measurement_velocity_.ready() || !setpoint_angle_.ready()
            || !setpoint_velocity_.ready() || !control_torque_.ready() || !joint_torque_.ready()
            || !eso_z3_.ready()) {
            return false;
        }

        input.measurement_angle = *measurement_angle_;
        input.measurement_velocity = *measurement_velocity_;
        input.setpoint_angle = *setpoint_angle_;
        input.setpoint_velocity = *setpoint_velocity_;
        input.control_torque = *control_torque_;
        input.joint_torque = *joint_torque_;
        input.eso_z3 = *eso_z3_;

        return std::isfinite(input.measurement_angle) && std::isfinite(input.measurement_velocity)
            && std::isfinite(input.setpoint_angle) && std::isfinite(input.setpoint_velocity)
            && std::isfinite(input.control_torque) && std::isfinite(input.joint_torque)
            && std::isfinite(input.eso_z3);
    }

    void update_baselines_(const InputSnapshot& input) {
        if (!baselines_initialized_) {
            z3_baseline_ = input.eso_z3;
            torque_baseline_ = std::abs(input.joint_torque);
            baselines_initialized_ = true;
            return;
        }

        const double baseline_alpha = std::clamp(update_dt_ * 0.5, 0.0, 1.0);
        z3_baseline_ = blend_(z3_baseline_, input.eso_z3, baseline_alpha);
        torque_baseline_ = blend_(torque_baseline_, std::abs(input.joint_torque), baseline_alpha);
    }

    [[nodiscard]] FeatureScores compute_scores_(const InputSnapshot& input) const {
        FeatureScores scores;

        const double command_velocity = std::abs(input.setpoint_velocity);
        const double command_torque = std::abs(input.control_torque);
        const double measured_velocity = std::abs(input.measurement_velocity);
        const double tracking_error = std::abs(input.setpoint_angle - input.measurement_angle);
        const double z3_delta = std::abs(input.eso_z3 - z3_baseline_);
        const double torque_delta = std::max(0.0, std::abs(input.joint_torque) - torque_baseline_);

        const bool command_active =
            command_velocity >= min_command_velocity_ || command_torque >= min_command_torque_
            || tracking_error >= min_tracking_error_;

        scores.z3 = normalized_activation_(z3_delta, z3_activation_threshold_);

        if (command_active && command_velocity >= min_command_velocity_) {
            const double blocked_ratio =
                std::clamp(1.0 - measured_velocity / std::max(command_velocity, kMinDenominator), 0.0, 1.0);
            scores.block = blocked_ratio;
        } else {
            scores.block = 0.0;
        }

        scores.tracking =
            command_active ? normalized_activation_(tracking_error, min_tracking_error_) : 0.0;
        scores.torque =
            command_active ? normalized_activation_(torque_delta, torque_activation_threshold_) : 0.0;
        return scores;
    }

    [[nodiscard]] double fuse_scores_(const FeatureScores& scores) const {
        if (!std::isfinite(scores.z3) || !std::isfinite(scores.block)
            || !std::isfinite(scores.tracking) || !std::isfinite(scores.torque)) {
            return nan_;
        }

        const double raw = z3_weight_ * scores.z3 + block_weight_ * scores.block
            + tracking_weight_ * scores.tracking + torque_weight_ * scores.torque;
        return std::clamp(raw, 0.0, 1.0);
    }

    double filter_confidence_(double confidence_raw) {
        if (!std::isfinite(confidence_raw)) {
            filtered_confidence_ = nan_;
            return filtered_confidence_;
        }

        if (!std::isfinite(filtered_confidence_)) {
            filtered_confidence_ = confidence_raw;
            return filtered_confidence_;
        }

        const double rate =
            confidence_raw >= filtered_confidence_ ? confidence_rise_rate_ : confidence_fall_rate_;
        const double alpha = std::clamp(rate * update_dt_, 0.0, 1.0);
        filtered_confidence_ = blend_(filtered_confidence_, confidence_raw, alpha);
        filtered_confidence_ = std::clamp(filtered_confidence_, 0.0, 1.0);
        return filtered_confidence_;
    }

    void publish_outputs_(const FeatureScores& scores, double confidence) {
        *ground_contact_confidence_ = confidence;
        *ground_contact_z3_score_ = scores.z3;
        *ground_contact_block_score_ = scores.block;
        *ground_contact_tracking_score_ = scores.tracking;
        *ground_contact_torque_score_ = scores.torque;
    }

    void publish_nan_outputs_() {
        baselines_initialized_ = false;
        filtered_confidence_ = nan_;
        *ground_contact_confidence_ = nan_;
        *ground_contact_z3_score_ = nan_;
        *ground_contact_block_score_ = nan_;
        *ground_contact_tracking_score_ = nan_;
        *ground_contact_torque_score_ = nan_;
    }

    [[nodiscard]] static double normalized_activation_(double value, double threshold) {
        if (!std::isfinite(value) || !std::isfinite(threshold) || threshold <= 0.0) {
            return nan_;
        }
        return std::clamp(value / threshold, 0.0, 1.0);
    }

    [[nodiscard]] static double blend_(double current, double target, double alpha) {
        return current + alpha * (target - current);
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kMinDenominator = 1e-6;

    std::string joint_name_;
    double update_dt_;
    double z3_weight_;
    double block_weight_;
    double tracking_weight_;
    double torque_weight_;
    double confidence_rise_rate_;
    double confidence_fall_rate_;
    double min_command_velocity_;
    double min_command_torque_;
    double min_tracking_error_;
    double z3_activation_threshold_;
    double torque_activation_threshold_;

    InputInterface<double> measurement_angle_;
    InputInterface<double> measurement_velocity_;
    InputInterface<double> setpoint_angle_;
    InputInterface<double> setpoint_velocity_;
    InputInterface<double> control_torque_;
    InputInterface<double> joint_torque_;
    InputInterface<double> eso_z3_;

    OutputInterface<double> ground_contact_confidence_;
    OutputInterface<double> ground_contact_z3_score_;
    OutputInterface<double> ground_contact_block_score_;
    OutputInterface<double> ground_contact_tracking_score_;
    OutputInterface<double> ground_contact_torque_score_;

    bool baselines_initialized_ = false;
    double z3_baseline_ = 0.0;
    double torque_baseline_ = 0.0;
    double filtered_confidence_ = nan_;
};

} // namespace rmcs_core::chassis::suspension

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::chassis::suspension::GroundContactConfidenceEstimator, rmcs_executor::Component)
