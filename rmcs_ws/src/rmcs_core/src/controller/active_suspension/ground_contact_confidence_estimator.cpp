#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "contact_confidence/motion_model.hpp"
#include "contact_confidence/reference_model.hpp"
#include "contact_confidence/scoring.hpp"
#include "contact_confidence/types.hpp"

namespace rmcs_core::chassis::suspension {

namespace cc = rmcs_core::chassis::suspension::contact_confidence;

class GroundContactConfidenceEstimator
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GroundContactConfidenceEstimator()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , joint_name_(get_parameter_or("joint_name", std::string("unknown_joint")))
        , reference_config_(load_reference_config_())
        , fusion_config_(load_fusion_config_())
        , filter_(load_filter_config_())
        , reference_learner_(reference_config_)
        , score_calculator_(fusion_config_) {
        register_interfaces_();
    }

    void update() override {
        cc::InputSnapshot input;
        if (!read_inputs_(input)) {
            publish_invalid_outputs_();
            return;
        }

        const cc::MotionObservation observation = reference_learner_.motion_model().observe(input);
        reference_learner_.update(input, observation);

        if (!reference_learner_.z3_ready()) {
            publish_warmup_outputs_();
            return;
        }

        const cc::FeatureScores scores = score_calculator_.compute(input, observation, reference_learner_);
        const double entry_confidence = score_calculator_.entry_confidence(scores);
        const double hold_confidence = score_calculator_.hold_confidence(scores, reference_learner_);
        const double confidence_raw = phase_logic_.compose(entry_confidence, hold_confidence);
        const double confidence = filter_.update(confidence_raw);
        publish_outputs_(scores, confidence);
    }

private:
    cc::ReferenceConfig load_reference_config_() {
        cc::ReferenceConfig config;
        config.min_command_torque = get_parameter_or("min_command_torque", 1.0);
        config.min_tracking_error = get_parameter_or("min_tracking_error", 0.01);
        config.min_progress_velocity = get_parameter_or(
            "min_progress_velocity", get_parameter_or("min_command_velocity", 0.05));
        config.min_free_motion_velocity = get_parameter_or("min_free_motion_velocity", 0.05);
        config.free_state_tracking_error_threshold =
            get_parameter_or("free_state_tracking_error_threshold", 0.005);
        config.free_state_block_score_threshold =
            get_parameter_or("free_state_block_score_threshold", 0.2);
        config.free_state_raw_z3_delta_threshold =
            get_parameter_or("free_state_raw_z3_delta_threshold", 0.5);
        config.grounded_z3_reference = get_parameter_or("grounded_z3_reference", cc::kNaN);
        config.grounded_z3_reject_margin = get_parameter_or("grounded_z3_reject_margin", 0.5);
        config.min_free_samples_for_ready = static_cast<std::size_t>(
            std::max<int64_t>(0, get_parameter_or("min_free_samples_for_ready", 200)));
        config.min_free_motion_samples_for_torque_ready = static_cast<std::size_t>(
            std::max<int64_t>(0, get_parameter_or("min_free_motion_samples_for_torque_ready", 100)));
        config.free_state_reference_window_size = static_cast<std::size_t>(
            std::max<int64_t>(1, get_parameter_or("free_state_reference_window_size", 400)));
        config.free_state_z3_residual_quantile = get_parameter_or("free_state_z3_residual_quantile", 0.9);
        config.min_z3_residual_scale = get_parameter_or("min_z3_residual_scale", 0.25);
        return config;
    }

    cc::FusionConfig load_fusion_config_() const {
        cc::FusionConfig config;
        config.z3_weight = get_parameter_or("z3_weight", 0.40);
        config.block_weight = get_parameter_or("block_weight", 0.30);
        config.tracking_weight = get_parameter_or("tracking_weight", 0.20);
        config.torque_weight = get_parameter_or("torque_weight", 0.10);
        config.entry_activate_threshold = get_parameter_or("entry_activate_threshold", 0.55);
        config.hold_activate_threshold = get_parameter_or("hold_activate_threshold", 0.55);
        config.hold_deactivate_threshold = get_parameter_or("hold_deactivate_threshold", 0.30);
        config.normalized_z3_strength_full_scale_threshold =
            get_parameter_or("normalized_z3_strength_full_scale_threshold", 3.0);
        config.normalized_z3_strength_presence_threshold =
            get_parameter_or("normalized_z3_strength_presence_threshold", 1.0);
        config.normalized_z3_strength_presence_score =
            get_parameter_or("normalized_z3_strength_presence_score", 0.7);
        config.torque_activation_threshold = get_parameter_or("torque_activation_threshold", 1.0);
        return config;
    }

    cc::FilterConfig load_filter_config_() const {
        cc::FilterConfig config;
        config.update_dt = get_parameter_or("dt", 0.001);
        config.confidence_rise_rate = get_parameter_or("confidence_rise_rate", 15.0);
        config.confidence_fall_rate = get_parameter_or("confidence_fall_rate", 6.0);
        return config;
    }

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
            get_parameter_or("setpoint_velocity", std::string("/chassis/joint/target_physical_velocity")),
            setpoint_velocity_, false);
        register_input(
            get_parameter_or("control_torque", std::string("/chassis/joint/control_torque")),
            control_torque_);
        register_input(
            get_parameter_or("joint_torque", std::string("/chassis/joint/torque")), joint_torque_);
        register_input(get_parameter_or("eso_z3", std::string("/chassis/joint/eso_z3")), eso_z3_);

        register_output(
            get_parameter_or(
                "ground_contact_confidence", std::string("/chassis/joint/ground_contact_confidence")),
            ground_contact_confidence_, cc::kNaN);
        register_output(
            get_parameter_or(
                "ground_contact_z3_score", std::string("/chassis/joint/ground_contact_z3_score")),
            ground_contact_z3_score_, cc::kNaN);
        register_output(
            get_parameter_or(
                "ground_contact_block_score", std::string("/chassis/joint/ground_contact_block_score")),
            ground_contact_block_score_, cc::kNaN);
        register_output(
            get_parameter_or(
                "ground_contact_tracking_score",
                std::string("/chassis/joint/ground_contact_tracking_score")),
            ground_contact_tracking_score_, cc::kNaN);
        register_output(
            get_parameter_or(
                "ground_contact_torque_score", std::string("/chassis/joint/ground_contact_torque_score")),
            ground_contact_torque_score_, cc::kNaN);
    }

    [[nodiscard]] bool read_inputs_(cc::InputSnapshot& input) const {
        if (!measurement_angle_.ready() || !measurement_velocity_.ready() || !setpoint_angle_.ready()
            || !control_torque_.ready() || !joint_torque_.ready() || !eso_z3_.ready()) {
            return false;
        }

        input.measurement_angle = *measurement_angle_;
        input.measurement_velocity = *measurement_velocity_;
        input.setpoint_angle = *setpoint_angle_;
        input.setpoint_velocity = setpoint_velocity_.ready() ? *setpoint_velocity_ : cc::kNaN;
        input.control_torque = *control_torque_;
        input.joint_torque = *joint_torque_;
        input.eso_z3 = *eso_z3_;

        return std::isfinite(input.measurement_angle) && std::isfinite(input.measurement_velocity)
            && std::isfinite(input.setpoint_angle) && std::isfinite(input.control_torque)
            && std::isfinite(input.joint_torque) && std::isfinite(input.eso_z3);
    }

    void publish_outputs_(const cc::FeatureScores& scores, double confidence) {
        *ground_contact_confidence_ = confidence;
        *ground_contact_z3_score_ = scores.z3;
        *ground_contact_block_score_ = scores.block;
        *ground_contact_tracking_score_ = scores.tracking;
        *ground_contact_torque_score_ = scores.torque;
    }

    void publish_invalid_outputs_() {
        filter_.reset();
        phase_logic_.reset();
        *ground_contact_confidence_ = cc::kNaN;
        *ground_contact_z3_score_ = cc::kNaN;
        *ground_contact_block_score_ = cc::kNaN;
        *ground_contact_tracking_score_ = cc::kNaN;
        *ground_contact_torque_score_ = cc::kNaN;
    }

    void publish_warmup_outputs_() {
        filter_.reset();
        phase_logic_.reset();
        *ground_contact_confidence_ = cc::kNaN;
        *ground_contact_z3_score_ = cc::kNaN;
        *ground_contact_block_score_ = cc::kNaN;
        *ground_contact_tracking_score_ = cc::kNaN;
        *ground_contact_torque_score_ = cc::kNaN;
    }

    std::string joint_name_;
    cc::ReferenceConfig reference_config_;
    cc::FusionConfig fusion_config_;
    cc::ConfidenceFilter filter_;
    cc::ConfidencePhaseLogic phase_logic_{fusion_config_};
    cc::ReferenceLearner reference_learner_;
    cc::ScoreCalculator score_calculator_;

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
};

} // namespace rmcs_core::chassis::suspension

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::chassis::suspension::GroundContactConfidenceEstimator, rmcs_executor::Component)
