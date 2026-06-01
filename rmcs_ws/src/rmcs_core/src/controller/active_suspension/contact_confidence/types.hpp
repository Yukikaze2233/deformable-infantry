#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>

namespace rmcs_core::chassis::suspension::contact_confidence {

inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
inline constexpr double kMinDenominator = 1e-6;
inline constexpr double kMinResidualScale = 1e-6;

struct InputSnapshot {
    double measurement_angle = kNaN;
    double measurement_velocity = kNaN;
    double setpoint_angle = kNaN;
    double setpoint_velocity = kNaN;
    double control_torque = kNaN;
    double joint_torque = kNaN;
    double eso_z3 = kNaN;
};

struct MotionObservation {
    double command_torque = 0.0;
    double measured_velocity = 0.0;
    double tracking_error = 0.0;
    double progress_velocity = 0.0;
    bool command_active = false;
    double block_score = 0.0;
    double tracking_score = 0.0;
};

struct FeatureScores {
    double z3 = kNaN;
    double block = kNaN;
    double tracking = kNaN;
    double torque = kNaN;
};

enum class ReferenceState {
    kWarmup,
    kReady,
};

struct ReferenceConfig {
    double min_command_torque = 1.0;
    double min_tracking_error = 0.01;
    double min_progress_velocity = 0.05;
    double min_free_motion_velocity = 0.05;
    double free_state_tracking_error_threshold = 0.005;
    double free_state_block_score_threshold = 0.2;
    double free_state_raw_z3_delta_threshold = 0.5;
    double grounded_z3_reference = kNaN;
    double grounded_z3_reject_margin = 0.5;
    std::size_t min_free_samples_for_ready = 200;
    std::size_t min_free_motion_samples_for_torque_ready = 100;
    std::size_t free_state_reference_window_size = 400;
    double free_state_z3_residual_quantile = 0.9;
    double min_z3_residual_scale = 0.25;
};

struct ReferenceModel {
    ReferenceState state = ReferenceState::kWarmup;
    std::deque<double> free_state_z3_samples;
    std::deque<double> free_state_torque_samples;
    double z3_baseline = kNaN;
    double torque_baseline = kNaN;
    double z3_residual_scale = kNaN;
};

struct FusionConfig {
    double z3_weight = 0.40;
    double block_weight = 0.30;
    double tracking_weight = 0.20;
    double torque_weight = 0.10;
    double entry_activate_threshold = 0.55;
    double hold_activate_threshold = 0.55;
    double hold_deactivate_threshold = 0.30;
    double normalized_z3_strength_full_scale_threshold = 3.0;
    double normalized_z3_strength_presence_threshold = 1.0;
    double normalized_z3_strength_presence_score = 0.7;
    double torque_activation_threshold = 1.0;
};

struct FilterConfig {
    double update_dt = 0.001;
    double confidence_rise_rate = 15.0;
    double confidence_fall_rate = 6.0;
};

inline double normalized_activation(double value, double threshold) {
    if (!std::isfinite(value) || !std::isfinite(threshold) || threshold <= 0.0) {
        return kNaN;
    }
    return std::clamp(value / threshold, 0.0, 1.0);
}

inline double blend(double current, double target, double alpha) {
    return current + alpha * (target - current);
}

} // namespace rmcs_core::chassis::suspension::contact_confidence
