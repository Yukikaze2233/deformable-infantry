#pragma once

#include <algorithm>
#include <cmath>

#include "types.hpp"

namespace rmcs_core::chassis::suspension::contact_confidence {

class MotionModel {
public:
    explicit MotionModel(const ReferenceConfig& config)
        : config_(config) {}

    [[nodiscard]] bool command_active(double command_torque, double tracking_error) const {
        return command_torque >= config_.min_command_torque
            || tracking_error >= config_.min_tracking_error;
    }

    [[nodiscard]] MotionObservation observe(const InputSnapshot& input) const {
        MotionObservation observation;
        observation.command_torque = std::abs(input.control_torque);
        observation.measured_velocity = std::abs(input.measurement_velocity);
        const double signed_tracking_error = input.setpoint_angle - input.measurement_angle;
        observation.tracking_error = std::abs(signed_tracking_error);
        observation.progress_velocity = signed_tracking_error == 0.0
            ? 0.0
            : (signed_tracking_error > 0.0 ? 1.0 : -1.0) * input.measurement_velocity;
        observation.command_active = command_active(observation.command_torque, observation.tracking_error);

        observation.tracking_score = observation.command_active
            ? normalized_activation(observation.tracking_error, config_.min_tracking_error)
            : 0.0;
        const double progress_score = observation.command_active
            ? normalized_activation(std::max(0.0, observation.progress_velocity), config_.min_progress_velocity)
            : 0.0;
        observation.block_score = observation.command_active
            ? std::clamp(observation.tracking_score * (1.0 - progress_score), 0.0, 1.0)
            : 0.0;
        return observation;
    }

    [[nodiscard]] bool far_from_grounded_reference(double eso_z3) const {
        if (!std::isfinite(config_.grounded_z3_reference)) {
            return true;
        }
        if (!std::isfinite(config_.grounded_z3_reject_margin) || config_.grounded_z3_reject_margin < 0.0) {
            return false;
        }
        return std::abs(eso_z3 - config_.grounded_z3_reference) > config_.grounded_z3_reject_margin;
    }

    [[nodiscard]] bool likely_free_state(
        const MotionObservation& observation, double raw_z3_delta) const {
        const bool motion_is_free =
            !observation.command_active
            || observation.progress_velocity >= config_.min_progress_velocity;
        return motion_is_free
            && observation.tracking_error <= config_.free_state_tracking_error_threshold
            && raw_z3_delta <= config_.free_state_raw_z3_delta_threshold
            && observation.block_score <= config_.free_state_block_score_threshold
            && observation.tracking_score <= config_.free_state_block_score_threshold;
    }

    [[nodiscard]] bool free_motion_state(
        const MotionObservation& observation, double raw_z3_delta, double eso_z3) const {
        return observation.command_active
            && observation.measured_velocity >= config_.min_free_motion_velocity
            && likely_free_state(observation, raw_z3_delta)
            && far_from_grounded_reference(eso_z3);
    }

private:
    ReferenceConfig config_;
};

} // namespace rmcs_core::chassis::suspension::contact_confidence
