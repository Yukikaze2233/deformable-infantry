#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include "motion_model.hpp"
#include "types.hpp"

namespace rmcs_core::chassis::suspension::contact_confidence {

class ReferenceLearner {
public:
    explicit ReferenceLearner(const ReferenceConfig& config)
        : config_(config)
        , motion_model_(config) {}

    [[nodiscard]] const MotionModel& motion_model() const { return motion_model_; }

    void update(const InputSnapshot& input, const MotionObservation& observation) {
        const double reference_center = model_.free_state_z3_samples.empty()
            ? input.eso_z3
            : std::accumulate(model_.free_state_z3_samples.begin(), model_.free_state_z3_samples.end(), 0.0)
                / static_cast<double>(model_.free_state_z3_samples.size());
        const double raw_z3_delta = std::abs(input.eso_z3 - reference_center);

        if (!motion_model_.far_from_grounded_reference(input.eso_z3)) {
            return;
        }
        if (!motion_model_.likely_free_state(observation, raw_z3_delta)) {
            return;
        }

        push_bounded(model_.free_state_z3_samples, input.eso_z3);
        if (motion_model_.free_motion_state(observation, raw_z3_delta, input.eso_z3)) {
            push_bounded(model_.free_state_torque_samples, std::abs(input.joint_torque));
        }
        refresh_reference_model();
    }

    [[nodiscard]] bool z3_ready() const {
        return model_.state == ReferenceState::kReady
            && model_.free_state_z3_samples.size() >= config_.min_free_samples_for_ready
            && std::isfinite(model_.z3_baseline) && std::isfinite(model_.z3_residual_scale)
            && model_.z3_residual_scale >= config_.min_z3_residual_scale;
    }

    [[nodiscard]] bool torque_ready() const {
        return model_.free_state_torque_samples.size() >= config_.min_free_motion_samples_for_torque_ready
            && std::isfinite(model_.torque_baseline);
    }

    [[nodiscard]] const ReferenceModel& model() const { return model_; }

private:
    void push_bounded(std::deque<double>& samples, double value) {
        samples.push_back(value);
        while (samples.size() > config_.free_state_reference_window_size) {
            samples.pop_front();
        }
    }

    [[nodiscard]] double quantile(const std::deque<double>& samples, double quantile_value) const {
        if (samples.empty()) {
            return kNaN;
        }

        std::vector<double> sorted(samples.begin(), samples.end());
        std::sort(sorted.begin(), sorted.end());
        const double clamped_quantile = std::clamp(quantile_value, 0.0, 1.0);
        const std::size_t index = static_cast<std::size_t>(
            std::round(clamped_quantile * static_cast<double>(sorted.size() - 1)));
        return sorted[index];
    }

    void refresh_reference_model() {
        if (model_.free_state_z3_samples.size() < config_.min_free_samples_for_ready) {
            return;
        }

        model_.z3_baseline = std::accumulate(
                                 model_.free_state_z3_samples.begin(), model_.free_state_z3_samples.end(), 0.0)
            / static_cast<double>(model_.free_state_z3_samples.size());

        std::deque<double> z3_residuals;
        z3_residuals.resize(model_.free_state_z3_samples.size());
        for (std::size_t i = 0; i < model_.free_state_z3_samples.size(); ++i) {
            z3_residuals[i] = std::abs(model_.free_state_z3_samples[i] - model_.z3_baseline);
        }
        model_.z3_residual_scale = std::max(
            quantile(z3_residuals, config_.free_state_z3_residual_quantile), config_.min_z3_residual_scale);
        model_.state = ReferenceState::kReady;

        if (model_.free_state_torque_samples.size() < config_.min_free_motion_samples_for_torque_ready) {
            return;
        }

        model_.torque_baseline = quantile(model_.free_state_torque_samples, 0.5);
    }

    ReferenceConfig config_;
    MotionModel motion_model_;
    ReferenceModel model_;
};

} // namespace rmcs_core::chassis::suspension::contact_confidence
