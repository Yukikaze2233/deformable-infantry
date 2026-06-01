#pragma once

#include <algorithm>
#include <cmath>

#include "reference_model.hpp"
#include "types.hpp"

namespace rmcs_core::chassis::suspension::contact_confidence {

class ScoreCalculator {
public:
    explicit ScoreCalculator(const FusionConfig& config)
        : config_(config) {}

    [[nodiscard]] FeatureScores compute(
        const InputSnapshot& input, const MotionObservation& observation,
        const ReferenceLearner& learner) const {
        FeatureScores scores;
        const auto& model = learner.model();
        const double raw_z3_delta = std::abs(input.eso_z3 - model.z3_baseline);
        const double torque_delta = std::max(0.0, std::abs(input.joint_torque) - model.torque_baseline);

        scores.z3 = normalized_z3_score(raw_z3_delta, model.z3_residual_scale);
        scores.block = observation.block_score;
        scores.tracking = observation.tracking_score;
        scores.torque = learner.torque_ready() && observation.command_active
            ? normalized_activation(torque_delta, config_.torque_activation_threshold)
            : 0.0;
        return scores;
    }

    [[nodiscard]] double entry_confidence(const FeatureScores& scores) const {
        if (!std::isfinite(scores.z3) || !std::isfinite(scores.block)
            || !std::isfinite(scores.tracking) || !std::isfinite(scores.torque)) {
            return kNaN;
        }

        const double raw = config_.block_weight * scores.block + config_.tracking_weight * scores.tracking;
        const double weight_sum = config_.block_weight + config_.tracking_weight;
        if (weight_sum <= kMinDenominator) {
            return 0.0;
        }
        return std::clamp(raw / weight_sum, 0.0, 1.0);
    }

    [[nodiscard]] double hold_confidence(const FeatureScores& scores, const ReferenceLearner& learner) const {
        if (!std::isfinite(scores.z3) || !std::isfinite(scores.block)
            || !std::isfinite(scores.tracking) || !std::isfinite(scores.torque)) {
            return kNaN;
        }

        double effective_z3_weight = config_.z3_weight;
        double effective_torque_weight = config_.torque_weight;
        if (learner.z3_ready() && !learner.torque_ready()) {
            effective_z3_weight += config_.torque_weight;
            effective_torque_weight = 0.0;
        } else if (!learner.z3_ready() && learner.torque_ready()) {
            effective_torque_weight += config_.z3_weight;
            effective_z3_weight = 0.0;
        }

        const double raw = effective_z3_weight * scores.z3 + effective_torque_weight * scores.torque;
        const double weight_sum = effective_z3_weight + effective_torque_weight;
        if (weight_sum <= kMinDenominator) {
            return 0.0;
        }
        return std::clamp(raw / weight_sum, 0.0, 1.0);
    }

private:
    [[nodiscard]] double normalized_z3_score(double raw_z3_delta, double z3_residual_scale) const {
        if (!std::isfinite(raw_z3_delta) || !std::isfinite(z3_residual_scale)
            || z3_residual_scale < kMinResidualScale) {
            return kNaN;
        }
        return z3_score_from_strength(raw_z3_delta / z3_residual_scale);
    }

    [[nodiscard]] double z3_score_from_strength(double normalized_strength) const {
        if (!std::isfinite(normalized_strength)
            || !std::isfinite(config_.normalized_z3_strength_presence_threshold)
            || !std::isfinite(config_.normalized_z3_strength_presence_score)
            || !std::isfinite(config_.normalized_z3_strength_full_scale_threshold)
            || config_.normalized_z3_strength_presence_threshold < 0.0
            || config_.normalized_z3_strength_full_scale_threshold <= 0.0
            || config_.normalized_z3_strength_presence_threshold
                > config_.normalized_z3_strength_full_scale_threshold) {
            return kNaN;
        }

        if (normalized_strength < config_.normalized_z3_strength_presence_threshold) {
            return 0.0;
        }

        const double clamped_presence_score =
            std::clamp(config_.normalized_z3_strength_presence_score, 0.0, 1.0);
        const double excess_score = normalized_activation(
            normalized_strength - config_.normalized_z3_strength_presence_threshold,
            std::max(
                config_.normalized_z3_strength_full_scale_threshold
                    - config_.normalized_z3_strength_presence_threshold,
                kMinDenominator));
        return std::clamp(
            clamped_presence_score + (1.0 - clamped_presence_score) * excess_score, 0.0, 1.0);
    }

    FusionConfig config_;
};

class ConfidenceFilter {
public:
    explicit ConfidenceFilter(const FilterConfig& config)
        : config_(config) {}

    [[nodiscard]] double update(double confidence_raw) {
        if (!std::isfinite(confidence_raw)) {
            filtered_confidence_ = kNaN;
            return filtered_confidence_;
        }

        if (!std::isfinite(filtered_confidence_)) {
            filtered_confidence_ = confidence_raw;
            return filtered_confidence_;
        }

        const double rate = confidence_raw >= filtered_confidence_
            ? config_.confidence_rise_rate
            : config_.confidence_fall_rate;
        const double alpha = std::clamp(rate * config_.update_dt, 0.0, 1.0);
        filtered_confidence_ = blend(filtered_confidence_, confidence_raw, alpha);
        filtered_confidence_ = std::clamp(filtered_confidence_, 0.0, 1.0);
        return filtered_confidence_;
    }

    void reset() { filtered_confidence_ = kNaN; }

private:
    FilterConfig config_;
    double filtered_confidence_ = kNaN;
};

class ConfidencePhaseLogic {
public:
    explicit ConfidencePhaseLogic(const FusionConfig& config)
        : config_(config) {}

    [[nodiscard]] double compose(double entry_confidence, double hold_confidence) {
        if (!std::isfinite(entry_confidence) || !std::isfinite(hold_confidence)) {
            hold_active_ = false;
            return kNaN;
        }

        if (!hold_active_) {
            if (entry_confidence >= config_.entry_activate_threshold
                || hold_confidence >= config_.hold_activate_threshold) {
                hold_active_ = true;
            }
        } else if (hold_confidence < config_.hold_deactivate_threshold) {
            hold_active_ = false;
        }

        return hold_active_ ? std::max(hold_confidence, entry_confidence) : entry_confidence;
    }

    void reset() { hold_active_ = false; }

private:
    FusionConfig config_;
    bool hold_active_ = false;
};

} // namespace rmcs_core::chassis::suspension::contact_confidence
