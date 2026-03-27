#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace rmcs_core::controller::kalman {

class KalmanCalculator {
public:
    KalmanCalculator() { reset(); }

    KalmanCalculator(double process_noise, double measurement_noise, double initial_covariance = 1.0)
        : process_noise_(std::max(process_noise, kMinPositive))
        , measurement_noise_(std::max(measurement_noise, kMinPositive))
        , initial_covariance_(std::max(initial_covariance, kMinPositive)) {
        reset();
    }

    void reset() {
        estimate_ = kNan;
        covariance_ = initial_covariance_;
        is_initialized_ = false;
    }

    void reset(double initial_estimate, double covariance = kNan) {
        estimate_ = initial_estimate;
        if (std::isfinite(covariance) && covariance > kMinPositive) {
            covariance_ = covariance;
        } else {
            covariance_ = initial_covariance_;
        }
        is_initialized_ = std::isfinite(initial_estimate);
    }

    double update(double measurement) {
        if (!std::isfinite(measurement)) {
            return kNan;
        }

        if (!is_initialized_) {
            estimate_ = measurement;
            covariance_ = initial_covariance_;
            is_initialized_ = true;
            return estimate_;
        }

        covariance_ = std::max(covariance_ + process_noise_, kMinPositive);
        const double kalman_gain = covariance_ / (covariance_ + measurement_noise_);
        estimate_ += kalman_gain * (measurement - estimate_);
        covariance_ = std::max((1.0 - kalman_gain) * covariance_, kMinPositive);
        return estimate_;
    }

    void set_process_noise(double process_noise) {
        process_noise_ = std::max(process_noise, kMinPositive);
    }

    void set_measurement_noise(double measurement_noise) {
        measurement_noise_ = std::max(measurement_noise, kMinPositive);
    }

    [[nodiscard]] double state() const { return estimate_; }
    [[nodiscard]] double covariance() const { return covariance_; }

private:
    static constexpr double kMinPositive = 1e-9;
    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

    double process_noise_ = 1e-3;
    double measurement_noise_ = 1e-2;
    double initial_covariance_ = 1.0;

    double estimate_ = kNan;
    double covariance_ = 1.0;
    bool is_initialized_ = false;
};

} // namespace rmcs_core::controller::kalman
