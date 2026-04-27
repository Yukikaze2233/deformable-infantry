#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "controller/adrc/ESO.hpp"
#include "controller/adrc/NLESF.hpp"
#include "controller/adrc/TD.hpp"

namespace rmcs_core::controller::chassis {

/// Lightweight, non-ROS ADRC joint servo extracted from DeformableJointController.
/// Manages TD-ESO-NLESF state for a single joint, supporting dual-mode
/// (normal tracking / suspension compliance) with on-the-fly config switching.
class JointServo {
public:
    struct ModeConfig {
        rmcs_core::controller::adrc::TD::Config td;
        rmcs_core::controller::adrc::ESO::Config eso;
        rmcs_core::controller::adrc::NLESF::Config nlesf;
        double output_min = -std::numeric_limits<double>::infinity();
        double output_max = std::numeric_limits<double>::infinity();
        double torque_feedforward_gain = 0.0;
    };

    struct Input {
        double measurement_angle = std::numeric_limits<double>::quiet_NaN();
        double setpoint_angle    = std::numeric_limits<double>::quiet_NaN();
        double feedforward_torque = 0.0;   ///< suspension_torque or other external feedforward
        bool   suspension_mode   = false;
    };

    struct Output {
        double control_torque = std::numeric_limits<double>::quiet_NaN();
        double eso_z2         = std::numeric_limits<double>::quiet_NaN();
        double eso_z3         = std::numeric_limits<double>::quiet_NaN();
    };

    JointServo() = default;

    /// One-time configuration.  Copies are stored internally.
    void configure(
        double dt, double b0, double kt, const ModeConfig& normal, const ModeConfig& suspension);

    /// Reset TD / ESO state (call on first valid frame or after disable).
    void reset(double measurement_angle, double setpoint_angle);

    /// Single-cycle update.  Returns false when the output is invalid (NaN).
    bool update(const Input& input, Output& output);

private:
    void apply_mode_config_(const ModeConfig& cfg);
    const ModeConfig& active_config_(bool suspension) const;
    void init_if_needed_(const Input& input);

    static constexpr double kQuietNan = std::numeric_limits<double>::quiet_NaN();

    double dt_ = 0.001;
    double b0_ = 1.0;
    double kt_ = 1.0;

    ModeConfig normal_config_;
    ModeConfig suspension_config_;

    rmcs_core::controller::adrc::TD    td_;
    rmcs_core::controller::adrc::ESO   eso_;
    rmcs_core::controller::adrc::NLESF nlesf_;

    bool initialized_          = false;
    bool last_suspension_mode_ = false;
    double last_u_             = 0.0;
};

} // namespace rmcs_core::controller::chassis
