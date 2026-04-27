#include "joint_servo.hpp"

namespace rmcs_core::controller::chassis {

void JointServo::configure(
    double dt, double b0, double kt, const ModeConfig& normal, const ModeConfig& suspension) {
    dt_ = dt;
    b0_ = b0;
    kt_ = kt;
    normal_config_     = normal;
    suspension_config_ = suspension;
    apply_mode_config_(normal_config_);
}

void JointServo::reset(double measurement_angle, double setpoint_angle) {
    td_.reset(setpoint_angle, 0.0);
    eso_.reset(measurement_angle);
    last_u_ = 0.0;
}

bool JointServo::update(const Input& input, Output& output) {
    output.control_torque = kQuietNan;
    output.eso_z2         = kQuietNan;
    output.eso_z3         = kQuietNan;

    if (!std::isfinite(input.measurement_angle) || !std::isfinite(input.setpoint_angle))
        return false;

    if (input.suspension_mode != last_suspension_mode_) {
        apply_mode_config_(active_config_(input.suspension_mode));
        last_suspension_mode_ = input.suspension_mode;
    }

    const auto& cfg = active_config_(input.suspension_mode);

    if (cfg.torque_feedforward_gain != 0.0 && !std::isfinite(input.feedforward_torque))
        return false;

    init_if_needed_(input);

    const auto td_out  = td_.update(input.setpoint_angle);
    const auto eso_out = eso_.update(input.measurement_angle, last_u_);

    const double e1 = td_out.x1 - eso_out.z1;
    const double e2 = td_out.x2 - eso_out.z2;

    double control = kt_ * nlesf_.compute(e1, e2, eso_out.z3, b0_).u;
    if (cfg.torque_feedforward_gain != 0.0)
        control += cfg.torque_feedforward_gain * input.feedforward_torque;

    control = std::clamp(control, cfg.output_min, cfg.output_max);
    if (!std::isfinite(control))
        return false;

    output.control_torque = control;
    output.eso_z2         = eso_out.z2;
    output.eso_z3         = eso_out.z3;
    last_u_               = control;
    return true;
}

void JointServo::apply_mode_config_(const ModeConfig& cfg) {
    td_.set_config(cfg.td);
    eso_.set_config(cfg.eso);
    nlesf_.set_config(cfg.nlesf);
}

const JointServo::ModeConfig& JointServo::active_config_(bool suspension) const {
    return suspension ? suspension_config_ : normal_config_;
}

void JointServo::init_if_needed_(const Input& input) {
    if (initialized_)
        return;
    reset(input.measurement_angle, input.setpoint_angle);
    initialized_ = true;
}

} // namespace rmcs_core::controller::chassis
