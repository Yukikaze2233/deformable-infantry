#include "joint_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rmcs_core::controller::chassis {

namespace {
constexpr double kGravity                     = 9.81;
constexpr double kMaxAttitudeRad              = 30.0 * std::numbers::pi / 180.0;
constexpr double kMinForceArmSin              = 0.1;
constexpr double kContactConfidenceEnterThr   = 0.55;
constexpr double kContactConfidenceExitThr    = 0.35;
constexpr double kMinimumArmingTime           = 0.02;

constexpr std::array<double, kJointCount> kPitchSigns{-1.0, 1.0, 1.0, -1.0};
constexpr std::array<double, kJointCount> kRollSigns{1.0, 1.0, -1.0, -1.0};
}

double JointController::AttitudePidAxis::update(double error, double rate, double dt) {
    if (!std::isfinite(error) || !std::isfinite(rate) || !std::isfinite(dt) || dt <= 0.0) {
        reset();
        return std::numeric_limits<double>::quiet_NaN();
    }
    integral = std::clamp(integral + error * dt, -integral_limit, integral_limit);
    return std::clamp(kp * error + ki * integral - kd * rate, -output_limit, output_limit);
}

void JointController::configure(const Config& config) {
    config_     = config;
    configured_ = true;
    target_active = false;

    pitch_pid_.kp = config.attitude_kp_pitch;
    pitch_pid_.ki = config.attitude_ki_pitch;
    pitch_pid_.kd = config.attitude_kd_pitch;
    pitch_pid_.integral_limit = config.attitude_integral_limit;
    pitch_pid_.output_limit   = config.attitude_output_limit;
    roll_pid_.kp = config.attitude_kp_roll;
    roll_pid_.ki = config.attitude_ki_roll;
    roll_pid_.kd = config.attitude_kd_roll;
    roll_pid_.integral_limit = config.attitude_integral_limit;
    roll_pid_.output_limit   = config.attitude_output_limit;
    pitch_pid_.reset();
    roll_pid_.reset();

    target_physical_angle_state.fill(config.min_angle);
    target_physical_velocity_state.fill(0.0);
    target_physical_acceleration_state.fill(0.0);
    requested_target_angles.fill(config.min_angle);
    leg_states_   = {};
    leg_commands_ = {};
    suspension_active_.fill(false);

    JointState::Config sc;
    sc.entry_offset       = config.entry_offset;
    sc.ride_height_offset = config.ride_height_offset;
    sc.hold_travel        = config.hold_travel;
    sc.velocity_threshold = config.activation_velocity_threshold;
    sc.min_arming_time    = kMinimumArmingTime;
    sc.deploy_angle       = config.min_angle;
    sc.max_angle          = config.max_angle;
    joint_state_.configure(sc);
}

void JointController::reset() {
    target_active = false;
    pitch_pid_.reset();
    roll_pid_.reset();
    leg_states_   = {};
    leg_commands_ = {};
    suspension_active_.fill(false);
    target_physical_velocity_state.fill(0.0);
    target_physical_acceleration_state.fill(0.0);
    if (configured_)
        requested_target_angles.fill(config_.min_angle);
}

JointController::WheelCartesianState JointController::compute_wheel_cartesian(
    const std::array<double, kJointCount>& angles,
    const std::array<double, kJointCount>& velocities) const {

    WheelCartesianState ws;
    ws.z.fill(std::numeric_limits<double>::quiet_NaN());
    ws.z_dot.fill(std::numeric_limits<double>::quiet_NaN());
    ws.chassis_z = std::numeric_limits<double>::quiet_NaN();
    ws.chassis_pitch = std::numeric_limits<double>::quiet_NaN();
    ws.chassis_roll  = std::numeric_limits<double>::quiet_NaN();

    for (size_t i = 0; i < kJointCount; ++i) {
        if (!std::isfinite(angles[i]) || !std::isfinite(velocities[i])) return ws;
        ws.z[i]     = -config_.rod_length * std::cos(angles[i]);
        ws.z_dot[i] =  config_.rod_length * std::sin(angles[i]) * velocities[i];
    }

    double z_front = (ws.z[kLeftFront] + ws.z[kRightFront]) * 0.5;
    double z_back  = (ws.z[kLeftBack]  + ws.z[kRightBack])  * 0.5;
    double z_left  = (ws.z[kLeftFront] + ws.z[kLeftBack])   * 0.5;
    double z_right = (ws.z[kRightFront] + ws.z[kRightBack])  * 0.5;
    double hx = std::max(config_.wheel_base_half_x, 1e-6);
    double hy = std::max(config_.wheel_base_half_y, 1e-6);

    double sum = 0;
    for (double v : ws.z) sum += v;
    ws.chassis_z     = sum / static_cast<double>(kJointCount);
    ws.chassis_pitch = (z_back - z_front) / (2 * hx);
    ws.chassis_roll  = (z_right - z_left) / (2 * hy);
    return ws;
}

std::array<double, kJointCount> JointController::compute_attitude_angle_corrections(const CycleInput& input) {
    std::array<double, kJointCount> corrections{};
    double cp = std::clamp(input.imu_pitch - input.imu_pitch_offset, -kMaxAttitudeRad, kMaxAttitudeRad);
    double cr = std::clamp(input.imu_roll  - input.imu_roll_offset,  -kMaxAttitudeRad, kMaxAttitudeRad);
    double pf = pitch_pid_.update(-cp, input.imu_pitch_rate, input.dt);
    double rf = roll_pid_.update(cr, -input.imu_roll_rate, input.dt);
    if (!std::isfinite(pf) || !std::isfinite(rf)) {
        corrections.fill(std::numeric_limits<double>::quiet_NaN());
        return corrections;
    }
    for (size_t i = 0; i < kJointCount; ++i)
        corrections[i] = -kPitchSigns[i] * pf - kRollSigns[i] * rf;
    return corrections;
}

void JointController::compute_support_forces(
    std::array<double, kJointCount>& out,
    const WheelCartesianState& ws,
    double z_ref,
    const Eigen::Vector2d& accel_est) const {

    for (size_t i = 0; i < kJointCount; ++i) {
        if (!std::isfinite(ws.z[i]) || !std::isfinite(ws.z_dot[i])) {
            out.fill(std::numeric_limits<double>::quiet_NaN());
            return;
        }
    }
    double grav = config_.gravity_comp_gain * config_.mass * kGravity
                / static_cast<double>(kJointCount);
    for (size_t i = 0; i < kJointCount; ++i) {
        double spring  = config_.Kz_linear * (ws.z[i] - z_ref);
        double damping = config_.D_leg_linear * ws.z_dot[i];
        double accel   = 0;
        if (config_.com_height > 0 && config_.wheel_base_half_x > 1e-6 && config_.wheel_base_half_y > 1e-6) {
            accel  = kPitchSigns[i] * config_.mass * accel_est.x() * config_.com_height
                   / (4 * config_.wheel_base_half_x);
            accel += kRollSigns[i]  * config_.mass * accel_est.y() * config_.com_height
                   / (4 * config_.wheel_base_half_y);
        }
        out[i] = std::max(grav + spring + damping + accel, 0.0);
    }
}

double JointController::force_to_torque(double force, double angle) const {
    double sa = std::max(std::sin(angle), kMinForceArmSin);
    return std::clamp(force * config_.rod_length * sa, -config_.torque_limit, config_.torque_limit);
}

double JointController::estimate_contact(const CycleInput& input, size_t index) const {
    double c = 1.0;
    if (std::isfinite(input.eso_z3[index]))
        c -= std::clamp(std::abs(input.eso_z3[index]) / 80.0, 0.0, 0.5);
    if (std::isfinite(input.joint_torques[index]))
        c += std::clamp(std::abs(input.joint_torques[index]) / 20.0, 0.0, 0.3);
    if (std::isfinite(input.physical_velocities[index]))
        c -= std::clamp(std::abs(input.physical_velocities[index]) / 10.0, 0.0, 0.2);
    return std::clamp(c, 0.0, 1.0);
}

void JointController::update_contact_estimates(const CycleInput& input) {
    for (size_t i = 0; i < kJointCount; ++i) {
        auto& ls = leg_states_[i];
        double confidence = estimate_contact(input, i);
        ls.contact_latched = confidence >= kContactConfidenceEnterThr
                           ? true
                           : (confidence <= kContactConfidenceExitThr ? false : ls.contact_latched);
    }
}

void JointController::update_target_trajectory(double dt) {
    if (!std::isfinite(dt) || dt <= 0.0)
        dt = 0.001;

    for (size_t i = 0; i < kJointCount; ++i) {
        double& ang = target_physical_angle_state[i];
        double& vel = target_physical_velocity_state[i];
        double& acc = target_physical_acceleration_state[i];
        double target = leg_commands_[i].final_target_angle;
        double vl = suspension_active_[i] ? config_.suspension_target_velocity_limit
                                          : config_.target_velocity_limit;
        double al = suspension_active_[i] ? config_.suspension_target_acceleration_limit
                                          : config_.target_acceleration_limit;
        vl = std::max(std::abs(vl), 1e-6);
        al = std::max(std::abs(al), 1e-6);
        if (!std::isfinite(target)) {
            vel = 0.0;
            acc = 0.0;
            continue;
        }
        double pos_err = target - ang;

        double des_vel =
            (pos_err > 0.0 ? 1.0 : -1.0) * std::min(std::sqrt(2.0 * al * std::abs(pos_err)), vl);
        acc = std::clamp((des_vel - vel) / dt, -al, al);
        vel = std::clamp(vel + acc * dt, -vl, vl);
        ang += vel * dt;

        double next_err = target - ang;
        if ((pos_err > 0 && next_err < 0) || (pos_err < 0 && next_err > 0)
            || (std::abs(next_err) < 1e-5 && std::abs(vel) < 1e-3)) {
            ang = target; vel = 0; acc = 0;
        }

        leg_commands_[i].target_velocity = vel;
        leg_commands_[i].target_acceleration = acc;
    }
}

JointController::CycleOutput JointController::update(const CycleInput& input) {
    CycleOutput output{};
    output.final_target_angles.fill(std::numeric_limits<double>::quiet_NaN());
    output.target_physical_velocities.fill(std::numeric_limits<double>::quiet_NaN());
    output.target_physical_accelerations.fill(std::numeric_limits<double>::quiet_NaN());
    output.suspension_torque.fill(std::numeric_limits<double>::quiet_NaN());
    output.support_forces.fill(std::numeric_limits<double>::quiet_NaN());
    output.contact_confidences.fill(std::numeric_limits<double>::quiet_NaN());

    if (!configured_) return output;

    if (!target_active) {
        bool all_finite = true;
        for (size_t i = 0; i < kJointCount; ++i)
            if (!std::isfinite(input.motor_angles[i]) || !std::isfinite(input.physical_angles[i]))
            { all_finite = false; break; }
        if (all_finite) {
            for (size_t i = 0; i < kJointCount; ++i) {
                target_physical_angle_state[i] = input.physical_angles[i];
                target_physical_velocity_state[i] = 0.0;
                target_physical_acceleration_state[i] = 0.0;
            }
            target_active = true;
        } else return output;
    }
    requested_target_angles = input.requested_target_angles;

    for (size_t i = 0; i < kJointCount; ++i) {
        leg_commands_[i] = LegCommand{};
        leg_commands_[i].requested_target_angle = requested_target_angles[i];
        leg_commands_[i].final_target_angle = requested_target_angles[i];
    }

    if (!input.suspension_requested) {
        pitch_pid_.reset();
        roll_pid_.reset();
        leg_states_ = {};
        suspension_active_.fill(false);
        update_target_trajectory(input.dt);
    } else {
        double deploy  = config_.min_angle;
        double ride    = std::clamp(deploy + config_.ride_height_offset, deploy, config_.max_angle);
        double z_ref   = -config_.rod_length * std::cos(deploy - config_.preload_angle);

        std::array<double, kJointCount> attitude_corrections = compute_attitude_angle_corrections(input);
        if (!std::isfinite(attitude_corrections[0])) {
            pitch_pid_.reset();
            roll_pid_.reset();
            leg_states_ = {};
            suspension_active_.fill(false);
            for (size_t i = 0; i < kJointCount; ++i)
                leg_commands_[i].final_target_angle = deploy;
            update_target_trajectory(input.dt);
        } else {
            update_contact_estimates(input);

            for (size_t i = 0; i < kJointCount; ++i) {
                JointState::PerLegInput state_input;
                state_input.physical_angle    = input.physical_angles[i];
                state_input.physical_velocity = input.physical_velocities[i];
                state_input.motor_angle       = input.motor_angles[i];
                state_input.deploy_requested  = std::isfinite(requested_target_angles[i])
                                             && requested_target_angles[i] <= deploy + config_.entry_offset;
                state_input.contact_ready     = leg_states_[i].contact_latched;
                state_input.dt                = input.dt;
                joint_state_.update(i, state_input, leg_states_[i]);
                suspension_active_[i] = leg_states_[i].active;
            }

            WheelCartesianState ws =
                compute_wheel_cartesian(input.physical_angles, input.physical_velocities);
            std::array<double, kJointCount> support_forces{};
            compute_support_forces(support_forces, ws, z_ref, input.control_acceleration);

            for (size_t i = 0; i < kJointCount; ++i) {
                output.support_forces[i] = support_forces[i];
                if (!leg_states_[i].active) continue;
                if (!std::isfinite(support_forces[i]) || !std::isfinite(input.physical_angles[i]))
                    continue;
                double corrected_ride = std::clamp(
                    ride + attitude_corrections[i], deploy, config_.max_angle);
                leg_commands_[i].final_target_angle = corrected_ride;
                leg_commands_[i].suspension_mode    = true;
                leg_commands_[i].suspension_torque =
                    force_to_torque(support_forces[i], input.physical_angles[i]);
            }

            update_target_trajectory(input.dt);
        }
    }
    for (size_t i = 0; i < kJointCount; ++i) {
        output.final_target_angles[i] = target_physical_angle_state[i];
        output.target_physical_velocities[i] = target_physical_velocity_state[i];
        output.target_physical_accelerations[i] = target_physical_acceleration_state[i];
        output.suspension_mode[i] = leg_commands_[i].suspension_mode;
        output.suspension_torque[i] = leg_commands_[i].suspension_torque;
        output.contact_confidences[i] = leg_states_[i].contact_latched ? 1.0 : 0.0;
        output.leg_phases[i] = leg_states_[i].phase;
    }
    return output;
}

} // namespace rmcs_core::controller::chassis
