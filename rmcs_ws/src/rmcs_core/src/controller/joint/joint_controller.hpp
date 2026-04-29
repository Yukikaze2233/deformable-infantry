#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <eigen3/Eigen/Dense>

#include "joint_state.hpp"

namespace rmcs_core::controller::chassis {

class JointController {
public:
    struct Config {
        double rod_length = 0;
        double Kz_linear = 0;
        double D_leg_linear = 0;
        double gravity_comp_gain = 0;
        double mass = 0;
        double torque_limit = 0;
        double preload_angle = 0;
        double entry_offset = 0;
        double ride_height_offset = 0;
        double hold_travel = 0;
        double activation_velocity_threshold = 0;
        double wheel_base_half_x = 0;
        double wheel_base_half_y = 0;
        double com_height = 0;
        double min_angle = 0;
        double max_angle = 0;

        double attitude_kp_pitch = 0;
        double attitude_ki_pitch = 0;
        double attitude_kd_pitch = 0;
        double attitude_kp_roll = 0;
        double attitude_ki_roll = 0;
        double attitude_kd_roll = 0;
        double attitude_integral_limit = 0;
        double attitude_output_limit = 0;

        double target_velocity_limit = 0;
        double target_acceleration_limit = 0;
        double suspension_target_velocity_limit = 0;
        double suspension_target_acceleration_limit = 0;
    };

    struct CycleInput {
        std::array<double, kJointCount> physical_angles{};
        std::array<double, kJointCount> physical_velocities{};
        std::array<double, kJointCount> joint_torques{};
        std::array<double, kJointCount> eso_z2{};
        std::array<double, kJointCount> eso_z3{};
        std::array<double, kJointCount> motor_angles{};

        double imu_pitch        = 0.0;
        double imu_roll         = 0.0;
        double imu_pitch_rate   = 0.0;
        double imu_roll_rate    = 0.0;
        double imu_pitch_offset = 0.0;
        double imu_roll_offset  = 0.0;

        Eigen::Vector2d control_acceleration = Eigen::Vector2d::Zero();

        std::array<double, kJointCount> requested_target_angles{};
        bool suspension_requested = false;
        double dt                 = 0.001;
    };

    struct CycleOutput {
        std::array<double, kJointCount> final_target_angles{};
        std::array<double, kJointCount> target_physical_velocities{};
        std::array<double, kJointCount> target_physical_accelerations{};
        std::array<bool, kJointCount> suspension_mode{};
        std::array<double, kJointCount> suspension_torque{};

        std::array<double, kJointCount> support_forces{};
        std::array<double, kJointCount> contact_confidences{};
        std::array<SuspensionPhase, kJointCount> leg_phases{};
    };

    JointController() = default;
    void configure(const Config& config);
    void reset();
    CycleOutput update(const CycleInput& input);

private:
    struct LegCommand {
        double requested_target_angle = std::numeric_limits<double>::quiet_NaN();
        double final_target_angle     = std::numeric_limits<double>::quiet_NaN();
        double target_velocity        = 0.0;
        double target_acceleration    = 0.0;
        bool suspension_mode          = false;
        double suspension_torque      = std::numeric_limits<double>::quiet_NaN();
    };

    struct AttitudePidAxis {
        double kp            = 20.0;
        double ki            = 0.0;
        double kd            = 0.0;
        double integral      = 0.0;
        double integral_limit = std::numeric_limits<double>::infinity();
        double output_limit   = std::numeric_limits<double>::infinity();

        void reset() { integral = 0.0; }
        double update(double error, double rate, double dt);
    };

    struct WheelCartesianState {
        std::array<double, kJointCount> z{};
        std::array<double, kJointCount> z_dot{};
        double chassis_z     = 0.0;
        double chassis_pitch = 0.0;
        double chassis_roll  = 0.0;
    };

    WheelCartesianState compute_wheel_cartesian(
        const std::array<double, kJointCount>& angles,
        const std::array<double, kJointCount>& velocities) const;

    std::array<double, kJointCount> compute_attitude_angle_corrections(const CycleInput& input);

    void compute_support_forces(
        std::array<double, kJointCount>& out,
        const WheelCartesianState& ws,
        double z_ref,
        const Eigen::Vector2d& accel_est) const;

    double force_to_torque(double force, double angle) const;

    double estimate_contact(const CycleInput& input, size_t index) const;
    void update_contact_estimates(const CycleInput& input);
    void update_target_trajectory(double dt);

    Config config_;
    bool configured_ = false;

    JointState joint_state_;
    AttitudePidAxis pitch_pid_, roll_pid_;

    std::array<JointState::PerLegState, kJointCount> leg_states_{};
    std::array<LegCommand, kJointCount> leg_commands_{};
    std::array<bool, kJointCount> suspension_active_{};

    std::array<double, kJointCount> target_physical_angle_state{};
    std::array<double, kJointCount> target_physical_velocity_state{};
    std::array<double, kJointCount> target_physical_acceleration_state{};
    bool target_active = false;
    std::array<double, kJointCount> requested_target_angles{};
};

} // namespace rmcs_core::controller::chassis
