#pragma once

#include <array>
#include <cstdint>
#include <limits>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_msgs/keyboard.hpp>

#include "controller/joint/joint_controller.hpp"
#include "controller/joint/joint_servo.hpp"

namespace rmcs_core::controller::chassis {

class DeformableSuspensionController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    enum class JointFeedbackSource : uint8_t { kLegacyEncoderAngle, kMotorAngle };

    DeformableSuspensionController();

    void before_updating() override;
    void update() override;
    void reset();

private:
    // ---- Joint feedback adapter ----
    void validate_joint_feedback_() const;
    void read_joint_feedback_(JointController::CycleInput& out) const;

    // ---- IMU calibration ----
    void reset_imu_calibration_();
    void update_imu_calibration_();

    // ---- Output publishing ----
    void clear_suspension_outputs_();
    void configure_servos_();
    void publish_suspension_outputs_(const JointController::CycleOutput& out);
    void publish_joint_targets_(
        const JointController::CycleOutput& out,
        const std::array<double, kJointCount>& current_physical_angles);
    void publish_nan_joint_targets_();

    // ---- Helpers ----
    static double degree_to_radian_(double degree);
    static double wrap_degree_(double degree);
    static double physical_to_motor_angle_(double physical_angle_rad);
    static double motor_to_physical_angle_(double motor_angle_rad);
    static double legacy_encoder_to_physical_angle_(
        double encoder_angle_degree, double joint_offset_degree);
    double compute_dt_() const;
    static bool symmetric_targets_requested_(const std::array<double, kJointCount>& angles);

    // ---- Constants ----
    static constexpr double kQuietNan   = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kRadianToDegree = 180.0 / std::numbers::pi;
    static constexpr double kDefaultDt  = 1e-3;
    static constexpr double kJointZeroPhysicalAngleRadian = 1.090830782496456;

    // ---- Core ----
    JointController joint_controller_;

    // ---- Per-leg ADRC servos (was DeformableJointController ×4) ----
    std::array<JointServo, kJointCount> servos_;
    std::array<double, kJointCount> last_eso_z3_{};

    // ---- Joint feedback input interfaces ----
    InputInterface<double> left_front_joint_angle_;
    InputInterface<double> left_back_joint_angle_;
    InputInterface<double> right_back_joint_angle_;
    InputInterface<double> right_front_joint_angle_;

    InputInterface<double> left_front_joint_physical_angle_;
    InputInterface<double> left_back_joint_physical_angle_;
    InputInterface<double> right_back_joint_physical_angle_;
    InputInterface<double> right_front_joint_physical_angle_;

    InputInterface<double> left_front_joint_physical_velocity_;
    InputInterface<double> left_back_joint_physical_velocity_;
    InputInterface<double> right_back_joint_physical_velocity_;
    InputInterface<double> right_front_joint_physical_velocity_;

    InputInterface<double> left_front_joint_torque_;
    InputInterface<double> left_back_joint_torque_;
    InputInterface<double> right_back_joint_torque_;
    InputInterface<double> right_front_joint_torque_;

    InputInterface<double> left_front_joint_encoder_angle_;
    InputInterface<double> left_back_joint_encoder_angle_;
    InputInterface<double> right_back_joint_encoder_angle_;
    InputInterface<double> right_front_joint_encoder_angle_;

    InputInterface<double> chassis_imu_pitch_;
    InputInterface<double> chassis_imu_roll_;
    InputInterface<double> chassis_imu_pitch_rate_;
    InputInterface<double> chassis_imu_roll_rate_;

    // Chassis intent inputs
    InputInterface<double> left_front_joint_requested_angle_;
    InputInterface<double> left_back_joint_requested_angle_;
    InputInterface<double> right_back_joint_requested_angle_;
    InputInterface<double> right_front_joint_requested_angle_;
    InputInterface<bool> suspension_requested_;
    InputInterface<double> control_acceleration_x_;
    InputInterface<double> control_acceleration_y_;
    InputInterface<double> update_rate_;

    // ---- Output interfaces ----
    OutputInterface<double> left_front_joint_target_angle_;
    OutputInterface<double> left_back_joint_target_angle_;
    OutputInterface<double> right_back_joint_target_angle_;
    OutputInterface<double> right_front_joint_target_angle_;

    OutputInterface<double> left_front_joint_target_physical_angle_;
    OutputInterface<double> left_back_joint_target_physical_angle_;
    OutputInterface<double> right_back_joint_target_physical_angle_;
    OutputInterface<double> right_front_joint_target_physical_angle_;

    OutputInterface<double> left_front_joint_target_physical_velocity_;
    OutputInterface<double> left_back_joint_target_physical_velocity_;
    OutputInterface<double> right_back_joint_target_physical_velocity_;
    OutputInterface<double> right_front_joint_target_physical_velocity_;

    OutputInterface<double> left_front_joint_target_physical_acceleration_;
    OutputInterface<double> left_back_joint_target_physical_acceleration_;
    OutputInterface<double> right_back_joint_target_physical_acceleration_;
    OutputInterface<double> right_front_joint_target_physical_acceleration_;

    OutputInterface<bool> left_front_joint_suspension_mode_;
    OutputInterface<bool> left_back_joint_suspension_mode_;
    OutputInterface<bool> right_back_joint_suspension_mode_;
    OutputInterface<bool> right_front_joint_suspension_mode_;

    OutputInterface<double> left_front_joint_suspension_torque_;
    OutputInterface<double> left_back_joint_suspension_torque_;
    OutputInterface<double> right_back_joint_suspension_torque_;
    OutputInterface<double> right_front_joint_suspension_torque_;

    OutputInterface<double> left_front_joint_control_angle_error_;
    OutputInterface<double> left_back_joint_control_angle_error_;
    OutputInterface<double> right_back_joint_control_angle_error_;
    OutputInterface<double> right_front_joint_control_angle_error_;

    OutputInterface<double> processed_encoder_angle_;

    // ---- ADRC servo outputs (consumed by deformable_infantry_command motors) ----
    OutputInterface<double> left_front_joint_control_torque_;
    OutputInterface<double> left_back_joint_control_torque_;
    OutputInterface<double> right_back_joint_control_torque_;
    OutputInterface<double> right_front_joint_control_torque_;

    // ---- Parameters ----
    double minimum_angle_degree_;
    double maximum_angle_degree_;
    double left_front_joint_offset_degree_;
    double left_back_joint_offset_degree_;
    double right_front_joint_offset_degree_;
    double right_back_joint_offset_degree_;

    JointFeedbackSource joint_feedback_source_ = JointFeedbackSource::kMotorAngle;

    // ---- IMU calibration state ----
    double imu_calibration_wait_time_;
    double imu_calibration_sample_time_;
    double imu_calibration_elapsed_ = 0.0;
    size_t imu_calibration_sample_count_ = 0;
    double imu_pitch_sum_ = 0.0;
    double imu_roll_sum_ = 0.0;
    bool imu_calibration_done_for_window_ = false;
    double imu_pitch_offset_ = 0.0;
    double imu_roll_offset_ = 0.0;

    // ---- Requested target cache (for IMU calibration symmetry detection) ----
    std::array<double, kJointCount> requested_target_physical_angles_rad_{};
};

} // namespace rmcs_core::controller::chassis
