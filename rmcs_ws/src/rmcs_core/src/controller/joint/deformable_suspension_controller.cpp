#include "deformable_suspension_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>

#include "controller/joint/joint_controller.hpp"

namespace rmcs_core::controller::chassis {

DeformableSuspensionController::DeformableSuspensionController()
    : Node(
          get_component_name(),
          rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
    , minimum_angle_degree_(get_parameter_or("minimum_angle", 15.0))
    , maximum_angle_degree_(get_parameter_or("maximum_angle", 55.0))
    , left_front_joint_offset_degree_(get_parameter_or("left_front_joint_offset", 0.0))
    , left_back_joint_offset_degree_(get_parameter_or("left_back_joint_offset", 0.0))
    , right_front_joint_offset_degree_(get_parameter_or("right_front_joint_offset", 0.0))
    , right_back_joint_offset_degree_(get_parameter_or("right_back_joint_offset", 0.0))
    , imu_calibration_wait_time_(
          std::max(get_parameter_or("chassis_imu_calibration_wait_s", 2.0), 0.0))
    , imu_calibration_sample_time_(
          std::max(get_parameter_or("chassis_imu_calibration_sample_s", 3.0), 1e-6)) {

    // Detect joint feedback source from parameter presence
    const bool has_left_front_offset  = has_parameter("left_front_joint_offset");
    const bool has_left_back_offset   = has_parameter("left_back_joint_offset");
    const bool has_right_front_offset = has_parameter("right_front_joint_offset");
    const bool has_right_back_offset  = has_parameter("right_back_joint_offset");
    const bool has_any_offset    = has_left_front_offset || has_left_back_offset
                                || has_right_front_offset || has_right_back_offset;
    const bool has_all_offsets   = has_left_front_offset && has_left_back_offset
                                && has_right_front_offset && has_right_back_offset;

    if (has_any_offset && !has_all_offsets)
        throw std::runtime_error(
            "deformable suspension: joint offsets must be configured for all four joints or "
            "removed entirely");

    joint_feedback_source_ =
        has_all_offsets ? JointFeedbackSource::kLegacyEncoderAngle
                        : JointFeedbackSource::kMotorAngle;

    // ---- Register inputs ----
    // Joint motor angles
    register_input("/chassis/left_front_joint/angle", left_front_joint_angle_, false);
    register_input("/chassis/left_back_joint/angle", left_back_joint_angle_, false);
    register_input("/chassis/right_back_joint/angle", right_back_joint_angle_, false);
    register_input("/chassis/right_front_joint/angle", right_front_joint_angle_, false);

    // Joint physical angles
    register_input(
        "/chassis/left_front_joint/physical_angle", left_front_joint_physical_angle_, false);
    register_input(
        "/chassis/left_back_joint/physical_angle", left_back_joint_physical_angle_, false);
    register_input(
        "/chassis/right_back_joint/physical_angle", right_back_joint_physical_angle_, false);
    register_input(
        "/chassis/right_front_joint/physical_angle", right_front_joint_physical_angle_, false);

    // Joint physical velocities
    register_input(
        "/chassis/left_front_joint/physical_velocity", left_front_joint_physical_velocity_, false);
    register_input(
        "/chassis/left_back_joint/physical_velocity", left_back_joint_physical_velocity_, false);
    register_input(
        "/chassis/right_back_joint/physical_velocity", right_back_joint_physical_velocity_, false);
    register_input(
        "/chassis/right_front_joint/physical_velocity", right_front_joint_physical_velocity_, false);

    // Joint torques
    register_input("/chassis/left_front_joint/torque", left_front_joint_torque_, false);
    register_input("/chassis/left_back_joint/torque", left_back_joint_torque_, false);
    register_input("/chassis/right_back_joint/torque", right_back_joint_torque_, false);
    register_input("/chassis/right_front_joint/torque", right_front_joint_torque_, false);

    // Legacy encoder angles
    register_input(
        "/chassis/left_front_joint/encoder_angle", left_front_joint_encoder_angle_, false);
    register_input(
        "/chassis/left_back_joint/encoder_angle", left_back_joint_encoder_angle_, false);
    register_input(
        "/chassis/right_front_joint/encoder_angle", right_front_joint_encoder_angle_, false);
    register_input(
        "/chassis/right_back_joint/encoder_angle", right_back_joint_encoder_angle_, false);

    // IMU
    register_input("/chassis/imu/pitch", chassis_imu_pitch_, false);
    register_input("/chassis/imu/roll", chassis_imu_roll_, false);
    register_input("/chassis/imu/pitch_rate", chassis_imu_pitch_rate_, false);
    register_input("/chassis/imu/roll_rate", chassis_imu_roll_rate_, false);

    // Chassis intent
    register_input(
        "/chassis/left_front_joint/requested_angle", left_front_joint_requested_angle_, false);
    register_input(
        "/chassis/left_back_joint/requested_angle", left_back_joint_requested_angle_, false);
    register_input(
        "/chassis/right_back_joint/requested_angle", right_back_joint_requested_angle_, false);
    register_input(
        "/chassis/right_front_joint/requested_angle", right_front_joint_requested_angle_, false);
    register_input("/chassis/suspension/requested", suspension_requested_, false);
    register_input("/chassis/control_acceleration/x", control_acceleration_x_, false);
    register_input("/chassis/control_acceleration/y", control_acceleration_y_, false);
    register_input("/predefined/update_rate", update_rate_);

    // ---- Register outputs ----
    register_output("/chassis/left_front_joint/target_angle", left_front_joint_target_angle_, kQuietNan);
    register_output("/chassis/left_back_joint/target_angle", left_back_joint_target_angle_, kQuietNan);
    register_output("/chassis/right_back_joint/target_angle", right_back_joint_target_angle_, kQuietNan);
    register_output("/chassis/right_front_joint/target_angle", right_front_joint_target_angle_, kQuietNan);

    register_output(
        "/chassis/left_front_joint/target_physical_angle",
        left_front_joint_target_physical_angle_, kQuietNan);
    register_output(
        "/chassis/left_back_joint/target_physical_angle",
        left_back_joint_target_physical_angle_, kQuietNan);
    register_output(
        "/chassis/right_back_joint/target_physical_angle",
        right_back_joint_target_physical_angle_, kQuietNan);
    register_output(
        "/chassis/right_front_joint/target_physical_angle",
        right_front_joint_target_physical_angle_, kQuietNan);

    register_output(
        "/chassis/left_front_joint/target_physical_velocity",
        left_front_joint_target_physical_velocity_, kQuietNan);
    register_output(
        "/chassis/left_back_joint/target_physical_velocity",
        left_back_joint_target_physical_velocity_, kQuietNan);
    register_output(
        "/chassis/right_back_joint/target_physical_velocity",
        right_back_joint_target_physical_velocity_, kQuietNan);
    register_output(
        "/chassis/right_front_joint/target_physical_velocity",
        right_front_joint_target_physical_velocity_, kQuietNan);

    register_output(
        "/chassis/left_front_joint/target_physical_acceleration",
        left_front_joint_target_physical_acceleration_, kQuietNan);
    register_output(
        "/chassis/left_back_joint/target_physical_acceleration",
        left_back_joint_target_physical_acceleration_, kQuietNan);
    register_output(
        "/chassis/right_back_joint/target_physical_acceleration",
        right_back_joint_target_physical_acceleration_, kQuietNan);
    register_output(
        "/chassis/right_front_joint/target_physical_acceleration",
        right_front_joint_target_physical_acceleration_, kQuietNan);

    register_output(
        "/chassis/left_front_joint/suspension_mode", left_front_joint_suspension_mode_, false);
    register_output(
        "/chassis/left_back_joint/suspension_mode", left_back_joint_suspension_mode_, false);
    register_output(
        "/chassis/right_back_joint/suspension_mode", right_back_joint_suspension_mode_, false);
    register_output(
        "/chassis/right_front_joint/suspension_mode", right_front_joint_suspension_mode_, false);

    register_output(
        "/chassis/left_front_joint/suspension_torque", left_front_joint_suspension_torque_, kQuietNan);
    register_output(
        "/chassis/left_back_joint/suspension_torque", left_back_joint_suspension_torque_, kQuietNan);
    register_output(
        "/chassis/right_back_joint/suspension_torque", right_back_joint_suspension_torque_, kQuietNan);
    register_output(
        "/chassis/right_front_joint/suspension_torque", right_front_joint_suspension_torque_, kQuietNan);

    register_output(
        "/chassis/left_front_joint/control_angle_error", left_front_joint_control_angle_error_, kQuietNan);
    register_output(
        "/chassis/left_back_joint/control_angle_error", left_back_joint_control_angle_error_, kQuietNan);
    register_output(
        "/chassis/right_back_joint/control_angle_error", right_back_joint_control_angle_error_, kQuietNan);
    register_output(
        "/chassis/right_front_joint/control_angle_error", right_front_joint_control_angle_error_, kQuietNan);

    register_output("/chassis/processed_encoder/angle", processed_encoder_angle_, kQuietNan);

    register_output(
        "/chassis/left_front_joint/control_torque", left_front_joint_control_torque_, kQuietNan);
    register_output(
        "/chassis/left_back_joint/control_torque", left_back_joint_control_torque_, kQuietNan);
    register_output(
        "/chassis/right_back_joint/control_torque", right_back_joint_control_torque_, kQuietNan);
    register_output(
        "/chassis/right_front_joint/control_torque", right_front_joint_control_torque_, kQuietNan);

    // ---- Configure JointController ----
    JointController::Config config;
    config.rod_length            = get_parameter_or("active_suspension_rod_length", 0.150);
    config.Kz_linear             = get_parameter_or("active_suspension_Kz_linear", 15000.0);
    config.D_leg_linear          = get_parameter_or("active_suspension_D_leg_linear", 200.0);
    config.gravity_comp_gain     = get_parameter_or("active_suspension_gravity_comp_gain", 1.0);
    config.mass                  = get_parameter_or("active_suspension_mass", 22.5);
    config.torque_limit          = std::abs(get_parameter_or("active_suspension_torque_limit", 80.0));
    config.preload_angle         = std::abs(get_parameter_or(
                                       "active_suspension_preload_angle_deg", 8.0))
                                 * std::numbers::pi / 180.0;
    config.entry_offset          = std::abs(get_parameter_or(
                                       "active_suspension_entry_offset_deg",
                                       get_parameter_or("active_suspension_enter_deploy_tolerance_deg", 1.5)))
                                 * std::numbers::pi / 180.0;
    config.ride_height_offset    = std::abs(get_parameter_or(
                                       "active_suspension_ride_height_offset_deg", 0.0))
                                 * std::numbers::pi / 180.0;
    config.hold_travel           = std::abs(get_parameter_or(
                                       "active_suspension_hold_travel_deg",
                                       get_parameter_or("active_suspension_exit_deploy_tolerance_deg", 3.0)))
                                 * std::numbers::pi / 180.0;
    config.activation_velocity_threshold =
        get_parameter_or("active_suspension_activation_velocity_threshold_deg", 15.0)
        * std::numbers::pi / 180.0;
    config.wheel_base_half_x     = get_parameter_or("active_suspension_wheel_base_half_x",
                                      0.2341741 / std::numbers::sqrt2);
    config.wheel_base_half_y     = get_parameter_or("active_suspension_wheel_base_half_y",
                                      0.2341741 / std::numbers::sqrt2);
    config.com_height            = get_parameter_or("active_suspension_com_height", 0.15);
    config.min_angle = degree_to_radian_(minimum_angle_degree_);
    config.max_angle = degree_to_radian_(maximum_angle_degree_);

    config.attitude_kp_pitch     = get_parameter_or("active_suspension_Kp", 200.0);
    config.attitude_ki_pitch     = get_parameter_or("active_suspension_pitch_ki", 0.0);
    config.attitude_kd_pitch     = get_parameter_or("active_suspension_Dp", 20.0);
    config.attitude_kp_roll      = get_parameter_or("active_suspension_Kr", 200.0);
    config.attitude_ki_roll      = get_parameter_or("active_suspension_roll_ki", 0.0);
    config.attitude_kd_roll      = get_parameter_or("active_suspension_Dr", 20.0);
    config.attitude_integral_limit
        = std::abs(get_parameter_or("active_suspension_pid_integral_limit_deg",
                                    maximum_angle_degree_ - minimum_angle_degree_))
        * std::numbers::pi / 180.0;
    config.attitude_output_limit
        = std::abs(get_parameter_or("active_suspension_pitch_angle_diff_limit_deg",
                                    maximum_angle_degree_ - minimum_angle_degree_))
        * std::numbers::pi / 180.0;

    config.target_velocity_limit = std::max(
        degree_to_radian_(std::abs(get_parameter_or("target_physical_velocity_limit", 180.0))),
        1e-6);
    config.target_acceleration_limit = std::max(
        degree_to_radian_(std::abs(get_parameter_or("target_physical_acceleration_limit", 720.0))),
        1e-6);
    config.suspension_target_velocity_limit = std::max(
        degree_to_radian_(std::abs(get_parameter_or(
            "active_suspension_target_velocity_limit_deg",
            get_parameter_or("target_physical_velocity_limit", 180.0)))), 1e-6);
    config.suspension_target_acceleration_limit = std::max(
        degree_to_radian_(std::abs(get_parameter_or(
            "active_suspension_target_acceleration_limit_deg",
            get_parameter_or("target_physical_acceleration_limit", 720.0)))), 1e-6);

    joint_controller_.configure(config);
    configure_servos_();
}

void DeformableSuspensionController::configure_servos_() {
    const double dt = std::max(get_parameter_or("adrc_dt", 0.001), 1e-9);
    const double b0 = get_parameter_or("adrc_b0", 1.0);
    const double kt = get_parameter_or("adrc_kt", 1.0);

    JointServo::ModeConfig normal;
    normal.td.h       = get_parameter_or("adrc_td_h", dt);
    normal.td.r       = get_parameter_or("adrc_td_r", 300.0);
    normal.td.max_vel = get_parameter_or("adrc_td_max_vel",
        std::numeric_limits<double>::infinity());
    normal.td.max_acc = get_parameter_or("adrc_td_max_acc",
        std::numeric_limits<double>::infinity());
    normal.eso.h         = dt;
    normal.eso.b0        = b0;
    normal.eso.w0        = get_parameter_or("adrc_eso_w0", 80.0);
    normal.eso.auto_beta = get_parameter_or("adrc_eso_auto_beta", true);
    normal.eso.beta1     = get_parameter_or("adrc_eso_beta1", 3.0 * normal.eso.w0);
    normal.eso.beta2     = get_parameter_or("adrc_eso_beta2",
        3.0 * normal.eso.w0 * normal.eso.w0);
    normal.eso.beta3     = get_parameter_or("adrc_eso_beta3",
        normal.eso.w0 * normal.eso.w0 * normal.eso.w0);
    normal.eso.z3_limit  = get_parameter_or("adrc_eso_z3_limit", 1e9);
    normal.nlesf.k1     = get_parameter_or("adrc_k1", 50.0);
    normal.nlesf.k2     = get_parameter_or("adrc_k2", 5.0);
    normal.nlesf.alpha1 = get_parameter_or("adrc_alpha1", 0.75);
    normal.nlesf.alpha2 = get_parameter_or("adrc_alpha2", 1.25);
    normal.nlesf.delta  = get_parameter_or("adrc_delta", 0.01);
    normal.nlesf.u_min  = get_parameter_or("adrc_u_min",
        -std::numeric_limits<double>::infinity());
    normal.nlesf.u_max  = get_parameter_or("adrc_u_max",
        std::numeric_limits<double>::infinity());
    normal.output_min   = get_parameter_or("adrc_output_min",
        -std::numeric_limits<double>::infinity());
    normal.output_max   = get_parameter_or("adrc_output_max",
        std::numeric_limits<double>::infinity());
    if (normal.output_min > normal.output_max)
        std::swap(normal.output_min, normal.output_max);
    normal.torque_feedforward_gain = get_parameter_or("adrc_torque_feedforward_gain", 0.0);

    JointServo::ModeConfig suspension = normal;
    suspension.td.h       = get_parameter_or("adrc_suspension_td_h", suspension.td.h);
    suspension.td.r       = get_parameter_or("adrc_suspension_td_r", suspension.td.r);
    suspension.td.max_vel = get_parameter_or("adrc_suspension_td_max_vel", suspension.td.max_vel);
    suspension.td.max_acc = get_parameter_or("adrc_suspension_td_max_acc", suspension.td.max_acc);
    suspension.eso.w0        = get_parameter_or("adrc_suspension_eso_w0", suspension.eso.w0);
    suspension.eso.auto_beta = get_parameter_or("adrc_suspension_eso_auto_beta",
        suspension.eso.auto_beta);
    suspension.eso.beta1     = get_parameter_or("adrc_suspension_eso_beta1", suspension.eso.beta1);
    suspension.eso.beta2     = get_parameter_or("adrc_suspension_eso_beta2", suspension.eso.beta2);
    suspension.eso.beta3     = get_parameter_or("adrc_suspension_eso_beta3", suspension.eso.beta3);
    suspension.eso.z3_limit  = get_parameter_or("adrc_suspension_eso_z3_limit",
        suspension.eso.z3_limit);
    suspension.nlesf.k1     = get_parameter_or("adrc_suspension_k1", suspension.nlesf.k1);
    suspension.nlesf.k2     = get_parameter_or("adrc_suspension_k2", suspension.nlesf.k2);
    suspension.nlesf.alpha1 = get_parameter_or("adrc_suspension_alpha1",
        suspension.nlesf.alpha1);
    suspension.nlesf.alpha2 = get_parameter_or("adrc_suspension_alpha2",
        suspension.nlesf.alpha2);
    suspension.nlesf.delta  = get_parameter_or("adrc_suspension_delta", suspension.nlesf.delta);
    suspension.nlesf.u_min  = get_parameter_or("adrc_suspension_u_min", suspension.nlesf.u_min);
    suspension.nlesf.u_max  = get_parameter_or("adrc_suspension_u_max", suspension.nlesf.u_max);
    suspension.output_min   = get_parameter_or("adrc_suspension_output_min",
        normal.output_min);
    suspension.output_max   = get_parameter_or("adrc_suspension_output_max",
        normal.output_max);
    if (suspension.output_min > suspension.output_max)
        std::swap(suspension.output_min, suspension.output_max);
    suspension.torque_feedforward_gain = get_parameter_or(
        "adrc_suspension_torque_feedforward_gain", -1.0);

    for (auto& servo : servos_)
        servo.configure(dt, b0, kt, normal, suspension);
}

void DeformableSuspensionController::before_updating() {
    if (!left_front_joint_torque_.ready())
        left_front_joint_torque_.make_and_bind_directly(0.0);
    if (!left_back_joint_torque_.ready())
        left_back_joint_torque_.make_and_bind_directly(0.0);
    if (!right_back_joint_torque_.ready())
        right_back_joint_torque_.make_and_bind_directly(0.0);
    if (!right_front_joint_torque_.ready())
        right_front_joint_torque_.make_and_bind_directly(0.0);
    if (!chassis_imu_pitch_.ready())
        chassis_imu_pitch_.make_and_bind_directly(0.0);
    if (!chassis_imu_roll_.ready())
        chassis_imu_roll_.make_and_bind_directly(0.0);
    if (!chassis_imu_pitch_rate_.ready())
        chassis_imu_pitch_rate_.make_and_bind_directly(0.0);
    if (!chassis_imu_roll_rate_.ready())
        chassis_imu_roll_rate_.make_and_bind_directly(0.0);
    if (!suspension_requested_.ready())
        suspension_requested_.make_and_bind_directly(false);
    validate_joint_feedback_();
}

void DeformableSuspensionController::update() {
    JointController::CycleInput cycle_input;
    read_joint_feedback_(cycle_input);

    cycle_input.imu_pitch       = *chassis_imu_pitch_;
    cycle_input.imu_roll        = *chassis_imu_roll_;
    cycle_input.imu_pitch_rate  = *chassis_imu_pitch_rate_;
    cycle_input.imu_roll_rate   = *chassis_imu_roll_rate_;
    cycle_input.imu_pitch_offset = imu_pitch_offset_;
    cycle_input.imu_roll_offset  = imu_roll_offset_;
    cycle_input.control_acceleration = Eigen::Vector2d(
        *control_acceleration_x_, *control_acceleration_y_);
    cycle_input.suspension_requested = *suspension_requested_;
    cycle_input.dt = compute_dt_();

    // Read requested angles from chassis controller and cache for symmetry detection
    const std::array<double, kJointCount> raw_requests{
        *left_front_joint_requested_angle_,
        *left_back_joint_requested_angle_,
        *right_back_joint_requested_angle_,
        *right_front_joint_requested_angle_,
    };
    for (size_t i = 0; i < kJointCount; ++i)
        requested_target_physical_angles_rad_[i] = degree_to_radian_(raw_requests[i]);
    cycle_input.requested_target_angles = requested_target_physical_angles_rad_;

    // Reject if not all physical angles are finite
    for (double angle : cycle_input.physical_angles)
        if (!std::isfinite(angle)) { publish_nan_joint_targets_(); return; }

    // Feed previous-cycle ESO z3 into contact estimation
    for (size_t i = 0; i < kJointCount; ++i) {
        cycle_input.eso_z2[i] = 0.0;
        cycle_input.eso_z3[i] = last_eso_z3_[i];
    }

    update_imu_calibration_();

    clear_suspension_outputs_();
    auto output = joint_controller_.update(cycle_input);

    publish_suspension_outputs_(output);
    publish_joint_targets_(output, cycle_input.physical_angles);

    // Run per-leg ADRC servos with suspension outputs — skip if chassis is in reset
    const bool all_requests_nan = !std::isfinite(raw_requests[0])
                               && !std::isfinite(raw_requests[1])
                               && !std::isfinite(raw_requests[2])
                               && !std::isfinite(raw_requests[3]);
    const std::array<OutputInterface<double>*, kJointCount> torque_outputs{
        &left_front_joint_control_torque_,
        &left_back_joint_control_torque_,
        &right_back_joint_control_torque_,
        &right_front_joint_control_torque_,
    };
    if (all_requests_nan) {
        for (auto* out : torque_outputs)
            **out = kQuietNan;
        return;
    }
    for (size_t i = 0; i < kJointCount; ++i) {
        JointServo::Input servo_in;
        servo_in.measurement_angle  = cycle_input.physical_angles[i];
        servo_in.setpoint_angle     = output.final_target_angles[i];
        servo_in.feedforward_torque = output.suspension_torque[i];
        servo_in.suspension_mode    = output.suspension_mode[i];

        JointServo::Output servo_out;
        if (servos_[i].update(servo_in, servo_out)) {
            **torque_outputs[i] = servo_out.control_torque;
            last_eso_z3_[i]    = servo_out.eso_z3;
        } else {
            **torque_outputs[i] = kQuietNan;
        }
    }
}

void DeformableSuspensionController::reset() {
    reset_imu_calibration_();
    joint_controller_.reset();
    for (auto& servo : servos_) {
        servo.reset(0.0, 0.0);
    }
    last_eso_z3_.fill(0.0);
    publish_nan_joint_targets_();
}

// -----------------------------------------------------------------------
// Joint feedback
// -----------------------------------------------------------------------

void DeformableSuspensionController::validate_joint_feedback_() const {
    const bool ready =
        joint_feedback_source_ == JointFeedbackSource::kMotorAngle
            ? left_front_joint_angle_.ready() && left_back_joint_angle_.ready()
                  && right_front_joint_angle_.ready() && right_back_joint_angle_.ready()
            : left_front_joint_encoder_angle_.ready()
                  && left_back_joint_encoder_angle_.ready()
                  && right_front_joint_encoder_angle_.ready()
                  && right_back_joint_encoder_angle_.ready();
    if (ready) return;

    throw std::runtime_error(
        joint_feedback_source_ == JointFeedbackSource::kMotorAngle
            ? "missing V2 joint feedback interfaces: expected /chassis/*_joint/angle"
            : "missing legacy joint feedback interfaces: expected /chassis/*_joint/encoder_angle");
}

void DeformableSuspensionController::read_joint_feedback_(
    JointController::CycleInput& output) const {
    output.motor_angles.fill(kQuietNan);
    output.physical_angles.fill(kQuietNan);
    output.physical_velocities.fill(kQuietNan);
    output.joint_torques.fill(kQuietNan);
    output.eso_z2.fill(kQuietNan);
    output.eso_z3.fill(kQuietNan);

    const std::array<const InputInterface<double>*, kJointCount> motor_inputs{
        &left_front_joint_angle_, &left_back_joint_angle_,
        &right_back_joint_angle_, &right_front_joint_angle_};
    const std::array<const InputInterface<double>*, kJointCount> physical_inputs{
        &left_front_joint_physical_angle_, &left_back_joint_physical_angle_,
        &right_back_joint_physical_angle_, &right_front_joint_physical_angle_};
    const std::array<const InputInterface<double>*, kJointCount> encoder_inputs{
        &left_front_joint_encoder_angle_, &left_back_joint_encoder_angle_,
        &right_back_joint_encoder_angle_, &right_front_joint_encoder_angle_};
    const std::array<double, kJointCount> encoder_offsets{
        left_front_joint_offset_degree_, left_back_joint_offset_degree_,
        right_back_joint_offset_degree_, right_front_joint_offset_degree_};
    const std::array<const InputInterface<double>*, kJointCount> velocity_inputs{
        &left_front_joint_physical_velocity_, &left_back_joint_physical_velocity_,
        &right_back_joint_physical_velocity_, &right_front_joint_physical_velocity_};
    const std::array<const InputInterface<double>*, kJointCount> torque_inputs{
        &left_front_joint_torque_, &left_back_joint_torque_,
        &right_back_joint_torque_, &right_front_joint_torque_};

    for (size_t i = 0; i < kJointCount; ++i) {
        if (joint_feedback_source_ == JointFeedbackSource::kMotorAngle
            && motor_inputs[i]->ready() && std::isfinite(*(*motor_inputs[i]))) {
            output.motor_angles[i] = *(*motor_inputs[i]);
            output.physical_angles[i] = motor_to_physical_angle_(output.motor_angles[i]);
        }
        if (joint_feedback_source_ == JointFeedbackSource::kLegacyEncoderAngle
            && encoder_inputs[i]->ready() && std::isfinite(*(*encoder_inputs[i]))) {
            output.physical_angles[i] =
                legacy_encoder_to_physical_angle_(*(*encoder_inputs[i]), encoder_offsets[i]);
            output.motor_angles[i] = physical_to_motor_angle_(output.physical_angles[i]);
        }
        if (physical_inputs[i]->ready() && std::isfinite(*(*physical_inputs[i]))) {
            output.physical_angles[i] = *(*physical_inputs[i]);
            if (!std::isfinite(output.motor_angles[i]))
                output.motor_angles[i] = physical_to_motor_angle_(output.physical_angles[i]);
        }
        if (velocity_inputs[i]->ready() && std::isfinite(*(*velocity_inputs[i])))
            output.physical_velocities[i] = *(*velocity_inputs[i]);
        if (torque_inputs[i]->ready() && std::isfinite(*(*torque_inputs[i])))
            output.joint_torques[i] = *(*torque_inputs[i]);
    }
}

// -----------------------------------------------------------------------
// IMU calibration
// -----------------------------------------------------------------------

void DeformableSuspensionController::reset_imu_calibration_() {
    imu_calibration_elapsed_      = 0.0;
    imu_calibration_sample_count_ = 0;
    imu_pitch_sum_                = 0.0;
    imu_roll_sum_                 = 0.0;
    imu_calibration_done_for_window_ = false;
}

void DeformableSuspensionController::update_imu_calibration_() {
    if (!symmetric_targets_requested_(requested_target_physical_angles_rad_)) {
        reset_imu_calibration_();
        return;
    }
    double pitch = *chassis_imu_pitch_, roll = *chassis_imu_roll_;
    if (!std::isfinite(pitch) || !std::isfinite(roll)) return;

    imu_calibration_elapsed_ += compute_dt_();
    if (imu_calibration_elapsed_ < imu_calibration_wait_time_) return;

    double end_time = imu_calibration_wait_time_ + imu_calibration_sample_time_;
    if (imu_calibration_elapsed_ < end_time) {
        imu_pitch_sum_ += pitch;
        imu_roll_sum_  += roll;
        ++imu_calibration_sample_count_;
        return;
    }
    if (imu_calibration_done_for_window_) return;
    imu_calibration_done_for_window_ = true;

    if (imu_calibration_sample_count_ == 0) {
        RCLCPP_WARN(get_logger(),
                    "[suspension imu calibration] skipped - no valid samples");
        return;
    }
    imu_pitch_offset_ =
        imu_pitch_sum_ / static_cast<double>(imu_calibration_sample_count_);
    imu_roll_offset_  =
        imu_roll_sum_  / static_cast<double>(imu_calibration_sample_count_);
    RCLCPP_INFO(get_logger(),
                "[suspension imu calibration] pitch_offset=% .3f deg  roll_offset=% .3f deg  "
                "(samples=%zu)",
                imu_pitch_offset_ * kRadianToDegree,
                imu_roll_offset_  * kRadianToDegree,
                imu_calibration_sample_count_);
}

// -----------------------------------------------------------------------
// Output publishing
// -----------------------------------------------------------------------

void DeformableSuspensionController::clear_suspension_outputs_() {
    *left_front_joint_suspension_mode_  = false;
    *left_back_joint_suspension_mode_   = false;
    *right_back_joint_suspension_mode_  = false;
    *right_front_joint_suspension_mode_ = false;
    *left_front_joint_suspension_torque_  = kQuietNan;
    *left_back_joint_suspension_torque_   = kQuietNan;
    *right_back_joint_suspension_torque_  = kQuietNan;
    *right_front_joint_suspension_torque_ = kQuietNan;
}

void DeformableSuspensionController::publish_suspension_outputs_(
    const JointController::CycleOutput& output) {
    *left_front_joint_suspension_mode_  = output.suspension_mode[kLeftFront];
    *left_back_joint_suspension_mode_   = output.suspension_mode[kLeftBack];
    *right_back_joint_suspension_mode_  = output.suspension_mode[kRightBack];
    *right_front_joint_suspension_mode_ = output.suspension_mode[kRightFront];
    *left_front_joint_suspension_torque_  = output.suspension_torque[kLeftFront];
    *left_back_joint_suspension_torque_   = output.suspension_torque[kLeftBack];
    *right_back_joint_suspension_torque_  = output.suspension_torque[kRightBack];
    *right_front_joint_suspension_torque_ = output.suspension_torque[kRightFront];
}

void DeformableSuspensionController::publish_joint_targets_(
    const JointController::CycleOutput& output,
    const std::array<double, kJointCount>& current_physical_angles) {

    auto to_motor = [](double physical) {
        return kJointZeroPhysicalAngleRadian - physical;
    };

    *left_front_joint_target_angle_ = to_motor(output.final_target_angles[kLeftFront]);
    *left_back_joint_target_angle_  = to_motor(output.final_target_angles[kLeftBack]);
    *right_back_joint_target_angle_ = to_motor(output.final_target_angles[kRightBack]);
    *right_front_joint_target_angle_= to_motor(output.final_target_angles[kRightFront]);

    *left_front_joint_target_physical_angle_   = output.final_target_angles[kLeftFront];
    *left_back_joint_target_physical_angle_    = output.final_target_angles[kLeftBack];
    *right_back_joint_target_physical_angle_   = output.final_target_angles[kRightBack];
    *right_front_joint_target_physical_angle_  = output.final_target_angles[kRightFront];

    *left_front_joint_target_physical_velocity_ =
        output.target_physical_velocities[kLeftFront];
    *left_back_joint_target_physical_velocity_ =
        output.target_physical_velocities[kLeftBack];
    *right_back_joint_target_physical_velocity_ =
        output.target_physical_velocities[kRightBack];
    *right_front_joint_target_physical_velocity_ =
        output.target_physical_velocities[kRightFront];

    *left_front_joint_target_physical_acceleration_ =
        output.target_physical_accelerations[kLeftFront];
    *left_back_joint_target_physical_acceleration_ =
        output.target_physical_accelerations[kLeftBack];
    *right_back_joint_target_physical_acceleration_ =
        output.target_physical_accelerations[kRightBack];
    *right_front_joint_target_physical_acceleration_ =
        output.target_physical_accelerations[kRightFront];

    *left_front_joint_control_angle_error_ = std::isfinite(current_physical_angles[kLeftFront])
        ? current_physical_angles[kLeftFront] - output.final_target_angles[kLeftFront] : kQuietNan;
    *left_back_joint_control_angle_error_ = std::isfinite(current_physical_angles[kLeftBack])
        ? current_physical_angles[kLeftBack]  - output.final_target_angles[kLeftBack]  : kQuietNan;
    *right_back_joint_control_angle_error_ = std::isfinite(current_physical_angles[kRightBack])
        ? current_physical_angles[kRightBack] - output.final_target_angles[kRightBack] : kQuietNan;
    *right_front_joint_control_angle_error_ = std::isfinite(current_physical_angles[kRightFront])
        ? current_physical_angles[kRightFront]- output.final_target_angles[kRightFront]: kQuietNan;

    bool all_finite = true;
    double sum = 0.0;
    for (double angle : current_physical_angles) {
        if (!std::isfinite(angle)) { all_finite = false; break; }
        sum += angle;
    }
    *processed_encoder_angle_ = all_finite
        ? (kRadianToDegree * sum / static_cast<double>(kJointCount))
        : kQuietNan;
}

void DeformableSuspensionController::publish_nan_joint_targets_() {
    clear_suspension_outputs_();
    *left_front_joint_target_angle_  = kQuietNan;
    *left_back_joint_target_angle_   = kQuietNan;
    *right_back_joint_target_angle_  = kQuietNan;
    *right_front_joint_target_angle_ = kQuietNan;
    *left_front_joint_target_physical_angle_   = kQuietNan;
    *left_back_joint_target_physical_angle_    = kQuietNan;
    *right_back_joint_target_physical_angle_   = kQuietNan;
    *right_front_joint_target_physical_angle_  = kQuietNan;
    *left_front_joint_target_physical_velocity_   = kQuietNan;
    *left_back_joint_target_physical_velocity_    = kQuietNan;
    *right_back_joint_target_physical_velocity_   = kQuietNan;
    *right_front_joint_target_physical_velocity_  = kQuietNan;
    *left_front_joint_target_physical_acceleration_   = kQuietNan;
    *left_back_joint_target_physical_acceleration_    = kQuietNan;
    *right_back_joint_target_physical_acceleration_   = kQuietNan;
    *right_front_joint_target_physical_acceleration_  = kQuietNan;
    *left_front_joint_control_angle_error_ = kQuietNan;
    *left_back_joint_control_angle_error_  = kQuietNan;
    *right_back_joint_control_angle_error_ = kQuietNan;
    *right_front_joint_control_angle_error_= kQuietNan;
    *processed_encoder_angle_ = kQuietNan;
    *left_front_joint_control_torque_  = kQuietNan;
    *left_back_joint_control_torque_   = kQuietNan;
    *right_back_joint_control_torque_  = kQuietNan;
    *right_front_joint_control_torque_ = kQuietNan;
}

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

double DeformableSuspensionController::compute_dt_() const {
    if (update_rate_.ready() && std::isfinite(*update_rate_) && *update_rate_ > 1e-6)
        return 1.0 / *update_rate_;
    return kDefaultDt;
}

double DeformableSuspensionController::degree_to_radian_(double degree) {
    return degree * std::numbers::pi / 180.0;
}

double DeformableSuspensionController::wrap_degree_(double degree) {
    degree = std::fmod(degree, 360.0);
    if (degree >= 180.0) degree -= 360.0;
    if (degree < -180.0) degree += 360.0;
    return degree;
}

double DeformableSuspensionController::physical_to_motor_angle_(double physical_angle_rad) {
    return kJointZeroPhysicalAngleRadian - physical_angle_rad;
}

double DeformableSuspensionController::motor_to_physical_angle_(double motor_angle_rad) {
    return kJointZeroPhysicalAngleRadian - motor_angle_rad;
}

double DeformableSuspensionController::legacy_encoder_to_physical_angle_(
    double encoder_angle_degree, double joint_offset_degree) {
    return degree_to_radian_(
        wrap_degree_(joint_offset_degree) - wrap_degree_(encoder_angle_degree));
}

bool DeformableSuspensionController::symmetric_targets_requested_(
    const std::array<double, kJointCount>& angles) {
    constexpr double epsilon = 1e-6;
    return std::abs(angles[kLeftFront] - angles[kLeftBack]) <= epsilon
        && std::abs(angles[kLeftFront] - angles[kRightBack]) <= epsilon
        && std::abs(angles[kLeftFront] - angles[kRightFront]) <= epsilon;
}

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::DeformableSuspensionController, rmcs_executor::Component)
