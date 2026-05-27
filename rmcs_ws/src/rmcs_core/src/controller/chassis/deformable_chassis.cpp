#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <numbers>
#include <stdexcept>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class DeformableChassis
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    enum class JointFeedbackSource : uint8_t { kLegacyEncoderAngle, kMotorAngle };
    enum JointIndex : size_t {
        kLeftFront = 0,
        kLeftBack = 1,
        kRightBack = 2,
        kRightFront = 3,
        kJointCount = 4,
    };

    struct JointFeedbackFrame {
        std::array<double, kJointCount> motor_angles{};
        std::array<double, kJointCount> physical_angles{};
    };

    DeformableChassis()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , following_velocity_controller_(10.0, 0.0, 0.0)
        , spin_ratio_(std::clamp(get_parameter_or("spin_ratio", 0.6), 0.0, 1.0))

        , min_angle_(get_parameter_or("min_angle", 15.0))
        , max_angle_(get_parameter_or("max_angle", 55.0))
        , left_front_joint_offset_(get_parameter_or("left_front_joint_offset", 0.0))
        , left_back_joint_offset_(get_parameter_or("left_back_joint_offset", 0.0))
        , right_front_joint_offset_(get_parameter_or("right_front_joint_offset", 0.0))
        , right_back_joint_offset_(get_parameter_or("right_back_joint_offset", 0.0))
        , target_physical_velocity_limit_(
              std::max(
                  deg_to_rad(std::abs(get_parameter_or("target_physical_velocity_limit", 180.0))),
                  1e-6))
        , target_physical_acceleration_limit_(
              std::max(
                  deg_to_rad(
                      std::abs(get_parameter_or("target_physical_acceleration_limit", 720.0))),
                  1e-6))
        {

        following_velocity_controller_.output_max = angular_velocity_max_;
        following_velocity_controller_.output_min = -angular_velocity_max_;

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob", rotary_knob_);
        register_input("/predefined/update_rate", update_rate_);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);
        register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_, false);

        register_input("/chassis/left_front_joint/angle", left_front_joint_angle_, false);
        register_input("/chassis/left_back_joint/angle", left_back_joint_angle_, false);
        register_input("/chassis/right_front_joint/angle", right_front_joint_angle_, false);
        register_input("/chassis/right_back_joint/angle", right_back_joint_angle_, false);

        register_input(
            "/chassis/left_front_joint/physical_angle", left_front_joint_physical_angle_, false);
        register_input(
            "/chassis/left_back_joint/physical_angle", left_back_joint_physical_angle_, false);
        register_input(
            "/chassis/right_front_joint/physical_angle", right_front_joint_physical_angle_, false);
        register_input(
            "/chassis/right_back_joint/physical_angle", right_back_joint_physical_angle_, false);
        register_input(
            "/chassis/left_front_joint/encoder_angle", left_front_joint_encoder_angle_, false);
        register_input(
            "/chassis/left_back_joint/encoder_angle", left_back_joint_encoder_angle_, false);
        register_input(
            "/chassis/right_front_joint/encoder_angle", right_front_joint_encoder_angle_, false);
        register_input(
            "/chassis/right_back_joint/encoder_angle", right_back_joint_encoder_angle_, false);
        register_output("/gimbal/scope/control_torque", scope_motor_control_torque, nan_);

        register_output("/chassis/angle", chassis_angle_, nan_);
        register_output("/chassis/control_angle", chassis_control_angle_, nan_);

        register_output("/chassis/control_mode", mode_);
        register_output("/chassis/control_velocity", chassis_control_velocity_);

        register_output("/chassis/left_front_joint/control_angle_error", lf_angle_error_, nan_);
        register_output("/chassis/left_back_joint/control_angle_error", lb_angle_error_, nan_);
        register_output("/chassis/right_front_joint/control_angle_error", rf_angle_error_, nan_);
        register_output("/chassis/right_back_joint/control_angle_error", rb_angle_error_, nan_);

        register_output(
            "/chassis/left_front_joint/target_angle", left_front_joint_target_angle_, nan_);
        register_output(
            "/chassis/left_back_joint/target_angle", left_back_joint_target_angle_, nan_);
        register_output(
            "/chassis/right_back_joint/target_angle", right_back_joint_target_angle_, nan_);
        register_output(
            "/chassis/right_front_joint/target_angle", right_front_joint_target_angle_, nan_);

        register_output(
            "/chassis/left_front_joint/target_physical_angle",
            left_front_joint_target_physical_angle_, nan_);
        register_output(
            "/chassis/left_back_joint/target_physical_angle",
            left_back_joint_target_physical_angle_, nan_);
        register_output(
            "/chassis/right_back_joint/target_physical_angle",
            right_back_joint_target_physical_angle_, nan_);
        register_output(
            "/chassis/right_front_joint/target_physical_angle",
            right_front_joint_target_physical_angle_, nan_);
        register_output(
            "/chassis/left_front_joint/target_physical_velocity",
            left_front_joint_target_physical_velocity_, nan_);
        register_output(
            "/chassis/left_back_joint/target_physical_velocity",
            left_back_joint_target_physical_velocity_, nan_);
        register_output(
            "/chassis/right_back_joint/target_physical_velocity",
            right_back_joint_target_physical_velocity_, nan_);
        register_output(
            "/chassis/right_front_joint/target_physical_velocity",
            right_front_joint_target_physical_velocity_, nan_);
        register_output(
            "/chassis/left_front_joint/target_physical_acceleration",
            left_front_joint_target_physical_acceleration_, nan_);
        register_output(
            "/chassis/left_back_joint/target_physical_acceleration",
            left_back_joint_target_physical_acceleration_, nan_);
        register_output(
            "/chassis/right_back_joint/target_physical_acceleration",
            right_back_joint_target_physical_acceleration_, nan_);
        register_output(
            "/chassis/right_front_joint/target_physical_acceleration",
            right_front_joint_target_physical_acceleration_, nan_);
        register_output("/chassis/processed_encoder/angle", processed_encoder_angle_, nan_);

        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        chassis_control_velocity_->vector << nan_, nan_, nan_;

        current_target_angle_ = max_angle_;
        lf_current_target_angle_ = max_angle_;
        lb_current_target_angle_ = max_angle_;
        rf_current_target_angle_ = max_angle_;
        rb_current_target_angle_ = max_angle_;

        const bool left_front_joint_offset = has_parameter("left_front_joint_offset");
        const bool left_back_joint_offset = has_parameter("left_back_joint_offset");
        const bool right_front_joint_offset = has_parameter("right_front_joint_offset");
        const bool right_back_joint_offset = has_parameter("right_back_joint_offset");

        const bool has_any_joint_offset = left_front_joint_offset || left_back_joint_offset
                                       || right_front_joint_offset || right_back_joint_offset;
        const bool has_all_joint_offsets = left_front_joint_offset && left_back_joint_offset
                                        && right_front_joint_offset && right_back_joint_offset;
        if (has_any_joint_offset && !has_all_joint_offsets)
            throw std::runtime_error(
                "deformable chassis joint offsets must be configured for all four joints or "
                "removed entirely");

        joint_feedback_source_ = has_all_joint_offsets ? JointFeedbackSource::kLegacyEncoderAngle
                                                       : JointFeedbackSource::kMotorAngle;
    }

    void before_updating() override {
        if (!gimbal_yaw_angle_.ready()) {
            gimbal_yaw_angle_.make_and_bind_directly(0.0);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/gimbal/yaw/angle\". Set to 0.0.");
        }
        if (!gimbal_yaw_angle_error_.ready()) {
            gimbal_yaw_angle_error_.make_and_bind_directly(0.0);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/gimbal/yaw/control_angle_error\". Set to 0.0.");
        }
        validate_joint_feedback_inputs();
    }

    void update() override {
        using rmcs_msgs::Switch;

        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;
        const auto keyboard = *keyboard_;

        do {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_all_controls();
                break;
            }

            update_mode_from_inputs_(switch_left, switch_right, keyboard);
            update_velocity_control();
            update_lift_target_toggle(keyboard);
            run_joint_intent_pipeline_();
        } while (false);

        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        last_keyboard_ = keyboard;
    }

private:
    static constexpr double inf_ = std::numeric_limits<double>::infinity();
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double translational_velocity_max_ = 10.0;
    static constexpr double angular_velocity_max_ = 30.0;
    static constexpr double rad_to_deg_ = 180.0 / std::numbers::pi;

    static double wrap_deg(double deg) {
        deg = std::fmod(deg, 360.0);
        if (deg >= 180.0)
            deg -= 360.0;
        if (deg < -180.0)
            deg += 360.0;
        return deg;
    }

    void validate_joint_feedback_inputs() const {
        const bool ready =
            joint_feedback_source_ == JointFeedbackSource::kMotorAngle
                ? left_front_joint_angle_.ready() && left_back_joint_angle_.ready()
                      && right_front_joint_angle_.ready() && right_back_joint_angle_.ready()
                : left_front_joint_encoder_angle_.ready() && left_back_joint_encoder_angle_.ready()
                      && right_front_joint_encoder_angle_.ready()
                      && right_back_joint_encoder_angle_.ready();

        if (ready)
            return;

        throw std::runtime_error(
            joint_feedback_source_ == JointFeedbackSource::kMotorAngle
                ? "missing V2 joint feedback interfaces: expected /chassis/*_joint/angle"
                : "missing legacy joint feedback interfaces: expected /chassis/*_joint/"
                  "encoder_angle");
    }

    void update_mode_from_inputs_(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right,
        const rmcs_msgs::Keyboard& keyboard) {
        auto mode = *mode_;
        if (switch_left == rmcs_msgs::Switch::DOWN)
            return;

        if (last_switch_right_ == rmcs_msgs::Switch::MIDDLE
            && switch_right == rmcs_msgs::Switch::DOWN) {
            if (mode == rmcs_msgs::ChassisMode::SPIN) {
                mode = rmcs_msgs::ChassisMode::STEP_DOWN;
            } else {
                mode = rmcs_msgs::ChassisMode::SPIN;
                spinning_forward_ = !spinning_forward_;
            }
        } else if (!last_keyboard_.c && keyboard.c) {
            if (mode == rmcs_msgs::ChassisMode::SPIN) {
                mode = rmcs_msgs::ChassisMode::AUTO;
            } else {
                mode = rmcs_msgs::ChassisMode::SPIN;
                spinning_forward_ = !spinning_forward_;
            }
        } else if (!last_keyboard_.x && keyboard.x) {
            mode = mode == rmcs_msgs::ChassisMode::LAUNCH_RAMP
                     ? rmcs_msgs::ChassisMode::AUTO
                     : rmcs_msgs::ChassisMode::LAUNCH_RAMP;
        } else if (!last_keyboard_.z && keyboard.z) {
            mode = mode == rmcs_msgs::ChassisMode::STEP_DOWN ? rmcs_msgs::ChassisMode::AUTO
                                                             : rmcs_msgs::ChassisMode::STEP_DOWN;
        }

        *mode_ = mode;
    }

    // JointFeedbackAdapter: normalize motor / physical / legacy encoder feedback into one frame.
    JointFeedbackFrame read_joint_feedback_frame_() const {
        JointFeedbackFrame joint_feedback;
        update_current_joint_feedback(joint_feedback.motor_angles, joint_feedback.physical_angles);
        return joint_feedback;
    }

    void refresh_requested_joint_targets_from_deploy_state_() {
        requested_target_physical_angles_rad_[kLeftFront] = deg_to_rad(lf_current_target_angle_);
        requested_target_physical_angles_rad_[kLeftBack] = deg_to_rad(lb_current_target_angle_);
        requested_target_physical_angles_rad_[kRightBack] = deg_to_rad(rb_current_target_angle_);
        requested_target_physical_angles_rad_[kRightFront] = deg_to_rad(rf_current_target_angle_);

        current_target_physical_angles_rad_ = requested_target_physical_angles_rad_;
    }

    void update_current_joint_feedback(
        std::array<double, kJointCount>& current_motor_angles,
        std::array<double, kJointCount>& current_physical_angles) const {
        const std::array<const InputInterface<double>*, kJointCount> motor_angle_inputs{
            &left_front_joint_angle_, &left_back_joint_angle_, &right_back_joint_angle_,
            &right_front_joint_angle_};
        const std::array<const InputInterface<double>*, kJointCount> physical_angle_inputs{
            &left_front_joint_physical_angle_, &left_back_joint_physical_angle_,
            &right_back_joint_physical_angle_, &right_front_joint_physical_angle_};
        current_motor_angles.fill(nan_);
        current_physical_angles.fill(nan_);

        for (size_t i = 0; i < kJointCount; ++i) {
            if (motor_angle_inputs[i]->ready() && std::isfinite(*(*motor_angle_inputs[i]))) {
                current_motor_angles[i] = *(*motor_angle_inputs[i]);
                current_physical_angles[i] = motor_to_physical_angle(current_motor_angles[i]);
            }

            if (physical_angle_inputs[i]->ready() && std::isfinite(*(*physical_angle_inputs[i]))) {
                current_physical_angles[i] = *(*physical_angle_inputs[i]);
            }
        }
    }

    bool initialize_joint_target_states_from_feedback(
        const std::array<double, kJointCount>& current_motor_angles,
        const std::array<double, kJointCount>& current_physical_angles) {
        for (size_t i = 0; i < kJointCount; ++i) {
            if (!std::isfinite(current_motor_angles[i])
                || !std::isfinite(current_physical_angles[i]))
                return false;
        }

        joint_target_angle_state_rad_ = current_motor_angles;
        joint_target_physical_angle_state_rad_ = current_physical_angles;
        joint_target_physical_velocity_state_rad_ = {0.0, 0.0, 0.0, 0.0};
        joint_target_physical_acceleration_state_rad_ = {0.0, 0.0, 0.0, 0.0};
        requested_target_physical_angles_rad_ = current_physical_angles;
        current_target_physical_angles_rad_ = current_physical_angles;
        joint_target_active_ = true;
        return true;
    }

    void reset_all_controls() {
        *mode_ = rmcs_msgs::ChassisMode::AUTO;

        chassis_control_velocity_->vector << nan_, nan_, nan_;
        *chassis_angle_ = nan_;
        *chassis_control_angle_ = nan_;

        current_target_angle_ = max_angle_;
        lf_current_target_angle_ = current_target_angle_;
        lb_current_target_angle_ = current_target_angle_;
        rb_current_target_angle_ = current_target_angle_;
        rf_current_target_angle_ = current_target_angle_;
        joint_target_active_ = false;

        *scope_motor_control_torque = nan_;

        *lf_angle_error_ = nan_;
        *lb_angle_error_ = nan_;
        *rf_angle_error_ = nan_;
        *rb_angle_error_ = nan_;

        *left_front_joint_target_angle_ = nan_;
        *left_back_joint_target_angle_ = nan_;
        *right_back_joint_target_angle_ = nan_;
        *right_front_joint_target_angle_ = nan_;

        *left_front_joint_target_physical_angle_ = nan_;
        *left_back_joint_target_physical_angle_ = nan_;
        *right_back_joint_target_physical_angle_ = nan_;
        *right_front_joint_target_physical_angle_ = nan_;
        *left_front_joint_target_physical_velocity_ = nan_;
        *left_back_joint_target_physical_velocity_ = nan_;
        *right_back_joint_target_physical_velocity_ = nan_;
        *right_front_joint_target_physical_velocity_ = nan_;
        *left_front_joint_target_physical_acceleration_ = nan_;
        *left_back_joint_target_physical_acceleration_ = nan_;
        *right_back_joint_target_physical_acceleration_ = nan_;
        *right_front_joint_target_physical_acceleration_ = nan_;
        *processed_encoder_angle_ = nan_;
    }

    void update_velocity_control() {
        const Eigen::Vector2d translational_velocity = update_translational_velocity_control();
        const double angular_velocity = update_angular_velocity_control();
        chassis_control_velocity_->vector << translational_velocity, angular_velocity;
    }

    double update_dt() const {
        if (update_rate_.ready() && std::isfinite(*update_rate_) && *update_rate_ > 1e-6)
            return 1.0 / *update_rate_;
        return default_dt_;
    }

    Eigen::Vector2d update_translational_velocity_control() {
        const auto keyboard = *keyboard_;
        const Eigen::Vector2d keyboard_move{keyboard.w - keyboard.s, keyboard.a - keyboard.d};

        Eigen::Vector2d translational_velocity =
            Eigen::Rotation2Dd{*gimbal_yaw_angle_} * ((*joystick_right_) + keyboard_move);

        if (translational_velocity.norm() > 1.0)
            translational_velocity.normalize();

        translational_velocity *= translational_velocity_max_;
        return translational_velocity;
    }

    double update_angular_velocity_control() {
        double angular_velocity = 0.0;
        double chassis_control_angle = nan_;

        switch (*mode_) {
        case rmcs_msgs::ChassisMode::AUTO: break;

        case rmcs_msgs::ChassisMode::SPIN: {
            angular_velocity =
                spin_ratio_ * (spinning_forward_ ? angular_velocity_max_ : -angular_velocity_max_);
            angular_velocity =
                std::clamp(angular_velocity, -angular_velocity_max_, angular_velocity_max_);
        } break;

        case rmcs_msgs::ChassisMode::STEP_DOWN: {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);

            // In step-down mode, front/back can both be used for alignment.
            constexpr double alignment = std::numbers::pi;
            while (err > alignment / 2) {
                chassis_control_angle -= alignment;
                if (chassis_control_angle < 0)
                    chassis_control_angle += 2 * std::numbers::pi;
                err -= alignment;
            }

            angular_velocity = following_velocity_controller_.update(err);
        } break;

        case rmcs_msgs::ChassisMode::LAUNCH_RAMP: {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);

            constexpr double alignment = 2 * std::numbers::pi;
            if (err > alignment / 2)
                err -= alignment;

            angular_velocity = following_velocity_controller_.update(err);
        } break;

        default: break;
        }

        *chassis_angle_ = 2 * std::numbers::pi - *gimbal_yaw_angle_;
        *chassis_control_angle_ = chassis_control_angle;

        return angular_velocity;
    }

    double calculate_unsigned_chassis_angle_error(double& chassis_control_angle) {
        chassis_control_angle = *gimbal_yaw_angle_error_;
        if (chassis_control_angle < 0)
            chassis_control_angle += 2 * std::numbers::pi;

        double err = chassis_control_angle + *gimbal_yaw_angle_;
        if (err >= 2 * std::numbers::pi)
            err -= 2 * std::numbers::pi;

        return err;
    }

    void update_lift_target_toggle(rmcs_msgs::Keyboard keyboard) {
        constexpr double rotary_knob_edge_threshold = 0.7;

        const bool keyboard_toggle_condition = !last_keyboard_.q && keyboard.q;
        const bool rotary_knob_toggle_condition =
            last_rotary_knob_ < rotary_knob_edge_threshold
            && *rotary_knob_ >= rotary_knob_edge_threshold;
        const bool front_high_rear_low = !last_keyboard_.b && keyboard.b;
        const bool front_low_rear_high = !last_keyboard_.g && keyboard.g;

        if (apply_symmetric_target) {
            lf_current_target_angle_ = current_target_angle_;
            lb_current_target_angle_ = current_target_angle_;
            rb_current_target_angle_ = current_target_angle_;
            rf_current_target_angle_ = current_target_angle_;
        }

        if (rotary_knob_toggle_condition || keyboard_toggle_condition) {
            current_target_angle_ =
                (std::abs(current_target_angle_ - max_angle_) < 1e-6) ? min_angle_ : max_angle_;
            apply_symmetric_target = true;
        } else if (front_high_rear_low) {
            lf_current_target_angle_ = max_angle_;
            rf_current_target_angle_ = max_angle_;
            lb_current_target_angle_ = min_angle_;
            rb_current_target_angle_ = min_angle_;
            apply_symmetric_target = false;
        } else if (front_low_rear_high) {
            lf_current_target_angle_ = min_angle_;
            rf_current_target_angle_ = min_angle_;
            lb_current_target_angle_ = max_angle_;
            rb_current_target_angle_ = max_angle_;
            apply_symmetric_target = false;
        }

        last_rotary_knob_ = *rotary_knob_;
    }

    // Chassis owns the high-level joint intent pipeline: read feedback, generate deploy targets,
    // then publish the resulting joint intent for the servo layer.
    void run_joint_intent_pipeline_() {
        const auto joint_feedback = read_joint_feedback_frame_();

        if (!joint_target_active_
            && !initialize_joint_target_states_from_feedback(
                joint_feedback.motor_angles, joint_feedback.physical_angles)) {
            publish_nan_joint_targets();
            return;
        }

        refresh_requested_joint_targets_from_deploy_state_();
        scope_motor_control(false);
        update_joint_target_trajectory();
        publish_joint_target_angles(joint_feedback.physical_angles);
    }

    static double deg_to_rad(double deg) { return deg * std::numbers::pi / 180.0; }

    static double physical_to_motor_angle(double physical_angle_rad) {
        return joint_zero_physical_angle_rad_ - physical_angle_rad;
    }

    static double motor_to_physical_angle(double motor_angle_rad) {
        return joint_zero_physical_angle_rad_ - motor_angle_rad;
    }

    void scope_motor_control(bool = false) {
        *scope_motor_control_torque = 0.3;
    }

    void update_joint_target_trajectory() {
        const double dt = update_dt();
        for (size_t i = 0; i < kJointCount; ++i) {
            double& angle_state = joint_target_physical_angle_state_rad_[i];
            double& velocity_state = joint_target_physical_velocity_state_rad_[i];
            double& acceleration_state = joint_target_physical_acceleration_state_rad_[i];
            const double target_angle = current_target_physical_angles_rad_[i];
            const double velocity_limit = target_physical_velocity_limit_;
            const double acceleration_limit = target_physical_acceleration_limit_;

            if (!std::isfinite(target_angle) || !std::isfinite(angle_state)) {
                continue;
            }

            const double position_error = target_angle - angle_state;
            const double stopping_distance =
                velocity_state * velocity_state / (2.0 * acceleration_limit);

            double desired_velocity = 0.0;
            if (std::abs(position_error) > 1e-6 && std::abs(position_error) > stopping_distance) {
                desired_velocity = std::copysign(velocity_limit, position_error);
            }

            const double velocity_error = desired_velocity - velocity_state;
            acceleration_state =
                std::clamp(velocity_error / dt, -acceleration_limit, acceleration_limit);

            velocity_state += acceleration_state * dt;
            velocity_state = std::clamp(velocity_state, -velocity_limit, velocity_limit);
            angle_state += velocity_state * dt;

            const double next_error = target_angle - angle_state;
            if ((position_error > 0.0 && next_error < 0.0)
                || (position_error < 0.0 && next_error > 0.0)
                || (std::abs(next_error) < 1e-5 && std::abs(velocity_state) < 1e-3)) {
                angle_state = target_angle;
                velocity_state = 0.0;
                acceleration_state = 0.0;
            }

            joint_target_angle_state_rad_[i] = physical_to_motor_angle(angle_state);
        }
    }

    void publish_joint_target_angles(
        const std::array<double, kJointCount>& current_physical_angles) {
        if (!joint_target_active_) {
            publish_nan_joint_targets();
            return;
        }

        *left_front_joint_target_angle_ = joint_target_angle_state_rad_[kLeftFront];
        *left_back_joint_target_angle_ = joint_target_angle_state_rad_[kLeftBack];
        *right_back_joint_target_angle_ = joint_target_angle_state_rad_[kRightBack];
        *right_front_joint_target_angle_ = joint_target_angle_state_rad_[kRightFront];

        *left_front_joint_target_physical_angle_ =
            joint_target_physical_angle_state_rad_[kLeftFront];
        *left_back_joint_target_physical_angle_ = joint_target_physical_angle_state_rad_[kLeftBack];
        *right_back_joint_target_physical_angle_ =
            joint_target_physical_angle_state_rad_[kRightBack];
        *right_front_joint_target_physical_angle_ =
            joint_target_physical_angle_state_rad_[kRightFront];

        *left_front_joint_target_physical_velocity_ =
            joint_target_physical_velocity_state_rad_[kLeftFront];
        *left_back_joint_target_physical_velocity_ =
            joint_target_physical_velocity_state_rad_[kLeftBack];
        *right_back_joint_target_physical_velocity_ =
            joint_target_physical_velocity_state_rad_[kRightBack];
        *right_front_joint_target_physical_velocity_ =
            joint_target_physical_velocity_state_rad_[kRightFront];

        *left_front_joint_target_physical_acceleration_ =
            joint_target_physical_acceleration_state_rad_[kLeftFront];
        *left_back_joint_target_physical_acceleration_ =
            joint_target_physical_acceleration_state_rad_[kLeftBack];
        *right_back_joint_target_physical_acceleration_ =
            joint_target_physical_acceleration_state_rad_[kRightBack];
        *right_front_joint_target_physical_acceleration_ =
            joint_target_physical_acceleration_state_rad_[kRightFront];

        *lf_angle_error_ = std::isfinite(current_physical_angles[kLeftFront])
                             ? current_physical_angles[kLeftFront]
                                   - joint_target_physical_angle_state_rad_[kLeftFront]
                             : nan_;
        *lb_angle_error_ = std::isfinite(current_physical_angles[kLeftBack])
                             ? current_physical_angles[kLeftBack]
                                   - joint_target_physical_angle_state_rad_[kLeftBack]
                             : nan_;
        *rb_angle_error_ = std::isfinite(current_physical_angles[kRightBack])
                             ? current_physical_angles[kRightBack]
                                   - joint_target_physical_angle_state_rad_[kRightBack]
                             : nan_;
        *rf_angle_error_ = std::isfinite(current_physical_angles[kRightFront])
                             ? current_physical_angles[kRightFront]
                                   - joint_target_physical_angle_state_rad_[kRightFront]
                             : nan_;

        bool all_joint_angles_finite = true;
        double physical_angle_sum = 0.0;
        for (double current_physical_angle : current_physical_angles) {
            if (!std::isfinite(current_physical_angle)) {
                all_joint_angles_finite = false;
                break;
            }
            physical_angle_sum += current_physical_angle;
        }

        *processed_encoder_angle_ = all_joint_angles_finite ? rad_to_deg_ * physical_angle_sum
                                                                  / static_cast<double>(kJointCount)
                                                            : nan_;
    }

    void publish_nan_joint_targets() {
        *left_front_joint_target_angle_ = nan_;
        *left_back_joint_target_angle_ = nan_;
        *right_back_joint_target_angle_ = nan_;
        *right_front_joint_target_angle_ = nan_;

        *left_front_joint_target_physical_angle_ = nan_;
        *left_back_joint_target_physical_angle_ = nan_;
        *right_back_joint_target_physical_angle_ = nan_;
        *right_front_joint_target_physical_angle_ = nan_;

        *left_front_joint_target_physical_velocity_ = nan_;
        *left_back_joint_target_physical_velocity_ = nan_;
        *right_back_joint_target_physical_velocity_ = nan_;
        *right_front_joint_target_physical_velocity_ = nan_;

        *left_front_joint_target_physical_acceleration_ = nan_;
        *left_back_joint_target_physical_acceleration_ = nan_;
        *right_back_joint_target_physical_acceleration_ = nan_;
        *right_front_joint_target_physical_acceleration_ = nan_;

        *lf_angle_error_ = nan_;
        *lb_angle_error_ = nan_;
        *rb_angle_error_ = nan_;
        *rf_angle_error_ = nan_;
    }

private:
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> rotary_knob_;
    InputInterface<double> update_rate_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();
    double last_rotary_knob_ = 0.0;

    InputInterface<double> gimbal_yaw_angle_, gimbal_yaw_angle_error_;
    OutputInterface<double> chassis_angle_, chassis_control_angle_;

    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;

    bool spinning_forward_ = true;
    bool apply_symmetric_target = true;
    pid::PidCalculator following_velocity_controller_;
    const double spin_ratio_;

    InputInterface<double> left_front_joint_angle_;
    InputInterface<double> left_back_joint_angle_;
    InputInterface<double> right_front_joint_angle_;
    InputInterface<double> right_back_joint_angle_;

    InputInterface<double> left_front_joint_physical_angle_;
    InputInterface<double> left_back_joint_physical_angle_;
    InputInterface<double> right_front_joint_physical_angle_;
    InputInterface<double> right_back_joint_physical_angle_;
    InputInterface<double> left_front_joint_encoder_angle_;
    InputInterface<double> left_back_joint_encoder_angle_;
    InputInterface<double> right_front_joint_encoder_angle_;
    InputInterface<double> right_back_joint_encoder_angle_;

    OutputInterface<double> scope_motor_control_torque;

    OutputInterface<double> lf_angle_error_;
    OutputInterface<double> lb_angle_error_;
    OutputInterface<double> rf_angle_error_;
    OutputInterface<double> rb_angle_error_;

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

    OutputInterface<double> processed_encoder_angle_;

    double min_angle_;
    double max_angle_;
    double left_front_joint_offset_;
    double left_back_joint_offset_;
    double right_front_joint_offset_;
    double right_back_joint_offset_;
    JointFeedbackSource joint_feedback_source_ = JointFeedbackSource::kLegacyEncoderAngle;

    double current_target_angle_;
    double lf_current_target_angle_, lb_current_target_angle_, rb_current_target_angle_,
        rf_current_target_angle_;

    bool joint_target_active_ = false;
    std::array<double, kJointCount> requested_target_physical_angles_rad_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> current_target_physical_angles_rad_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> joint_target_angle_state_rad_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> joint_target_physical_angle_state_rad_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> joint_target_physical_velocity_state_rad_ = {
        0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> joint_target_physical_acceleration_state_rad_ = {
        0.0, 0.0, 0.0, 0.0};
    double target_physical_velocity_limit_;
    double target_physical_acceleration_limit_;
    static constexpr double default_dt_ = 1e-3;
    static constexpr double joint_zero_physical_angle_rad_ = 1.090830782496456;

    static constexpr double pi_ = std::numbers::pi;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::DeformableChassis, rmcs_executor::Component)
