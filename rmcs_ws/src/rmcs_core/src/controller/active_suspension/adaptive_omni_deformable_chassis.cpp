#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
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
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::active_suspension {

class AdaptiveOmniDeformableChassis
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    enum JointIndex : size_t {
        kLeftFront = 0,
        kLeftBack = 1,
        kRightBack = 2,
        kRightFront = 3,
        kJointCount = 4,
    };

    AdaptiveOmniDeformableChassis()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , following_velocity_controller_(10.0, 0.0, 0.0)
        , min_angle_deg_(get_parameter_or("min_angle", 20.0))
        , max_angle_deg_(get_parameter_or("max_angle", 58.0))
        , target_physical_velocity_limit_(
              deg_to_rad(get_parameter_or("target_physical_velocity_limit", 180.0)))
        , target_physical_acceleration_limit_(
              deg_to_rad(get_parameter_or("target_physical_acceleration_limit", 720.0))) {

        following_velocity_controller_.output_max = angular_velocity_max_;
        following_velocity_controller_.output_min = -angular_velocity_max_;

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob", rotary_knob_);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);
        register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_, false);

        register_input("/chassis/left_front_joint/angle", left_front_joint_angle_, false);
        register_input("/chassis/left_back_joint/angle", left_back_joint_angle_, false);
        register_input("/chassis/right_back_joint/angle", right_back_joint_angle_, false);
        register_input("/chassis/right_front_joint/angle", right_front_joint_angle_, false);

        register_output("/chassis/control_mode", mode_);
        register_output("/chassis/control_velocity", chassis_control_velocity_);
        register_output("/chassis/angle", chassis_angle_, nan_);
        register_output("/chassis/control_angle", chassis_control_angle_, nan_);

        register_output(
            "/chassis/left_front_joint/base_target_angle", left_front_joint_target_angle_, nan_);
        register_output(
            "/chassis/left_back_joint/base_target_angle", left_back_joint_target_angle_, nan_);
        register_output(
            "/chassis/right_back_joint/base_target_angle", right_back_joint_target_angle_, nan_);
        register_output(
            "/chassis/right_front_joint/base_target_angle", right_front_joint_target_angle_, nan_);

        register_output(
            "/chassis/left_front_joint/base_target_physical_angle",
            left_front_joint_target_physical_angle_, nan_);
        register_output(
            "/chassis/left_back_joint/base_target_physical_angle",
            left_back_joint_target_physical_angle_, nan_);
        register_output(
            "/chassis/right_back_joint/base_target_physical_angle",
            right_back_joint_target_physical_angle_, nan_);
        register_output(
            "/chassis/right_front_joint/base_target_physical_angle",
            right_front_joint_target_physical_angle_, nan_);

        register_output(
            "/chassis/left_front_joint/base_target_physical_velocity",
            left_front_joint_target_physical_velocity_, nan_);
        register_output(
            "/chassis/left_back_joint/base_target_physical_velocity",
            left_back_joint_target_physical_velocity_, nan_);
        register_output(
            "/chassis/right_back_joint/base_target_physical_velocity",
            right_back_joint_target_physical_velocity_, nan_);
        register_output(
            "/chassis/right_front_joint/base_target_physical_velocity",
            right_front_joint_target_physical_velocity_, nan_);

        register_output(
            "/chassis/left_front_joint/base_target_physical_acceleration",
            left_front_joint_target_physical_acceleration_, nan_);
        register_output(
            "/chassis/left_back_joint/base_target_physical_acceleration",
            left_back_joint_target_physical_acceleration_, nan_);
        register_output(
            "/chassis/right_back_joint/base_target_physical_acceleration",
            right_back_joint_target_physical_acceleration_, nan_);
        register_output(
            "/chassis/right_front_joint/base_target_physical_acceleration",
            right_front_joint_target_physical_acceleration_, nan_);

        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        chassis_control_velocity_->vector << nan_, nan_, nan_;

        current_target_angle_deg_ = max_angle_deg_;
        lf_current_target_angle_deg_ = max_angle_deg_;
        lb_current_target_angle_deg_ = max_angle_deg_;
        rb_current_target_angle_deg_ = max_angle_deg_;
        rf_current_target_angle_deg_ = max_angle_deg_;
    }

    void before_updating() override {
        if (!gimbal_yaw_angle_.ready()) {
            gimbal_yaw_angle_.make_and_bind_directly(0.0);
            RCLCPP_WARN(get_logger(), "Adaptive omni chassis fallback: /gimbal/yaw/angle -> 0.0");
        }
        if (!gimbal_yaw_angle_error_.ready()) {
            gimbal_yaw_angle_error_.make_and_bind_directly(0.0);
            RCLCPP_WARN(
                get_logger(),
                "Adaptive omni chassis fallback: /gimbal/yaw/control_angle_error -> 0.0");
        }
    }

    void update() override {
        using rmcs_msgs::Switch;

        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;
        const auto keyboard = *keyboard_;

        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
            last_switch_right_ = switch_right;
            last_switch_left_ = switch_left;
            last_keyboard_ = keyboard;
            if (rotary_knob_.ready() && std::isfinite(*rotary_knob_))
                last_rotary_knob_ = *rotary_knob_;
            return;
        }

        auto mode = *mode_;
        if (switch_left != Switch::DOWN) {
            if (last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN) {
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
                mode = mode == rmcs_msgs::ChassisMode::STEP_DOWN
                         ? rmcs_msgs::ChassisMode::AUTO
                         : rmcs_msgs::ChassisMode::STEP_DOWN;
            }
            *mode_ = mode;
        }

        update_velocity_control();
        update_lift_target_toggle(switch_left, switch_right, keyboard);
        update_joint_target_trajectory();
        publish_joint_target_angles();

        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        last_keyboard_ = keyboard;
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double translational_velocity_max_ = 10.0;
    static constexpr double angular_velocity_max_ = 25.0;
    static constexpr double joint_zero_physical_angle_rad_ = 62.5 * std::numbers::pi / 180.0;
    static constexpr double dt_ = 1e-3;

    static double deg_to_rad(double deg) { return deg * std::numbers::pi / 180.0; }

    static double physical_to_motor_angle(double physical_angle_rad) {
        return joint_zero_physical_angle_rad_ - physical_angle_rad;
    }

    static double motor_to_physical_angle(double motor_angle_rad) {
        return joint_zero_physical_angle_rad_ - motor_angle_rad;
    }

<<<<<<<< HEAD:rmcs_ws/src/rmcs_core/src/controller/active_suspension/adaptive_omni_deformable_chassis.cpp
    void reset_all_controls() {
        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        chassis_control_velocity_->vector << nan_, nan_, nan_;
        *chassis_angle_ = nan_;
        *chassis_control_angle_ = nan_;

        current_target_angle_deg_ = max_angle_deg_;
        lf_current_target_angle_deg_ = current_target_angle_deg_;
        lb_current_target_angle_deg_ = current_target_angle_deg_;
        rb_current_target_angle_deg_ = current_target_angle_deg_;
        rf_current_target_angle_deg_ = current_target_angle_deg_;
        joint_target_active_ = false;

        publish_nan_joint_targets();
========
    void scope_motor_control() {
        if (current_target_angle_ == min_angle_ && *mode_ != rmcs_msgs::ChassisMode::SPIN){
            // *scope_motor_control_torque = -0.3;
            // if (*scope_motor_velocity <= std::abs(0.1)){
            //     *scope_motor_control_torque = 0.18 * 1.0 / 36.0;
            // }
        }
        else{
            // *scope_motor_control_torque = 0.3;
            // if (*scope_motor_velocity <= std::abs(0.1)){
            //     *scope_motor_control_torque = -0.18 * 1.0 / 36.0;
            // }
        }
>>>>>>>> ab86bdbf5d59eab51b4e0008daf9f6ac73de44ad:rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp
    }

    void update_velocity_control() {
        const Eigen::Vector2d translational_velocity = update_translational_velocity_control();
        const double angular_velocity = update_angular_velocity_control();
        chassis_control_velocity_->vector << translational_velocity, angular_velocity;
    }

    Eigen::Vector2d update_translational_velocity_control() const {
        const auto keyboard = *keyboard_;
        const Eigen::Vector2d keyboard_move{keyboard.w - keyboard.s, keyboard.a - keyboard.d};

        Eigen::Vector2d translational_velocity =
            Eigen::Rotation2Dd{*gimbal_yaw_angle_} * ((*joystick_right_) + keyboard_move);
        if (translational_velocity.norm() > 1.0)
            translational_velocity.normalize();
        return translational_velocity * translational_velocity_max_;
    }

    double update_angular_velocity_control() {
        double angular_velocity = 0.0;
        double chassis_control_angle = nan_;

        switch (*mode_) {
        case rmcs_msgs::ChassisMode::AUTO: break;

        case rmcs_msgs::ChassisMode::SPIN:
            angular_velocity = 0.6 * (spinning_forward_ ? angular_velocity_max_ : -angular_velocity_max_);
            angular_velocity = std::clamp(
                angular_velocity, -angular_velocity_max_, angular_velocity_max_);
            break;

        case rmcs_msgs::ChassisMode::STEP_DOWN: {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);
            constexpr double alignment = std::numbers::pi;
            while (err > alignment / 2) {
                chassis_control_angle -= alignment;
                if (chassis_control_angle < 0)
                    chassis_control_angle += 2 * std::numbers::pi;
                err -= alignment;
            }
            angular_velocity = following_velocity_controller_.update(err);
            break;
        }

        case rmcs_msgs::ChassisMode::LAUNCH_RAMP: {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);
            constexpr double alignment = 2 * std::numbers::pi;
            if (err > alignment / 2)
                err -= alignment;
            angular_velocity = following_velocity_controller_.update(err);
            break;
        }

        default: break;
        }

        *chassis_angle_ = 2 * std::numbers::pi - *gimbal_yaw_angle_;
        *chassis_control_angle_ = chassis_control_angle;
        return angular_velocity;
    }

    double calculate_unsigned_chassis_angle_error(double& chassis_control_angle) const {
        chassis_control_angle = *gimbal_yaw_angle_error_;
        if (chassis_control_angle < 0)
            chassis_control_angle += 2 * std::numbers::pi;

        double err = chassis_control_angle + *gimbal_yaw_angle_;
        if (err >= 2 * std::numbers::pi)
            err -= 2 * std::numbers::pi;
        return err;
    }

    void update_lift_target_toggle(
        rmcs_msgs::Switch left_switch, rmcs_msgs::Switch right_switch,
        const rmcs_msgs::Keyboard& keyboard) {
        constexpr double rotary_knob_edge_threshold = 0.7;
        const bool switch_toggle_condition =
            (left_switch == rmcs_msgs::Switch::MIDDLE) && (right_switch == rmcs_msgs::Switch::UP);
        const bool keyboard_toggle_condition = !last_keyboard_.q && keyboard.q;

        const bool last_switch_toggle_condition = (last_switch_left_ == rmcs_msgs::Switch::MIDDLE)
                                               && (last_switch_right_ == rmcs_msgs::Switch::UP);

        const bool rotary_knob_front_high_rear_low = last_rotary_knob_ > -rotary_knob_edge_threshold
                                                  && *rotary_knob_ <= -rotary_knob_edge_threshold;
        const bool rotary_knob_front_low_rear_high = last_rotary_knob_ < rotary_knob_edge_threshold
                                                  && *rotary_knob_ >= rotary_knob_edge_threshold;
        const bool front_high_rear_low =
            (!last_keyboard_.b && keyboard.b) || rotary_knob_front_high_rear_low;
        const bool front_low_rear_high =
            (!last_keyboard_.g && keyboard.g) || rotary_knob_front_low_rear_high;
        const bool uphill = keyboard.ctrl;

        if ((switch_toggle_condition && !last_switch_toggle_condition)
            || keyboard_toggle_condition) {
            if (apply_symmetric_target_) {
                current_target_angle_deg_ =
                    (std::abs(current_target_angle_deg_ - max_angle_deg_) < 1e-6) ? min_angle_deg_
                                                                                   : max_angle_deg_;
            }
            apply_symmetric_target_ = true;
        } else if (uphill) {
            lf_current_target_angle_deg_ = min_angle_deg_;
            rf_current_target_angle_deg_ = min_angle_deg_;
            lb_current_target_angle_deg_ = min_angle_deg_ + 25.0;
            rb_current_target_angle_deg_ = min_angle_deg_ + 25.0;
            apply_symmetric_target_ = false;
        } else if (front_high_rear_low) {
            lf_current_target_angle_deg_ = max_angle_deg_;
            rf_current_target_angle_deg_ = max_angle_deg_;
            lb_current_target_angle_deg_ = min_angle_deg_;
            rb_current_target_angle_deg_ = min_angle_deg_;
            apply_symmetric_target_ = false;
        } else if (front_low_rear_high) {
            lf_current_target_angle_deg_ = min_angle_deg_;
            rf_current_target_angle_deg_ = min_angle_deg_;
            lb_current_target_angle_deg_ = max_angle_deg_;
            rb_current_target_angle_deg_ = max_angle_deg_;
            apply_symmetric_target_ = false;
        }

        if (apply_symmetric_target_) {
            lf_current_target_angle_deg_ = current_target_angle_deg_;
            lb_current_target_angle_deg_ = current_target_angle_deg_;
            rb_current_target_angle_deg_ = current_target_angle_deg_;
            rf_current_target_angle_deg_ = current_target_angle_deg_;
        }

        last_rotary_knob_ = *rotary_knob_;
    }

    bool initialize_joint_target_state() {
        const std::array<InputInterface<double>*, kJointCount> motor_angle_inputs{
            &left_front_joint_angle_, &left_back_joint_angle_, &right_back_joint_angle_,
            &right_front_joint_angle_};

        for (size_t i = 0; i < kJointCount; ++i) {
            if (!motor_angle_inputs[i]->ready() || !std::isfinite(*(*motor_angle_inputs[i])))
                return false;
        }

        joint_target_angle_state_rad_ = {
            *left_front_joint_angle_, *left_back_joint_angle_, *right_back_joint_angle_,
            *right_front_joint_angle_};

        for (size_t i = 0; i < kJointCount; ++i) {
            joint_target_physical_angle_state_rad_[i] =
                motor_to_physical_angle(joint_target_angle_state_rad_[i]);
            joint_target_physical_velocity_state_rad_[i] = 0.0;
            joint_target_physical_acceleration_state_rad_[i] = 0.0;
        }

        joint_target_active_ = true;
        return true;
    }

    void update_joint_target_trajectory() {
        if (!joint_target_active_ && !initialize_joint_target_state()) {
            publish_nan_joint_targets();
            return;
        }

        const std::array<double, kJointCount> desired_physical_angles{
            deg_to_rad(lf_current_target_angle_deg_), deg_to_rad(lb_current_target_angle_deg_),
            deg_to_rad(rb_current_target_angle_deg_), deg_to_rad(rf_current_target_angle_deg_)};

        for (size_t i = 0; i < kJointCount; ++i) {
            double& angle_state = joint_target_physical_angle_state_rad_[i];
            double& velocity_state = joint_target_physical_velocity_state_rad_[i];
            double& acceleration_state = joint_target_physical_acceleration_state_rad_[i];
            const double target_angle = desired_physical_angles[i];

            const double position_error = target_angle - angle_state;
            const double stopping_distance =
                velocity_state * velocity_state / (2.0 * target_physical_acceleration_limit_);

            double desired_velocity = 0.0;
            if (std::abs(position_error) > 1e-6 && std::abs(position_error) > stopping_distance)
                desired_velocity = std::copysign(target_physical_velocity_limit_, position_error);

            const double velocity_error = desired_velocity - velocity_state;
            acceleration_state = std::clamp(
                velocity_error / dt_, -target_physical_acceleration_limit_,
                target_physical_acceleration_limit_);

            velocity_state += acceleration_state * dt_;
            velocity_state = std::clamp(
                velocity_state, -target_physical_velocity_limit_, target_physical_velocity_limit_);
            angle_state += velocity_state * dt_;

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

    void publish_joint_target_angles() {
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
    }

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> rotary_knob_;

    InputInterface<double> gimbal_yaw_angle_;
    InputInterface<double> gimbal_yaw_angle_error_;

    InputInterface<double> left_front_joint_angle_;
    InputInterface<double> left_back_joint_angle_;
    InputInterface<double> right_back_joint_angle_;
    InputInterface<double> right_front_joint_angle_;

    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
    OutputInterface<double> chassis_angle_;
    OutputInterface<double> chassis_control_angle_;

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

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();
    double last_rotary_knob_ = 0.0;

    bool spinning_forward_ = true;
    bool apply_symmetric_target_ = true;
    bool joint_target_active_ = false;

    pid::PidCalculator following_velocity_controller_;

    double min_angle_deg_;
    double max_angle_deg_;
    double current_target_angle_deg_ = 0.0;
    double lf_current_target_angle_deg_ = 0.0;
    double lb_current_target_angle_deg_ = 0.0;
    double rb_current_target_angle_deg_ = 0.0;
    double rf_current_target_angle_deg_ = 0.0;

    std::array<double, kJointCount> joint_target_angle_state_rad_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> joint_target_physical_angle_state_rad_ = {
        0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> joint_target_physical_velocity_state_rad_ = {
        0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> joint_target_physical_acceleration_state_rad_ = {
        0.0, 0.0, 0.0, 0.0};

    double target_physical_velocity_limit_;
    double target_physical_acceleration_limit_;
};

} // namespace rmcs_core::controller::active_suspension

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::active_suspension::AdaptiveOmniDeformableChassis,
    rmcs_executor::Component)
