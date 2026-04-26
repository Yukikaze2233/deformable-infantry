#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

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
    DeformableChassis()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , following_velocity_controller_(10.0, 0.0, 0.0)
        , minimum_angle_degree_(get_parameter_or("minimum_angle", 15.0))
        , maximum_angle_degree_(get_parameter_or("maximum_angle", 55.0))
        , active_suspension_enabled_(get_parameter_or("active_suspension_enable", false))
        , active_suspension_control_acceleration_limit_(
              std::abs(get_parameter_or("active_suspension_control_acceleration_limit", 6.0))) {

        following_velocity_controller_.output_max = kAngularVelocityMax;
        following_velocity_controller_.output_min = -kAngularVelocityMax;

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

        register_output("/gimbal/scope/control_torque", scope_motor_control_torque_, kQuietNan);

        register_output("/chassis/angle", chassis_angle_, kQuietNan);
        register_output("/chassis/control_angle", chassis_control_angle_, kQuietNan);
        register_output("/chassis/control_mode", mode_);
        register_output("/chassis/control_velocity", chassis_control_velocity_);

        // Chassis intent outputs (consumed by suspension controller)
        register_output(
            "/chassis/left_front_joint/requested_angle", left_front_requested_angle_, kQuietNan);
        register_output(
            "/chassis/left_back_joint/requested_angle", left_back_requested_angle_, kQuietNan);
        register_output(
            "/chassis/right_back_joint/requested_angle", right_back_requested_angle_, kQuietNan);
        register_output(
            "/chassis/right_front_joint/requested_angle", right_front_requested_angle_, kQuietNan);
        register_output("/chassis/suspension/requested", suspension_requested_, false);
        register_output("/chassis/control_acceleration/x", control_acceleration_x_, kQuietNan);
        register_output("/chassis/control_acceleration/y", control_acceleration_y_, kQuietNan);

        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        chassis_control_velocity_->vector << 0.0, 0.0, 0.0;

        symmetric_deploy_target_degree_          = maximum_angle_degree_;
        left_front_deploy_target_degree_         = maximum_angle_degree_;
        left_back_deploy_target_degree_          = maximum_angle_degree_;
        right_back_deploy_target_degree_         = maximum_angle_degree_;
        right_front_deploy_target_degree_        = maximum_angle_degree_;
    }

    void before_updating() override {
        if (!gimbal_yaw_angle_.ready()) {
            gimbal_yaw_angle_.make_and_bind_directly(0.0);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/gimbal/yaw/angle\". Set to 0.0.");
        }
        if (!gimbal_yaw_angle_error_.ready()) {
            gimbal_yaw_angle_error_.make_and_bind_directly(0.0);
            RCLCPP_WARN(
                get_logger(),
                "Failed to fetch \"/gimbal/yaw/control_angle_error\". Set to 0.0.");
        }
    }

    void update() override {
        using rmcs_msgs::Switch;

        const auto switch_right = *switch_right_;
        const auto switch_left  = *switch_left_;
        const auto keyboard     = *keyboard_;

        do {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_all_controls_();
                break;
            }
            update_mode_(switch_left, switch_right, keyboard);
            update_velocity_control_();
            update_lift_targets_(switch_left, switch_right, keyboard);
            publish_requested_angles_();
        } while (false);

        last_switch_right_ = switch_right;
        last_switch_left_  = switch_left;
        last_keyboard_     = keyboard;
    }

private:
    // ---- Constants ----
    static constexpr double kQuietNan              = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kTranslationalVelocityMax = 10.0;
    static constexpr double kAngularVelocityMax     = 25.0;
    static constexpr double kDefaultDt              = 1e-3;

    // ---- Mode switching ----
    void update_mode_(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right,
        const rmcs_msgs::Keyboard& keyboard) {
        auto mode = *mode_;
        if (switch_left == rmcs_msgs::Switch::DOWN) return;
        if (last_switch_right_ == rmcs_msgs::Switch::MIDDLE
            && switch_right == rmcs_msgs::Switch::DOWN) {
            mode = (mode == rmcs_msgs::ChassisMode::SPIN)
                       ? rmcs_msgs::ChassisMode::STEP_DOWN
                       : rmcs_msgs::ChassisMode::SPIN;
            if (mode == rmcs_msgs::ChassisMode::SPIN) spinning_forward_ = !spinning_forward_;
        } else if (!last_keyboard_.c && keyboard.c) {
            mode = (mode == rmcs_msgs::ChassisMode::SPIN)
                       ? rmcs_msgs::ChassisMode::AUTO
                       : rmcs_msgs::ChassisMode::SPIN;
            if (mode == rmcs_msgs::ChassisMode::SPIN) spinning_forward_ = !spinning_forward_;
        } else if (!last_keyboard_.x && keyboard.x) {
            mode = (mode == rmcs_msgs::ChassisMode::LAUNCH_RAMP)
                       ? rmcs_msgs::ChassisMode::AUTO
                       : rmcs_msgs::ChassisMode::LAUNCH_RAMP;
        } else if (!last_keyboard_.z && keyboard.z) {
            mode = (mode == rmcs_msgs::ChassisMode::STEP_DOWN)
                       ? rmcs_msgs::ChassisMode::AUTO
                       : rmcs_msgs::ChassisMode::STEP_DOWN;
        }
        *mode_ = mode;
    }

    // ---- Velocity control ----
    void update_velocity_control_() {
        Eigen::Vector2d translational_velocity = compute_translational_velocity_();
        double angular_velocity = compute_angular_velocity_();
        update_control_acceleration_estimate_(translational_velocity);
        chassis_control_velocity_->vector << translational_velocity, angular_velocity;
    }

    Eigen::Vector2d compute_translational_velocity_() {
        auto keyboard = *keyboard_;
        Eigen::Vector2d keyboard_move{keyboard.w - keyboard.s, keyboard.a - keyboard.d};
        Eigen::Vector2d translational_velocity =
            Eigen::Rotation2Dd{*gimbal_yaw_angle_} * ((*joystick_right_) + keyboard_move);
        if (translational_velocity.norm() > 1.0) translational_velocity.normalize();
        return translational_velocity * kTranslationalVelocityMax;
    }

    double compute_angular_velocity_() {
        double angular_velocity = 0.0;
        double chassis_control_angle = kQuietNan;
        switch (*mode_) {
        case rmcs_msgs::ChassisMode::AUTO: break;
        case rmcs_msgs::ChassisMode::SPIN:
            angular_velocity =
                0.6 * (spinning_forward_ ? kAngularVelocityMax : -kAngularVelocityMax);
            angular_velocity =
                std::clamp(angular_velocity, -kAngularVelocityMax, kAngularVelocityMax);
            break;
        case rmcs_msgs::ChassisMode::STEP_DOWN:
        case rmcs_msgs::ChassisMode::LAUNCH_RAMP: {
            double error = compute_chassis_angle_error_(chassis_control_angle);
            if (error > std::numbers::pi) error -= 2 * std::numbers::pi;
            angular_velocity = following_velocity_controller_.update(error);
        } break;
        default: break;
        }
        *chassis_angle_         = 2 * std::numbers::pi - *gimbal_yaw_angle_;
        *chassis_control_angle_ = chassis_control_angle;
        return angular_velocity;
    }

    double compute_chassis_angle_error_(double& chassis_control_angle) {
        chassis_control_angle = *gimbal_yaw_angle_error_;
        if (chassis_control_angle < 0) chassis_control_angle += 2 * std::numbers::pi;
        double error = chassis_control_angle + *gimbal_yaw_angle_;
        if (error >= 2 * std::numbers::pi) error -= 2 * std::numbers::pi;
        return error;
    }

    // ---- Control acceleration estimate ----
    void reset_control_acceleration_estimate_() {
        control_acceleration_estimate_.setZero();
        last_translational_velocity_.setZero();
        last_translational_velocity_valid_ = false;
    }

    void update_control_acceleration_estimate_(const Eigen::Vector2d& translational_velocity) {
        if (!translational_velocity.array().isFinite().all()) {
            reset_control_acceleration_estimate_();
            return;
        }
        if (!last_translational_velocity_valid_) {
            last_translational_velocity_      = translational_velocity;
            control_acceleration_estimate_.setZero();
            last_translational_velocity_valid_ = true;
            return;
        }
        double dt  = compute_dt_();
        Eigen::Vector2d max_acceleration =
            Eigen::Vector2d::Constant(active_suspension_control_acceleration_limit_);
        control_acceleration_estimate_ =
            ((translational_velocity - last_translational_velocity_) / dt)
                .cwiseMax(-max_acceleration)
                .cwiseMin(max_acceleration);
        last_translational_velocity_ = translational_velocity;
    }

    // ---- Lift target toggle ----
    void update_lift_targets_(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right,
        rmcs_msgs::Keyboard keyboard) {

        constexpr double kRotaryEdge = 0.7;
        bool switch_toggle =
            (switch_left == rmcs_msgs::Switch::MIDDLE) && (switch_right == rmcs_msgs::Switch::UP);
        bool keyboard_toggle = !last_keyboard_.q && keyboard.q;
        bool last_switch_toggle =
            (last_switch_left_ == rmcs_msgs::Switch::MIDDLE)
            && (last_switch_right_ == rmcs_msgs::Switch::UP);
        bool front_high =
            (!last_keyboard_.b && keyboard.b)
            || (last_rotary_knob_ > -kRotaryEdge && *rotary_knob_ <= -kRotaryEdge);
        bool front_low =
            (!last_keyboard_.g && keyboard.g)
            || (last_rotary_knob_ < kRotaryEdge && *rotary_knob_ >= kRotaryEdge);

        if (apply_symmetric_target_) {
            left_front_deploy_target_degree_  = symmetric_deploy_target_degree_;
            left_back_deploy_target_degree_   = symmetric_deploy_target_degree_;
            right_back_deploy_target_degree_  = symmetric_deploy_target_degree_;
            right_front_deploy_target_degree_ = symmetric_deploy_target_degree_;
        }
        if ((switch_toggle && !last_switch_toggle) || keyboard_toggle) {
            symmetric_deploy_target_degree_ =
                (std::abs(symmetric_deploy_target_degree_ - maximum_angle_degree_) < 1e-6)
                    ? minimum_angle_degree_
                    : maximum_angle_degree_;
            apply_symmetric_target_ = true;
        } else if (front_high) {
            left_front_deploy_target_degree_  = maximum_angle_degree_;
            right_front_deploy_target_degree_ = maximum_angle_degree_;
            left_back_deploy_target_degree_   = minimum_angle_degree_;
            right_back_deploy_target_degree_  = minimum_angle_degree_;
            apply_symmetric_target_ = false;
        } else if (front_low) {
            left_front_deploy_target_degree_  = minimum_angle_degree_;
            right_front_deploy_target_degree_ = minimum_angle_degree_;
            left_back_deploy_target_degree_   = maximum_angle_degree_;
            right_back_deploy_target_degree_  = maximum_angle_degree_;
            apply_symmetric_target_ = false;
        }
        last_rotary_knob_ = *rotary_knob_;
    }

    void publish_requested_angles_() {
        *left_front_requested_angle_  = left_front_deploy_target_degree_;
        *left_back_requested_angle_   = left_back_deploy_target_degree_;
        *right_back_requested_angle_  = right_back_deploy_target_degree_;
        *right_front_requested_angle_ = right_front_deploy_target_degree_;

        *suspension_requested_ = suspension_requested_by_input_();

        *control_acceleration_x_ = control_acceleration_estimate_.x();
        *control_acceleration_y_ = control_acceleration_estimate_.y();

        scope_motor_control_(prone_override_requested_()
                             || symmetric_deploy_target_degree_ == minimum_angle_degree_);
    }

    // ---- Suspension request logic ----
    bool prone_override_requested_() const {
        return keyboard_.ready() && keyboard_->ctrl;
    }

    bool suspension_requested_by_switch_() const {
        return switch_left_.ready() && switch_right_.ready()
            && *switch_left_ == rmcs_msgs::Switch::DOWN
            && *switch_right_ == rmcs_msgs::Switch::UP;
    }

    bool suspension_requested_by_input_() const {
        return active_suspension_enabled_
            && (prone_override_requested_() || suspension_requested_by_switch_());
    }

    // ---- Scope motor ----
    void scope_motor_control_(bool prone) {
        bool active_deploy = prone || symmetric_deploy_target_degree_ == minimum_angle_degree_;
        if (active_deploy && *mode_ != rmcs_msgs::ChassisMode::SPIN)
            *scope_motor_control_torque_ = -0.3;
        else
            *scope_motor_control_torque_ = 0.3;
    }

    // ---- Utility ----
    double compute_dt_() const {
        if (update_rate_.ready() && std::isfinite(*update_rate_) && *update_rate_ > 1e-6)
            return 1.0 / *update_rate_;
        return kDefaultDt;
    }

    void reset_all_controls_() {
        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        reset_control_acceleration_estimate_();

        chassis_control_velocity_->vector << kQuietNan, kQuietNan, kQuietNan;
        *chassis_angle_         = kQuietNan;
        *chassis_control_angle_ = kQuietNan;

        symmetric_deploy_target_degree_   = maximum_angle_degree_;
        left_front_deploy_target_degree_  = maximum_angle_degree_;
        left_back_deploy_target_degree_   = maximum_angle_degree_;
        right_back_deploy_target_degree_  = maximum_angle_degree_;
        right_front_deploy_target_degree_ = maximum_angle_degree_;
        apply_symmetric_target_ = true;

        *scope_motor_control_torque_ = kQuietNan;
        *left_front_requested_angle_  = kQuietNan;
        *left_back_requested_angle_   = kQuietNan;
        *right_back_requested_angle_  = kQuietNan;
        *right_front_requested_angle_ = kQuietNan;
        *suspension_requested_        = false;
        *control_acceleration_x_      = kQuietNan;
        *control_acceleration_y_      = kQuietNan;
    }

    // ---- Input interfaces ----
    InputInterface<Eigen::Vector2d> joystick_right_, joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_, switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> rotary_knob_, update_rate_;

    InputInterface<double> gimbal_yaw_angle_, gimbal_yaw_angle_error_;

    // ---- Previous state ----
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_  = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_   = rmcs_msgs::Keyboard::zero();
    double last_rotary_knob_ = 0.0;

    // ---- Output interfaces ----
    OutputInterface<double> chassis_angle_, chassis_control_angle_;
    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
    OutputInterface<double> scope_motor_control_torque_;

    OutputInterface<double> left_front_requested_angle_;
    OutputInterface<double> left_back_requested_angle_;
    OutputInterface<double> right_back_requested_angle_;
    OutputInterface<double> right_front_requested_angle_;
    OutputInterface<bool> suspension_requested_;
    OutputInterface<double> control_acceleration_x_;
    OutputInterface<double> control_acceleration_y_;

    // ---- Chassis state ----
    bool spinning_forward_         = true;
    bool apply_symmetric_target_   = true;
    pid::PidCalculator following_velocity_controller_;

    // ---- Lift target state ----
    double symmetric_deploy_target_degree_;
    double left_front_deploy_target_degree_;
    double left_back_deploy_target_degree_;
    double right_back_deploy_target_degree_;
    double right_front_deploy_target_degree_;

    // ---- Parameters ----
    double minimum_angle_degree_;
    double maximum_angle_degree_;
    bool active_suspension_enabled_;
    double active_suspension_control_acceleration_limit_;

    // ---- Acceleration estimate ----
    Eigen::Vector2d control_acceleration_estimate_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d last_translational_velocity_   = Eigen::Vector2d::Zero();
    bool last_translational_velocity_valid_ = false;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::DeformableChassis, rmcs_executor::Component)
