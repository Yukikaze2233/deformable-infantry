#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/chassis/qcp_solver.hpp"
#include "controller/pid/matrix_pid_calculator.hpp"
#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::active_suspension {

class AdaptiveDeformableOmniWheelController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AdaptiveDeformableOmniWheelController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , mass_(get_parameter("mess").as_double())
        , moment_of_inertia_(get_parameter("moment_of_inertia").as_double())
        , wheel_radius_(get_parameter("wheel_radius").as_double())
        , friction_coefficient_(get_parameter("friction_coefficient").as_double())
        , k1_(get_parameter("k1").as_double())
        , k2_(get_parameter("k2").as_double())
        , no_load_power_(get_parameter("no_load_power").as_double())
        , min_contact_weight_(get_parameter_or("min_contact_weight", 0.15))
        , translational_velocity_pid_calculator_(5.0, 0.0, 0.0)
        , angular_velocity_pid_calculator_(5.0, 0.0, 0.0)
        , wheel_velocity_pid_(0.6, 0.0, 0.0) {

        register_input("/chassis/left_front_wheel/max_torque", left_front_wheel_max_torque_);
        register_input("/chassis/left_back_wheel/max_torque", left_back_wheel_max_torque_);
        register_input("/chassis/right_back_wheel/max_torque", right_back_wheel_max_torque_);
        register_input("/chassis/right_front_wheel/max_torque", right_front_wheel_max_torque_);

        register_input("/chassis/left_front_wheel/velocity", left_front_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_velocity_);
        register_input("/chassis/right_front_wheel/velocity", right_front_velocity_);

        register_input("/chassis/left_front_contact/confidence", left_front_contact_confidence_);
        register_input("/chassis/left_back_contact/confidence", left_back_contact_confidence_);
        register_input("/chassis/right_back_contact/confidence", right_back_contact_confidence_);
        register_input("/chassis/right_front_contact/confidence", right_front_contact_confidence_);

        register_input("/chassis/control_velocity", chassis_control_velocity_);
        register_input("/chassis/control_power_limit", power_limit_);
        register_input("/chassis/radius", chassis_radius_);

        register_output("/chassis/left_front_wheel/control_torque", left_front_control_torque_, nan_);
        register_output("/chassis/left_back_wheel/control_torque", left_back_control_torque_, nan_);
        register_output("/chassis/right_back_wheel/control_torque", right_back_control_torque_, nan_);
        register_output("/chassis/right_front_wheel/control_torque", right_front_control_torque_, nan_);
    }

    void update() override {
        const Eigen::Vector3d control_velocity = chassis_control_velocity_->vector;
        if (!control_velocity.allFinite() || !std::isfinite(*power_limit_)
            || !std::isfinite(*chassis_radius_) || *chassis_radius_ <= 1e-6) {
            return reset_all_controls();
        }

        const Eigen::Vector4d wheel_velocities = {
            *left_front_velocity_, *left_back_velocity_, *right_back_velocity_, *right_front_velocity_};
        if (!all_finite_(wheel_velocities))
            return reset_all_controls();

        const Eigen::Vector4d wheel_max_torque{
            *left_front_wheel_max_torque_, *left_back_wheel_max_torque_, *right_back_wheel_max_torque_,
            *right_front_wheel_max_torque_};
        if (!all_finite_(wheel_max_torque))
            return reset_all_controls();

        const Eigen::Vector4d contact_weight = weighted_contact_();
        if (!all_finite_(contact_weight))
            return reset_all_controls();

        const auto chassis_velocity = calculate_chassis_velocity_weighted_(wheel_velocities, contact_weight);
        if (!all_finite_(chassis_velocity))
            return reset_all_controls();

        auto chassis_control_torque = calculate_chassis_control_torque_(chassis_velocity);
        if (!all_finite_(chassis_control_torque.torque) || !all_finite_(chassis_control_torque.lambda))
            return reset_all_controls();

        const auto wheel_pid_torques = calculate_wheel_pid_torques_(wheel_velocities, chassis_velocity);
        if (!all_finite_(wheel_pid_torques))
            return reset_all_controls();

        chassis_control_torque.torque = constrain_chassis_control_torque_(
            wheel_velocities, chassis_control_torque, wheel_pid_torques, contact_weight);
        if (!all_finite_(chassis_control_torque.torque))
            return reset_all_controls();

        const auto wheel_control_torques = calculate_wheel_control_torques_weighted_(
            chassis_control_torque, wheel_pid_torques, contact_weight, wheel_max_torque);
        if (!all_finite_(wheel_control_torques))
            return reset_all_controls();

        *left_front_control_torque_ = wheel_control_torques[0];
        *left_back_control_torque_ = wheel_control_torques[1];
        *right_back_control_torque_ = wheel_control_torques[2];
        *right_front_control_torque_ = wheel_control_torques[3];
    }

private:
    struct ChassisControlTorque {
        Eigen::Vector2d torque = Eigen::Vector2d::Zero();
        Eigen::Vector2d lambda = Eigen::Vector2d::UnitX();
    };

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double g_ = 9.81;

    void reset_all_controls() {
        translational_velocity_pid_calculator_.reset();
        angular_velocity_pid_calculator_.reset();
        wheel_velocity_pid_.reset();

        *left_front_control_torque_ = 0.0;
        *left_back_control_torque_ = 0.0;
        *right_back_control_torque_ = 0.0;
        *right_front_control_torque_ = 0.0;
    }

    static bool all_finite_(const Eigen::Vector2d& values) { return values.allFinite(); }
    static bool all_finite_(const Eigen::Vector3d& values) { return values.allFinite(); }
    static bool all_finite_(const Eigen::Vector4d& values) { return values.allFinite(); }

    Eigen::Vector4d weighted_contact_() const {
        Eigen::Vector4d weight{
            *left_front_contact_confidence_, *left_back_contact_confidence_,
            *right_back_contact_confidence_, *right_front_contact_confidence_};
        for (int i = 0; i < weight.size(); ++i) {
            if (!std::isfinite(weight[i]))
                weight[i] = 1.0;
            weight[i] = std::clamp(weight[i], min_contact_weight_, 1.0);
        }
        return weight;
    }

    Eigen::Matrix<double, 4, 3> wheel_kinematics_(double chassis_radius) const {
        const double a_plus_b = std::numbers::sqrt2 * chassis_radius;
        Eigen::Matrix<double, 4, 3> matrix;
        matrix << -1.0, 1.0, a_plus_b, -1.0, -1.0, a_plus_b, 1.0, -1.0, a_plus_b, 1.0, 1.0,
            a_plus_b;
        matrix *= -1.0 / (std::numbers::sqrt2 * wheel_radius_);
        return matrix;
    }

    Eigen::Vector3d calculate_chassis_velocity_weighted_(
        const Eigen::Vector4d& wheel_velocities, const Eigen::Vector4d& contact_weight) const {
        const Eigen::Matrix<double, 4, 3> a = wheel_kinematics_(std::max(*chassis_radius_, 1e-6));
        const Eigen::Matrix4d w = contact_weight.asDiagonal();
        const Eigen::Matrix3d lhs =
            a.transpose() * w * a + Eigen::Matrix3d::Identity() * 1e-6;
        const Eigen::Vector3d rhs = a.transpose() * w * wheel_velocities;
        return lhs.ldlt().solve(rhs);
    }

    ChassisControlTorque calculate_chassis_control_torque_(const Eigen::Vector3d& chassis_velocity) {
        ChassisControlTorque result;
        Eigen::Vector3d err = chassis_control_velocity_->vector - chassis_velocity;
        const Eigen::Vector2d translational_torque =
            (-std::numbers::sqrt2 / 4.0 * wheel_radius_) * mass_
            * translational_velocity_pid_calculator_.update(err.head<2>());

        result.torque.x() = translational_torque.norm();

        const double a_plus_b = std::numbers::sqrt2 * std::max(*chassis_radius_, 1e-6);
        result.torque.y() = (-std::numbers::sqrt2 / 4.0 * wheel_radius_)
                          * (moment_of_inertia_ / a_plus_b)
                          * angular_velocity_pid_calculator_.update(err[2]);

        Eigen::Vector2d direction = Eigen::Vector2d::UnitX();
        if (result.torque.x() > 1e-6)
            direction = translational_torque / result.torque.x();
        result.lambda = {-direction.x() + direction.y(), -direction.x() - direction.y()};
        return result;
    }

    Eigen::Vector4d expected_wheel_velocity_(const Eigen::Vector3d& chassis_velocity) const {
        return wheel_kinematics_(std::max(*chassis_radius_, 1e-6)) * chassis_velocity;
    }

    Eigen::Vector4d calculate_wheel_pid_torques_(
        const Eigen::Vector4d& wheel_velocities, const Eigen::Vector3d& chassis_velocity) {
        return wheel_velocity_pid_.update(expected_wheel_velocity_(chassis_velocity) - wheel_velocities);
    }

    Eigen::Vector2d constrain_chassis_control_torque_(
        const Eigen::Vector4d& wheel_velocities, const ChassisControlTorque& chassis_control_torque,
        const Eigen::Vector4d& wheel_pid_torques, const Eigen::Vector4d& contact_weight) const {
        const double x_max = chassis_control_torque.torque.x();
        const double y_max = chassis_control_torque.torque.y();
        const double y_sign = y_max >= 0.0 ? 1.0 : -1.0;
        const double lambda_1 = chassis_control_torque.lambda.x();
        const double lambda_2 = chassis_control_torque.lambda.y();

        const double effective_contact = contact_weight.mean();
        const double rhombus_top =
            (friction_coefficient_ * effective_contact * mass_ * g_ * wheel_radius_) / 4.0;
        const double rhombus_right =
            rhombus_top / std::max(std::max(std::abs(lambda_1), std::abs(lambda_2)), 1e-6);

        const double d = lambda_1 * (wheel_velocities[0] - wheel_velocities[2]
                                 + 2.0 * k1_ * (wheel_pid_torques[0] - wheel_pid_torques[2]))
                       + lambda_2 * (wheel_velocities[1] - wheel_velocities[3]
                                 + 2.0 * k1_ * (wheel_pid_torques[1] - wheel_pid_torques[3]));
        const double e =
            y_sign * (2.0 * k1_ * wheel_pid_torques.sum() + wheel_velocities.sum());
        const double f = k1_ * wheel_pid_torques.array().square().sum()
                       + (wheel_pid_torques.array() * wheel_velocities.array()).sum()
                       + k2_ * wheel_velocities.array().square().sum() - no_load_power_
                       - *power_limit_;

        Eigen::Vector2d constrained = qcp_solver_.solve(
            {1.0, 1.0}, {x_max, std::abs(y_max)}, {rhombus_right, rhombus_top},
            {4.0 * k1_, 0.0, 4.0 * k1_, d, e, f});
        constrained.y() *= y_sign;
        return constrained;
    }

    Eigen::Vector4d calculate_wheel_control_torques_weighted_(
        const ChassisControlTorque& chassis_control_torque, const Eigen::Vector4d& wheel_pid_torques,
        const Eigen::Vector4d& contact_weight, const Eigen::Vector4d& wheel_max_torque) const {
        const Eigen::Vector4d s{
            chassis_control_torque.lambda.x(), chassis_control_torque.lambda.y(),
            -chassis_control_torque.lambda.x(), -chassis_control_torque.lambda.y()};

        Eigen::Matrix<double, 2, 4> c;
        c.row(0) = s.transpose();
        c.row(1) = Eigen::RowVector4d::Ones();

        Eigen::Vector2d d;
        d.x() = chassis_control_torque.torque.x() * s.squaredNorm();
        d.y() = 4.0 * chassis_control_torque.torque.y();

        const Eigen::Matrix4d w = contact_weight.asDiagonal();
        const Eigen::Matrix2d normal = c * w * c.transpose();
        Eigen::Vector4d wheel_torques =
            w * c.transpose() * normal.ldlt().solve(d);
        wheel_torques += wheel_pid_torques;

        double scale = 1.0;
        for (int i = 0; i < wheel_torques.size(); ++i) {
            const double allowed = std::max(wheel_max_torque[i] * contact_weight[i], 1e-6);
            if (std::abs(wheel_torques[i]) > allowed)
                scale = std::min(scale, allowed / std::abs(wheel_torques[i]));
        }
        if (scale < 1.0)
            wheel_torques *= scale;
        return wheel_torques;
    }

    const double mass_;
    const double moment_of_inertia_;
    const double wheel_radius_;
    const double friction_coefficient_;
    const double k1_;
    const double k2_;
    const double no_load_power_;
    const double min_contact_weight_;

    InputInterface<double> left_front_wheel_max_torque_;
    InputInterface<double> left_back_wheel_max_torque_;
    InputInterface<double> right_back_wheel_max_torque_;
    InputInterface<double> right_front_wheel_max_torque_;

    InputInterface<double> left_front_velocity_;
    InputInterface<double> left_back_velocity_;
    InputInterface<double> right_back_velocity_;
    InputInterface<double> right_front_velocity_;

    InputInterface<double> left_front_contact_confidence_;
    InputInterface<double> left_back_contact_confidence_;
    InputInterface<double> right_back_contact_confidence_;
    InputInterface<double> right_front_contact_confidence_;

    InputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
    InputInterface<double> power_limit_;
    InputInterface<double> chassis_radius_;

    pid::MatrixPidCalculator<2> translational_velocity_pid_calculator_;
    pid::PidCalculator angular_velocity_pid_calculator_;
    pid::MatrixPidCalculator<4> wheel_velocity_pid_;

    rmcs_core::controller::chassis::QcpSolver qcp_solver_;

    OutputInterface<double> left_front_control_torque_;
    OutputInterface<double> left_back_control_torque_;
    OutputInterface<double> right_back_control_torque_;
    OutputInterface<double> right_front_control_torque_;
};

} // namespace rmcs_core::controller::active_suspension

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::active_suspension::AdaptiveDeformableOmniWheelController,
    rmcs_executor::Component)
