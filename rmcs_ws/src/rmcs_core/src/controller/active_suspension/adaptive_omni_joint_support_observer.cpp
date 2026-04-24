#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include <rclcpp/node.hpp>

#include <rmcs_executor/component.hpp>

#include "controller/adrc/ESO.hpp"

namespace rmcs_core::controller::active_suspension {

class AdaptiveOmniJointSupportObserver
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

    AdaptiveOmniJointSupportObserver()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/chassis/left_front_joint/physical_angle", left_front_joint_physical_angle_);
        register_input("/chassis/left_back_joint/physical_angle", left_back_joint_physical_angle_);
        register_input("/chassis/right_back_joint/physical_angle", right_back_joint_physical_angle_);
        register_input("/chassis/right_front_joint/physical_angle", right_front_joint_physical_angle_);

        register_input("/chassis/left_front_joint/torque", left_front_joint_torque_);
        register_input("/chassis/left_back_joint/torque", left_back_joint_torque_);
        register_input("/chassis/right_back_joint/torque", right_back_joint_torque_);
        register_input("/chassis/right_front_joint/torque", right_front_joint_torque_);

        register_output("/chassis/left_front_joint/support_observer_z3", left_front_support_observer_z3_, nan_);
        register_output("/chassis/left_back_joint/support_observer_z3", left_back_support_observer_z3_, nan_);
        register_output("/chassis/right_back_joint/support_observer_z3", right_back_support_observer_z3_, nan_);
        register_output("/chassis/right_front_joint/support_observer_z3", right_front_support_observer_z3_, nan_);

        const auto cfg = load_eso_config_();
        for (auto& observer : observers_)
            observer.set_config(cfg);
    }

    void update() override {
        update_joint_(
            observers_[kLeftFront], initialized_[kLeftFront], left_front_joint_physical_angle_,
            left_front_joint_torque_, *left_front_support_observer_z3_);
        update_joint_(
            observers_[kLeftBack], initialized_[kLeftBack], left_back_joint_physical_angle_,
            left_back_joint_torque_, *left_back_support_observer_z3_);
        update_joint_(
            observers_[kRightBack], initialized_[kRightBack], right_back_joint_physical_angle_,
            right_back_joint_torque_, *right_back_support_observer_z3_);
        update_joint_(
            observers_[kRightFront], initialized_[kRightFront], right_front_joint_physical_angle_,
            right_front_joint_torque_, *right_front_support_observer_z3_);
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    adrc::ESO::Config load_eso_config_() {
        adrc::ESO::Config cfg;
        cfg.h = get_parameter_or("dt", 0.001);
        cfg.b0 = get_parameter_or("b0", 1.0);
        cfg.w0 = get_parameter_or("eso_w0", 80.0);
        cfg.auto_beta = get_parameter_or("eso_auto_beta", true);
        cfg.beta1 = get_parameter_or("eso_beta1", 3.0 * cfg.w0);
        cfg.beta2 = get_parameter_or("eso_beta2", 3.0 * cfg.w0 * cfg.w0);
        cfg.beta3 = get_parameter_or("eso_beta3", cfg.w0 * cfg.w0 * cfg.w0);
        cfg.z3_limit = get_parameter_or("eso_z3_limit", 1e9);
        return cfg;
    }

    static void update_joint_(
        adrc::ESO& observer, bool& initialized, const InputInterface<double>& physical_angle,
        const InputInterface<double>& joint_torque, double& support_observer_z3) {
        const double measurement = *physical_angle;
        const double control_input = *joint_torque;
        if (!std::isfinite(measurement) || !std::isfinite(control_input)) {
            initialized = false;
            support_observer_z3 = nan_;
            return;
        }

        if (!initialized) {
            observer.reset(measurement);
            initialized = true;
            // Let downstream estimators fall back for one tick instead of treating warm-up as
            // a valid zero-support observation.
            support_observer_z3 = nan_;
            return;
        }

        const auto observer_output = observer.update(measurement, control_input);
        support_observer_z3 = std::isfinite(observer_output.z3) ? observer_output.z3 : nan_;
        if (!std::isfinite(support_observer_z3))
            initialized = false;
    }

    std::array<adrc::ESO, kJointCount> observers_{};
    std::array<bool, kJointCount> initialized_ = {false, false, false, false};

    InputInterface<double> left_front_joint_physical_angle_;
    InputInterface<double> left_back_joint_physical_angle_;
    InputInterface<double> right_back_joint_physical_angle_;
    InputInterface<double> right_front_joint_physical_angle_;

    InputInterface<double> left_front_joint_torque_;
    InputInterface<double> left_back_joint_torque_;
    InputInterface<double> right_back_joint_torque_;
    InputInterface<double> right_front_joint_torque_;

    OutputInterface<double> left_front_support_observer_z3_;
    OutputInterface<double> left_back_support_observer_z3_;
    OutputInterface<double> right_back_support_observer_z3_;
    OutputInterface<double> right_front_support_observer_z3_;
};

} // namespace rmcs_core::controller::active_suspension

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::active_suspension::AdaptiveOmniJointSupportObserver,
    rmcs_executor::Component)
