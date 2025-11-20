
#include <cstdlib>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <eigen3/Eigen/Dense>
#include <tuple>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class ChassisLiftController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ChassisLiftController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , balance_pid{  
            {5.0, 0.1, 0.05},  
            {5.0, 0.1, 0.05}, 
            {5.0, 0.1, 0.05},  
            {5.0, 0.1, 0.05}   
            }
        {        
        register_input("/chassis/lift/target_angle", target_angle_);
        register_input("/chassis/lift/left_front_wheel/encoder_angle", left_front_wheel_angle_);
        register_input("/chassis/lift/left_back_wheel/encoder_angle", left_back_wheel_angle_);
        register_input("/chassis/lift/right_front_wheel/encoder_angle", right_front_wheel_angle_);
        register_input("/chassis/lift/right_back_wheel/encoder_angle", right_back_wheel_angle_);

        register_output("/chassis/lift/left_front_wheel/control_torque", left_front_wheel_torque_, nan_);
        register_output("/chassis/lift/left_back_wheel/control_torque", left_back_wheel_torque_, nan_);
        register_output("/chassis/lift/right_front_wheel/control_torque", right_front_wheel_torque_, nan_);
        register_output("/chassis/lift/right_back_wheel/control_torque", right_back_wheel_torque_, nan_);
        }

    void update() override {
        double err_lf = *target_angle_ - *left_front_wheel_angle_;
        double err_lb = *target_angle_ - *left_back_wheel_angle_;
        double err_rf = *target_angle_ - *right_front_wheel_angle_;
        double err_rb = *target_angle_ - *right_back_wheel_angle_;

        auto calc_torque = [this](rmcs_core::controller::pid::PidCalculator& pid, double error) {
            double torque = pid.update(error);
            return std::clamp(torque, -max_torque_, max_torque_);
        };

        *left_front_wheel_torque_ = calc_torque(balance_pid[0], err_lf);
        *left_back_wheel_torque_  = calc_torque(balance_pid[1], err_lb);
        *right_front_wheel_torque_ = calc_torque(balance_pid[2], err_rf);
        *right_back_wheel_torque_  = calc_torque(balance_pid[3], err_rb);
        
    }
private:

    void stop_lift(){
         *left_front_wheel_torque_ = 0.0;
          *left_back_wheel_torque_ = 0.0;
        *right_front_wheel_torque_ = 0.0;
         *right_back_wheel_torque_ = 0.0;
        RCLCPP_INFO(get_logger(), "Stopping lift");
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;


    InputInterface<double> target_angle_;
    InputInterface<double> left_front_wheel_angle_;
    InputInterface<double> left_front_wheel_encoder_angle_;
    InputInterface<double> left_back_wheel_angle_;
    InputInterface<double> right_front_wheel_angle_;
    InputInterface<double> right_back_wheel_angle_;

    OutputInterface<double> left_front_wheel_torque_;
    OutputInterface<double> left_back_wheel_torque_;
    OutputInterface<double> right_front_wheel_torque_;
    OutputInterface<double> right_back_wheel_torque_;

    rmcs_core::controller::pid::PidCalculator balance_pid[4];

    const double max_torque_ = 6.0;  
};

} // namespace rmcs_core::controller::chassis


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ChassisLiftController, rmcs_executor::Component)