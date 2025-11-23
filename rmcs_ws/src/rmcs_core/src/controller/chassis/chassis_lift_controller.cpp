#include <cstdlib>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>

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
        , wheel_pids{  
            {14.0, 0.001, 0.05},  
            {14.0, 0.001, 0.05},  
            {14.0, 0.001, 0.05},  
            {14.0, 0.001, 0.05}   
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

        for (auto& pid : wheel_pids) {
            pid.output_max = max_torque_;
            pid.output_min = -max_torque_;
        } 
    }

    void update() override {

        double target = *target_angle_;
        double lf = *left_front_wheel_angle_;
        double lb = *left_back_wheel_angle_;
        double rf = *right_front_wheel_angle_;
        double rb = *right_back_wheel_angle_;

        target = (target < min_angle_) ? min_angle_ : (target > max_angle_) ? max_angle_ : target;  //Soft Limit

        auto calc_error = [target](double current) {
            double err = target - current;
            return std::atan2(std::sin(err), std::cos(err));  
        };
        double lf_err = calc_error(lf);
        double lb_err = calc_error(lb);
        double rf_err = calc_error(rf);
        double rb_err = calc_error(rb);

        *left_front_wheel_torque_ = wheel_pids[0].update(lf_err);
        *left_back_wheel_torque_  = wheel_pids[1].update(lb_err);
        *right_front_wheel_torque_ = wheel_pids[2].update(rf_err);
        *right_back_wheel_torque_  = wheel_pids[3].update(rb_err);

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

    InputInterface<double> target_angle_;
    InputInterface<double> left_front_wheel_angle_;
    InputInterface<double> left_back_wheel_angle_;
    InputInterface<double> right_front_wheel_angle_;
    InputInterface<double> right_back_wheel_angle_;

    OutputInterface<double> left_front_wheel_torque_;
    OutputInterface<double> left_back_wheel_torque_;
    OutputInterface<double> right_front_wheel_torque_;
    OutputInterface<double> right_back_wheel_torque_;

    rmcs_core::controller::pid::PidCalculator wheel_pids[4];


    const double max_torque_ = 6.0;
    const double max_angle_ = 80.24;
    const double min_angle_ = 1.00;                    
};

} // namespace rmcs_core::controller::chassis


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ChassisLiftController, rmcs_executor::Component)