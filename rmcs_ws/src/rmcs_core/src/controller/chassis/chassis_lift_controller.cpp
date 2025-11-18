
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
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        
        register_input("/chassis/lift/left_front_wheel/angle", left_front_wheel_angle_);
        register_input("/chassis/lift/left_front_wheel/encoder_angle", left_front_wheel_encoder_angle_);
        register_input("/chassis/lift/left_back_wheel/angle", left_back_wheel_angle_);
        register_input("/chassis/lift/right_front_wheel/angle", right_front_wheel_angle_);
        register_input("/chassis/lift/right_back_wheel/angle", right_back_wheel_angle_);

        register_output("/chassis/lift/left_front_wheel/control_torque", left_front_wheel_torque_, nan_);
        register_output("/chassis/lift/left_back_wheel/control_torque", left_back_wheel_torque_, nan_);
        register_output("/chassis/lift/right_front_wheel/control_torque", right_front_wheel_torque_, nan_);
        register_output("/chassis/lift/right_back_wheel/control_torque", right_back_wheel_torque_, nan_);
        }

    void update() override {
        
        auto switch_right_state = *switch_right_;
        auto switch_left_state = *switch_left_;
        
        auto joystick_right_state = *joystick_right_;

        auto left_front_wheel_angle = *left_front_wheel_angle_;
        auto left_back_wheel_angle = *left_back_wheel_angle_;
        auto right_front_wheel_angle = *right_front_wheel_angle_;
        auto right_back_wheel_angle = *right_back_wheel_angle_;

        
        // if (switch_right_state == rmcs_msgs::Switch::DOWN && 
        //     switch_left_state == rmcs_msgs::Switch::DOWN) {
        //     stop_lift();
        //     return;
        // }
        // else if(switch_right_state == rmcs_msgs::Switch::MIDDLE && 
        //         switch_left_state == rmcs_msgs::Switch::MIDDLE){
        //     double target_torque = 2.5/*joystick_right_state.x() * max_torque_*/;

        //     if(std::abs(target_torque) < 1.2){
        //         target_torque = 0.0;
        //     }

        //     auto [left_front_compensation, left_back_compensation, right_front_compensation, right_back_compensation] = torque_calculator(left_front_wheel_angle, left_back_wheel_angle, right_front_wheel_angle, right_back_wheel_angle);
            
        //     double left_front_target_torque = std::clamp(target_torque + left_front_compensation, -max_torque_, max_torque_);
        //     double left_back_target_torque = std::clamp(target_torque + left_back_compensation, -max_torque_, max_torque_);
        //     double right_front_target_torque = std::clamp(target_torque + right_front_compensation, -max_torque_, max_torque_);
        //     double right_back_target_torque = std::clamp(target_torque + right_back_compensation, -max_torque_, max_torque_);
            
        //     *left_front_wheel_torque_ = target_torque/*left_front_target_torque*/;
        //     *left_back_wheel_torque_ = left_back_target_torque;
        //     *right_front_wheel_torque_ = right_front_target_torque;
        //     *right_back_wheel_torque_ = right_back_target_torque;
            
        // }
        // else {
        //     stop_lift();
        //     return;
        // }
         *left_front_wheel_torque_ = -0.7;
        RCLCPP_INFO(
            get_logger(), "Max control torque of wheel motor: %f",
            *left_front_wheel_encoder_angle_);
         
    }
private:

    void stop_lift(){
         *left_front_wheel_torque_ = 0.0;
          *left_back_wheel_torque_ = 0.0;
        *right_front_wheel_torque_ = 0.0;
         *right_back_wheel_torque_ = 0.0;
        RCLCPP_INFO(get_logger(), "Stopping lift");
    }

    std::tuple<double, double,double,double> torque_calculator(double angle1,double angle2,double angle3,double angle4){
        double avg_angle = ( angle1 + angle2 + angle3 + angle4 ) / 4.0;
        double angle_diff1 = angle1 - avg_angle ;
        double angle_diff2 = angle2 - avg_angle ;
        double angle_diff3 = angle3 - avg_angle ;
        double angle_diff4 = angle4 - avg_angle ;
        double compensation1 = balance_pid[0].update(angle_diff1);
        double compensation2 = balance_pid[1].update(angle_diff2);
        double compensation3 = balance_pid[2].update(angle_diff3);
        double compensation4 = balance_pid[3].update(angle_diff4);
        return std::make_tuple(compensation1, compensation2, compensation3, compensation4);
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

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