
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
            {2.0, 0.1, 0.05},  
            {2.0, 0.1, 0.05}, 
            {2.0, 0.1, 0.05},  
            {2.0, 0.1, 0.05}   
            }
        , overall_pid{10.0, 0.5, 2.0}
        , angle_tolerance_(0.01)
        {        
        register_input("/chassis/lift/target_angle", target_angle_);
        register_input("/chassis/lift/encoder_angle", encoder_angle_);
        register_input("/chassis/lift/left_front_wheel/angle", left_front_wheel_angle_);
        register_input("/chassis/lift/left_back_wheel/angle", left_back_wheel_angle_);
        register_input("/chassis/lift/right_front_wheel/angle", right_front_wheel_angle_);
        register_input("/chassis/lift/right_back_wheel/angle", right_back_wheel_angle_);

        register_output("/chassis/lift/left_front_wheel/control_torque", left_front_wheel_torque_, nan_);
        register_output("/chassis/lift/left_back_wheel/control_torque", left_back_wheel_torque_, nan_);
        register_output("/chassis/lift/right_front_wheel/control_torque", right_front_wheel_torque_, nan_);
        register_output("/chassis/lift/right_back_wheel/control_torque", right_back_wheel_torque_, nan_);

        overall_pid.output_max = max_torque_ * 0.8;  
        overall_pid.output_min = -max_torque_ * 0.8;
        }

    void update() override {

        double target = *target_angle_;
        double current_overall = *encoder_angle_;  
        double lf = *left_front_wheel_angle_;
        double lb = *left_back_wheel_angle_;
        double rf = *right_front_wheel_angle_;
        double rb = *right_back_wheel_angle_;

        double overall_err = target - current_overall;
        overall_err = std::atan2(std::sin(overall_err), std::cos(overall_err));

        double base_torque = overall_pid.update(overall_err);

        auto [comp_lf, comp_lb, comp_rf, comp_rb] = torque_calculator(lf, lb, rf, rb);
        comp_lf = -comp_lf;
        comp_lb = -comp_lb;
        comp_rf = -comp_rf;
        comp_rb = -comp_rb;

        auto calc_final_torque = [this](double base, double comp) {
            double total = base + comp;
            return std::clamp(total, -max_torque_, max_torque_);
        };

        if (std::abs(overall_err) < angle_tolerance_) {

            *left_front_wheel_torque_ = calc_final_torque(base_torque, comp_lf);
            *left_back_wheel_torque_  = calc_final_torque(base_torque, comp_lb);
            *right_front_wheel_torque_ = calc_final_torque(base_torque, comp_rf);
            *right_back_wheel_torque_  = calc_final_torque(base_torque, comp_rb);

        }
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


    InputInterface<double> target_angle_;
    InputInterface<double> encoder_angle_;
    InputInterface<double> left_front_wheel_angle_;
    InputInterface<double> left_back_wheel_angle_;
    InputInterface<double> right_front_wheel_angle_;
    InputInterface<double> right_back_wheel_angle_;

    OutputInterface<double> left_front_wheel_torque_;
    OutputInterface<double> left_back_wheel_torque_;
    OutputInterface<double> right_front_wheel_torque_;
    OutputInterface<double> right_back_wheel_torque_;

    rmcs_core::controller::pid::PidCalculator balance_pid[4];
    rmcs_core::controller::pid::PidCalculator overall_pid;

    const double max_torque_ = 6.0;
    const double angle_tolerance_;  
};

} // namespace rmcs_core::controller::chassis


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ChassisLiftController, rmcs_executor::Component)