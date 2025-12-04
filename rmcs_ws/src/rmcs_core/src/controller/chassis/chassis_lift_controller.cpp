#include <cstdlib>
#include <numbers>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <ctime>

#include "controller/pid/pid_calculator.hpp"
#include "std_msgs/msg/int32.hpp"

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
            {0.5, 0.001, 0.01},  
            {1.0, 0.001, 0.01},  
            {1.0, 0.001, 0.01},  
            {1.0, 0.001, 0.01} 
        ,  
        }
        , min_angle_( get_parameter_or("min_angle", 15) )
        , max_angle_( get_parameter_or("max_angle", 55) )
    {        
        register_input("/chassis/lift/target_angle", target_angle_);/*15-55*/
        register_input("/chassis/lift/left_front_wheel/encoder_angle", left_front_wheel_angle_);
        register_input("/chassis/lift/left_back_wheel/encoder_angle", left_back_wheel_angle_);
        register_input("/chassis/lift/right_front_wheel/encoder_angle", right_front_wheel_angle_);
        register_input("/chassis/lift/right_back_wheel/encoder_angle", right_back_wheel_angle_);

        register_input("/chassis/lift/left_front_wheel/velocity", left_front_wheel_velocity_);
        register_input("/chassis/lift/left_back_wheel/velocity", left_back_wheel_velocity_);
        register_input("/chassis/lift/right_front_wheel/velocity", right_front_wheel_velocity_);
        register_input("/chassis/lift/right_back_wheel/velocity", right_back_wheel_velocity_);

        L0 = get_parameter("Initial_length").as_double();
        Bx = get_parameter("Rod_relative_horizontal_coordinate").as_double();
        By = get_parameter("Rod_relative_longitudinal_coordinate").as_double();
        L = get_parameter("Rod_length").as_double();
        Load = get_parameter("Load").as_double();
        reduction_ratio = get_parameter("reduction_ratio").as_double();

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
        double target = trapezoidal_calculator(*target_angle_);
        if (!chassis_lift_controller_) {
            chassis_lift_controller_ = create_subscription<std_msgs::msg::Int32>(
                "/chassis/lift/target_angle", rclcpp::QoS{10}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                    stop_lift();
                    init_calculator(std::move(msg));
                });
        }
        s_lf -= *left_front_wheel_velocity_/(2 * pi) * dt * reduction_ratio;
        s_lb -= *left_back_wheel_velocity_/(2 * pi) * dt * reduction_ratio;
        s_rf -= *right_front_wheel_velocity_/(2 * pi) * dt * reduction_ratio;
        s_rb -= *right_back_wheel_velocity_/(2 * pi) * dt * reduction_ratio;

        double lf_err = s_lf - target;
        double lb_err = s_lb - target;
        double rf_err = s_rf - target;
        double rb_err = s_rb - target;

        double lf_torque = std::clamp(wheel_pids[0].update(lf_err),-0.8,0.8) + frictional_resistance * get_double_sign(lf_err) + feedback_forward(*left_front_wheel_angle_);
        double lb_torque = std::clamp(wheel_pids[1].update(lb_err),-0.8,0.8) + frictional_resistance * get_double_sign(lb_err) + feedback_forward(*left_back_wheel_angle_);
        double rf_torque = std::clamp(wheel_pids[2].update(rf_err),-0.8,0.8) + frictional_resistance * get_double_sign(rf_err) + feedback_forward(*right_front_wheel_angle_);
        double rb_torque = std::clamp(wheel_pids[3].update(rb_err),-0.8,0.8) + frictional_resistance * get_double_sign(rb_err) + feedback_forward(*right_back_wheel_angle_);

        *left_front_wheel_torque_ = lf_torque;
        *left_back_wheel_torque_  = lb_torque;
        *right_front_wheel_torque_ = rf_torque;
        *right_back_wheel_torque_  = rb_torque;

        
    }

private:

    void stop_lift(){
        *left_front_wheel_torque_ = 0.0;
        *left_back_wheel_torque_ = 0.0;
        *right_front_wheel_torque_ = 0.0;
        *right_back_wheel_torque_ = 0.0;
        RCLCPP_INFO(get_logger(), "Stopping lift");
    }

    double feedback_forward(double theta ) const{
        double F_a = 95 * Load * std::cos(theta) / 26;
        if(std::abs(*left_front_wheel_velocity_) > 0.1){
            return F_a * 0.001 / (2 * pi) * 0.7 * get_double_sign(*left_front_wheel_velocity_);
        }else{
            if(std::abs(*left_front_wheel_torque_) > 0.1){
                return F_a * 0.001 / (2 * pi) * 0.7 * get_double_sign(*left_front_wheel_torque_);
            }else{
                return 0;
            }
        }

    }

    double trapezoidal_calculator(double alpha) const{
        double term = Bx * cos(alpha * pi / 180) + By * sin(alpha * pi / 180);
        double s = term + sqrt(L * L - (Bx * std::sin(alpha * pi / 180) - By * std::cos(alpha * pi / 180) + 15) * (Bx * std::sin(alpha * pi / 180) - By * std::cos(alpha * pi / 180) + 15)) ;
        return s;
    }

    double angle_calculator(double s) const {

        double A = 2 * s * Bx + 30 * By;   
        double B = 2 * s * By - 30 * Bx;  
        double C = s * s + Bx * Bx + By * By - 9775.0;  

        double cos_val = C / std::sqrt(A * A + B * B);
        cos_val = std::max(-1.0, std::min(1.0, cos_val));
    
        return std::atan2(B, A) - std::acos(cos_val);
    }

    void init_calculator(std_msgs::msg::Int32::UniquePtr){
        lf = trapezoidal_calculator(65 - *left_front_wheel_angle_);
        lb = trapezoidal_calculator(65 - *left_back_wheel_angle_);
        rf = trapezoidal_calculator(65 - *right_front_wheel_angle_);
        rb = trapezoidal_calculator(65 - *right_back_wheel_angle_);
        s_lf = lf;
        s_lb = lb;
        s_rf = rf;
        s_rb = rb;
    }

    static int get_double_sign(double x) {
        return std::signbit(x) ? -1 : 1;
    }


    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr chassis_lift_controller_;

    InputInterface<double> target_angle_;
    InputInterface<double> left_front_wheel_angle_;
    InputInterface<double> left_back_wheel_angle_;
    InputInterface<double> right_front_wheel_angle_;
    InputInterface<double> right_back_wheel_angle_;
    InputInterface<double> left_front_wheel_velocity_;
    InputInterface<double> left_back_wheel_velocity_;
    InputInterface<double> right_front_wheel_velocity_;
    InputInterface<double> right_back_wheel_velocity_;

    OutputInterface<double> left_front_wheel_torque_;
    OutputInterface<double> left_back_wheel_torque_;
    OutputInterface<double> right_front_wheel_torque_;
    OutputInterface<double> right_back_wheel_torque_;

    rmcs_core::controller::pid::PidCalculator wheel_pids[4];

    const double min_angle_ ;
    const double max_angle_ ;

    const double max_torque_ = 6.0;


    double lf, lb, rf, rb;
    double s_lf, s_lb, s_rf, s_rb;

    double L0;
    double Bx, By;
    double L;
    double Load;
    double frictional_resistance = 0.49158;
    double dt = 0.001;
    double pi = std::numbers::pi;
    double reduction_ratio;

};

} // namespace rmcs_core::controller::chassis


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ChassisLiftController, rmcs_executor::Component)