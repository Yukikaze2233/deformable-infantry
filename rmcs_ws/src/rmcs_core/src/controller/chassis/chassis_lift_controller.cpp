#include <cstdlib>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <algorithm>

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
            {0.03436926, 0.000001, 0.001},  
            {1.0, 0.001, 0.01},  
            {1.0, 0.001, 0.01},  
            {1.0, 0.001, 0.01}   
        }
        , min_angle_( get_parameter_or("min_angle", -1.57) )
        , max_angle_( get_parameter_or("max_angle", 1.57) )
    {        
        register_input("/chassis/lift/target_angle", target_angle_);
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

        double target = trapezoidal_calaulator(*target_angle_);
        chassis_lift_controller_ = create_subscription<std_msgs::msg::Int32>(
        "/chassis/lift/target_angle", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
            init_calculator(std::move(msg));
        });


        target = (target < min_angle_) ? min_angle_ : (target > max_angle_) ? max_angle_ : target;
        
        auto calc_error = [target](double current) {
            double err = target - current;
            return std::atan2(std::sin(err), std::cos(err));  
        };

        s_lf = lf + *left_front_wheel_velocity_/(2 * pi) * dt;
        s_lb = lb + *left_back_wheel_velocity_/(2 * pi) * dt;
        s_rf = rf + *right_front_wheel_velocity_/(2 * pi) * dt;
        s_rb = rb + *right_back_wheel_velocity_/(2 * pi) * dt;

        double lf_err = calc_error(s_lf);
        double lb_err = calc_error(s_lb);
        double rf_err = calc_error(s_rf);
        double rb_err = calc_error(s_rb);

        *left_front_wheel_torque_ = std::clamp(wheel_pids[0].update(lf_err),-0.8,0.8);

        // *left_front_wheel_torque_ = 0.5;
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

    double trapezoidal_calaulator(double alpha) const{
        double term = Bx * cos(alpha) + By * sin(alpha);
        double s = term + sqrt(L * L - (By * std::sin(alpha) - By * std::cos(alpha) + 15) * (By * std::sin(alpha) - By * std::cos(alpha) + 15)) - L0;
        return s;
    }

    void init_calculator(std_msgs::msg::Int32::UniquePtr){
        lf = trapezoidal_calaulator(*left_front_wheel_angle_);
        lb = trapezoidal_calaulator(*left_back_wheel_angle_);
        rf = trapezoidal_calaulator(*right_front_wheel_angle_);
        rb = trapezoidal_calaulator(*right_back_wheel_angle_);
        s_lf = lf;
        s_lb = lb;
        s_rf = rf;
        s_rb = rb;
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
    double dt = 0.001;
    double pi = 3.1415926;
};

} // namespace rmcs_core::controller::chassis


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ChassisLiftController, rmcs_executor::Component)