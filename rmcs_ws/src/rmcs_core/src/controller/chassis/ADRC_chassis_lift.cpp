#include <cstdlib>
#include <numbers>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <ctime>
#include <limits>

#include "std_msgs/msg/int32.hpp"
#include "controller/adrc/adrc_calculator.hpp" 

namespace rmcs_core::controller::chassis {

class ChassisLiftController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ChassisLiftController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , wheel_adrcs{adrc::ADRCController(0.001), adrc::ADRCController(0.001), 
                      adrc::ADRCController(0.001), adrc::ADRCController(0.001)}
        , min_angle_(get_parameter_or("min_angle", 15.0))
        , max_angle_(get_parameter_or("max_angle", 55.0))
    {        
        register_input("/chassis/lift/target_angle", target_angle_);
        register_input("/chassis/lift/left_front_wheel/encoder_angle", left_front_wheel_angle_);
        register_input("/chassis/lift/left_back_wheel/encoder_angle", left_back_wheel_angle_);
        register_input("/chassis/lift/right_front_wheel/encoder_angle", right_front_wheel_angle_);
        register_input("/chassis/lift/right_back_wheel/encoder_angle", right_back_wheel_angle_);

        L0 = get_parameter("Initial_length").as_double();
        Bx = get_parameter("Rod_relative_horizontal_coordinate").as_double();
        By = get_parameter("Rod_relative_longitudinal_coordinate").as_double();
        L = get_parameter("Rod_length").as_double();

        register_output("/chassis/lift/left_front_wheel/control_torque", left_front_wheel_torque_, nan_);
        register_output("/chassis/lift/left_back_wheel/control_torque", left_back_wheel_torque_, nan_);
        register_output("/chassis/lift/right_front_wheel/control_torque", right_front_wheel_torque_, nan_);
        register_output("/chassis/lift/right_back_wheel/control_torque", right_back_wheel_torque_, nan_);

        wheel_adrcs[0].set_params(50.0, 1.0, 2.0, 0.5, 0.2, 0.01, 25.0, 6.0);
        for (int i = 1; i < 4; ++i) {
            wheel_adrcs[i].set_params(50.0, 1.0, 2.0, 0.5, 0.2, 0.01, 25.0, 6.0);
        }

        init_displacement();
    }

    void update() override {
        double target_angle_clamped = std::clamp(*target_angle_, min_angle_, max_angle_);
        double target_s = trapezoidal_calculator(target_angle_clamped);
        s_lf = trapezoidal_calculator(65 - *left_front_wheel_angle_);
        s_lb = trapezoidal_calculator(65 - *left_back_wheel_angle_);
        s_rf = trapezoidal_calculator(65 - *right_front_wheel_angle_);
        s_rb = trapezoidal_calculator(65 - *right_back_wheel_angle_);

        double lf_adrc_output = wheel_adrcs[0].compute_adrc_output(target_s, s_lf, last_u[0]);
        last_u[0] = lf_adrc_output;
        double lf_err = s_lf - last_u[0];
        
        double lb_adrc_output = wheel_adrcs[1].compute_adrc_output(target_s, s_lb, last_u[1]);
        last_u[1] = lb_adrc_output;
        double lb_err = s_lb - last_u[1];
        
        double rf_adrc_output = wheel_adrcs[2].compute_adrc_output(target_s, s_rf, last_u[2]);
        last_u[2] = rf_adrc_output;
        double rf_err = s_rf - last_u[2];
        
        double rb_adrc_output = wheel_adrcs[3].compute_adrc_output(target_s, s_rb, last_u[3]);
        last_u[3] = rb_adrc_output;
        double rb_err = s_rb - last_u[3];

        double lf_torque = calculate_torque(lf_err);
        double lb_torque = calculate_torque(lb_err);
        double rf_torque = calculate_torque(rf_err);
        double rb_torque = calculate_torque(rb_err);

        *left_front_wheel_torque_ = std::clamp(lf_torque, -max_torque_, max_torque_);
        *left_back_wheel_torque_  = std::clamp(lb_torque, -max_torque_, max_torque_);
        *right_front_wheel_torque_ = std::clamp(rf_torque, -max_torque_, max_torque_);
        *right_back_wheel_torque_  = std::clamp(rb_torque, -max_torque_, max_torque_);
    }

private:
    void init_displacement() {
        s_lf = s_lb = s_rf = s_rb = 0.0;
        last_u[0] = last_u[1] = last_u[2] = last_u[3] = 0.0;
    }

    static double calculate_torque(double s_err) {
        const double torque_gain = 0.2; 
        double base_torque = s_err * torque_gain; 
        return std::clamp(base_torque, -4.0, 4.0); 
    }

    void stop_lift() {
        *left_front_wheel_torque_ = 0.0;
        *left_back_wheel_torque_ = 0.0;
        *right_front_wheel_torque_ = 0.0;
        *right_back_wheel_torque_ = 0.0;
        RCLCPP_INFO(get_logger(), "Stopping lift");
    }

    double trapezoidal_calculator(double alpha) const {
        double alpha_rad = alpha * pi / 180;
        double term = Bx * cos(alpha_rad) + By * sin(alpha_rad);
        double sqrt_term = Bx * std::sin(alpha_rad) - By * std::cos(alpha_rad) + 15.0;
        double sqrt_arg = std::max(L * L - sqrt_term * sqrt_term, 0.0);
        return term + sqrt(sqrt_arg) - L0; 
    }

    void init_calculator(std_msgs::msg::Int32::UniquePtr) {
        lf = trapezoidal_calculator(65 - *left_front_wheel_angle_);
        lb = trapezoidal_calculator(65 - *left_back_wheel_angle_);
        rf = trapezoidal_calculator(65 - *right_front_wheel_angle_);
        rb = trapezoidal_calculator(65 - *right_back_wheel_angle_);
        s_lf = lf; s_lb = lb; s_rf = rf; s_rb = rb;
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

    adrc::ADRCController wheel_adrcs[4];
    double last_u[4]; // 上一周期ADRC控制量

    const double min_angle_;
    const double max_angle_;
    const double max_torque_ = 4.0;

    double lf, lb, rf, rb;
    double s_lf, s_lb, s_rf, s_rb;

    double L0;
    double Bx, By;
    double L;
    double dt = 0.001; 
    double pi = std::numbers::pi;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ChassisLiftController, rmcs_executor::Component)