#include <cstdlib>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <random>
#include <chrono>
#include <algorithm>
#include <ctime>
#include <limits>

#include "std_msgs/msg/int32.hpp"
#include "controller/adrc/adrc_link_calculator.hpp" 

namespace rmcs_core::virtue::chassis {

class ChassisLiftController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ChassisLiftController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , engine(seed)
        , min_angle_(get_parameter_or("min_angle", 15.0))
        , max_angle_(get_parameter_or("max_angle", 55.0))
    {        

        L0 = get_parameter("Initial_length").as_double();
        Bx = get_parameter("Rod_relative_horizontal_coordinate").as_double();
        By = get_parameter("Rod_relative_longitudinal_coordinate").as_double();
        L = get_parameter("Rod_length").as_double();



        wheel_adrcs.set_params(500.0, 1.0, 167.0, /*0.5, 0.2, 0.01,*/ 0.39408, 65.507);

        init_displacement();
    }

    void update() override {
        double target_angle_clamped = std::clamp(30.0 /*+ 15 * std::sin(0.1 * std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())*/, min_angle_, max_angle_);
        double target_s = trapezoidal_calculator(target_angle_clamped);
        s_lf = speed_simulate(torque_simulate(test_torque_));

        double s_lf_error = target_s - s_lf;
        double lf_adrc_output = wheel_adrcs.compute_adrc_output(target_s, s_lf);
        last_u = lf_adrc_output;
        double lf_torque = lf_adrc_output;
        // RCLCPP_INFO(get_logger(),"s_lf:%f",s_lf);   
        RCLCPP_INFO(get_logger(),"%f    %f    %f ",s_lf_error,lf_adrc_output,torque_simulate(test_torque_));
        test_torque_ = std::clamp(lf_torque, -max_torque_, max_torque_);
    }

private:
    void init_displacement() {
        s_lf = trapezoidal_calculator(int_dist(engine));
    }

    double torque_simulate(double torque){
        if((torque > 0) && (torque < 0.4)){
            return 0;
        }else if(torque >= 0.4) {
            return torque - 0.2;//double_dist1(engine);
        }else if((torque < 0) && (torque > -0.4)){
            return 0;
        }else{
            return torque + 0.2;
        }
    }



    double speed_simulate(double torque){
        double angular_acceleration = torque / 0.001;
        speed_simulater_ += angular_acceleration * 0.001; 
        return speed_simulater_/(2 * pi);
    }

    void stop_lift() {
        test_torque_ = 0.0;
        RCLCPP_INFO(get_logger(), "Stopping lift");
    }

    double trapezoidal_calculator(double alpha) const {
        double alpha_rad = alpha * pi / 180;
        double term = Bx * cos(alpha_rad) + By * sin(alpha_rad);
        double sqrt_term = Bx * std::sin(alpha_rad) - By * std::cos(alpha_rad) + 15.0;
        double sqrt_arg = std::max(L * L - sqrt_term * sqrt_term, 0.0);
        return term + sqrt(sqrt_arg) - L0; 
    }

    double angle_calculator(double s) const {
        double A = 2 * s * Bx + 30 * By;   
        double B = 2 * s * By - 30 * Bx;  
        double C = s * s + Bx * Bx + By * By - 9775.0;  

        double cos_val = C / std::sqrt(A * A + B * B);
        cos_val = std::max(-1.0, std::min(1.0, cos_val));
    
        return std::atan2(B, A) - std::acos(cos_val);
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    unsigned int seed = static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count());
    std::mt19937 engine;
    std::uniform_int_distribution<int> int_dist{15, 55};
    std::uniform_real_distribution<double> double_dist1{0.04, 0.2};
 

    controller::adrc::LinkADRCController wheel_adrcs;

    const double min_angle_ = 15.0;
    const double max_angle_ = 55.0;
    const double max_torque_ = 4.0;
    double speed_simulater_ = 0.0;
    double last_u = 0.0;
    double test_angle_ = 0.0, test_torque_ = 0.0;
    double s_lf;

    double L0;
    double Bx, By;
    double L;
    double pi = std::numbers::pi;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::virtue::chassis::ChassisLiftController, rmcs_executor::Component)