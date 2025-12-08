#include <algorithm>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "adrc_calculator.hpp"

namespace rmcs_core::controller::adrc {

class AdrcController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AdrcController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))

            {
        register_input(get_parameter("measurement").as_string(), measurement_);
        register_input(get_parameter("target").as_string(), target_);
        
        register_output(get_parameter("control").as_string(), control_error);
        KT = get_parameter("kt").as_double();

        adrc_calculator.set_params(
            R = get_parameter("r").as_double(),   // R
            B0 = get_parameter("b0").as_double(),    // B0
            BETA01 = get_parameter("beta01").as_double(),  // BETA01
            BETA02 = get_parameter("beta02").as_double(),   // BETA02
            BETA03 = get_parameter("beta03").as_double(),    // BETA03
            ALPHA1 = get_parameter("alpha1").as_double(),    // ALPHA1
            ALPHA2 = get_parameter("alpha2").as_double(),    // ALPHA2
            DELTA = get_parameter("delta").as_double(),   // DELTA
            KP = get_parameter("kp").as_double(),   // KP
            KD = get_parameter("kd").as_double()   // KD
        );

        output_max = get_parameter_or("output_max",inf);
        output_min = get_parameter_or("output_min",-inf);
    }

    void update() override {
        if((*target_ - last_target) != 0){
            last_control = *control_error;
            last_target = *target_;
        }else{
            *control_error = std::clamp(KT * adrc_calculator.update(*target_ ,*measurement_ ,last_control) * dt ,output_min ,output_max);
            last_control = *control_error ;
        }
    }

private:
    ADRCController adrc_calculator;
    double R;
    double B0;
    double BETA01;
    double BETA02;
    double BETA03;
    double ALPHA1;
    double ALPHA2;
    double DELTA;
    double KP;
    double KD;
    double KT = 1; //扭矩转化系数
    double last_target = 0, last_control = 0;
    double output_max = inf ,output_min = -inf;
    double dt = 0.001;

    InputInterface<double> measurement_;
    InputInterface<double> target_;

    OutputInterface<double> control_error;
        static constexpr double inf = std::numeric_limits<double>::infinity();
};

} // namespace rmcs_core::controller::adrc

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::adrc::AdrcController, rmcs_executor::Component)
