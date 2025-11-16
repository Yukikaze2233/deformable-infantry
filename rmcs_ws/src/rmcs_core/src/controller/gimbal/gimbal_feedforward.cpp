#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <eigen3/Eigen/Dense>
#include "fstream"
namespace rmcs_core::controller::gimbal {
class GimbalFeedforward final : 
    public rmcs_executor::Component,
    public rclcpp::Node
{
public:
    GimbalFeedforward() : rclcpp::Node(get_component_name(), 
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
    {
        register_input(get_parameter("control_Object").as_string(), control_Object);
        register_output(get_parameter("feedforward").as_string(), feed_forward);
        Kv = get_parameter_or("Kv",0.2148);
        Ka = get_parameter_or("Ka",0.3823);

        firc = get_parameter_or("firc",7.2);
    }

    void update() override
    {
        *feed_forward = calculate_feedforward_();
    }

private:
    double calculate_feedforward_(){
        if(!calculate_initialized_){
            previous_dx = 0;
            previous_x = 0;
            calculate_initialized_ = true;
        }
        x = *control_Object;
        dx = (x - previous_x)/dt;
        ddx = (dx - previous_dx)/dt;

        previous_dx = dx;
        previous_x = x;

        return (Kv * dx) + (Ka * ddx) + firc; 
    }    

    std::string filename_;
    InputInterface<double> control_Object;
    OutputInterface<double> feedforward;


    double dt =0.001;
    double x = 0;
    double dx = 0;
    double ddx = 0;
    double previous_dx;
    double previous_x;
    double firc;
    double Kv;
    double Ka;
    bool calculate_initialized_ = false;
    OutputInterface<double> feed_forward;
    std::ofstream csv_file_;
}; 

}
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::GimbalFeedforward, rmcs_executor::Component)