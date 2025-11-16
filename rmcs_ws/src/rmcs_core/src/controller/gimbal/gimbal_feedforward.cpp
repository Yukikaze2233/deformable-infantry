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
        
        register_output(get_parameter("feedforward").as_string(), feedforward);


        register_input(get_parameter("control_Object").as_string(), control_Object);

        firc = get_parameter_or("firc",0.0);
    }

    void update() override
    {
        const double dt = 0.001;
    }

private:    

    std::string filename_;
    InputInterface<double> control_Object;
    InputInterface<double> transfer_function;
     
    OutputInterface<double> feedforward;


    Eigen::Matrix<double,1,3> transfer_function_;
    Eigen::Matrix<double,3,1> control_Object_;
    double firc;
    double gain1;
    double gain2;
    std::ofstream csv_file_;
}; 

}
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::GimbalFeedforward, rmcs_executor::Component)