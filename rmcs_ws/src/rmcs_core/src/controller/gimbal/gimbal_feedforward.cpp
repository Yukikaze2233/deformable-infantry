#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>


namespace rmcs_core::controller::gimbal {


class GimbalFeedforward 
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GimbalFeedforward() 
        : rclcpp::Node(
            get_component_name(), 
            rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
    {
        register_input(get_parameter("angle_error").as_string(), control_angle_);
        register_input(get_parameter("control_nonfeedforward").as_string(), control_);
        register_output(get_parameter("control").as_string(), control_output_);
        Kv = get_parameter("kv").as_double();
        Ka = get_parameter("ka").as_double();
        firc = get_parameter("firc").as_double();
    }

    void update() override
    {
        double feedforward = 0.0;
        
        feedforward = angle_control_calculate_feedforward_();
        control_object_ = *control_angle_;
        
        *control_output_ = feedforward + function_compensation(control_object_) + *control_;
    }

private:
    double angle_control_calculate_feedforward_(){
        const double current_x = *control_angle_;

        if(!calculate_initialized_){
            previous_x_ = 0;
            previous_dx_ = 0;
            calculate_initialized_ = true;
        }
        const double dx = (current_x - previous_x_) / dt_;
        const double ddx = (dx - previous_dx_) / dt_;
        angle_velocity = dx;
        previous_dx_ = dx;
        previous_x_ = current_x;

        return (Kv * dx) + (Ka * ddx) ; 
    }

    double function_compensation(double sign_velocity) const{
        if (std::abs(angle_velocity) > 0.2 ){
            if ((angle_velocity) < 0){
                return -firc;
            } else {
                return firc;
            }
        } else {
            if(sign_velocity > 0.1){
                return firc;
            } else if(sign_velocity < -0.1){
                return -firc;
            } else {
                return 0.0;
            }
        }
    }   


    InputInterface<double> control_angle_;
    InputInterface<double> control_;
    OutputInterface<double> control_output_;

    double dt_ =0.001;
    double angle_velocity = 0.0;
    double previous_x_ = 0.0;
    double previous_dx_ = 0.0;
    double firc;
    double Kv;
    double Ka;
    double control_object_;
    bool calculate_initialized_ = false;
}; 

}
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::GimbalFeedforward, rmcs_executor::Component)
