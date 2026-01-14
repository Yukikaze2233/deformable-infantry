#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
namespace rmcs_core::controller::gimbal {
class GimbalFeedforward final : 
    public rmcs_executor::Component,
    public rclcpp::Node
{
public:
    GimbalFeedforward() : rclcpp::Node(get_component_name(), 
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
    {
        register_input(get_parameter("target_velocity").as_string(), control_velocity_);
        register_input(get_parameter("target_angle").as_string(), control_angle_);
        register_input(get_parameter("control_nonfeedforward").as_string(), control_);
        register_output(get_parameter("control").as_string(), control_output_);
        Kv = get_parameter("kv").as_double();
        Ka = get_parameter("ka").as_double();
        firc = get_parameter("firc").as_double();
        order_number = get_parameter("order_number").as_double();
    }

    void update() override
    {
         double feedforward = 0.0;
        if (order_number < 1.0 || order_number > 2.0){
            RCLCPP_ERROR(get_logger(), "Gimbal Feedforward order_number parameter is invalid");
            return;
        }else if(order_number == 1.0){
            feedforward = velocity_control_calculate_feedforward_();
            control_object_ = *control_velocity_;
        }else if(order_number == 2.0){
            feedforward = angle_control_calculate_feedforward_();
            control_object_ = *control_angle_;
        }
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
        current_dx = dx;
        previous_dx_ = dx;
        previous_x_ = current_x;

        return (Kv * dx) + (Ka * ddx) ; 
    }
    double velocity_control_calculate_feedforward_(){
        const double current_x = *control_velocity_;
        return (Kv * current_x) ; 
    }

    double function_compensation(double feedforward) const{
        if (std::abs(current_dx) > 0.2 ){
            if (current_dx < 0){
                return -firc;
            } else {
                return firc;
            }
        } else {
            if(feedforward > 0.1){
                return firc;
            } else if(feedforward < -0.1){
                return -firc;
            } else {
                return 0.0;
            }
        }
    }   


    InputInterface<double> control_velocity_;
    InputInterface<double> control_angle_;
    InputInterface<double> control_;
    OutputInterface<double> control_output_;

    double order_number = 2.0;
    double dt_ =0.001;
    double current_dx;
    double previous_x_;
    double previous_dx_;
    double firc;
    double Kv;
    double Ka;
    double control_object_;
    bool calculate_initialized_ = false;
}; 

}
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::GimbalFeedforward, rmcs_executor::Component)

/*
    前馈的场合一共有四种：
    一种是速控，速度误差为输入；
    一种是速控，角度误差为输入；
    一种是角控，速度误差为输入；
    一种是角控，角度误差为输入。
    速控的前馈是一致的，都是Kv*速度方向
    角控的前馈也是一致的，都是Kv*速度方向 + Ka*加速度方向
    通过order_number参数来区分速控和角控
    通过输入的误差类型来区分速度误差和角度误差
*/