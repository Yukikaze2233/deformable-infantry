#include <eigen3/Eigen/Dense>
#include <string>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <controller/chassis/csv_file_save.hpp>

namespace rmcs_core::controller::gimbal {

class TestYawController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    TestYawController()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        // threshold_ = get_parameter("threshold").as_double();
        register_input("/remote/switch/right", switch_right_);
        register_input("/gimbal/yaw/velocity", yaw_velocity_);
        // register_input("/gimbal/yaw/target_velocity", yaw_target_velocity_);

        register_output("/gimbal/yaw/velocity_error", yaw_velocity_error, 0.0);
        register_output("/gimbal/yaw/controller_torque", yaw_control_torque_, 0.0);
        // csv_file_.init_csv_recorder(csv_name1, csv_name2, csv_name3);
            
    }

    void update() override {
        // *yaw_velocity_error = *yaw_target_velocity_ - *yaw_velocity_;
        T += 0.001;
        auto switch_right = *switch_right_;
        if(switch_right == rmcs_msgs::Switch::UP){
            if(std::abs(*yaw_velocity_) > threshold_) {
                *yaw_control_torque_ += 0.0;
            }else{
                *yaw_control_torque_ = T * 0.1;
            } 
        }
        // csv_file_.record_data(*yaw_velocity_, *yaw_target_velocity_, *yaw_control_torque_);
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<rmcs_msgs::Switch> switch_right_;
    double threshold_ = 1.0;
    double T =0.0;
    std::string csv_name1 = "yaw_velocity";
    std::string csv_name2 = "yaw_target_velocity";
    std::string csv_name3 = "yaw_control_torque";

    // debug::CSVfilesave csv_file_;
    InputInterface<double> yaw_velocity_;
    InputInterface<double> yaw_target_velocity_;

    OutputInterface<double> yaw_control_torque_;
    OutputInterface<double> yaw_velocity_error;

};
} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::TestYawController, rmcs_executor::Component)