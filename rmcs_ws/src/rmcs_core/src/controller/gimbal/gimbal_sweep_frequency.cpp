
#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <cmath>
#include "/workspaces/RMCS/rmcs_ws/src/rmcs_core/src/controller/chassis/csv_file_save.hpp"

namespace rmcs_core::controller::gimbal {

class FrequencySweepController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    FrequencySweepController()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        duration = get_parameter("duration").as_double();
        top_torque = get_parameter("top_torque").as_double();
        register_output("/gimbal/yaw/torque", yaw_current, 0.0);
        register_input("/gimbal/yaw/angle", yaw_position);

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        std::string col1 = "set_current_A";  // 第一列：指令电流（A）
        std::string col2 = "yaw_pos_rad";    // 第二列：云台yaw轴角度（rad）
        std::string col3 = "current_freq_Hz";// 第三列：瞬时扫频频率（Hz）

        csv_recorder.init_csv_recorder(col1, col2, col3);
    }

    double phase_calculate() {
        double start_freq = 0.1; // Hz
        double end_freq = 50.0; // Hz
        double freq_step = (end_freq - start_freq) / duration;
        freq = start_freq + freq_step * time;

        return 2 * M_PI * (start_freq * time + 0.5 * freq_step * time * time);
    }

    void update() override {
        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;

        if (switch_left == rmcs_msgs::Switch::UP && switch_right == rmcs_msgs::Switch::UP){
            if(!calculate_initialized_){
                time = 0.0;
                calculate_initialized_ = true;
            }

            if(time > duration){
                time = duration; 
                *yaw_current = 0.0;
            }else{
                double phase = phase_calculate();
                *yaw_current = top_torque * std::sin(phase);
                time += dt_;
                csv_recorder.record_data(*yaw_current, *yaw_position, freq);
            }
        }
    }

private:

    OutputInterface<double> yaw_current;
    
    debug::CSVfilesave csv_recorder;

    InputInterface<double> yaw_position;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    double duration = 100.0;
    double freq = 0.0;
    double time =0.0;
    double dt_ = 0.001; 
    double top_torque = 1.0;

    bool calculate_initialized_ = false;

};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::FrequencySweepController, rmcs_executor::Component)