#include <cstdlib>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <thread>

#include <rmcs_msgs/switch.hpp>
#include "controller/pid/pid_calculator.hpp"
#include "std_msgs/msg/int32.hpp"
#include "controller/chassis/CSV_file_save.hpp"
#include "filter/low_pass_filter.hpp"

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
            {0.023, 0.000, 0.09},  
            {1.0, 0.001, 0.01},  
            {1.0, 0.001, 0.01},  
            {1.0, 0.001, 0.01}   
        }
        
        , csv_saver_()
        , lift_left_front_angle_filter_(100.0 /* cutoff_frequency */, 1000.0 /* sampling_frequency */)
        , min_angle_( get_parameter_or("min_angle", 15) )
        , max_angle_( get_parameter_or("max_angle", 55) )
        , is_recording_enabled_(get_parameter_or("enable_csv_recording", true))  
        , record_interval_(std::chrono::microseconds(1000)) 
    {        
        // register_input("/chassis/lift/target_angle", target_angle_);/*15-55*/
        register_input("/remote/joystick/left", remote_left_joystic_);
        register_input("/remote/joystick/right", remote_right_joystic_);
        register_input("/remote/switch/left", remote_left_switch_);
        register_input("/remote/switch/right", remote_right_switch_);
        register_input("/chassis/lift/left_front_wheel/encoder_angle", left_front_wheel_angle_);
        register_input("/chassis/lift/left_front_wheel/torque", left_front_wheel_back_torque_);         //debug    

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

        // register_output("/chassis/lift/left_front_wheel/control_torque", left_front_wheel_torque_, nan_);
        register_output("/chassis/lift/left_back_wheel/control_torque", left_back_wheel_torque_, nan_);
        register_output("/chassis/lift/right_front_wheel/control_torque", right_front_wheel_torque_, nan_);
        register_output("/chassis/lift/right_back_wheel/control_torque", right_back_wheel_torque_, nan_);

        register_output("/chassis/lift/lf/control_angle_error", lf_angle_error_, nan_);
        for (auto& pid : wheel_pids) {
            pid.output_max = max_torque_;
            pid.output_min = -max_torque_;
        } 
        start_time_ = std::chrono::steady_clock::now();
    }

    void update() override {
        if ((*remote_left_switch_ == rmcs_msgs::Switch::DOWN || *remote_left_switch_ == rmcs_msgs::Switch::UNKNOWN)
            && (*remote_right_switch_ == rmcs_msgs::Switch::DOWN || *remote_right_switch_ == rmcs_msgs::Switch::UNKNOWN)) {
            stop_lift();
            test_init = false;
            // stop all !!
        } else if ((*remote_left_switch_ == rmcs_msgs::Switch::MIDDLE ) && (*remote_right_switch_ == rmcs_msgs::Switch::DOWN )) {
            *left_front_wheel_torque_ = -1.0* remote_left_joystic_-> x();
            *left_back_wheel_torque_ = -1.0* remote_left_joystic_-> y();
            *right_front_wheel_torque_ = -1.0* remote_right_joystic_-> x();
            *right_back_wheel_torque_ = -2.0* remote_right_joystic_-> y();
            RCLCPP_INFO(get_logger(), "lf_angle%f",319 - *left_front_wheel_angle_);
            RCLCPP_INFO(get_logger(), "lb_angle--%f",211.7 - *left_back_wheel_angle_);
            if(*right_front_wheel_angle_ > 180){
                RCLCPP_INFO(get_logger(), "rf_angle----%f",401 - *right_front_wheel_angle_);
            }else{
                RCLCPP_INFO(get_logger(), "rf_angle----%f",41 - *right_front_wheel_angle_);
            }
            RCLCPP_INFO(get_logger(), "rb_angle------%f",339 - *right_back_wheel_angle_);
    
            test_init = false;
        } else if ((*remote_left_switch_ == rmcs_msgs::Switch::MIDDLE ) && (*remote_right_switch_ == rmcs_msgs::Switch::MIDDLE )) {
            test_init = false;
        } else {
            if(test_init == false){
                init_calculator();
                test_init = true;
                // csv_saver_.init_csv_recorder("s1","s2","s3");
            }

            double target = trapezoidal_calculator(45.0/* *target_angle_ */);

            s_lf -= *left_front_wheel_velocity_/(2 * pi) * dt * reduction_ratio;
            s_lb -= *left_back_wheel_velocity_/(2 * pi) * dt * reduction_ratio;
            s_rf -= *right_front_wheel_velocity_/(2 * pi) * dt * reduction_ratio;
            s_rb -= *right_back_wheel_velocity_/(2 * pi) * dt * reduction_ratio;
    
            double lf_err = s_lf - target;
            double lb_err = s_lb - target;
            double rf_err = s_rf - target;
            double rb_err = s_rb - target;

            // *left_front_wheel_torque_ = std::clamp(wheel_pids[0].update(lf_err),-0.6,0.6);
            // *left_back_wheel_torque_  = std::clamp(wheel_pids[1].update(lb_err),-0.6,0.6);
            // *right_front_wheel_torque_ = std::clamp(wheel_pids[2].update(rf_err),-0.6,0.6);
            // *right_back_wheel_torque_  = std::clamp(wheel_pids[3].update(rb_err),-0.6,0.6);

            // auto now = std::chrono::steady_clock::now();

            // double elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();
            // *left_front_wheel_torque_ = std::clamp(-0.1 * elapsed_time , 0.8, 0.8);

            *lf_angle_error_ = lf_err;  // smc

            torque += k * 0.001;
            if (*left_front_wheel_velocity_ > 0.040277 || *left_front_wheel_velocity_ < -0.032222){
                k = 0;
            }else{
                k = 0.01;
            }

            *left_front_wheel_torque_ = torque;

            // csv_saver_.record_data((65 - *left_front_wheel_angle_ ), *left_front_wheel_torque_ , *left_front_wheel_velocity_);                        //debug
            // RCLCPP_INFO(get_logger(), "left_front_wheel_torque_:%f", *left_front_wheel_torque_);
            // RCLCPP_INFO(get_logger(), "lf_err:%f", lf_err);
            // RCLCPP_INFO(get_logger(), "now_angle:%f", 65 - *left_front_wheel_angle_);
            // RCLCPP_INFO(get_logger(), "left_front_wheel_velocity_:%f", *left_front_wheel_velocity_);
            // RCLCPP_INFO(get_logger(), "s_lf:%f", s_lf);
            // RCLCPP_INFO(get_logger(), "angle_calculator:%f", angle_calculator(s_lf) * 180 / pi);
        }

    }

private:

    void stop_lift(){
        *left_front_wheel_torque_ = 0.0;
        *left_back_wheel_torque_ = 0.0;
        *right_front_wheel_torque_ = 0.0;
        *right_back_wheel_torque_ = 0.0;
        RCLCPP_INFO(get_logger(), "Stopping lift");
        *lf_angle_error_= 0.0;
    }

    double trapezoidal_calculator(double alpha) const{
        double term = Bx * cos(alpha * pi / 180) + By * sin(alpha * pi / 180);
        double s = term + sqrt(L * L - (Bx * std::sin(alpha * pi / 180) - By * std::cos(alpha * pi / 180) + 15) * (Bx * std::sin(alpha * pi / 180) - By * std::cos(alpha * pi / 180) + 15)) ;
        return s;
    }

    double angle_calculator(double s) const {

        double A = 2 * s * Bx + 30 * By;   
        double B = 2 * s * By - 30 * Bx;  
        double C = s * s + Bx * Bx + By * By - 9775.0;  

        double cos_val = C / std::sqrt(A * A + B * B);
        cos_val = std::max(-1.0, std::min(1.0, cos_val));
    
        return std::atan2(B, A) - std::acos(cos_val);
    }

    void init_calculator(){
        lf = trapezoidal_calculator( 319 - *left_front_wheel_angle_);
        lb = trapezoidal_calculator(211.9 - *left_back_wheel_angle_);
        rf = trapezoidal_calculator(65 - *right_front_wheel_angle_);
        rb = trapezoidal_calculator(65 - *right_back_wheel_angle_);
        s_lf = lf;
        s_lb = lb;
        s_rf = rf;
        s_rb = rb;
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr chassis_lift_controller_;

    InputInterface<rmcs_msgs::Switch> remote_left_switch_;
    InputInterface<rmcs_msgs::Switch> remote_right_switch_;

    InputInterface<Eigen::Vector2d> remote_left_joystic_;
    InputInterface<Eigen::Vector2d> remote_right_joystic_;

    // InputInterface<double> target_angle_;
    InputInterface<double> left_front_wheel_angle_;
    InputInterface<double> left_back_wheel_angle_;
    InputInterface<double> right_front_wheel_angle_;
    InputInterface<double> right_back_wheel_angle_;
    InputInterface<double> left_front_wheel_velocity_;
    InputInterface<double> left_back_wheel_velocity_;
    InputInterface<double> right_front_wheel_velocity_;
    InputInterface<double> right_back_wheel_velocity_;

    InputInterface<double> left_front_wheel_back_torque_;//debug

    OutputInterface<double> left_front_wheel_torque_;
    OutputInterface<double> left_back_wheel_torque_;
    OutputInterface<double> right_front_wheel_torque_;
    OutputInterface<double> right_back_wheel_torque_;

    OutputInterface<double> lf_angle_error_;

    rmcs_core::controller::pid::PidCalculator wheel_pids[4];
    rmcs_core::controller::debug::CSVfilesave csv_saver_;

    rmcs_core::filter::LowPassFilter<1> lift_left_front_angle_filter_;

    const double min_angle_ ;
    const double max_angle_ ;

    const double max_torque_ = 6.0;
    std::chrono::steady_clock::time_point start_time_;  

    double lf, lb, rf, rb;
    double s_lf, s_lb, s_rf, s_rb;


    double torque = 0.0;
    double k = 0.0;

    double L0;
    double Bx, By;
    double dt = 0.001;
    double reduction_ratio = 14.105;
    double L;
    double pi = std::numbers::pi;
    bool test_init = false;

    bool is_recording_enabled_;          // 是否启用记录
    std::thread record_thread_;          // 独立记录线程
    std::ofstream csv_file_;             // CSV文件流
    std::string csv_file_path_;          // CSV文件路径
    std::chrono::microseconds record_interval_;  // 记录间隔（1ms）
    const std::chrono::steady_clock::time_point node_start_time_ = std::chrono::steady_clock::now();  // 节点启动时间

};

} // namespace rmcs_core::controller::chassis


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ChassisLiftController, rmcs_executor::Component)