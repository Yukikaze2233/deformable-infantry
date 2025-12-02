#include <cstdlib>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <fstream>
// #include <filesystem>
#include <chrono>
// #include <iomanip>
#include <ctime>
#include <thread>

#include <rmcs_msgs/switch.hpp>
#include "controller/pid/pid_calculator.hpp"
#include "std_msgs/msg/int32.hpp"

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
        reduction_ratio = get_parameter("reduction_ratio").as_double();

        register_output("/chassis/lift/left_front_wheel/control_torque", left_front_wheel_torque_, nan_);
        register_output("/chassis/lift/left_back_wheel/control_torque", left_back_wheel_torque_, nan_);
        register_output("/chassis/lift/right_front_wheel/control_torque", right_front_wheel_torque_, nan_);
        register_output("/chassis/lift/right_back_wheel/control_torque", right_back_wheel_torque_, nan_);

        for (auto& pid : wheel_pids) {
            pid.output_max = max_torque_;
            pid.output_min = -max_torque_;
        } 
    }

    void update() override {
        if ((*remote_left_switch_ == rmcs_msgs::Switch::DOWN || *remote_left_switch_ == rmcs_msgs::Switch::UNKNOWN)
            && (*remote_right_switch_ == rmcs_msgs::Switch::DOWN || *remote_right_switch_ == rmcs_msgs::Switch::UNKNOWN)) {
            stop_lift();
            test_init = false;
            // stop all !!
        } else if ((*remote_left_switch_ == rmcs_msgs::Switch::MIDDLE ) && (*remote_right_switch_ == rmcs_msgs::Switch::DOWN )) {
            *left_front_wheel_torque_ = -0.8* remote_left_joystic_-> x();
            // RCLCPP_INFO(get_logger(), "left_front_wheel_torque_%f", *left_front_wheel_torque_);
            test_init = false;
        }  else {
        if(test_init == false){
            init_calculator();
            // init_csv_recorder();
            test_init = true;
        }

        double target = trapezoidal_calculator(45.0/* *target_angle_ */);
        // chassis_lift_controller_ = create_subscription<std_msgs::msg::Int32>(
        // "/chassis/lift/target_angle", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
        //     stop_lift();
        //     init_calculator(std::move(msg));
        // });

        // target = (target < min_angle_) ? min_angle_ : (target > max_angle_) ? max_angle_ : target;


        s_lf = trapezoidal_calculator(65 - *left_front_wheel_angle_);
        s_lb = trapezoidal_calculator(65 - *left_back_wheel_angle_);
        s_rf = trapezoidal_calculator(65 - *right_front_wheel_torque_);
        s_rb = trapezoidal_calculator(65 - *right_back_wheel_angle_);
 
        double lf_err = s_lf - target;
        double lb_err = s_lb - target;
        double rf_err = s_rf - target;
        double rb_err = s_rb - target;

        *left_front_wheel_torque_ = std::clamp(wheel_pids[0].update(lf_err),-0.6,0.6);
        *left_back_wheel_torque_  = std::clamp(wheel_pids[1].update(lb_err),-0.6,0.6);
        *right_front_wheel_torque_ = std::clamp(wheel_pids[2].update(rf_err),-0.6,0.6);
        *right_back_wheel_torque_  = std::clamp(wheel_pids[3].update(rb_err),-0.6,0.6);

        // RCLCPP_INFO(get_logger(), "left_front_wheel_torque_:%f", *left_front_wheel_torque_);
        // RCLCPP_INFO(get_logger(), "target:%f", target);
        // RCLCPP_INFO(get_logger(), "lf_err:%f", lf_err);
        // RCLCPP_INFO(get_logger(), "now_angle:%f", 65 - *left_front_wheel_angle_);
        // RCLCPP_INFO(get_logger(), "left_front_wheel_velocity_:%f", *left_front_wheel_velocity_);
        // RCLCPP_INFO(get_logger(), "s_lf:%f", s_lf);
        // RCLCPP_INFO(get_logger(), "angle_calculator:%f", angle_calculator(s_lf) * 180 / pi);
        // record_data();
        }

    }

// private:
//     void init_csv_recorder() {
//         if (!is_recording_enabled_) {
//             RCLCPP_INFO(get_logger(), "CSV recording is disabled via parameter");
//             return;
//         }

//         auto current_path = std::filesystem::current_path();

//         // 创建带时间戳的文件名（格式：chassis_lift_YYYYMMDD_HHMMSS.csv）
//         auto now = std::chrono::system_clock::now();
//         auto now_t = std::chrono::system_clock::to_time_t(now);
//         thread_local std::tm tm_buf;  
//         std::tm* tm = localtime_r(&now_t, &tm_buf); 

//         std::ostringstream filename_os;
//         filename_os << std::put_time(tm, "chassis_lift_%Y%m%d_%H%M%S.csv");
//         csv_file_path_ = (current_path / filename_os.str()).string();

//         csv_file_.open(csv_file_path_, std::ios::out | std::ios::app);
//         if (!csv_file_.is_open()) {
//             RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", csv_file_path_.c_str());
//             is_recording_enabled_ = false;
//             return;
//         }

//         csv_file_ << std::fixed << std::setprecision(6);  
//         csv_file_ << "timestamp_s , s_lf , trapezoidal_calculator(*target_angle_) , trapezoidal_calculator(*left_front_angle）" << std::endl;

//         RCLCPP_INFO(get_logger(), "CSV recording started. File path: %s", csv_file_path_.c_str());
//     }

//     void record_data() {
//         auto last_record_time = std::chrono::steady_clock::now();

        
//         auto now = std::chrono::steady_clock::now();
//         auto elapsed = now - last_record_time;
//         if (elapsed < record_interval_) {
//             std::this_thread::sleep_for(record_interval_ - elapsed);

//         }

//         auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(
//             now - node_start_time_
//         ).count();

//         csv_file_ << std::fixed << std::setprecision(6);
//         csv_file_ << timestamp << "," << (s_lf - trapezoidal_calculator(45.0/* *target_angle_ */)) << ","<< trapezoidal_calculator(45) << "," << trapezoidal_calculator(*left_front_wheel_angle_) << std::endl;

//         csv_file_.flush();

//         last_record_time = now;
        
//     }

    void stop_lift(){
        *left_front_wheel_torque_ = 0.0;
        *left_back_wheel_torque_ = 0.0;
        *right_front_wheel_torque_ = 0.0;
        *right_back_wheel_torque_ = 0.0;
        RCLCPP_INFO(get_logger(), "Stopping lift");
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
        lf = trapezoidal_calculator(65 - *left_front_wheel_angle_);
        lb = trapezoidal_calculator(65 - *left_back_wheel_angle_);
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

    OutputInterface<double> left_front_wheel_torque_;
    OutputInterface<double> left_back_wheel_torque_;
    OutputInterface<double> right_front_wheel_torque_;
    OutputInterface<double> right_back_wheel_torque_;

    rmcs_core::controller::pid::PidCalculator wheel_pids[4];

    const double min_angle_ ;
    const double max_angle_ ;

    const double max_torque_ = 6.0;


    double lf, lb, rf, rb;
    double s_lf, s_lb, s_rf, s_rb;

    double L0;
    double Bx, By;
    double L;
    double pi = std::numbers::pi;
    double reduction_ratio;
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