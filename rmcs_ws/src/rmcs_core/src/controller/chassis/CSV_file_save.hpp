        #pragma once

        #include <cstdlib>
        #include <rclcpp/node.hpp>
        #include <rmcs_executor/component.hpp>
        #include <rmcs_msgs/switch.hpp>
        #include <eigen3/Eigen/Dense>
        #include <cmath>
        #include <fstream>
        #include <filesystem>
        #include <chrono>
        #include <iomanip>
        #include <string>
        #include <thread>
        #include <ctime>
        namespace rmcs_core::controller::debug {

        class CSVfilesave
        {
        public:
            CSVfilesave()
                :record_interval_(std::chrono::microseconds(1000))  
            {}

            void init_csv_recorder(const std::string& name_1 ,const std::string& name_2 ,const std::string& name_3 ) {
                auto current_path = std::filesystem::current_path();

                // 创建带时间戳的文件名（格式：chassis_lift_YYYYMMDD_HHMMSS.csv）
                auto now = std::chrono::system_clock::now();
                auto now_t = std::chrono::system_clock::to_time_t(now);
                thread_local std::tm tm_buf;  
                std::tm* tm = localtime_r(&now_t, &tm_buf); 

                std::ostringstream filename_os;
                filename_os << std::put_time(tm, "data_%Y%m%d_%H%M%S.csv");
                csv_file_path_ = (current_path / filename_os.str()).string();

                csv_file_.open(csv_file_path_, std::ios::out | std::ios::app);
                
                csv_file_ << std::fixed << std::setprecision(6);  
                csv_file_ << "timestamp_s" << ","<< name_1 << "," << name_2 << "," << name_3 << std::endl;

            }

            void record_data(double a,double b,double c) {
                auto last_record_time = std::chrono::steady_clock::now();
                auto now = std::chrono::steady_clock::now();
                // auto elapsed = now - last_record_time;
                // if (elapsed < record_interval_) {
                //     std::this_thread::sleep_for(record_interval_ - elapsed);
                // }

                auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(
                    now - node_start_time_
                ).count();

                csv_file_ << std::fixed << std::setprecision(6);
                csv_file_ << timestamp << "," << a << "," << b << "," << c << std::endl;

                csv_file_.flush();

                last_record_time = now;
            }
        private:
            std::thread record_thread_;          
            std::ofstream csv_file_;            
            std::string csv_file_path_;          
            std::chrono::microseconds record_interval_;  // 记录间隔（1ms）留有修改接口
            const std::chrono::steady_clock::time_point node_start_time_ = std::chrono::steady_clock::now();  

        };

        } // namespace rmcs_core::controller::debug