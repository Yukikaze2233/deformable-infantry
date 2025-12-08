#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <algorithm>

// 兼容C++17以下的clamp（统一使用宏定义，避免混用）
#ifndef __cpp_lib_clamp
#define clamp(val, min_val, max_val) (std::max(min_val, std::min(val, max_val)))
#endif

class ScrewADRC {
private:
    double Bx;          // B点X坐标(mm)
    double By;          // B点Y坐标(mm)
    double L0;          // 零位参数(mm)
    const double L;     // 连接杆长度(mm)，固定为100
    const double pi;    // 圆周率常量

    // 2. ADRC参数（保持不变）
    double r;           // 快速跟踪因子
    double b0;          // 控制量系数
    double beta01;      // 观测器增益1
    double beta02;      // 观测器增益2
    double beta03;      // 观测器增益3
    double alpha1;      // fal函数幂次1
    double alpha2;      // fal函数幂次2
    double delta;       // fal函数线性段阈值
    double kp;          // 比例增益
    double kd;          // 微分增益

    // 3. ADRC状态变量（保持不变）
    double v1, v2;      // TD状态：跟踪信号、跟踪信号微分
    double z1, z2, z3;  // ESO状态：角度观测、角速度观测、总扰动观测
    double s_min, s_max;// 滑台位移限幅范围(mm)

    // ADRC核心函数（私有，内部调用）
    static double fst(double e, double ed, double r, double h);
    static double fal(double e, double alpha, double delta);
public:
    double sample_time; // 控制周期(s)
    double rad2deg(double rad) const {
        return rad * 180.0 / pi;
    }
    double deg2rad(double deg) const {
        return deg * pi / 180.0;
    }   
    // 构造函数（初始化顺序与声明一致）
    ScrewADRC(double Bx, double By, double L0 = NAN, double sample_time = 0.001) 
        : Bx(Bx), By(By), L0(L0), L(100.0), pi(M_PI), sample_time(sample_time) {
        // 初始化滑台限位
        s_min = -50.0;
        s_max = 150.0;

        // 初始化ADRC参数
        r = 50.0;
        b0 = 1.0;
        beta01 = 100.0;
        beta02 = 20.0;
        beta03 = 5.0;
        alpha1 = 0.5;
        alpha2 = 0.2;
        delta = 0.01;
        kp = 20.0;
        kd = 5.0;

        // 初始化ADRC状态变量
        v1 = 0.0;
        v2 = 0.0;
        z1 = 0.0;
        z2 = 0.0;
        z3 = 0.0;
    }

    // ------------------------------
    // 几何映射函数（修正后）
    // ------------------------------
    void calibrate_L0(double theta0_deg = 0.0, double s0 = 0.0) {
        double k = trapezoidal_calculator(theta0_deg);
        L0 = k - s0; 
    }
    double theta_to_s(double theta_rad) const {
        double theta_deg = rad2deg(theta_rad); 
        double k = trapezoidal_calculator(theta_deg);
        double s = k - L0; 
        return std::clamp(s, s_min, s_max); 
    }
    double s_to_theta(double s) const {
        double k = L0 + s; 
        return angle_calculator(k); 
    }
    double trapezoidal_calculator(double alpha_deg) const {
        double alpha_rad = deg2rad(alpha_deg); 
        double term = Bx * cos(alpha_rad) + By * sin(alpha_rad);
        double sqrt_term = Bx * sin(alpha_rad) - By * cos(alpha_rad) + 15.0;
        double sqrt_arg = L * L - sqrt_term * sqrt_term;
        double k = term + sqrt(sqrt_arg); 
        return k;
    }
    double angle_calculator(double k) const {
        double A = 2 * k * Bx + 30 * By;   
        double B = 2 * k * By - 30 * Bx;  
        double C = k * k + Bx * Bx + By * By - 9775.0;  
        double denom = sqrt(A * A + B * B);
        double cos_val = C / denom;
        cos_val = std::clamp(cos_val, -1.0, 1.0);
        return atan2(B, A) - acos(cos_val);
    }
    // ------------------------------
    // ADRC核心函数（修正映射调用逻辑）
    // ------------------------------
    void td_update(double v_cmd) {
        double h = sample_time;
        double e = v1 - v_cmd;
        v2 += h * fst(e, v2, r, h);
        v1 += h * v2;
    }

    void eso_update(double theta_meas, double u) {
        double h = sample_time;
        double e = z1 - theta_meas;
        z1 += h * (z2 - beta01 * e);
        z2 += h * (z3 + b0 * u - beta02 * fal(e, 0.5, delta));
        z3 += h * (-beta03 * fal(e, 0.2, delta));
    }

    double nlsef() const {
        double e1 = v1 - z1;
        double e2 = v2 - z2;
        double u0 = kp * fal(e1, alpha1, delta) + kd * fal(e2, alpha2, delta);
        double u = (u0 - z3) / b0;
        return u;
    }

    // 控制主函数（修正核心逻辑）
    double control_step(double theta_cmd, double theta_meas) {
        // 1. TD跟踪微分器更新（输入是弧度，符合ADRC逻辑）
        td_update(theta_cmd);
        // 2. ESO观测：u_prev是滑台位移（z1是弧度→转s）
        double u_prev = theta_to_s(z1); 
        eso_update(theta_meas, u_prev);
        // 3. NLSEF计算角度控制量
        double u_theta = nlsef();
        // 4. 预测目标角度（弧度）→ 转滑台位移s
        double theta_target = z1 + u_theta * sample_time;
        double s = theta_to_s(theta_target); // 自动处理单位+L0+限幅
        return s;
    }
};

// ------------------------------
// ADRC私有函数实现
// ------------------------------
double ScrewADRC::fst(double e, double ed, double r, double h) {
    double d = r * pow(h, 2);
    double a0 = h * ed;
    double a = e + a0;
    if (fabs(a) > d) {
        return -r * std::copysign(1.0, a);
    } else {
        return -r * (a / d);
    }
}

double ScrewADRC::fal(double e, double alpha, double delta) {
    if (fabs(e) > delta) {
        return std::copysign(pow(fabs(e), alpha), e);
    } else {
        return e / pow(delta, 1 - alpha);
    }
}

// ------------------------------
// 示例主函数
// ------------------------------
int main() {
    try {
        // 1. 初始化机构参数
        double Bx = 80.0, By = 30.0;
        ScrewADRC adrc(Bx, By);

        // 2. 零位校准（θ=0度时滑台位移s=0）
        adrc.calibrate_L0(0.0, 0.0);

        // 3. 模拟控制过程（目标角度30度=π/6弧度）
        double theta_cmd_rad = M_PI / 6.0; // ADRC内部用弧度
        double theta_meas_rad = 0.0;       // 初始角度（弧度）

        for (int i = 0; i < 1000; ++i) {
            // 单步ADRC控制：输出滑台位移s
            double s = adrc.control_step(theta_cmd_rad, theta_meas_rad);

            // 模拟执行器：滑台移动s → 角度变化（调用修正后的s_to_theta）
            theta_meas_rad = adrc.s_to_theta(s);

            // 打印状态（弧度+角度，便于验证）
            if (i % 100 == 0) {
                std::cout << "第" << i << "步："
                          << "目标θ=" << std::fixed << std::setprecision(3) 
                          << theta_cmd_rad << " rad (" << adrc.rad2deg(theta_cmd_rad) << "°), "
                          << "实际θ=" << theta_meas_rad << " rad (" << adrc.rad2deg(theta_meas_rad) << "°), "
                          << "滑台s=" << s << " mm" << std::endl;
            }

            // 控制周期延时
            std::this_thread::sleep_for(std::chrono::duration<double>(adrc.sample_time));
        }
    } catch (const std::invalid_argument& e) {
        std::cerr << "错误：" << e.what() << std::endl;
        return -1;
    }

    return 0;
}