#ifndef PURE_ADRC_CONTROLLER_H
#define PURE_ADRC_CONTROLLER_H

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <limits> // 用于std::numeric_limits

namespace rmcs_core::controller::adrc{

class LinkADRCController {
public:
    // 沿用指定的参数枚举类型
    enum class ADRCParam {
        R,          
        B0,        
        WC,
        W0,    
        KP,         
        KD          
    };

    // 构造函数：沿用指定名称，初始化默认参数+ADRC核心状态
    explicit LinkADRCController()
    {
        init_default_params(); // 初始化默认参数（指定函数名）
        // 显式初始化所有状态变量，彻底解决未定义问题
        m_current_time = 0.0;
        m_step_size = 0.001;    // 明确赋值，与原代码一致（1ms步长）
        m_corrected_ctrl = 0.0;
        m_ref_scale = 0.5;      // 参考信号缩放增益，显式初始化
        // 跟踪微分器（TD）状态初始化
        m_td_state << 0.0, 0.0;
        // 自定义参考信号函数指针初始化
        m_custom_ref = nullptr;
    }

    // 沿用指定的参数设置接口（单参数设置）
    void set_param(ADRCParam param_type, double value) {
        switch (param_type) {
            case ADRCParam::R:         r_ = value; break;
            case ADRCParam::B0:        b0_ = value; break;
            case ADRCParam::WC:        w_c = value; break;
            case ADRCParam::W0:        w_0 = value; break;
            case ADRCParam::KP:        kp_ = value; break;
            case ADRCParam::KD:        kd_ = value; break;
        }
    }

    // 沿用指定的参数设置接口（批量参数设置）
    void set_params(double wc, double b0, double w0, double kp, double kd) {
        r_ = wc * wc; // 与原代码m_err_gain1（250000.0）对应，保持原逻辑
        b0_ = b0;
        w_c = wc;
        w_0 = w0;
        kp_ = kp;
        kd_ = kd;
        // 批量设置时同步更新原代码核心增益（保持控制逻辑一致）
        update_adrc_core_gains();
    }

    // 沿用指定接口：计算ADRC输出（对应原run_adrc_step核心逻辑）
    double compute_adrc_output(double cmd, double meas) {
        double u = lsef(cmd); // 原代码原始控制量计算（沿用lsef函数名）
        eso_update_link(u, meas); // 原代码ESO状态更新（沿用指定函数名）
        // 非线性校正（原代码核心逻辑，保留）
        nonlinear_correction(u);
        // 更新跟踪微分器状态（原代码逻辑）
        update_td_state();
        // 更新时间（原代码逻辑）
        m_current_time += m_step_size;
        return m_corrected_ctrl;
    }

    // 沿用指定接口：更新ADRC并返回输出（与compute_adrc_output逻辑一致，保留重载）
    double update_link(double cmd, double meas){//target ,measurement 
        double u = lsef(cmd);
        eso_update_link(u, meas);
        nonlinear_correction(u);
        update_td_state();
        m_current_time += m_step_size;
        return m_corrected_ctrl;
    }

    // 新增：设置自定义参考信号（原代码接口，保留易用性）
    void set_custom_ref(double (*ref_func)(double time))
    {
        m_custom_ref = ref_func;
    }

    // 新增：获取ESO观测状态（原代码接口，映射到z1_/z2_/z3_）
    void get_eso_state(double out_state[3])
    {
        out_state[0] = z1_;
        out_state[1] = z2_;
        out_state[2] = z3_;
    }

    // 新增：设置控制步长（原代码接口，保留）
    void set_step_size(double step)
    {
        m_step_size = step; // 明确赋值，无未定义风险
    }

private:
    // 沿用指定函数名：ESO状态更新（核心逻辑与原代码一致，映射Eigen矩阵）
    void eso_update_link(double u,double y_meas) {
        // 原代码ESO状态加权（Gain2模块）：替换为Eigen矩阵运算
        Eigen::Matrix<double,3,1> gain2_out = ad * xbar; // 对应原代码gain2_out0/1/2
        // 原代码：计算ESO下一拍状态（映射到xbar_new）
        Eigen::Matrix<double,3,1> bd_u = bd * u;
        Eigen::Matrix<double,3,1> ad_ld_u = ad * ld * (y_meas - c * xbar);
        xbar_new = gain2_out + bd_u + ad_ld_u;

        // 原代码：更新ESO观测状态z1_/z2_/z3_（对应原m_eso_state）
        Eigen::Matrix<double,3,1> xhat = xbar + ld * (y_meas - c * xbar);
        z1_ = xhat(0);
        z2_ = xhat(1);
        z3_ = xhat(2);

        // 原代码：ESO状态迭代更新（对应原m_eso_state = m_next_eso_state）
        xbar = xbar_new;
    } 

    // 沿用指定函数名：非线性状态误差反馈（LSEF，原代码原始控制量计算逻辑）
    double lsef(double target) const {
        // 原代码：参考信号生成（支持自定义参考信号，兼容原逻辑）
        double ref_signal = target;
        if (m_custom_ref != nullptr) {
            ref_signal = m_custom_ref(m_current_time);
        } else {
            ref_signal = std::sin(m_current_time) * m_ref_scale; // 原默认正弦波
        }

        // 原代码：核心状态误差计算（对应原state_error）
        double state_error = (0.0 * m_td_state(0) + m_td_state(1)) - z1_;

        // 原代码：扰动补偿项（对应原raw_ctrl_tmp）
        double raw_ctrl_tmp = ld(2) * state_error; // ld(2)对应原m_gain4（3638.1）

        // 原代码：原始控制量计算（完全保留逻辑，替换为指定参数名）
        double u0 = ((ref_signal - (kp_ * state_error + z1_)) * r_ 
                   - (kd_ * state_error + z2_) * w_0) 
                   - (raw_ctrl_tmp + z3_);

        // 原代码：控制量输出增益（对应原m_ctrl_gain1/2/3/4，映射到b0_）
        return (u0 - z3_) / b0_;
    }

    // 沿用指定函数名：初始化默认参数（参数值与原代码保持一致）
    void init_default_params() {       
        // 原代码核心增益参数：映射到指定参数名
        r_ = 250000.0;        // 对应原m_err_gain1
        b0_ = 1.0;            // 对应原m_ctrl_gain1（适配）
        w_c = 500.0;          // 对应原m_err_gain2
        w_0 = 1000.0;         // 对应原m_err_gain3
        kp_ = 0.39408;        // 对应原m_err_gain2（0.394075...）
        kd_ = 65.507;         // 对应原m_err_gain4（65.5065...）

        // 原代码Gain2/ESO相关矩阵：映射到Eigen矩阵（与原代码一致）
        c << 1.0, 0.0, 0.0;
        bd << 5.0E-7, 0.001, 0.0;  // 对应原m_ctrl_gain1/3/0
        ld << 0.39408, 65.507, 3638.1; // 对应原m_err_gain2/4 + m_gain4
        // 原代码AD矩阵（对应ESO状态更新，与原逻辑一致）
        ad << 1.0, 0.0, 0.0, 
              0.001, 1.0, 0.0, 
              5.0E-7, 0.001, 1.0;
        // 原代码ESO状态初始化（对应m_eso_state/m_next_eso_state）
        xbar << 0.0, 0.0, 0.0;
        xbar_new << 0.0, 0.0, 0.0;

        // 原代码ESO观测状态初始化（z1_/z2_/z3_对应原m_eso_state）
        z1_ = 0.0; 
        z2_ = 0.0; 
        z3_ = 0.0; 
    }

    // 私有辅助函数：非线性校正（原代码核心逻辑，保留）
    void nonlinear_correction(double raw_ctrl)
    {
        if (raw_ctrl > 0.4)
        {
            m_corrected_ctrl = raw_ctrl - 0.2;
        }
        else if (raw_ctrl < -0.4)
        {
            m_corrected_ctrl = raw_ctrl + 0.2;
        }
        else
        {
            m_corrected_ctrl = 0.0;
        }
    }

    // 私有辅助函数：更新跟踪微分器（TD）状态（原代码逻辑，保留）
    void update_td_state()
    {
        // 原代码TD状态导数计算
        double td_state1_dot = -0.0 * m_td_state(0) - 0.0 * m_td_state(1) + m_corrected_ctrl;
        double td_state2_dot = m_td_state(0);

        // 欧拉法更新状态（与原代码一致，m_step_size已明确初始化）
        m_td_state(0) += td_state1_dot * m_step_size;
        m_td_state(1) += td_state2_dot * m_step_size;
    }

    // 私有辅助函数：更新ADRC核心增益（保持原逻辑与参数一致性）
    void update_adrc_core_gains()
    {
        // 同步更新Eigen矩阵中的增益值（与原代码对应）
        bd(0) = 5.0E-7;    // 原m_ctrl_gain1
        bd(1) = 0.001;     // 原m_ctrl_gain3
        ld(0) = kp_;       // 原m_err_gain2
        ld(1) = kd_;       // 原m_err_gain4
        ld(2) = 3638.1;    // 原m_gain4
        ad(2,0) = 5.0E-7;  // 原m_ctrl_gain1
        ad(2,1) = 0.001;   // 原m_ctrl_gain3
    }

    // -------------------------- 明确声明所有成员变量，解决未定义问题 --------------------------
    // 沿用指定的参数成员（与原代码增益一一映射）
    double r_ = 0.0;
    double b0_ = 0.0;
    double w_c = 0.0;
    double w_0 = 0.0;
    double kp_ = 0.0;
    double kd_ = 0.0;
    double z1_ = 0.0; 
    double z2_ = 0.0; 
    double z3_ = 0.0; 

    // 原代码保留参数（均显式赋予默认值，无未定义风险）
    double m_ref_scale = 0.5;       // 参考信号缩放增益
    double m_corrected_ctrl = 0.0;  // 非线性校正后控制量
    double m_current_time = 0.0;    // 当前时间
    double m_step_size = 0.001;     // 控制步长（显式默认值1ms，彻底解决未定义问题）

    // 沿用指定的Eigen矩阵成员（替换原普通数组，均默认初始化）
    Eigen::Matrix<double,1,3> c;
    Eigen::Matrix<double,3,1> bd, ld, xbar, xbar_new;
    Eigen::Matrix<double,3,3> ad;
    // 跟踪微分器（TD）状态（Eigen矩阵，替换原m_td_state数组）
    Eigen::Matrix<double,2,1> m_td_state;

    // 自定义参考信号函数指针（原代码接口，保留，默认初始化nullptr）
    double (*m_custom_ref)(double time) = nullptr;

    // 沿用指定的常量
    static constexpr double inf = std::numeric_limits<double>::infinity();
};

} // namespace rmcs_core::controller::adrc

#endif // PURE_ADRC_CONTROLLER_H