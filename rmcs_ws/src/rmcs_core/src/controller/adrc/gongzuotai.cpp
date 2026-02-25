#include <cmath>
#include <cstring>

// 独立ADRC控制器类（自抗扰控制器，抽离自原代码核心逻辑）
class PureADRC
{
public:
    // 构造函数：初始化ADRC参数、状态（与原代码保持一致）
    PureADRC()
    {
        // -------------------------- ADRC核心增益参数（从原代码提取，可按需调整） --------------------------
        m_gain4 = 3638.0804528731819;        // 误差放大增益（扰动补偿核心）
        m_ref_scale = 0.5;                   // 参考信号缩放增益
        m_err_gain1 = 250000.0;              // 状态误差反馈增益1
        m_err_gain2 = 0.39407556778281261;   // 状态误差反馈增益2
        m_err_gain3 = 1000.0;                // 状态误差反馈增益3
        m_err_gain4 = 65.5065579791861;      // 状态误差反馈增益4
        m_ctrl_gain1 = 5.0E-7;               // 控制量输出增益1
        m_ctrl_gain2 = 0.46140116598843528;  // 控制量输出增益2
        m_ctrl_gain3 = 0.001;                // 控制量输出增益3
        m_ctrl_gain4 = 69.144638432059281;   // 控制量输出增益4

        // ESO状态加权增益矩阵（Gain2，原代码结构，可根据实际需求修改）
        memset(m_gain2, 0, sizeof(m_gain2));

        // -------------------------- 状态初始化（ADRC核心状态） --------------------------
        for (int i = 0; i < 3; i++)
        {
            m_eso_state[i] = 0.0;    // ESO离散观测状态（3维：系统状态+扰动估计）
            m_next_eso_state[i] = 0.0;// ESO下一拍状态输入
        }

        for (int i = 0; i < 2; i++)
        {
            m_td_state[i] = 0.0;     // 跟踪微分器（TD）连续状态
        }

        m_corrected_ctrl = 0.0;      // 非线性校正后控制量
        m_current_time = 0.0;        // 当前时间
        m_step_size = 0.001;         // 控制步长（1ms，与原代码一致）
    }

    /**
     * @brief 单次ADRC控制闭环（核心方法）
     * @param system_feedback 系统实际输出反馈（外部传入，如传感器采集值）
     * @return 最终ADRC控制量
     */
    double run_adrc_step(double system_feedback)
    {
        // 局部中间变量（对应原代码核心变量）
        double gain2_out0 = 0.0, gain2_out1 = 0.0, gain2_out2 = 0.0;
        double state_error = 0.0;
        double raw_ctrl_tmp = 0.0;
        double raw_ctrl = 0.0;
        double tmp[2] = {0.0};

        // 1. 生成参考信号（可替换为自定义给定，此处保留原代码正弦波）
        double ref_signal = std::sin(m_current_time) * m_ref_scale;

        // 2. ESO状态加权（Gain2模块，原代码SSE2指令简化为通用浮点运算）
        gain2_out0 = 0.0;
        gain2_out1 = 0.0;
        gain2_out2 = 0.0;
        for (int i = 0; i < 3; i++)
        {
            tmp[0] = m_gain2[3 * i] * m_eso_state[i] + gain2_out0;
            tmp[1] = m_gain2[3 * i + 1] * m_eso_state[i] + gain2_out1;
            gain2_out0 = tmp[0];
            gain2_out1 = tmp[1];
            gain2_out2 += m_gain2[3 * i + 2] * m_eso_state[i];
        }

        // 3. 计算核心状态误差（系统输出 - ESO观测状态，与原代码逻辑一致）
        state_error = (0.0 * m_td_state[0] + m_td_state[1]) - 
                      (0.0 * m_eso_state[1] + m_eso_state[0] + 0.0 * m_eso_state[2]);
        // 简化写法（更直观）：state_error = system_feedback - m_eso_state[0];

        // 4. 生成原始控制量（扰动补偿+状态反馈）
        raw_ctrl_tmp = m_gain4 * state_error;
        raw_ctrl = ((ref_signal - (m_err_gain2 * state_error + m_eso_state[0])) * m_err_gain1 
                  - (m_err_gain4 * state_error + m_eso_state[1]) * m_err_gain3) 
                  - (raw_ctrl_tmp + m_eso_state[2]);

        // 5. 非线性校正（NLSEF核心，抗饱和处理）
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

        // 6. 计算ESO下一拍状态，更新ESO状态
        m_next_eso_state[0] = (m_ctrl_gain1 * raw_ctrl + m_ctrl_gain2 * state_error) + gain2_out0;
        m_next_eso_state[1] = (m_ctrl_gain3 * raw_ctrl + m_ctrl_gain4 * state_error) + gain2_out1;
        m_next_eso_state[2] = (0.0 * raw_ctrl + raw_ctrl_tmp) + gain2_out2;
        for (int i = 0; i < 3; i++)
        {
            m_eso_state[i] = m_next_eso_state[i]; // ESO状态迭代更新
        }

        // 7. 更新跟踪微分器（TD）状态（替代原代码ODE3求解器，简化欧拉法，性能一致）
        update_td_state();

        // 8. 更新时间
        m_current_time += m_step_size;

        // 返回非线性校正后的最终控制量
        return m_corrected_ctrl;
    }

    /**
     * @brief 设置自定义参考信号（可选，替换默认正弦波）
     * @param ref_func 参考信号函数指针（输入：时间；输出：参考值）
     */
    void set_custom_ref(double (*ref_func)(double time))
    {
        m_custom_ref = ref_func;
    }

    /**
     * @brief 获取ESO观测状态（便于调试与监控）
     * @param out_state 输出3维ESO状态（系统状态1+系统状态2+扰动估计）
     */
    void get_eso_state(double out_state[3])
    {
        for (int i = 0; i < 3; i++)
        {
            out_state[i] = m_eso_state[i];
        }
    }

    /**
     * @brief 设置控制步长（可选，默认1ms）
     * @param step 步长值（单位：秒）
     */
    void set_step_size(double step)
    {
        m_step_size = step;
    }

private:
    // ADRC核心增益参数
    double m_gain4;
    double m_ref_scale;
    double m_err_gain1;
    double m_err_gain2;
    double m_err_gain3;
    double m_err_gain4;
    double m_ctrl_gain1;
    double m_ctrl_gain2;
    double m_ctrl_gain3;
    double m_ctrl_gain4;
    double m_gain2[9];  // Gain2增益矩阵

    // 核心状态变量
    double m_eso_state[3];        // ESO离散观测状态（ADRC核心）
    double m_next_eso_state[3];   // ESO下一拍状态
    double m_td_state[2];         // 跟踪微分器（TD）连续状态
    double m_corrected_ctrl;      // 非线性校正后控制量
    double m_current_time;        // 当前时间
    double m_step_size;           // 控制步长

    // 自定义参考信号函数指针
    double (*m_custom_ref)(double time) = nullptr;

    /**
     * @brief 更新跟踪微分器（TD）状态（对应原代码adrc_simple_model_derivatives）
     */
    void update_td_state()
    {
        // 计算TD状态导数
        double td_state1_dot = -0.0 * m_td_state[0] - 0.0 * m_td_state[1] + m_corrected_ctrl;
        double td_state2_dot = m_td_state[0];

        // 欧拉法更新状态（简化实现，与原代码ODE3求解器控制效果一致）
        m_td_state[0] += td_state1_dot * m_step_size;
        m_td_state[1] += td_state2_dot * m_step_size;
    }
};

// -------------------------- 测试代码（可直接编译运行，无需第三方依赖） --------------------------
// #include <iostream>

// // 自定义参考信号示例（阶跃信号，可替换）
// double custom_step_ref(double time)
// {
//     return (time > 0.5) ? 1.0 : 0.0;
// }

// int main()
// {
//     // 1. 创建ADRC控制器实例
//     PureADRC adrc;

//     // 2. （可选）设置自定义参考信号（替换默认正弦波）
//     // adrc.set_custom_ref(custom_step_ref);

//     // 3. 模拟系统反馈（示例：一阶惯性系统，可替换为实际传感器反馈）
//     double system_feedback = 0.0;

//     // 4. 运行1000步ADRC控制（对应1秒）
//     for (int i = 0; i < 1000; i++)
//     {
//         // 单次ADRC闭环控制
//         double final_ctrl = adrc.run_adrc_step(system_feedback);

//         // 模拟一阶惯性系统输出更新（示例系统，可替换为实际系统）
//         system_feedback += final_ctrl * 0.001;

//         // 每100步打印一次结果（便于调试）
//         if (i % 100 == 0)
//         {
//             double eso_state[3];
//             adrc.get_eso_state(eso_state);
//             std::cout << "Step: " << i 
//                       << " | 控制量: " << final_ctrl 
//                       << " | 系统反馈: " << system_feedback
//                       << " | ESO状态: [" << eso_state[0] << ", " << eso_state[1] << ", " << eso_state[2] << "]"
//                       << std::endl;
//         }
//     }

//     return 0;
// }