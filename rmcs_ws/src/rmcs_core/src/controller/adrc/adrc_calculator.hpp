#ifndef PURE_ADRC_CONTROLLER_H
#define PURE_ADRC_CONTROLLER_H

#include <cmath>

namespace rmcs_core::controller::adrc{

class ADRCController {
public:
    enum class ADRCParam {
        R,          
        B0,        
        BETA01,     
        BETA02,     
        BETA03,     
        ALPHA1,     
        ALPHA2,     
        DELTA,      
        KP,         
        KD          
    };

    explicit ADRCController(double sample_time = 0.001)
        : sample_time_(sample_time) {
        init_default_params(); // 初始化默认参数
    }

    void set_param(ADRCParam param_type, double value) {
        switch (param_type) {
            case ADRCParam::R:          r_ = value; break;
            case ADRCParam::B0:         b0_ = value; break;
            case ADRCParam::BETA01:     beta01_ = value; break;
            case ADRCParam::BETA02:     beta02_ = value; break;
            case ADRCParam::BETA03:     beta03_ = value; break;
            case ADRCParam::ALPHA1:     alpha1_ = value; break;
            case ADRCParam::ALPHA2:     alpha2_ = value; break;
            case ADRCParam::DELTA:      delta_ = value; break;
            case ADRCParam::KP:         kp_ = value; break;
            case ADRCParam::KD:         kd_ = value; break;
        }
    }

    void set_params(double r, double b0, double beta01, double beta02, double beta03,
                    double alpha1, double alpha2, double delta, double kp, double kd) {
        r_ = r;
        b0_ = b0;
        beta01_ = beta01;
        beta02_ = beta02;
        beta03_ = beta03;
        alpha1_ = alpha1;
        alpha2_ = alpha2;
        delta_ = delta;
        kp_ = kp;
        kd_ = kd;
    }

    double compute_adrc_output(double theta_cmd_rad, double theta_meas_rad, double u_prev_rad) {
        td_update(theta_cmd_rad);
        eso_update(theta_meas_rad, u_prev_rad);
        return nlsef();
    }

    double update(double theta_cmd_rad, double theta_meas_rad, double u_prev_rad){
        td_update(theta_cmd_rad);
        eso_update(theta_meas_rad, u_prev_rad);
        return nlsef();
    }

private:
    void td_update(double v_cmd) {
        double h = sample_time_;
        double e = v1_ - v_cmd;
        v2_ += h * fst(e, v2_, r_, h); 
        v1_ += h * v2_; 
    }

    void eso_update(double theta_meas, double u) {
        double h = sample_time_;
        double e = z1_ - theta_meas;
        z1_ += h * (z2_ - beta01_ * e); 
        z2_ += h * (z3_ + b0_ * u - beta02_ * fal(e, 0.5, delta_)); 
        z3_ += h * (-beta03_ * fal(e, 0.2, delta_)); 
    }

    double nlsef() const {
        double e1 = v1_ - z1_; 
        double e2 = v2_ - z2_; 
        double u0 = kp_ * fal(e1, alpha1_, delta_) + kd_ * fal(e2, alpha2_, delta_);
        return (u0 - z3_) / b0_;
    }

    static double fst(double e, double ed, double r, double h) {
        double d = r * pow(h, 2);
        double a0 = h * ed;
        double a = e + a0;
        return std::fabs(a) > d ? -r * std::copysign(1.0, a) : -r * (a / d);
    }

    static double fal(double e, double alpha, double delta) {
        if (std::fabs(e) > delta) {
            return std::copysign(pow(std::fabs(e), alpha), e);
        } else {
            return e / pow(delta, 1 - alpha);
        }
    }

    void init_default_params() {
        r_ = 50.0;          
        b0_ = 1.0;          
        beta01_ = 100.0;    
        beta02_ = 20.0;     
        beta03_ = 5.0;      
        alpha1_ = 0.5;     
        alpha2_ = 0.2;      
        delta_ = 0.01;      
        kp_ = 20.0;         
        kd_ = 5.0;          
    }

    double r_, b0_, beta01_, beta02_, beta03_;
    double alpha1_, alpha2_, delta_, kp_, kd_;
    double v1_ = 0.0; 
    double v2_ = 0.0; 
    double z1_ = 0.0; 
    double z2_ = 0.0; 
    double z3_ = 0.0; 
    double sample_time_ = 0.001;
    static constexpr double inf = std::numeric_limits<double>::infinity();
};

#endif // PURE_ADRC_CONTROLLER_H

}