#ifndef PURE_ADRC_CONTROLLER_H
#define PURE_ADRC_CONTROLLER_H

#include <cmath>
#include <eigen3/Eigen/Dense>

namespace rmcs_core::controller::adrc{

class LinkADRCController {
public:
    enum class ADRCParam {
        R,          
        B0,        
        WC,
        W0,    
        KP,         
        KD          
    };

    explicit LinkADRCController()
    {
        init_default_params(); // 初始化默认参数
    }

    void set_param(ADRCParam param_type, double value) {
        switch (param_type) {
            case ADRCParam::B0:         b0_ = value; break;
            case ADRCParam::WC:         w_c = value; break;
            case ADRCParam::W0:         w_0 = value; break;
            case ADRCParam::KP:         kp_ = value; break;
            case ADRCParam::KD:         kd_ = value; break;
        }
    }

    void set_params(double wc, double b0, double w0, double kp, double kd) {
        r_ = wc * wc;
        b0_ = b0;
        w_c = wc;
        w_0 = w0;
        kp_ = kp;
        kd_ = kd;
    }

    double compute_adrc_output(double cmd, double meas) {
        double a = lsef(cmd);
        eso_update_link(a, meas);
        return a;
    }

    
    double update_link(double cmd, double meas){//target ,measurement 
        double a = lsef(cmd);
        eso_update_link(a, meas);
        return a;
    }

private:
    void eso_update_link(double u,double y_meas) {
        Eigen::Matrix<double,3,1> bd_u, ad_u, ad_ld_u ,xhat;
        bd_u << bd(0)*u , bd(1)*u , bd(2)*u ;
        ad_u = ad * xbar;
        ad_ld_u = ad * ld * (y_meas - c * xbar);
        xbar_new = ad_u + bd_u + ad_ld_u;
        xhat = xbar + ld * (y_meas - c * xbar);
        z1_ = xhat(0);
        z2_ = xhat(1);
        z3_ = xhat(2);
        xbar = xbar_new;
    } 


    double lsef(double target) const {
        double e1 = target - z1_; 
        double u0 = e1 * r_ - z2_ * w_c * 2;
        return (u0 - z3_) / b0_;
    }


    void init_default_params() {       
        r_ = 250000.0;  
        b0_ = 1.0;          
        w_c = 500;
        w_0 = 167;    
        kp_ = 20.0;         
        kd_ = 5.0;
        c << 1.0, 0.0, 0.0;
        bd << 5.0E-7, 0.001, 0.0; 
        ld << 0.39408, 65.507, 3638.1;     
        ad << 1.0, 0.0, 0.0, 0.001, 1.0, 0.0, 5.0E-7, 0.001, 1.0;
        xbar << 0.0, 0.0, 0.0;
        xbar_new << 0.0, 0.0, 0.0;
    }

    double r_, b0_, w_c, w_0;
    double kp_, kd_;
    double z1_ = 0.0; 
    double z2_ = 0.0; 
    double z3_ = 0.0; 
    Eigen::Matrix<double,1,3> c ;
    Eigen::Matrix<double,3,1> bd,ld,z,xbar,xbar_new ;
    Eigen::Matrix<double,3,3> ad ;
    static constexpr double inf = std::numeric_limits<double>::infinity();
};

}
#endif // PURE_ADRC_CONTROLLER_H
