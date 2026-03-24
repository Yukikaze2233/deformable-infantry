#ifndef RMCS_CORE_CONTROLLER_ADRC_CALCULATOR_HPP
#define RMCS_CORE_CONTROLLER_ADRC_CALCULATOR_HPP

#include <algorithm>
#include <cmath>
#include <limits>

namespace rmcs_core::controller::adrc {

class ADRCController {
public:
    struct Config {
        double dt = 0.001;              // Controller period [s]
        double b0 = 1.0;                // Nominal plant gain
        double u_limit = 1000.0;        // Output saturation

        // TD
        double td_r = 1000.0;           // Tracking speed factor
        double td_h0 = 0.003;           // TD filter step, usually 2~5*dt

        // ESO (linear ESO bandwidth method)
        double eso_omega = 120.0;
        bool auto_eso_beta = true;
        double beta01 = 360.0;
        double beta02 = 43200.0;
        double beta03 = 1728000.0;

        // NLSEF
        double kp = 200.0;
        double kd = 20.0;
        double alpha1 = 0.75;
        double alpha2 = 1.25;
        double delta = 0.01;
    };

    struct State {
        // TD
        double v1 = 0.0;
        double v2 = 0.0;
        // ESO
        double z1 = 0.0;
        double z2 = 0.0;
        double z3 = 0.0;
        // Last output for ESO recursion
        double u_last = 0.0;
    };

    struct Output {
        double u = 0.0;                 // Final control output
        double u0 = 0.0;                // NLSEF output before disturbance compensation
        double e1 = 0.0;                // Position error in NLSEF
        double e2 = 0.0;                // Velocity error in NLSEF
        double td_acc = 0.0;            // fhan output (TD acceleration)
        State state;                    // Updated state after this step
    };

    ADRCController()
        : ADRCController(Config{}) {}

    explicit ADRCController(const Config& config) {
        set_config(config);
        reset();
    }

    void set_config(const Config& config) {
        cfg_ = config;
        sanitize_config();
        if (cfg_.auto_eso_beta) {
            cfg_.beta01 = 3.0 * cfg_.eso_omega;
            cfg_.beta02 = 3.0 * cfg_.eso_omega * cfg_.eso_omega;
            cfg_.beta03 = cfg_.eso_omega * cfg_.eso_omega * cfg_.eso_omega;
        }
    }

    const Config& config() const { return cfg_; }
    const State& state() const { return state_; }

    void reset(double init_reference = 0.0, double init_measurement = 0.0) {
        state_.v1 = init_reference;
        state_.v2 = 0.0;
        state_.z1 = init_measurement;
        state_.z2 = 0.0;
        state_.z3 = 0.0;
        state_.u_last = 0.0;
    }

    Output update(double reference, double measurement) {
        Output out;

        // 1) TD
        const double td_acc = fhan(state_.v1 - reference, state_.v2, cfg_.td_r, cfg_.td_h0);
        const double v1_next = state_.v1 + cfg_.dt * state_.v2;
        const double v2_next = state_.v2 + cfg_.dt * td_acc;

        // 2) ESO (linear)
        const double eso_err = state_.z1 - measurement;
        const double z1_next = state_.z1 + cfg_.dt * (state_.z2 - cfg_.beta01 * eso_err);
        const double z2_next = state_.z2 + cfg_.dt * (state_.z3 + cfg_.b0 * state_.u_last - cfg_.beta02 * eso_err);
        const double z3_next = state_.z3 - cfg_.dt * cfg_.beta03 * eso_err;

        // 3) NLSEF
        const double e1 = v1_next - z1_next;
        const double e2 = v2_next - z2_next;
        const double u0 = cfg_.kp * fal(e1, cfg_.alpha1, cfg_.delta)
                        + cfg_.kd * fal(e2, cfg_.alpha2, cfg_.delta);

        // 4) Disturbance compensation + saturation
        const double u_raw = (u0 - z3_next) / cfg_.b0;
        const double u = clamp(u_raw, -cfg_.u_limit, cfg_.u_limit);

        // 5) Commit state
        state_.v1 = v1_next;
        state_.v2 = v2_next;
        state_.z1 = z1_next;
        state_.z2 = z2_next;
        state_.z3 = z3_next;
        state_.u_last = u;

        out.u = u;
        out.u0 = u0;
        out.e1 = e1;
        out.e2 = e2;
        out.td_acc = td_acc;
        out.state = state_;
        return out;
    }

private:
    static double sign(double x) {
        if (x > 0.0) {
            return 1.0;
        }
        if (x < 0.0) {
            return -1.0;
        }
        return 0.0;
    }

    static double clamp(double x, double low, double high) {
        return std::max(low, std::min(x, high));
    }

    static double fal(double e, double alpha, double delta) {
        const double abs_e = std::fabs(e);
        if (abs_e > delta) {
            return std::pow(abs_e, alpha) * sign(e);
        }
        return e / std::pow(delta, 1.0 - alpha);
    }

    static double fhan(double y, double x, double r, double h0) {
        const double d = r * h0 * h0;
        const double a0 = h0 * x;
        const double y1 = y + a0;
        const double a1 = std::sqrt(d * (d + 8.0 * std::fabs(y1)));

        const double a = (std::fabs(y1) > d)
            ? (a0 + sign(y1) * (a1 - d) / 2.0)
            : (a0 + y1 / h0);

        return (std::fabs(a) < d)
            ? (-r * (a / d))
            : (-r * sign(a));
    }

    void sanitize_config() {
        constexpr double kEps = 1e-9;
        cfg_.dt = std::max(cfg_.dt, kEps);
        cfg_.b0 = (std::fabs(cfg_.b0) < kEps)
            ? ((cfg_.b0 >= 0.0) ? kEps : -kEps)
            : cfg_.b0;
        cfg_.u_limit = std::max(cfg_.u_limit, 0.0);
        cfg_.td_r = std::max(cfg_.td_r, kEps);
        cfg_.td_h0 = std::max(cfg_.td_h0, kEps);
        cfg_.eso_omega = std::max(cfg_.eso_omega, kEps);
        cfg_.delta = std::max(cfg_.delta, kEps);
    }

    Config cfg_;
    State state_;
};

} // namespace rmcs_core::controller::adrc

#endif // RMCS_CORE_CONTROLLER_ADRC_CALCULATOR_HPP
