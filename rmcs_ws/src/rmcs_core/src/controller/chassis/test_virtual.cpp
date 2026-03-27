#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <limits>
#include <numbers>
#include <random>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/adrc/ESO.hpp"
#include "controller/adrc/NLESF.hpp"
#include "controller/adrc/TD.hpp"

namespace rmcs_core::virtue::chassis {

class ChassisLiftController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ChassisLiftController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , random_engine_(
              static_cast<unsigned int>(
                  std::chrono::system_clock::now().time_since_epoch().count()))
        , td_(default_td_config_())
        , eso_(default_eso_config_())
        , nlesf_(default_nlesf_config_()) {

        register_output("/test/motor/torque", motor_torque_output_, 0.0);
        register_output("/test/motor/velocity_rad_s", motor_velocity_output_, 0.0);
        register_output("/test/mechanism/angle_deg", mechanism_angle_deg_output_, 0.0);
        register_output("/test/target/angle_deg", target_theta_deg_output_, 0.0);
        register_output("/test/adrc/z3", z3_output_, 0.0);
        register_output("/test/model/gravity_torque", gravity_torque_output_, 0.0);

        load_parameters_();
        initialize_state_();
    }

    void update() override {
        const double target_theta_rad = resolve_target_theta_rad_();

        const auto td_out  = td_.update(target_theta_rad);
        const auto eso_out = eso_.update(theta_rad_, last_torque_cmd_);

        const double e1 = td_out.x1 - eso_out.z1;
        const double e2 = td_out.x2 - eso_out.z2;
        const auto nlesf_out = nlesf_.compute(e1, e2, eso_out.z3, adrc_b0_);

        const double torque_cmd = std::clamp(
            adrc_output_scale_ * nlesf_out.u,
            -torque_limit_,
            torque_limit_);

        last_torque_cmd_ = apply_contact_torque_guard_(torque_cmd);
        simulate_step_(last_torque_cmd_);

        const double target_theta_deg = target_theta_rad * kRadToDeg;

        *motor_torque_output_        = last_torque_cmd_;
        *motor_velocity_output_      = theta_dot_rad_;
        *mechanism_angle_deg_output_ = theta_rad_ * kRadToDeg;
        *target_theta_deg_output_    = target_theta_deg;
        *z3_output_                  = eso_out.z3;
        *gravity_torque_output_      = gravity_gain_ * std::cos(theta_rad_);

        elapsed_time_s_ += dt_;

        if ((loop_counter_++ % log_every_n_) == 0) {
            RCLCPP_INFO(
                get_logger(),
                "target=%.3f deg, angle=%.3f deg, motor_vel=%.3f rad/s, torque=%.3f",
                target_theta_deg,
                theta_rad_ * kRadToDeg,
                theta_dot_rad_ / 36.0,
                last_torque_cmd_);
        }
    }

private:
    static constexpr double kDegToRad = std::numbers::pi / 180.0;
    static constexpr double kRadToDeg = 180.0 / std::numbers::pi;

    static controller::adrc::TD::Config default_td_config_() {
        controller::adrc::TD::Config cfg;
        cfg.h = 0.003;
        cfg.r = 300.0;
        return cfg;
    }

    static controller::adrc::ESO::Config default_eso_config_() {
        controller::adrc::ESO::Config cfg;
        cfg.h = 0.001;
        cfg.b0 = 100.0;
        cfg.w0 = 120.0;
        cfg.auto_beta = true;
        return cfg;
    }

    static controller::adrc::NLESF::Config default_nlesf_config_() {
        controller::adrc::NLESF::Config cfg;
        cfg.k1 = 20.0;
        cfg.k2 = 2.0;
        cfg.alpha1 = 0.75;
        cfg.alpha2 = 1.25;
        cfg.delta = 0.02;
        cfg.u_min = -28.0;
        cfg.u_max = 28.0;
        return cfg;
    }

    static std::string normalize_mode_(std::string mode) {
        std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        return mode;
    }

    double min_angle_rad_() const { return min_angle_deg_ * kDegToRad; }
    double max_angle_rad_() const { return max_angle_deg_ * kDegToRad; }

    double clamp_angle_rad_(double value_rad) const {
        return std::clamp(value_rad, min_angle_rad_(), max_angle_rad_());
    }

    void load_parameters_() {
        min_angle_deg_ = get_parameter_or("physical_min_angle_deg", get_parameter_or("min_angle", 15.0));
        max_angle_deg_ = get_parameter_or("physical_max_angle_deg", get_parameter_or("max_angle", 55.0));

        inertia_ = get_parameter_or("inertia", 0.04);
        viscous_damping_ = get_parameter_or("viscous_damping", 0.02);
        reduction_ratio_ = get_parameter_or("reduction_ratio", 36.0);
        gravity_gain_ = get_parameter_or("gravity_gain", 11.2);
        torque_limit_ = get_parameter_or("torque_limit", 28.0);

        disturbance_min_ = get_parameter_or("disturbance_min", 0.0);
        disturbance_max_ = get_parameter_or("disturbance_max", 0.0);
        if (disturbance_min_ > disturbance_max_)
            std::swap(disturbance_min_, disturbance_max_);
        disturbance_distribution_ =
            std::uniform_real_distribution<double>(disturbance_min_, disturbance_max_);

        limit_restitution_ = std::clamp(get_parameter_or("limit_restitution", 0.0), 0.0, 1.0);
        limit_smooth_zone_deg_ = std::max(0.0, get_parameter_or("limit_smooth_zone_deg", 3.0));
        limit_inward_torque_scale_min_ = std::clamp(
            get_parameter_or("limit_inward_torque_scale_min", 1.0),
            0.0,
            1.0);
        limit_approach_damping_ = std::max(0.0, get_parameter_or("limit_approach_damping", 1.2));
        limit_contact_guard_enable_ = get_parameter_or("limit_contact_guard_enable", true);
        limit_contact_hold_margin_ = std::max(0.0, get_parameter_or("limit_contact_hold_margin", 0.8));
        limit_release_speed_rad_s_ = std::max(
            0.0,
            get_parameter_or("limit_release_speed_deg_s", 0.5) * kDegToRad);

        adrc_b0_ = get_parameter_or("adrc_b0", 0.694);
        adrc_output_scale_ = get_parameter_or("kt", 1.0);

        auto td_cfg = default_td_config_();
        td_cfg.h = get_parameter_or("adrc_td_h", td_cfg.h);
        td_cfg.r = get_parameter_or("adrc_td_r", td_cfg.r);
        td_.set_config(td_cfg);

        auto eso_cfg = eso_.config();
        eso_cfg.h = get_parameter_or("adrc_eso_h", eso_cfg.h);
        eso_cfg.b0 = adrc_b0_;
        eso_cfg.w0 = get_parameter_or("adrc_eso_w0", eso_cfg.w0);
        eso_cfg.auto_beta = get_parameter_or("adrc_eso_auto_beta", eso_cfg.auto_beta);
        eso_.set_config(eso_cfg);

        auto nlesf_cfg = nlesf_.config();
        nlesf_cfg.k1 = get_parameter_or("adrc_nlesf_k1", nlesf_cfg.k1);
        nlesf_cfg.k2 = get_parameter_or("adrc_nlesf_k2", nlesf_cfg.k2);
        nlesf_cfg.alpha1 = get_parameter_or("adrc_nlesf_alpha1", nlesf_cfg.alpha1);
        nlesf_cfg.alpha2 = get_parameter_or("adrc_nlesf_alpha2", nlesf_cfg.alpha2);
        nlesf_cfg.delta = get_parameter_or("adrc_nlesf_delta", nlesf_cfg.delta);
        nlesf_cfg.u_min = get_parameter_or("adrc_nlesf_u_min", -torque_limit_);
        nlesf_cfg.u_max = get_parameter_or("adrc_nlesf_u_max", torque_limit_);
        nlesf_.set_config(nlesf_cfg);

        fixed_target_angle_deg_ = std::clamp(
            get_parameter_or("fixed_target_angle_deg", get_parameter_or("fixed_target_angle", 55.0)),
            min_angle_deg_,
            max_angle_deg_);

        target_mode_ = normalize_mode_(get_parameter_or("target_mode", std::string{"yaml"}));

        code_target_center_deg_ = std::clamp(
            get_parameter_or("code_target_center_deg", fixed_target_angle_deg_),
            min_angle_deg_,
            max_angle_deg_);
        code_target_amplitude_deg_ = std::max(0.0, get_parameter_or("code_target_amplitude_deg", 0.0));
        code_target_period_s_ = std::max(0.0, get_parameter_or("code_target_period_s", 6.0));

        initial_angle_deg_ = std::clamp(
            get_parameter_or("initial_angle_deg", min_angle_deg_),
            min_angle_deg_,
            max_angle_deg_);

        log_every_n_ = std::max(1, get_parameter_or("log_every_n", 100));
    }

    void initialize_state_() {
        theta_rad_ = initial_angle_deg_ * kDegToRad;
        theta_dot_rad_ = 0.0;
        last_torque_cmd_ = 0.0;
        elapsed_time_s_ = 0.0;
        latched_limit_side_ = 0;
        last_limit_contact_ = false;
        loop_counter_ = 0;

        td_.reset(theta_rad_, 0.0);
        eso_.reset(theta_rad_);

        *motor_torque_output_ = 0.0;
        *motor_velocity_output_ = 0.0;
        *mechanism_angle_deg_output_ = theta_rad_ * kRadToDeg;
        *target_theta_deg_output_ = resolve_target_theta_rad_() * kRadToDeg;
    }

    double code_target_angle_rad_(double time_s) const {
        if (code_target_period_s_ <= 1e-9 || code_target_amplitude_deg_ <= 1e-9)
            return clamp_angle_rad_(code_target_center_deg_ * kDegToRad);

        const double omega = 2.0 * std::numbers::pi / code_target_period_s_;
        const double angle_deg =
            code_target_center_deg_ + code_target_amplitude_deg_ * std::sin(omega * time_s);
        return clamp_angle_rad_(angle_deg * kDegToRad);
    }

    double resolve_target_theta_rad_() {
        if (target_mode_ == "code")
            return code_target_angle_rad_(elapsed_time_s_);

        double target_angle_deg = fixed_target_angle_deg_;
        if (!get_parameter("fixed_target_angle_deg", target_angle_deg))
            (void)get_parameter("fixed_target_angle", target_angle_deg);

        if (!std::isfinite(target_angle_deg))
            target_angle_deg = fixed_target_angle_deg_;

        fixed_target_angle_deg_ = std::clamp(target_angle_deg, min_angle_deg_, max_angle_deg_);
        return fixed_target_angle_deg_ * kDegToRad;
    }

    double apply_contact_torque_guard_(double torque_cmd) const {
        if (!limit_contact_guard_enable_)
            return torque_cmd;

        constexpr double kContactEps = 1e-8;
        const bool near_upper = theta_rad_ >= (max_angle_rad_() - kContactEps);
        const bool near_lower = theta_rad_ <= (min_angle_rad_() + kContactEps);

        const bool upper_contact_active =
            (latched_limit_side_ == 1) || (last_limit_contact_ && near_upper);
        const bool lower_contact_active =
            (latched_limit_side_ == -1) || (last_limit_contact_ && near_lower);

        const double mechanism_velocity_rad = theta_dot_rad_ / reduction_ratio_;
        const double gravity_torque = gravity_gain_ * std::cos(theta_rad_);
        const double viscous_torque = viscous_damping_ * mechanism_velocity_rad;
        const double neutral_hold_torque = gravity_torque + viscous_torque;

        double guarded_torque = torque_cmd;
        if (upper_contact_active && guarded_torque > 0.0) {
            const double max_inward_torque =
                std::max(0.0, neutral_hold_torque + limit_contact_hold_margin_);
            guarded_torque = std::min(guarded_torque, max_inward_torque);
        }
        if (lower_contact_active && guarded_torque < 0.0) {
            const double min_inward_torque =
                std::min(0.0, neutral_hold_torque - limit_contact_hold_margin_);
            guarded_torque = std::max(guarded_torque, min_inward_torque);
        }

        return std::clamp(guarded_torque, -torque_limit_, torque_limit_);
    }

    void simulate_step_(double torque_cmd) {
        const double min_theta = min_angle_rad_();
        const double max_theta = max_angle_rad_();

        const double previous_theta = theta_rad_;
        const double mechanism_velocity = theta_dot_rad_ / reduction_ratio_;

        const double random_damping = disturbance_distribution_(random_engine_);
        const double velocity_sign =
            (mechanism_velocity > 1e-9) ? 1.0
            : ((mechanism_velocity < -1e-9) ? -1.0 : ((torque_cmd >= 0.0) ? 1.0 : -1.0));

        const double disturbance_torque = velocity_sign * random_damping;
        const double viscous_torque = viscous_damping_ * mechanism_velocity;
        const double gravity_torque = gravity_gain_ * std::cos(theta_rad_);

        auto smoothstep = [](double x) {
            const double xc = std::clamp(x, 0.0, 1.0);
            return xc * xc * (3.0 - 2.0 * xc);
        };

        double shaped_torque_cmd = torque_cmd;
        const double smooth_zone_rad = limit_smooth_zone_deg_ * kDegToRad;

        if (smooth_zone_rad > 1e-9) {
            if (shaped_torque_cmd > 0.0) {
                const double distance_to_upper = max_theta - theta_rad_;
                const double near_upper = 1.0 - smoothstep(distance_to_upper / smooth_zone_rad);
                const double torque_scale =
                    1.0 - near_upper * (1.0 - limit_inward_torque_scale_min_);
                shaped_torque_cmd *= torque_scale;
                if (mechanism_velocity > 0.0)
                    shaped_torque_cmd -= limit_approach_damping_ * near_upper * mechanism_velocity;
            } else if (shaped_torque_cmd < 0.0) {
                const double distance_to_lower = theta_rad_ - min_theta;
                const double near_lower = 1.0 - smoothstep(distance_to_lower / smooth_zone_rad);
                const double torque_scale =
                    1.0 - near_lower * (1.0 - limit_inward_torque_scale_min_);
                shaped_torque_cmd *= torque_scale;
                if (mechanism_velocity < 0.0)
                    shaped_torque_cmd -= limit_approach_damping_ * near_lower * mechanism_velocity;
            }
        }

        const double theta_ddot =
            (shaped_torque_cmd - disturbance_torque - viscous_torque - gravity_torque) / inertia_;

        const double theta_dot_next = theta_dot_rad_ + theta_ddot * dt_;
        const double mechanism_velocity_next = theta_dot_next / reduction_ratio_;
        const double theta_next = theta_rad_ + mechanism_velocity * dt_;

        bool limit_contact = false;
        if (latched_limit_side_ == -1) {
            if (mechanism_velocity_next <= limit_release_speed_rad_s_) {
                theta_rad_ = min_theta;
                theta_dot_rad_ = 0.0;
                limit_contact = true;
            } else {
                latched_limit_side_ = 0;
                theta_rad_ = min_theta;
                theta_dot_rad_ = theta_dot_next;
            }
        } else if (latched_limit_side_ == 1) {
            if (mechanism_velocity_next >= -limit_release_speed_rad_s_) {
                theta_rad_ = max_theta;
                theta_dot_rad_ = 0.0;
                limit_contact = true;
            } else {
                latched_limit_side_ = 0;
                theta_rad_ = max_theta;
                theta_dot_rad_ = theta_dot_next;
            }
        } else if (theta_next <= min_theta) {
            theta_rad_ = min_theta;
            if (theta_dot_next < 0.0) {
                const bool crossed_from_inside = previous_theta > min_theta + 1e-9;
                if (crossed_from_inside) {
                    const double post_impact_speed = -limit_restitution_ * theta_dot_next;
                    theta_dot_rad_ = post_impact_speed;
                    if (std::fabs(post_impact_speed / reduction_ratio_) <= limit_release_speed_rad_s_) {
                        latched_limit_side_ = -1;
                        theta_dot_rad_ = 0.0;
                    }
                    limit_contact = true;
                } else {
                    theta_dot_rad_ = 0.0;
                    latched_limit_side_ = -1;
                    limit_contact = true;
                }
            } else {
                theta_dot_rad_ = theta_dot_next;
            }
        } else if (theta_next >= max_theta) {
            theta_rad_ = max_theta;
            if (theta_dot_next > 0.0) {
                const bool crossed_from_inside = previous_theta < max_theta - 1e-9;
                if (crossed_from_inside) {
                    const double post_impact_speed = -limit_restitution_ * theta_dot_next;
                    theta_dot_rad_ = post_impact_speed;
                    if (std::fabs(post_impact_speed / reduction_ratio_) <= limit_release_speed_rad_s_) {
                        latched_limit_side_ = 1;
                        theta_dot_rad_ = 0.0;
                    }
                    limit_contact = true;
                } else {
                    theta_dot_rad_ = 0.0;
                    latched_limit_side_ = 1;
                    limit_contact = true;
                }
            } else {
                theta_dot_rad_ = theta_dot_next;
            }
        } else {
            latched_limit_side_ = 0;
            theta_rad_ = theta_next;
            theta_dot_rad_ = theta_dot_next;
        }

        last_limit_contact_ = limit_contact;
    }

    OutputInterface<double> motor_torque_output_;
    OutputInterface<double> motor_velocity_output_;
    OutputInterface<double> mechanism_angle_deg_output_;
    OutputInterface<double> target_theta_deg_output_;
    OutputInterface<double> z3_output_;
    OutputInterface<double> gravity_torque_output_;

    const double dt_ = 0.001;

    double min_angle_deg_ = 15.0;
    double max_angle_deg_ = 55.0;
    double inertia_ = 0.04;
    double viscous_damping_ = 0.02;
    double reduction_ratio_ = 36.0;
    double gravity_gain_ = 11.2;
    double torque_limit_ = 28.0;

    double disturbance_min_ = 0.0;
    double disturbance_max_ = 0.0;
    std::uniform_real_distribution<double> disturbance_distribution_{0.0, 0.0};

    double limit_restitution_ = 0.0;
    double limit_smooth_zone_deg_ = 3.0;
    double limit_inward_torque_scale_min_ = 1.0;
    double limit_approach_damping_ = 1.2;
    bool limit_contact_guard_enable_ = true;
    double limit_contact_hold_margin_ = 0.8;
    double limit_release_speed_rad_s_ = 0.5 * kDegToRad;

    double adrc_b0_ = 0.694;
    double adrc_output_scale_ = 1.0;

    double fixed_target_angle_deg_ = 55.0;
    std::string target_mode_ = "yaml";
    double code_target_center_deg_ = 55.0;
    double code_target_amplitude_deg_ = 0.0;
    double code_target_period_s_ = 6.0;
    double initial_angle_deg_ = 15.0;

    double theta_rad_ = 15.0 * kDegToRad;
    double theta_dot_rad_ = 0.0;
    double last_torque_cmd_ = 0.0;
    double elapsed_time_s_ = 0.0;

    bool last_limit_contact_ = false;
    int latched_limit_side_ = 0;

    int log_every_n_ = 100;
    int loop_counter_ = 0;

    std::mt19937 random_engine_;

    controller::adrc::TD td_;
    controller::adrc::ESO eso_;
    controller::adrc::NLESF nlesf_;
};

} // namespace rmcs_core::virtue::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::virtue::chassis::ChassisLiftController, rmcs_executor::Component)
