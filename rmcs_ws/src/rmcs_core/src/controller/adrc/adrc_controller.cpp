#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "adrc_calculator.hpp"

namespace rmcs_core::controller::adrc {

class AdrcController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AdrcController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , adrc_calculator_(load_config()) {
        register_input(get_parameter("measurement").as_string(), measurement_);

        if (has_parameter("setpoint")) {
            register_input(get_parameter("setpoint").as_string(), setpoint_);
            use_error_input_mode_ = false;
        } else if (has_parameter("target")) {
            register_input(get_parameter("target").as_string(), setpoint_);
            use_error_input_mode_ = false;
        } else {
            use_error_input_mode_ = true;
        }

        register_output(get_parameter("control").as_string(), control_);

        const auto init_reference = has_parameter("init_reference")
            ? get_parameter("init_reference").as_double()
            : 0.0;
        const auto init_measurement = has_parameter("init_measurement")
            ? get_parameter("init_measurement").as_double()
            : 0.0;
        adrc_calculator_.reset(init_reference, init_measurement);

        if (has_parameter("kt")) {
            kt_ = get_parameter("kt").as_double();
        }
        output_max_ = get_parameter_or("output_max", inf_);
        output_min_ = get_parameter_or("output_min", -inf_);

        if (use_error_input_mode_) {
            RCLCPP_WARN(
                get_logger(),
                "ADRC setpoint/target was not configured. Running in error-input mode: measurement topic is treated as error (setpoint - measurement).");
        }
    }

    void update() override {
        const double measurement_or_error = *measurement_;
        if (!std::isfinite(measurement_or_error)) {
            return;
        }

        double reference = 0.0;
        double measurement = 0.0;

        if (use_error_input_mode_) {
            // Error mode: input is e = r - y, convert to equivalent (r=0, y=-e).
            reference = 0.0;
            measurement = -measurement_or_error;
        } else {
            const double setpoint = *setpoint_;
            if (!std::isfinite(setpoint)) {
                return;
            }
            reference = setpoint;
            measurement = measurement_or_error;
        }

        const auto out = adrc_calculator_.update(reference, measurement);
        const double scaled_u = kt_ * out.u;
        *control_ = std::clamp(scaled_u, output_min_, output_max_);
    }

private:
    ADRCController::Config load_config() {
        ADRCController::Config cfg;
        if (has_parameter("dt")) cfg.dt = get_parameter("dt").as_double();
        if (has_parameter("b0")) cfg.b0 = get_parameter("b0").as_double();
        if (has_parameter("u_limit")) cfg.u_limit = get_parameter("u_limit").as_double();

        if (has_parameter("td_r")) cfg.td_r = get_parameter("td_r").as_double();
        if (has_parameter("td_h0")) cfg.td_h0 = get_parameter("td_h0").as_double();

        if (has_parameter("eso_omega")) cfg.eso_omega = get_parameter("eso_omega").as_double();
        if (has_parameter("auto_eso_beta")) cfg.auto_eso_beta = get_parameter("auto_eso_beta").as_bool();
        if (has_parameter("beta01")) cfg.beta01 = get_parameter("beta01").as_double();
        if (has_parameter("beta02")) cfg.beta02 = get_parameter("beta02").as_double();
        if (has_parameter("beta03")) cfg.beta03 = get_parameter("beta03").as_double();

        if (has_parameter("kp")) cfg.kp = get_parameter("kp").as_double();
        if (has_parameter("kd")) cfg.kd = get_parameter("kd").as_double();
        if (has_parameter("alpha1")) cfg.alpha1 = get_parameter("alpha1").as_double();
        if (has_parameter("alpha2")) cfg.alpha2 = get_parameter("alpha2").as_double();
        if (has_parameter("delta")) cfg.delta = get_parameter("delta").as_double();
        return cfg;
    }

    InputInterface<double> measurement_;
    InputInterface<double> setpoint_;
    OutputInterface<double> control_;

    ADRCController adrc_calculator_;
    bool use_error_input_mode_ = false;
    double kt_ = 1.0;
    double output_max_ = std::numeric_limits<double>::infinity();
    double output_min_ = -std::numeric_limits<double>::infinity();
    static constexpr double inf_ = std::numeric_limits<double>::infinity();
};

} // namespace rmcs_core::controller::adrc

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::adrc::AdrcController, rmcs_executor::Component)
