#include <cmath>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/kalman/kalman_calculator.hpp"

namespace rmcs_core::controller::kalman {

class KalmanFilterController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    KalmanFilterController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , kalman_filter_(
              get_parameter_or("process_noise", 1e-3),
              get_parameter_or("measurement_noise", 1e-2),
              get_parameter_or("initial_covariance", 1.0)) {
        register_input(get_parameter("measurement").as_string(), measurement_);

        if (has_parameter("output")) {
            register_output(get_parameter("output").as_string(), output_);
        } else {
            register_output(get_parameter("control").as_string(), output_);
        }

        if (has_parameter("initial_estimate")) {
            kalman_filter_.reset(
                get_parameter("initial_estimate").as_double(),
                get_parameter_or("initial_covariance", 1.0));
        } else {
            kalman_filter_.reset();
        }
    }

    void update() override {
        const double measurement = *measurement_;
        if (!std::isfinite(measurement)) {
            return;
        }

        const double output_value = kalman_filter_.update(measurement);
        if (!std::isfinite(output_value)) {
            return;
        }
        *output_ = output_value;
    }

private:
    InputInterface<double> measurement_;
    OutputInterface<double> output_;
    KalmanCalculator kalman_filter_;
};

} // namespace rmcs_core::controller::kalman

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::kalman::KalmanFilterController, rmcs_executor::Component)
