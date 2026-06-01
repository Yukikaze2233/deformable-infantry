#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::chassis::suspension {

class LoadWeightAllocator
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    LoadWeightAllocator()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , contact_confidence_weight_(get_parameter_or("contact_confidence_weight", 0.50))
        , min_contact_confidence_(get_parameter_or("min_contact_confidence", 0.20))
        , z3_score_weight_(get_parameter_or("z3_score_weight", 0.35))
        , torque_score_weight_(get_parameter_or("torque_score_weight", 0.15))
        , min_support_signal_(get_parameter_or("min_support_signal", 0.05)) {
        register_interfaces_();
    }

    void update() override {
        Inputs inputs;
        if (!read_inputs_(inputs)) {
            publish_nan_outputs_();
            return;
        }

        const auto support_signals = compute_support_signals_(inputs);
        if (!support_signals.has_value()) {
            publish_nan_outputs_();
            return;
        }

        publish_outputs_(*support_signals);
    }

private:
    struct Inputs {
        std::array<double, 4> contact_confidence{};
        std::array<double, 4> z3_score{};
        std::array<double, 4> torque_score{};
    };

    void register_interfaces_() {
        const std::array<std::string, 4> leg_names = {
            "left_front",
            "left_back",
            "right_back",
            "right_front",
        };
        const std::array<std::string, 4> joint_names = {
            "left_front_joint",
            "left_back_joint",
            "right_back_joint",
            "right_front_joint",
        };

        for (std::size_t i = 0; i < leg_names.size(); ++i) {
            const std::string leg_prefix = "leg_" + std::to_string(i + 1);
            register_input(
                get_parameter_or(
                    leg_prefix + "_contact_confidence",
                    "/chassis/" + joint_names[i] + "/ground_contact_confidence"),
                contact_confidence_inputs_[i]);
            register_input(
                get_parameter_or(
                    leg_prefix + "_z3_score", "/chassis/" + joint_names[i] + "/ground_contact_z3_score"),
                z3_score_inputs_[i]);
            register_input(
                get_parameter_or(
                    leg_prefix + "_torque_score",
                    "/chassis/" + joint_names[i] + "/ground_contact_torque_score"),
                torque_score_inputs_[i]);
            register_output(
                get_parameter_or(
                    leg_prefix + "_load_weight",
                    "/chassis/suspension/load_weight/" + leg_names[i]),
                load_weight_outputs_[i], nan_);
        }
    }

    [[nodiscard]] bool read_inputs_(Inputs& inputs) const {
        for (std::size_t i = 0; i < contact_confidence_inputs_.size(); ++i) {
            if (!contact_confidence_inputs_[i].ready() || !z3_score_inputs_[i].ready()
                || !torque_score_inputs_[i].ready()) {
                return false;
            }

            inputs.contact_confidence[i] = *contact_confidence_inputs_[i];
            inputs.z3_score[i] = *z3_score_inputs_[i];
            inputs.torque_score[i] = *torque_score_inputs_[i];
            if (!std::isfinite(inputs.contact_confidence[i]) || !std::isfinite(inputs.z3_score[i])
                || !std::isfinite(inputs.torque_score[i])) {
                return false;
            }
        }

        return true;
    }

    [[nodiscard]] std::optional<std::array<double, 4>> compute_support_signals_(const Inputs& inputs) const {
        if (!std::isfinite(contact_confidence_weight_) || !std::isfinite(z3_score_weight_)
            || !std::isfinite(torque_score_weight_) || !std::isfinite(min_contact_confidence_)
            || !std::isfinite(min_support_signal_)
            || contact_confidence_weight_ < 0.0 || z3_score_weight_ < 0.0 || torque_score_weight_ < 0.0
            || min_contact_confidence_ < 0.0 || min_support_signal_ < 0.0) {
            return std::nullopt;
        }

        std::array<double, 4> support_signals{};
        double support_sum = 0.0;
        for (std::size_t i = 0; i < support_signals.size(); ++i) {
            if (inputs.contact_confidence[i] < min_contact_confidence_) {
                support_signals[i] = 0.0;
                continue;
            }

            const double support = contact_confidence_weight_ * inputs.contact_confidence[i]
                + z3_score_weight_ * inputs.z3_score[i] + torque_score_weight_ * inputs.torque_score[i];
            support_signals[i] = support >= min_support_signal_ ? support : 0.0;
            support_sum += support_signals[i];
        }

        if (support_sum <= kMinDenominator) {
            support_signals.fill(0.25);
            return support_signals;
        }

        for (double& weight : support_signals) {
            weight /= support_sum;
        }
        return support_signals;
    }

    void publish_outputs_(const std::array<double, 4>& weights) {
        for (std::size_t i = 0; i < weights.size(); ++i) {
            *load_weight_outputs_[i] = weights[i];
        }
    }

    void publish_nan_outputs_() {
        for (auto& output : load_weight_outputs_) {
            *output = nan_;
        }
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kMinDenominator = 1e-9;

    double contact_confidence_weight_;
    double min_contact_confidence_;
    double z3_score_weight_;
    double torque_score_weight_;
    double min_support_signal_;

    std::array<InputInterface<double>, 4> contact_confidence_inputs_;
    std::array<InputInterface<double>, 4> z3_score_inputs_;
    std::array<InputInterface<double>, 4> torque_score_inputs_;
    std::array<OutputInterface<double>, 4> load_weight_outputs_;
};

} // namespace rmcs_core::chassis::suspension

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::chassis::suspension::LoadWeightAllocator, rmcs_executor::Component)
