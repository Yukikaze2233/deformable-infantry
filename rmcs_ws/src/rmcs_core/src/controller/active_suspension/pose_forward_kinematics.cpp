#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::chassis::suspension {

class PoseForwardKinematics
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PoseForwardKinematics()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , l_(get_parameter_or("l", 0.0))
        , L_(get_parameter_or("L", 0.0))
        , h_(get_parameter_or("h", 0.0)) {
        const auto a_topic = get_parameter_or("a_topic", std::string("/chassis/suspension/plane/a"));
        const auto b_topic = get_parameter_or("b_topic", std::string("/chassis/suspension/plane/b"));
        const auto c_topic = get_parameter_or("c_topic", std::string("/chassis/suspension/plane/c"));
        const auto output_prefix =
            get_parameter_or("output_prefix", std::string("/chassis/suspension/pose_forward_kinematics"));

        register_input(a_topic, a_input_);
        register_input(b_topic, b_input_);
        register_input(c_topic, c_input_);

        for (std::size_t i = 0; i < angle_outputs_.size(); ++i) {
            register_output(
                output_prefix + "/theta_" + std::to_string(i + 1), angle_outputs_[i], nan_);
        }
    }

    void update() override {
        if (!inputs_ready_()) {
            publish_nan_outputs_();
            return;
        }

        Solution solution;
        if (!solve_(*a_input_, *b_input_, *c_input_, solution)) {
            publish_nan_outputs_();
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "pose_forward_kinematics: unable to find a valid outer-expansion solution.");
            return;
        }

        for (std::size_t i = 0; i < angle_outputs_.size(); ++i) {
            *angle_outputs_[i] = solution.angles[i];
        }
    }

private:
    struct Point {
        double x;
        double y;
        double z;
    };

    struct Solution {
        double H;
        std::array<double, 4> angles;
    };

    [[nodiscard]] bool inputs_ready_() const {
        return a_input_.ready() && std::isfinite(*a_input_) && b_input_.ready() && std::isfinite(*b_input_)
            && c_input_.ready() && std::isfinite(*c_input_);
    }

    void publish_nan_outputs_() {
        for (auto& angle : angle_outputs_) {
            *angle = nan_;
        }
    }

    [[nodiscard]] bool solve_(double a, double b, double c, Solution& solution) const {
        if (!std::isfinite(l_) || !std::isfinite(L_) || !std::isfinite(h_) || l_ < 0.0 || L_ <= 0.0
            || h_ <= 0.0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "pose_forward_kinematics: parameters l, L, h must be finite and satisfy l>=0, L>0, h>0.");
            return false;
        }

        if (std::abs(c) <= epsilon_) {
            return false;
        }

        const double slope = std::hypot(a, b) / std::abs(c);
        if (slope > max_slope_ + epsilon_) {
            return false;
        }

        const double radial_sq = L_ * L_ - h_ * h_;
        if (radial_sq < -epsilon_) {
            return false;
        }

        const double expansion = std::sqrt(std::max(0.0, radial_sq) / 2.0);
        const double T         = l_ + expansion;

        const std::array<double, 4> gamma = {
            -(a + b) / c,
            -(a - b) / c,
            +(a + b) / c,
            +(a - b) / c,
        };

        std::optional<Solution> best_solution;
        for (double gamma_k : gamma) {
            const double candidate_H = h_ + gamma_k * T;

            Solution candidate_solution{
                .H      = candidate_H,
                .angles = {},
            };

            bool valid = true;
            double min_gap = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < gamma.size(); ++i) {
                const auto t = select_outer_root_(gamma[i], candidate_H, T);
                if (!t) {
                    valid = false;
                    break;
                }

                const auto b_point = point_from_t_(i, *t, gamma[i]);
                min_gap = std::min(min_gap, candidate_solution.H - b_point.z);
                candidate_solution.angles[i] = angle_with_xoy_(i, candidate_H, b_point);
                if (!std::isfinite(candidate_solution.angles[i])) {
                    valid = false;
                    break;
                }
            }

            if (!valid) {
                continue;
            }

            if (std::abs(min_gap - h_) > validation_tolerance_) {
                continue;
            }

            if (!best_solution || candidate_solution.H > best_solution->H) {
                best_solution = candidate_solution;
            }
        }

        if (!best_solution) {
            return false;
        }

        solution = *best_solution;
        return true;
    }

    [[nodiscard]] std::optional<double> select_outer_root_(double gamma, double H, double T) const {
        const double A = 2.0 + gamma * gamma;
        const double discriminant =
            A * L_ * L_ - 2.0 * (H - gamma * l_) * (H - gamma * l_);

        if (discriminant < -epsilon_) {
            return std::nullopt;
        }

        const double sqrt_discriminant = std::sqrt(std::max(0.0, discriminant));
        std::array<double, 2> roots     = {
            (2.0 * l_ + gamma * H - sqrt_discriminant) / A,
            (2.0 * l_ + gamma * H + sqrt_discriminant) / A,
        };

        std::optional<double> best_root;
        for (double root : roots) {
            if (!is_valid_outer_root_(root, gamma, H, T)) {
                continue;
            }

            if (!best_root || root > *best_root) {
                best_root = root;
            }
        }

        return best_root;
    }

    [[nodiscard]] bool is_valid_outer_root_(double t, double gamma, double H, double T) const {
        if (!std::isfinite(t) || t < l_ - validation_tolerance_ || t > T + validation_tolerance_) {
            return false;
        }

        const double gap = H - gamma * t;
        if (gap <= 0.0 || gap < h_ - validation_tolerance_) {
            return false;
        }

        const double residual =
            2.0 * (t - l_) * (t - l_) + gap * gap - L_ * L_;
        const double scale = std::max({1.0, std::abs(H), std::abs(gamma * t), L_ * L_});
        return std::abs(residual) <= validation_tolerance_ * scale;
    }

    [[nodiscard]] static Point point_from_t_(std::size_t index, double t, double gamma) {
        switch (index) {
        case 0:
            return {t, t, gamma * t};
        case 1:
            return {t, -t, gamma * t};
        case 2:
            return {-t, -t, gamma * t};
        case 3:
            return {-t, t, gamma * t};
        default:
            return {nan_, nan_, nan_};
        }
    }

    [[nodiscard]] static Point anchor_point_(std::size_t index, double l, double H) {
        switch (index) {
        case 0:
            return {l, l, H};
        case 1:
            return {l, -l, H};
        case 2:
            return {-l, -l, H};
        case 3:
            return {-l, l, H};
        default:
            return {nan_, nan_, nan_};
        }
    }

    [[nodiscard]] double angle_with_xoy_(std::size_t index, double H, const Point& b_point) const {
        const auto a_point = anchor_point_(index, l_, H);
        const double horizontal =
            std::hypot(b_point.x - a_point.x, b_point.y - a_point.y);
        const double vertical = std::abs(b_point.z - a_point.z);
        return std::atan2(vertical, horizontal);
    }

    static constexpr double epsilon_              = 1e-9;
    static constexpr double validation_tolerance_ = 1e-6;
    static constexpr double max_slope_            = 1.0 / 1.7320508075688772;
    static constexpr double nan_                  = std::numeric_limits<double>::quiet_NaN();

    double l_;
    double L_;
    double h_;

    InputInterface<double> a_input_;
    InputInterface<double> b_input_;
    InputInterface<double> c_input_;

    std::array<OutputInterface<double>, 4> angle_outputs_;
};

} // namespace rmcs_core::chassis::suspension

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::chassis::suspension::PoseForwardKinematics, rmcs_executor::Component)
