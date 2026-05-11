#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "active_suspension_geometry.hpp"

namespace rmcs_core::chassis::suspension {

namespace geom = rmcs_core::chassis::suspension::geometry;

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
        , h_(get_parameter_or("h", 0.0))
        , yaw_topic_(get_parameter_or("yaw_topic", std::string("/chassis/imu/yaw")))
        , static_yaw_(get_parameter_or("static_yaw", 0.0))
        , use_yaw_topic_(get_parameter_or("use_yaw_topic", false))
        , yaw_in_degrees_(get_parameter_or("yaw_in_degrees", false))
        , angles_in_degrees_(get_parameter_or("angles_in_degrees", false)) {
        const auto a_topic =
            get_parameter_or("a_topic", std::string("/chassis/suspension/ground_inverse/plane/a"));
        const auto b_topic =
            get_parameter_or("b_topic", std::string("/chassis/suspension/ground_inverse/plane/b"));
        const auto c_topic =
            get_parameter_or("c_topic", std::string("/chassis/suspension/ground_inverse/plane/c"));
        const auto d_topic =
            get_parameter_or("d_topic", std::string("/chassis/suspension/ground_inverse/plane/d"));
        const auto output_prefix =
            get_parameter_or("output_prefix", std::string("/chassis/suspension/pose_forward_kinematics"));

        register_input(yaw_topic_, yaw_input_);
        register_input(a_topic, a_input_);
        register_input(b_topic, b_input_);
        register_input(c_topic, c_input_);
        register_input(d_topic, d_input_);

        for (std::size_t i = 0; i < theta_outputs_.size(); ++i) {
            register_output(
                output_prefix + "/theta_" + std::to_string(i + 1), theta_outputs_[i], nan_);
            register_output(
                output_prefix + "/theta_xoy_" + std::to_string(i + 1), theta_xoy_outputs_[i], nan_);
        }
    }

    void update() override {
        if (!inputs_ready_()) {
            publish_nan_outputs_();
            return;
        }

        if (!validate_geometry_()) {
            publish_nan_outputs_();
            return;
        }

        Solution solution;
        if (!solve_(solution)) {
            publish_nan_outputs_();
            return;
        }

        for (std::size_t i = 0; i < theta_outputs_.size(); ++i) {
            *theta_outputs_[i]     = geom::from_radians(solution.theta[i], angles_in_degrees_);
            *theta_xoy_outputs_[i] = geom::from_radians(solution.theta_xoy[i], angles_in_degrees_);
        }
    }

private:
    struct LocalPlane {
        double a;
        double b;
        double c;
        double d;
    };

    struct Solution {
        std::array<double, 4> theta;
        std::array<double, 4> theta_xoy;
        std::array<geom::Vec3, 4> b_points;
    };

    [[nodiscard]] bool inputs_ready_() const {
        const bool plane_ready =
            a_input_.ready() && std::isfinite(*a_input_) && b_input_.ready() && std::isfinite(*b_input_)
            && c_input_.ready() && std::isfinite(*c_input_) && d_input_.ready() && std::isfinite(*d_input_);
        if (!plane_ready) {
            return false;
        }

        if (!use_yaw_topic_) {
            return std::isfinite(static_yaw_);
        }

        return yaw_input_.ready() && std::isfinite(*yaw_input_);
    }

    void publish_nan_outputs_() {
        for (auto& output : theta_outputs_) {
            *output = nan_;
        }
        for (auto& output : theta_xoy_outputs_) {
            *output = nan_;
        }
    }

    [[nodiscard]] bool validate_geometry_() const {
        if (!std::isfinite(l_) || !std::isfinite(L_) || !std::isfinite(h_) || L_ <= 0.0 || h_ < 0.0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "pose_forward_kinematics: invalid yaml parameters.");
            return false;
        }

        if (l_ <= 0.0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "pose_forward_kinematics: l must be positive.");
            return false;
        }
        return true;
    }

    [[nodiscard]] bool solve_(Solution& solution) const {
        const geom::Plane input_plane{
            .normal = {*a_input_, *b_input_, *c_input_},
            .d      = *d_input_,
        };

        geom::Plane plane;
        std::string plane_error;
        if (!geom::normalize_plane_to_world_up(input_plane, plane, plane_error)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "pose_forward_kinematics: %s", plane_error.c_str());
            return false;
        }

        const LocalPlane local_plane = {
            .a = plane.normal.x,
            .b = plane.normal.y,
            .c = plane.normal.z,
            .d = plane.d,
        };
        const double yaw = normalized_yaw_(use_yaw_topic_ ? *yaw_input_ : static_yaw_);
        for (std::size_t i = 0; i < solution.theta.size(); ++i) {
            const auto local_a_point = yaw_rotated_anchor_point_(i, l_, yaw);
            const auto local_radial_direction = yaw_rotated_radial_direction_(i, yaw);
            const double dot_plane_radial =
                local_plane.a * local_radial_direction.x + local_plane.b * local_radial_direction.y;
            const double dot_plane_normal = local_plane.c;
            const double rhs =
                (local_plane.d - local_plane.a * local_a_point.x - local_plane.b * local_a_point.y) / L_;

            const double amplitude = std::hypot(dot_plane_radial, dot_plane_normal);
            if (amplitude <= geom::kTolerance) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "pose_forward_kinematics: leg %zu has a degenerate solve amplitude.", i + 1);
                return false;
            }

            const double normalized_rhs = rhs / amplitude;
            if (normalized_rhs < -1.0 - geom::kValidationTolerance
                || normalized_rhs > 1.0 + geom::kValidationTolerance) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "pose_forward_kinematics: leg %zu target plane is outside the reachable range.", i + 1);
                return false;
            }

            const double clamped_rhs = std::clamp(normalized_rhs, -1.0, 1.0);
            const double phi = std::atan2(dot_plane_normal, dot_plane_radial);
            const double alpha = std::acos(clamped_rhs);

            const std::array<double, 2> candidates = {
                alpha - phi,
                -alpha - phi,
            };

            bool found = false;
            double best_theta = nan_;
            for (double candidate : candidates) {
                const double normalized_candidate = normalize_theta_(candidate);
                if (normalized_candidate < -geom::kValidationTolerance
                    || normalized_candidate > std::numbers::pi / 2 + geom::kValidationTolerance) {
                    continue;
                }

                const double theta = std::clamp(normalized_candidate, 0.0, std::numbers::pi / 2);
                const auto local_b_point = local_a_point
                    + local_radial_direction * (L_ * std::cos(theta))
                    - geom::Vec3{0.0, 0.0, 1.0} * (L_ * std::sin(theta));
                if (std::abs(local_plane_eval_(local_plane, local_b_point))
                    > geom::kValidationTolerance * std::max(1.0, L_)) {
                    continue;
                }

                if (!found || theta < best_theta) {
                    found = true;
                    best_theta = theta;
                    solution.b_points[i] = local_b_point;
                }
            }

            if (!found) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "pose_forward_kinematics: no valid theta_%zu in [0, pi/2].", i + 1);
                return false;
            }

            solution.theta[i] = best_theta;
            const auto world_a_point = local_a_point;
            const auto world_b_point = solution.b_points[i];
            solution.theta_xoy[i] = angle_with_xoy_(world_a_point, world_b_point);
        }

        const double min_distance = std::min({
            L_ * std::sin(solution.theta[0]),
            L_ * std::sin(solution.theta[1]),
            L_ * std::sin(solution.theta[2]),
            L_ * std::sin(solution.theta[3]),
        });
        if (std::abs(min_distance - h_) > geom::kGeometryTolerance * std::max(1.0, L_)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "pose_forward_kinematics: h is inconsistent with the solved theta_i.");
            return false;
        }

        if (!coplanar_with_plane_(local_plane, solution.b_points)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "pose_forward_kinematics: reconstructed B_i are not coplanar with the input plane.");
            return false;
        }

        return true;
    }

    [[nodiscard]] static double normalize_theta_(double theta) {
        while (theta <= -std::numbers::pi) {
            theta += 2.0 * std::numbers::pi;
        }
        while (theta > std::numbers::pi) {
            theta -= 2.0 * std::numbers::pi;
        }
        return theta;
    }

    [[nodiscard]] static double local_plane_eval_(const LocalPlane& plane, const geom::Vec3& point) {
        return plane.a * point.x + plane.b * point.y + plane.c * point.z - plane.d;
    }

    [[nodiscard]] static geom::Vec3 local_anchor_point_(std::size_t index, double l) {
        const double half_length = l / 2.0;
        switch (index) {
        case 0:
            return {+half_length, +half_length, 0.0};
        case 1:
            return {+half_length, -half_length, 0.0};
        case 2:
            return {-half_length, -half_length, 0.0};
        case 3:
            return {-half_length, +half_length, 0.0};
        default:
            return {nan_, nan_, nan_};
        }
    }

    [[nodiscard]] static geom::Vec3 local_radial_direction_(std::size_t index) {
        constexpr double inv_sqrt2 = 0.7071067811865475;
        switch (index) {
        case 0:
            return {+inv_sqrt2, +inv_sqrt2, 0.0};
        case 1:
            return {+inv_sqrt2, -inv_sqrt2, 0.0};
        case 2:
            return {-inv_sqrt2, -inv_sqrt2, 0.0};
        case 3:
            return {-inv_sqrt2, +inv_sqrt2, 0.0};
        default:
            return {nan_, nan_, nan_};
        }
    }

    [[nodiscard]] static geom::Vec3 yaw_rotated_anchor_point_(std::size_t index, double l, double yaw) {
        return geom::rotate_z(local_anchor_point_(index, l), yaw);
    }

    [[nodiscard]] static geom::Vec3 yaw_rotated_radial_direction_(std::size_t index, double yaw) {
        return geom::rotate_z(local_radial_direction_(index), yaw);
    }

    [[nodiscard]] static bool coplanar_with_plane_(
        const LocalPlane& plane, const std::array<geom::Vec3, 4>& points) {
        const double scale = std::max(1.0, std::abs(plane.d));
        return std::all_of(points.begin(), points.end(), [&](const auto& point) {
            return std::abs(local_plane_eval_(plane, point)) <= geom::kValidationTolerance * scale;
        });
    }

    [[nodiscard]] static double angle_with_xoy_(const geom::Vec3& a_point, const geom::Vec3& b_point) {
        const geom::Vec3 segment = b_point - a_point;
        return std::atan2(std::abs(segment.z), std::hypot(segment.x, segment.y));
    }

    [[nodiscard]] double normalized_yaw_(double yaw) const {
        return geom::to_radians(yaw, yaw_in_degrees_);
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    double l_;
    double L_;
    double h_;
    std::string yaw_topic_;
    double static_yaw_;
    bool use_yaw_topic_ = false;
    bool yaw_in_degrees_ = false;
    bool angles_in_degrees_ = false;

    InputInterface<double> yaw_input_;
    InputInterface<double> a_input_;
    InputInterface<double> b_input_;
    InputInterface<double> c_input_;
    InputInterface<double> d_input_;

    std::array<OutputInterface<double>, 4> theta_outputs_;
    std::array<OutputInterface<double>, 4> theta_xoy_outputs_;
};

} // namespace rmcs_core::chassis::suspension

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::chassis::suspension::PoseForwardKinematics, rmcs_executor::Component)
