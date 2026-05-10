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

namespace rmcs_core::chassis::suspension {

class GroundInverseCalculation
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GroundInverseCalculation()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , l_(get_parameter_or("l", 0.0))
        , L_(get_parameter_or("L", 0.0))
        , h_(get_parameter_or("h", 0.0))
        , plane_normal_(
              get_parameter_or("a1", 0.0), get_parameter_or("b1", 0.0),
              get_parameter_or("c1", 1.0))
        , a_points_({
              read_point_("point_1"),
              read_point_("point_2"),
              read_point_("point_3"),
              read_point_("point_4"),
          })
        , angles_in_degrees_(get_parameter_or("angles_in_degrees", false)) {
        register_input(
            get_parameter_or(
                "theta_1_topic", std::string("/chassis/suspension/ground_inverse/theta_1")),
            theta_inputs_[0]);
        register_input(
            get_parameter_or(
                "theta_2_topic", std::string("/chassis/suspension/ground_inverse/theta_2")),
            theta_inputs_[1]);
        register_input(
            get_parameter_or(
                "theta_3_topic", std::string("/chassis/suspension/ground_inverse/theta_3")),
            theta_inputs_[2]);
        register_input(
            get_parameter_or(
                "theta_4_topic", std::string("/chassis/suspension/ground_inverse/theta_4")),
            theta_inputs_[3]);

        const auto output_prefix =
            get_parameter_or("output_prefix", std::string("/chassis/suspension/ground_inverse"));

        register_output(output_prefix + "/plane/a", plane_a_output_, nan_);
        register_output(output_prefix + "/plane/b", plane_b_output_, nan_);
        register_output(output_prefix + "/plane/c", plane_c_output_, nan_);
        register_output(output_prefix + "/plane/d", plane_d_output_, nan_);

        register_output(output_prefix + "/slope/pitch", slope_pitch_output_, nan_);
        register_output(output_prefix + "/slope/roll", slope_roll_output_, nan_);
        register_output(output_prefix + "/slope/tilt", slope_tilt_output_, nan_);
    }

    void update() override {
        if (!inputs_ready_()) {
            publish_nan_outputs_();
            return;
        }

        Plane plane;
        if (!solve_(plane)) {
            publish_nan_outputs_();
            return;
        }

        *plane_a_output_ = plane.a;
        *plane_b_output_ = plane.b;
        *plane_c_output_ = plane.c;
        *plane_d_output_ = plane.d;

        *slope_pitch_output_ = std::atan2(-plane.a, plane.c);
        *slope_roll_output_  = std::atan2(-plane.b, plane.c);
        *slope_tilt_output_  = std::atan2(std::hypot(plane.a, plane.b), plane.c);
    }

private:
    struct Vec3 {
        double x;
        double y;
        double z;

        Vec3 operator+(const Vec3& other) const {
            return {x + other.x, y + other.y, z + other.z};
        }

        Vec3 operator-(const Vec3& other) const {
            return {x - other.x, y - other.y, z - other.z};
        }

        Vec3 operator*(double scalar) const {
            return {x * scalar, y * scalar, z * scalar};
        }
    };

    struct Plane {
        double a;
        double b;
        double c;
        double d;
    };

    [[nodiscard]] static double dot_(const Vec3& lhs, const Vec3& rhs) {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    [[nodiscard]] static Vec3 cross_(const Vec3& lhs, const Vec3& rhs) {
        return {
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x,
        };
    }

    [[nodiscard]] static double norm_(const Vec3& vector) {
        return std::sqrt(dot_(vector, vector));
    }

    [[nodiscard]] static bool finite_(const Vec3& vector) {
        return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
    }

    [[nodiscard]] Vec3 read_point_(const std::string& prefix) const {
        return {
            get_parameter_or(prefix + "_x", 0.0),
            get_parameter_or(prefix + "_y", 0.0),
            get_parameter_or(prefix + "_z", 0.0),
        };
    }

    [[nodiscard]] bool inputs_ready_() const {
        return std::all_of(theta_inputs_.begin(), theta_inputs_.end(), [](const auto& input) {
            return input.ready() && std::isfinite(*input);
        });
    }

    void publish_nan_outputs_() {
        *plane_a_output_    = nan_;
        *plane_b_output_    = nan_;
        *plane_c_output_    = nan_;
        *plane_d_output_    = nan_;
        *slope_pitch_output_ = nan_;
        *slope_roll_output_  = nan_;
        *slope_tilt_output_  = nan_;
    }

    [[nodiscard]] bool solve_(Plane& plane) const {
        if (!validate_geometry_()) {
            return false;
        }

        const double normal_norm = norm_(plane_normal_);
        const Vec3 n             = plane_normal_ * (1.0 / normal_norm);

        std::array<Vec3, 4> b_points;
        std::array<double, 4> normal_distances{};

        for (std::size_t i = 0; i < a_points_.size(); ++i) {
            const double theta = normalized_theta_(theta_inputs_[i]);
            if (!std::isfinite(theta) || theta < -tolerance_ || theta > std::numbers::pi / 2 + tolerance_) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "ground_inverse_calculation: theta_%zu is outside [0, pi/2].", i + 1);
                return false;
            }

            const double point_radius = norm_(a_points_[i]);
            if (point_radius <= tolerance_) {
                return false;
            }

            const Vec3 radial_direction = a_points_[i] * (1.0 / point_radius);
            b_points[i] = a_points_[i] + radial_direction * (L_ * std::cos(theta))
                - n * (L_ * std::sin(theta));
            normal_distances[i] = L_ * std::sin(theta);
        }

        const double min_distance =
            *std::min_element(normal_distances.begin(), normal_distances.end());
        if (std::abs(min_distance - h_) > consistency_tolerance_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "ground_inverse_calculation: h is inconsistent with L * sin(theta_i).");
            return false;
        }

        const Vec3 edge_12 = b_points[1] - b_points[0];
        const Vec3 edge_13 = b_points[2] - b_points[0];
        const Vec3 edge_14 = b_points[3] - b_points[0];

        Vec3 normal = cross_(edge_12, edge_13);
        const double normal_length = norm_(normal);
        if (normal_length <= tolerance_) {
            normal = cross_(edge_12, edge_14);
        }

        if (norm_(normal) <= tolerance_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "ground_inverse_calculation: B_i plane is degenerate.");
            return false;
        }

        const double coplanarity_error = dot_(normal, edge_14);
        const double scale =
            std::max({1.0, norm_(edge_12), norm_(edge_13), norm_(edge_14)});
        if (std::abs(coplanarity_error) > coplanarity_tolerance_ * scale * scale * scale) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "ground_inverse_calculation: computed B_i are not coplanar.");
            return false;
        }

        const double inv_normal_length = 1.0 / norm_(normal);
        normal                         = normal * inv_normal_length;
        if (normal.z < 0.0) {
            normal = normal * -1.0;
        }

        plane.a = normal.x;
        plane.b = normal.y;
        plane.c = normal.z;
        plane.d = dot_(normal, b_points[0]);
        return true;
    }

    [[nodiscard]] bool validate_geometry_() const {
        if (!finite_(plane_normal_) || !std::isfinite(l_) || !std::isfinite(L_) || !std::isfinite(h_)
            || l_ <= 0.0 || L_ <= 0.0 || h_ < 0.0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "ground_inverse_calculation: invalid yaml parameters.");
            return false;
        }

        const double normal_norm = norm_(plane_normal_);
        if (normal_norm <= tolerance_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "ground_inverse_calculation: plane normal must be non-zero.");
            return false;
        }

        const double target_radius = l_ / std::sqrt(2.0);
        for (const auto& point : a_points_) {
            if (!finite_(point)) {
                return false;
            }

            const double plane_residual = dot_(plane_normal_, point);
            if (std::abs(plane_residual) > geometry_tolerance_ * std::max(1.0, normal_norm)) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "ground_inverse_calculation: A_i is not on the configured square plane.");
                return false;
            }

            const double radius = norm_(point);
            if (std::abs(radius - target_radius) > geometry_tolerance_) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "ground_inverse_calculation: A_i radius is inconsistent with l.");
                return false;
            }
        }

        return true;
    }

    [[nodiscard]] double normalized_theta_(const InputInterface<double>& input) const {
        double theta = std::abs(*input);
        if (angles_in_degrees_) {
            theta *= std::numbers::pi / 180.0;
        }
        return theta;
    }

    static constexpr double nan_                   = std::numeric_limits<double>::quiet_NaN();
    static constexpr double tolerance_             = 1e-9;
    static constexpr double geometry_tolerance_    = 1e-5;
    static constexpr double consistency_tolerance_ = 1e-5;
    static constexpr double coplanarity_tolerance_ = 1e-6;

    double l_;
    double L_;
    double h_;
    Vec3 plane_normal_;
    std::array<Vec3, 4> a_points_;
    bool angles_in_degrees_ = false;

    std::array<InputInterface<double>, 4> theta_inputs_;

    OutputInterface<double> plane_a_output_;
    OutputInterface<double> plane_b_output_;
    OutputInterface<double> plane_c_output_;
    OutputInterface<double> plane_d_output_;
    OutputInterface<double> slope_pitch_output_;
    OutputInterface<double> slope_roll_output_;
    OutputInterface<double> slope_tilt_output_;
};

} // namespace rmcs_core::chassis::suspension

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::chassis::suspension::GroundInverseCalculation, rmcs_executor::Component)
