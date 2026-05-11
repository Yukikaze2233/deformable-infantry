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
        , yaw_topic_(get_parameter_or("yaw_topic", std::string("/chassis/imu/yaw")))
        , pitch_topic_(get_parameter_or("pitch_topic", std::string("/chassis/imu/pitch")))
        , roll_topic_(get_parameter_or("roll_topic", std::string("/chassis/imu/roll")))
        , static_yaw_(get_parameter_or("static_yaw", 0.0))
        , static_pitch_(get_parameter_or("static_pitch", 0.0))
        , static_roll_(get_parameter_or("static_roll", 0.0))
        , use_attitude_topics_(get_parameter_or("use_attitude_topics", false))
        , attitude_in_degrees_(get_parameter_or("attitude_in_degrees", false))
        , angles_in_degrees_(get_parameter_or("angles_in_degrees", false)) {
        register_input(yaw_topic_, yaw_input_);
        register_input(pitch_topic_, pitch_input_);
        register_input(roll_topic_, roll_input_);
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

        geom::LocalGeometry geometry;
        if (!validate_geometry_(geometry)) {
            publish_nan_outputs_();
            return;
        }

        geom::Plane plane;
        if (!solve_(geometry, plane)) {
            publish_nan_outputs_();
            return;
        }

        *plane_a_output_ = plane.normal.x;
        *plane_b_output_ = plane.normal.y;
        *plane_c_output_ = plane.normal.z;
        *plane_d_output_ = plane.d;

        geom::Plane world_observation_plane;
        std::string world_plane_error;
        if (!geom::normalize_plane_to_world_up(plane, world_observation_plane, world_plane_error)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "ground_inverse_calculation: %s", world_plane_error.c_str());
            publish_nan_outputs_();
            return;
        }

        // These are world-frame observations against the world xoy plane.
        *slope_pitch_output_ =
            std::atan2(-world_observation_plane.normal.x, world_observation_plane.normal.z);
        *slope_roll_output_ =
            std::atan2(-world_observation_plane.normal.y, world_observation_plane.normal.z);
        *slope_tilt_output_ = std::atan2(
            std::hypot(world_observation_plane.normal.x, world_observation_plane.normal.y),
            world_observation_plane.normal.z);

        RCLCPP_INFO(get_logger(), "坡度：%f", std::sqrt(plane.normal.x * plane.normal.x + plane.normal.y * plane.normal.y) / plane.normal.z);
    }

private:
    [[nodiscard]] bool inputs_ready_() const {
        const bool theta_ready =
            std::all_of(theta_inputs_.begin(), theta_inputs_.end(), [](const auto& input) {
                return input.ready() && std::isfinite(*input);
            });
        if (!theta_ready) {
            return false;
        }

        if (!use_attitude_topics_) {
            return std::isfinite(static_yaw_) && std::isfinite(static_pitch_) && std::isfinite(static_roll_);
        }

        return yaw_input_.ready() && std::isfinite(*yaw_input_) && pitch_input_.ready()
            && std::isfinite(*pitch_input_) && roll_input_.ready() && std::isfinite(*roll_input_);
    }

    void publish_nan_outputs_() {
        *plane_a_output_     = nan_;
        *plane_b_output_     = nan_;
        *plane_c_output_     = nan_;
        *plane_d_output_     = nan_;
        *slope_pitch_output_ = nan_;
        *slope_roll_output_  = nan_;
        *slope_tilt_output_  = nan_;
    }

    [[nodiscard]] bool validate_geometry_(geom::LocalGeometry& geometry) const {
        if (!std::isfinite(l_) || !std::isfinite(L_) || !std::isfinite(h_) || L_ <= 0.0 || h_ < 0.0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "ground_inverse_calculation: invalid yaml parameters.");
            return false;
        }

        const double yaw = normalized_attitude_(use_attitude_topics_ ? *yaw_input_ : static_yaw_);
        const double pitch = normalized_attitude_(use_attitude_topics_ ? *pitch_input_ : static_pitch_);
        const double roll = normalized_attitude_(use_attitude_topics_ ? *roll_input_ : static_roll_);

        std::string error;
        if (!geom::build_local_geometry_from_attitude(yaw, pitch, roll, l_, geometry, error)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "ground_inverse_calculation: %s", error.c_str());
            return false;
        }

        return true;
    }

    [[nodiscard]] bool solve_(const geom::LocalGeometry& geometry, geom::Plane& plane) const {
        std::array<geom::Vec3, 4> b_points;
        std::array<double, 4> normal_distances{};

        for (std::size_t i = 0; i < b_points.size(); ++i) {
            const double theta = normalized_theta_(theta_inputs_[i]);
            if (!std::isfinite(theta) || theta < -geom::kTolerance || theta > std::numbers::pi / 2 + geom::kTolerance) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "ground_inverse_calculation: theta_%zu is outside [0, pi/2].", i + 1);
                return false;
            }

            b_points[i] = geometry.points[i]
                + geometry.radial_directions[i] * (L_ * std::cos(theta))
                - geometry.n * (L_ * std::sin(theta));
            normal_distances[i] = L_ * std::sin(theta);
        }

        const double min_distance =
            *std::min_element(normal_distances.begin(), normal_distances.end());
        if (std::abs(min_distance - h_) > geom::kGeometryTolerance * std::max(1.0, L_)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "ground_inverse_calculation: h is inconsistent with L * sin(theta_i).");
            return false;
        }

        const geom::Vec3 edge_12 = b_points[1] - b_points[0];
        const geom::Vec3 edge_13 = b_points[2] - b_points[0];
        const geom::Vec3 edge_14 = b_points[3] - b_points[0];

        geom::Vec3 raw_normal = geom::cross(edge_12, edge_13);
        if (geom::norm(raw_normal) <= geom::kTolerance) {
            raw_normal = geom::cross(edge_12, edge_14);
        }

        if (geom::norm(raw_normal) <= geom::kTolerance) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "ground_inverse_calculation: B_i plane is degenerate.");
            return false;
        }

        const double coplanarity_error = geom::dot(raw_normal, edge_14);
        const double scale =
            std::max({1.0, geom::norm(edge_12), geom::norm(edge_13), geom::norm(edge_14)});
        if (std::abs(coplanarity_error) > geom::kValidationTolerance * scale * scale * scale) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "ground_inverse_calculation: computed B_i are not coplanar.");
            return false;
        }

        geom::Plane candidate_plane{
            .normal = raw_normal,
            .d      = geom::dot(raw_normal, b_points[0]),
        };

        std::string error;
        if (!geom::normalize_plane(candidate_plane, geometry.n, plane, error)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "ground_inverse_calculation: %s", error.c_str());
            return false;
        }

        return true;
    }

    [[nodiscard]] double normalized_theta_(const InputInterface<double>& input) const {
        return geom::to_radians(*input, angles_in_degrees_);
    }

    [[nodiscard]] double normalized_attitude_(double angle) const {
        return geom::to_radians(angle, attitude_in_degrees_);
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    double l_;
    double L_;
    double h_;
    std::string yaw_topic_;
    std::string pitch_topic_;
    std::string roll_topic_;
    double static_yaw_;
    double static_pitch_;
    double static_roll_;
    bool use_attitude_topics_ = false;
    bool attitude_in_degrees_ = false;
    bool angles_in_degrees_ = false;

    InputInterface<double> yaw_input_;
    InputInterface<double> pitch_input_;
    InputInterface<double> roll_input_;
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
