#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::active_suspension {

class AdaptiveOmniContactEstimator
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    enum JointIndex : size_t {
        kLeftFront = 0,
        kLeftBack = 1,
        kRightBack = 2,
        kRightFront = 3,
        kJointCount = 4,
    };

    AdaptiveOmniContactEstimator()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , wheel_radius_(get_parameter_or("wheel_radius", 0.055))
        , radius_base_(get_parameter_or("radius_base", 0.2341741))
        , pivot_offset_(get_parameter_or("pivot_offset", 0.17389))
        , body_length_(get_parameter_or(
              "body_length", std::numbers::sqrt2 * std::max(radius_base_ - pivot_offset_, 0.03)))
        , body_width_(get_parameter_or(
              "body_width", std::numbers::sqrt2 * std::max(radius_base_ - pivot_offset_, 0.03)))
        , arm_length_(get_parameter_or("arm_length", 0.155))
        , confidence_alpha_(get_parameter_or("confidence_alpha", 0.15))
        , support_force_alpha_(get_parameter_or("support_force_alpha", 0.20))
        , support_reference_rise_alpha_(std::clamp(
              get_parameter_or("support_reference_rise_alpha", 0.05), 0.0, 1.0))
        , support_reference_fall_alpha_(std::clamp(
              get_parameter_or("support_reference_fall_alpha", 0.002), 0.0, 1.0))
        , moving_speed_threshold_(get_parameter_or("moving_speed_threshold", 1.0))
        , slip_ratio_gain_(get_parameter_or("slip_ratio_gain", 1.5))
        , wheel_torque_reference_(get_parameter_or("wheel_torque_reference", 0.8))
        , joint_torque_reference_(get_parameter_or("joint_torque_reference", 5.0))
        , local_support_low_ratio_(std::clamp(
              get_parameter_or(
                  "local_support_low_ratio", get_parameter_or("min_contact_force_fraction", 0.45)),
              0.0, 1.0))
        , local_support_high_ratio_(std::clamp(
              get_parameter_or(
                  "local_support_high_ratio", get_parameter_or("full_contact_force_fraction", 0.80)),
              0.0, 2.0))
        , global_support_low_ratio_(std::clamp(
              get_parameter_or(
                  "global_support_low_ratio", get_parameter_or("total_support_low_ratio", 0.60)),
              0.0, 1.0))
        , global_support_high_ratio_(std::clamp(
              get_parameter_or(
                  "global_support_high_ratio", get_parameter_or("total_support_high_ratio", 0.90)),
              0.0, 2.0))
        , shock_support_jump_ratio_(std::max(get_parameter_or("shock_support_jump_ratio", 0.35), 0.0))
        , shock_angle_rate_threshold_(deg_to_rad(get_parameter_or("shock_angle_rate_threshold_deg", 40.0)))
        , shock_joint_rate_threshold_(deg_to_rad(get_parameter_or("shock_joint_rate_threshold_deg", 220.0)))
        , shock_residual_threshold_(std::max(get_parameter_or("shock_residual_threshold", 0.35), 0.0))
        , shock_hold_time_(std::max(get_parameter_or("shock_hold_time", 0.15), 0.0))
        , quasi_static_speed_threshold_(std::max(get_parameter_or("quasi_static_speed_threshold", 0.25), 0.0))
        , quasi_static_angle_rate_threshold_(
              deg_to_rad(get_parameter_or("quasi_static_angle_rate_threshold_deg", 4.0)))
        , quasi_static_joint_rate_threshold_(
              deg_to_rad(get_parameter_or("quasi_static_joint_rate_threshold_deg", 35.0)))
        , quasi_static_residual_threshold_(
              std::max(get_parameter_or("quasi_static_residual_threshold", 0.20), 0.0))
        , quasi_static_hold_time_(std::max(get_parameter_or("quasi_static_hold_time", 0.30), 0.0))
        , reference_update_load_share_threshold_(std::clamp(
              get_parameter_or("reference_update_load_share_threshold", 0.08), 0.0, 1.0))
        , load_share_floor_(get_parameter_or("load_share_floor", 0.05))
        , confidence_floor_(get_parameter_or("confidence_floor", 0.15)) {

        register_input("/chassis/control_velocity", chassis_control_velocity_);
        register_input("/chassis/radius", chassis_radius_);

        register_input("/chassis/left_front_wheel/velocity", left_front_wheel_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_wheel_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_wheel_velocity_);
        register_input("/chassis/right_front_wheel/velocity", right_front_wheel_velocity_);

        register_input("/chassis/left_front_wheel/torque", left_front_wheel_torque_);
        register_input("/chassis/left_back_wheel/torque", left_back_wheel_torque_);
        register_input("/chassis/right_back_wheel/torque", right_back_wheel_torque_);
        register_input("/chassis/right_front_wheel/torque", right_front_wheel_torque_);

        register_input("/chassis/left_front_joint/torque", left_front_joint_torque_);
        register_input("/chassis/left_back_joint/torque", left_back_joint_torque_);
        register_input("/chassis/right_back_joint/torque", right_back_joint_torque_);
        register_input("/chassis/right_front_joint/torque", right_front_joint_torque_);

        register_input(
            "/chassis/left_front_joint/physical_angle", left_front_joint_physical_angle_, false);
        register_input(
            "/chassis/left_back_joint/physical_angle", left_back_joint_physical_angle_, false);
        register_input(
            "/chassis/right_back_joint/physical_angle", right_back_joint_physical_angle_, false);
        register_input(
            "/chassis/right_front_joint/physical_angle", right_front_joint_physical_angle_, false);

        register_input(
            "/chassis/left_front_joint/support_observer_z3", left_front_joint_support_observer_z3_,
            false);
        register_input(
            "/chassis/left_back_joint/support_observer_z3", left_back_joint_support_observer_z3_,
            false);
        register_input(
            "/chassis/right_back_joint/support_observer_z3", right_back_joint_support_observer_z3_,
            false);
        register_input(
            "/chassis/right_front_joint/support_observer_z3",
            right_front_joint_support_observer_z3_, false);

        register_input("/chassis/imu/pitch", chassis_imu_pitch_, false);
        register_input("/chassis/imu/roll", chassis_imu_roll_, false);

        register_output(
            "/chassis/left_front_contact/confidence", left_front_contact_confidence_, 1.0);
        register_output("/chassis/left_back_contact/confidence", left_back_contact_confidence_, 1.0);
        register_output(
            "/chassis/right_back_contact/confidence", right_back_contact_confidence_, 1.0);
        register_output(
            "/chassis/right_front_contact/confidence", right_front_contact_confidence_, 1.0);

        register_output("/chassis/left_front_contact/residual", left_front_contact_residual_, 0.0);
        register_output("/chassis/left_back_contact/residual", left_back_contact_residual_, 0.0);
        register_output("/chassis/right_back_contact/residual", right_back_contact_residual_, 0.0);
        register_output(
            "/chassis/right_front_contact/residual", right_front_contact_residual_, 0.0);

        register_output(
            "/chassis/left_front_contact/load_share", left_front_contact_load_share_, 0.25);
        register_output(
            "/chassis/left_back_contact/load_share", left_back_contact_load_share_, 0.25);
        register_output(
            "/chassis/right_back_contact/load_share", right_back_contact_load_share_, 0.25);
        register_output(
            "/chassis/right_front_contact/load_share", right_front_contact_load_share_, 0.25);

        register_output(
            "/chassis/left_front_contact/normal_force_estimate",
            left_front_contact_normal_force_estimate_, nan_);
        register_output(
            "/chassis/left_back_contact/normal_force_estimate",
            left_back_contact_normal_force_estimate_, nan_);
        register_output(
            "/chassis/right_back_contact/normal_force_estimate",
            right_back_contact_normal_force_estimate_, nan_);
        register_output(
            "/chassis/right_front_contact/normal_force_estimate",
            right_front_contact_normal_force_estimate_, nan_);
        register_output("/chassis/contact/normal_force_total", contact_normal_force_total_, nan_);

        register_output("/chassis/contact/confidence_mean", contact_confidence_mean_, 1.0);
    }

    void before_updating() override {
        if (!chassis_imu_pitch_.ready())
            chassis_imu_pitch_.make_and_bind_directly(0.0);
        if (!chassis_imu_roll_.ready())
            chassis_imu_roll_.make_and_bind_directly(0.0);
    }

    void update() override {
        sanitize_state_();

        const auto wheel_velocities = read_inputs_(
            left_front_wheel_velocity_, left_back_wheel_velocity_, right_back_wheel_velocity_,
            right_front_wheel_velocity_);
        const auto wheel_torques = read_inputs_(
            left_front_wheel_torque_, left_back_wheel_torque_, right_back_wheel_torque_,
            right_front_wheel_torque_);
        const auto joint_torques = read_inputs_(
            left_front_joint_torque_, left_back_joint_torque_, right_back_joint_torque_,
            right_front_joint_torque_);

        const Eigen::Vector3d raw_control_velocity = chassis_control_velocity_->vector;
        const bool control_velocity_valid = raw_control_velocity.allFinite();
        const double command_norm = control_velocity_valid ? raw_control_velocity.norm() : 0.0;
        const double chassis_radius =
            (chassis_radius_.ready() && std::isfinite(*chassis_radius_) && *chassis_radius_ > 1e-6)
                ? *chassis_radius_
                : radius_base_;

        std::array<double, 4> expected_wheel_velocities{};
        bool kinematics_valid = control_velocity_valid && all_finite_(wheel_velocities);
        if (kinematics_valid) {
            expected_wheel_velocities =
                expected_wheel_velocity_(raw_control_velocity, std::max(chassis_radius, 1e-6));
            kinematics_valid = all_finite_(expected_wheel_velocities);
        }

        const auto physical_angles = read_optional_inputs_(
            left_front_joint_physical_angle_, left_back_joint_physical_angle_,
            right_back_joint_physical_angle_, right_front_joint_physical_angle_);
        const auto support_observer_z3 = read_optional_inputs_(
            left_front_joint_support_observer_z3_, left_back_joint_support_observer_z3_,
            right_back_joint_support_observer_z3_, right_front_joint_support_observer_z3_);
        const double pitch =
            (chassis_imu_pitch_.ready() && std::isfinite(*chassis_imu_pitch_)) ? *chassis_imu_pitch_
                                                                                : 0.0;
        const double roll =
            (chassis_imu_roll_.ready() && std::isfinite(*chassis_imu_roll_)) ? *chassis_imu_roll_
                                                                              : 0.0;

        std::array<double, 4> residual = {0.0, 0.0, 0.0, 0.0};
        std::array<double, 4> slip_score = {1.0, 1.0, 1.0, 1.0};
        std::array<double, 4> support_proxy = {nan_, nan_, nan_, nan_};
        std::array<double, 4> support_ratio = {nan_, nan_, nan_, nan_};
        std::array<double, 4> support_score = {1.0, 1.0, 1.0, 1.0};
        std::array<double, 4> legacy_support_score = {nan_, nan_, nan_, nan_};
        std::array<double, 4> support_estimate = {nan_, nan_, nan_, nan_};
        std::array<double, 4> predicted_support_reference = {nan_, nan_, nan_, nan_};
        std::array<bool, 4> observer_available = {false, false, false, false};
        std::array<ReferenceFeatureVector, 4> reference_features = zero_reference_features_array_();

        for (size_t i = 0; i < kJointCount; ++i) {
            if (kinematics_valid) {
                const double expected = expected_wheel_velocities[i];
                const double measured = wheel_velocities[i];
                const double denom = std::max(std::abs(expected), 1.0);
                residual[i] = std::abs(measured - expected) / denom;
                slip_score[i] = 1.0 / (1.0 + slip_ratio_gain_ * residual[i]);
            }

            legacy_support_score[i] =
                legacy_support_score_(wheel_torques[i], joint_torques[i]);
            if (std::isfinite(physical_angles[i])) {
                reference_features[i] = reference_features_(physical_angles[i], pitch, roll);
                if (support_reference_initialized_[i]) {
                    predicted_support_reference[i] =
                        support_reference_from_model_(i, reference_features[i]);
                }
            }

            const double observer_proxy =
                support_proxy_from_observer_(
                    i, support_observer_z3[i], physical_angles[i], pitch, roll);
            if (!std::isfinite(observer_proxy)) {
                support_estimate[i] = legacy_support_proxy_(
                    legacy_support_score[i], predicted_support_reference[i],
                    filtered_support_proxy_[i], last_support_proxy_[i]);
                support_score[i] = std::isfinite(legacy_support_score[i]) ? legacy_support_score[i]
                                                                          : last_confidence_[i];
                continue;
            }

            observer_available[i] = true;
            if (!std::isfinite(filtered_support_proxy_[i])) {
                filtered_support_proxy_[i] = observer_proxy;
            } else {
                filtered_support_proxy_[i] =
                    (1.0 - support_force_alpha_) * filtered_support_proxy_[i]
                    + support_force_alpha_ * observer_proxy;
            }
            support_proxy[i] = std::max(filtered_support_proxy_[i], reference_floor_());
            support_estimate[i] = support_proxy[i];

            reference_features[i] = reference_features_(physical_angles[i], pitch, roll);
            if (!support_reference_initialized_[i]) {
                initialize_support_reference_model_(i, support_proxy[i]);
            }

            predicted_support_reference[i] =
                support_reference_from_model_(i, reference_features[i]);
            support_ratio[i] = relative_support_ratio_(
                support_proxy[i], predicted_support_reference[i]);

            const double observer_score = support_presence_score_(
                support_ratio[i], local_support_low_ratio_, local_support_high_ratio_);
            support_score[i] = observer_score;
        }

        if (!all_finite_(support_score)) {
            publish_neutral_confidence();
            update_motion_snapshot_(pitch, roll, physical_angles, support_proxy, observer_available);
            return;
        }

        const auto motion_metrics = compute_motion_metrics_(
            support_proxy, predicted_support_reference, physical_angles, pitch, roll, residual,
            observer_available, kinematics_valid);
        update_shock_state_(detect_shock_(motion_metrics));
        const bool shock_active = shock_active_();

        const bool quasi_static_candidate = is_quasi_static_candidate_(command_norm, motion_metrics);
        update_quasi_static_window_(!shock_active && quasi_static_candidate);
        const bool reference_learning_enabled = quasi_static_ready_();

        const auto load_share = normalize_load_share_(support_estimate);
        const double global_support_score = compute_global_support_score_(
            support_proxy, predicted_support_reference, legacy_support_score, observer_available);

        std::array<double, 4> confidence = last_confidence_;
        for (size_t i = 0; i < kJointCount; ++i) {
            const double support_score_blended = std::min(support_score[i], global_support_score);
            const double share_score = std::clamp(
                load_share[i] / std::max(0.25, load_share_floor_), 0.0, 1.0);

            double raw_confidence = 0.85 * support_score_blended + 0.15 * share_score;
            if (kinematics_valid && command_norm > moving_speed_threshold_) {
                raw_confidence = 0.65 * support_score_blended + 0.35 * slip_score[i];
            }

            const double blended = std::clamp(raw_confidence, confidence_floor_, 1.0);
            const double filtered =
                (1.0 - confidence_alpha_) * last_confidence_[i] + confidence_alpha_ * blended;
            if (!std::isfinite(filtered)) {
                confidence[i] = last_confidence_[i];
                continue;
            }
            confidence[i] = filtered;
            last_confidence_[i] = filtered;
        }

        update_support_reference_models_(
            support_proxy, reference_features, load_share, observer_available,
            reference_learning_enabled);
        update_motion_snapshot_(pitch, roll, physical_angles, support_proxy, observer_available);
        publish_contact_estimate(
            confidence, residual, load_share, support_estimate, sum_finite_array_(support_estimate));
    }

private:
    static constexpr double dt_ = 1e-3;
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr int kReferenceFeatureCount_ = 5;

    using ReferenceFeatureVector = Eigen::Matrix<double, kReferenceFeatureCount_, 1>;

    struct MotionMetrics {
        double pitch_rate = 0.0;
        double roll_rate = 0.0;
        double max_joint_rate = 0.0;
        double max_support_jump_ratio = 0.0;
        double mean_residual = 0.0;
        bool residual_valid = false;
    };

    static double deg_to_rad(double deg) { return deg * std::numbers::pi / 180.0; }

    static std::uint64_t duration_to_ticks_(double seconds) {
        return seconds <= 0.0 ? 0 : static_cast<std::uint64_t>(std::llround(seconds / dt_));
    }

    static bool all_finite_(const std::array<double, 4>& values) {
        return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
    }

    template <typename InterfaceT>
    static std::array<double, 4> read_inputs_(
        const InterfaceT& left_front, const InterfaceT& left_back, const InterfaceT& right_back,
        const InterfaceT& right_front) {
        return {*left_front, *left_back, *right_back, *right_front};
    }

    template <typename InterfaceT>
    static std::array<double, 4> read_optional_inputs_(
        const InterfaceT& left_front, const InterfaceT& left_back, const InterfaceT& right_back,
        const InterfaceT& right_front) {
        return {
            (left_front.ready() && std::isfinite(*left_front)) ? *left_front : nan_,
            (left_back.ready() && std::isfinite(*left_back)) ? *left_back : nan_,
            (right_back.ready() && std::isfinite(*right_back)) ? *right_back : nan_,
            (right_front.ready() && std::isfinite(*right_front)) ? *right_front : nan_,
        };
    }

    static double sum_array_(const std::array<double, 4>& values) {
        double sum = 0.0;
        for (double value : values)
            sum += value;
        return sum;
    }

    static double mean_array_(const std::array<double, 4>& values) {
        return sum_array_(values) / static_cast<double>(kJointCount);
    }

    static double sum_finite_array_(const std::array<double, 4>& values) {
        double sum = 0.0;
        bool has_finite = false;
        for (double value : values) {
            if (!std::isfinite(value))
                continue;
            sum += value;
            has_finite = true;
        }
        return has_finite ? sum : nan_;
    }

    static double inverse_lerp_clamped_(double value, double low, double high) {
        if (!std::isfinite(value) || !std::isfinite(low) || !std::isfinite(high))
            return 0.0;
        if (high <= low)
            return value >= high ? 1.0 : 0.0;
        return std::clamp((value - low) / (high - low), 0.0, 1.0);
    }

    static std::array<ReferenceFeatureVector, 4> zero_reference_features_array_() {
        std::array<ReferenceFeatureVector, 4> features;
        for (auto& feature : features)
            feature.setZero();
        return features;
    }

    static std::array<ReferenceFeatureVector, 4> zero_reference_weights_array_() {
        return zero_reference_features_array_();
    }

    static constexpr double reference_floor_() { return 1e-3; }

    void sanitize_state_() {
        for (double& confidence : last_confidence_) {
            if (!std::isfinite(confidence))
                confidence = 1.0;
        }
        for (size_t i = 0; i < kJointCount; ++i) {
            if (!support_reference_weights_[i].allFinite()) {
                support_reference_weights_[i].setZero();
                support_reference_initialized_[i] = false;
            }
            if (!std::isfinite(filtered_support_proxy_[i]))
                filtered_support_proxy_[i] = nan_;
        }
    }

    std::array<double, 4> expected_wheel_velocity_(
        const Eigen::Vector3d& control_velocity, double chassis_radius) const {
        const double x = control_velocity.x();
        const double y = control_velocity.y();
        const double z = control_velocity.z();
        const double a_plus_b = std::numbers::sqrt2 * chassis_radius;

        std::array<double, 4> wheel_control_velocity{
            -x + y + a_plus_b * z,
            -x - y + a_plus_b * z,
            +x - y + a_plus_b * z,
            +x + y + a_plus_b * z,
        };
        for (double& velocity : wheel_control_velocity)
            velocity *= -1.0 / (std::numbers::sqrt2 * wheel_radius_);
        return wheel_control_velocity;
    }

    Eigen::Matrix3d rotation_world_body_(double pitch, double roll) const {
        const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
        return (ry * rx).toRotationMatrix();
    }

    Eigen::Vector3d body_corner_(size_t index) const {
        const double hx = 0.5 * body_length_;
        const double hy = 0.5 * body_width_;
        switch (index) {
        case kLeftFront: return {+hx, +hy, 0.0};
        case kLeftBack: return {-hx, +hy, 0.0};
        case kRightBack: return {-hx, -hy, 0.0};
        case kRightFront: return {+hx, -hy, 0.0};
        default: return {0.0, 0.0, 0.0};
        }
    }

    Eigen::Vector3d radial_direction_(size_t index) const {
        const Eigen::Vector3d corner = body_corner_(index);
        Eigen::Vector3d radial{corner.x(), corner.y(), 0.0};
        const double norm = radial.norm();
        if (!std::isfinite(norm) || norm < 1e-9)
            return {1.0, 0.0, 0.0};
        return radial / norm;
    }

    Eigen::Vector3d wheel_bottom_derivative_body_(size_t index, double physical_angle_rad) const {
        const Eigen::Vector3d radial = radial_direction_(index);
        const Eigen::Vector3d arm_horizontal = arm_length_ * radial;
        const Eigen::Vector3d arm_vertical{0.0, 0.0, -arm_length_};
        return -std::sin(physical_angle_rad) * arm_horizontal
             + std::cos(physical_angle_rad) * arm_vertical;
    }

    double clearance_jacobian_abs_(
        size_t index, double physical_angle_rad, double pitch, double roll) const {
        const Eigen::Vector3d derivative_body =
            wheel_bottom_derivative_body_(index, physical_angle_rad);
        const double jacobian = -(rotation_world_body_(pitch, roll) * derivative_body).z();
        return std::abs(jacobian);
    }

    double support_proxy_from_observer_(
        size_t index, double support_observer_z3, double physical_angle_rad, double pitch,
        double roll) const {
        if (!std::isfinite(support_observer_z3) || !std::isfinite(physical_angle_rad))
            return nan_;

        const double jacobian = clearance_jacobian_abs_(index, physical_angle_rad, pitch, roll);
        if (!std::isfinite(jacobian) || jacobian < 1e-6)
            return nan_;

        return std::abs(support_observer_z3) / jacobian;
    }

    double legacy_support_score_(double wheel_torque, double joint_torque) const {
        if (!std::isfinite(wheel_torque) || !std::isfinite(joint_torque))
            return nan_;

        const double wheel_load_score = std::clamp(
            std::abs(wheel_torque) / std::max(wheel_torque_reference_, 1e-6), 0.0, 1.0);
        const double joint_support_score = std::clamp(
            std::abs(joint_torque) / std::max(joint_torque_reference_, 1e-6), 0.0, 1.0);
        return std::clamp(0.65 * joint_support_score + 0.35 * wheel_load_score, 0.0, 1.0);
    }

    double legacy_support_proxy_(
        double legacy_support_score, double predicted_support_reference,
        double filtered_support_proxy, double last_support_proxy) const {
        if (!std::isfinite(legacy_support_score))
            return nan_;

        double proxy_scale = nan_;
        if (std::isfinite(predicted_support_reference))
            proxy_scale = predicted_support_reference;
        else if (std::isfinite(filtered_support_proxy))
            proxy_scale = filtered_support_proxy;
        else if (std::isfinite(last_support_proxy))
            proxy_scale = last_support_proxy;
        else
            proxy_scale = 1.0;

        return legacy_support_score * std::max(proxy_scale, reference_floor_());
    }

    ReferenceFeatureVector reference_features_(
        double physical_angle_rad, double pitch, double roll) const {
        ReferenceFeatureVector features;
        features << 1.0, std::sin(physical_angle_rad), std::cos(physical_angle_rad), pitch, roll;
        return features;
    }

    void initialize_support_reference_model_(size_t index, double support_proxy) {
        support_reference_weights_[index].setZero();
        support_reference_weights_[index][0] = std::max(support_proxy, reference_floor_());
        support_reference_initialized_[index] = true;
    }

    double support_reference_from_model_(
        size_t index, const ReferenceFeatureVector& features) const {
        if (!support_reference_initialized_[index])
            return nan_;
        return std::max(support_reference_weights_[index].dot(features), reference_floor_());
    }

    void update_support_reference_model_(
        size_t index, double support_proxy, const ReferenceFeatureVector& features) {
        if (!support_reference_initialized_[index]) {
            initialize_support_reference_model_(index, support_proxy);
            return;
        }

        const double predicted = support_reference_from_model_(index, features);
        const double error = support_proxy - predicted;
        const double learning_rate =
            error >= 0.0 ? support_reference_rise_alpha_ : support_reference_fall_alpha_;
        const double regularization = 1.0 + features.squaredNorm();
        support_reference_weights_[index] += learning_rate * (error / regularization) * features;
        support_reference_weights_[index][0] =
            std::max(support_reference_weights_[index][0], reference_floor_());
    }

    static double relative_support_ratio_(double support_proxy, double reference) {
        return support_proxy / std::max(reference, reference_floor_());
    }

    double support_presence_score_(double support_ratio, double low_threshold, double high_threshold) const {
        return inverse_lerp_clamped_(support_ratio, low_threshold, high_threshold);
    }

    MotionMetrics compute_motion_metrics_(
        const std::array<double, 4>& support_proxy,
        const std::array<double, 4>& predicted_support_reference,
        const std::array<double, 4>& physical_angles, double pitch, double roll,
        const std::array<double, 4>& residual, const std::array<bool, 4>& observer_available,
        bool kinematics_valid) const {
        MotionMetrics metrics;
        metrics.pitch_rate =
            std::isfinite(last_pitch_) ? (pitch - last_pitch_) / dt_ : 0.0;
        metrics.roll_rate =
            std::isfinite(last_roll_) ? (roll - last_roll_) / dt_ : 0.0;

        double residual_sum = 0.0;
        size_t residual_count = 0;
        for (size_t i = 0; i < kJointCount; ++i) {
            if (observer_available[i] && std::isfinite(support_proxy[i])
                && std::isfinite(last_support_proxy_[i])) {
                const double reference =
                    std::isfinite(predicted_support_reference[i]) ? predicted_support_reference[i]
                                                                  : support_proxy[i];
                const double jump_ratio =
                    std::abs(support_proxy[i] - last_support_proxy_[i])
                    / std::max(reference, reference_floor_());
                metrics.max_support_jump_ratio =
                    std::max(metrics.max_support_jump_ratio, jump_ratio);
            }

            if (std::isfinite(physical_angles[i]) && std::isfinite(last_physical_angle_[i])) {
                const double joint_rate =
                    std::abs(physical_angles[i] - last_physical_angle_[i]) / dt_;
                metrics.max_joint_rate = std::max(metrics.max_joint_rate, joint_rate);
            }

            if (kinematics_valid && std::isfinite(residual[i])) {
                residual_sum += residual[i];
                ++residual_count;
            }
        }

        metrics.residual_valid = residual_count > 0;
        if (metrics.residual_valid) {
            metrics.mean_residual = residual_sum / static_cast<double>(residual_count);
        }
        return metrics;
    }

    bool detect_shock_(const MotionMetrics& metrics) const {
        return metrics.max_support_jump_ratio > shock_support_jump_ratio_
            || std::abs(metrics.pitch_rate) > shock_angle_rate_threshold_
            || std::abs(metrics.roll_rate) > shock_angle_rate_threshold_
            || metrics.max_joint_rate > shock_joint_rate_threshold_
            || (metrics.residual_valid && metrics.mean_residual > shock_residual_threshold_);
    }

    void update_shock_state_(bool shock_detected) {
        if (shock_detected) {
            shock_hold_ticks_remaining_ = duration_to_ticks_(shock_hold_time_);
            return;
        }
        if (shock_hold_ticks_remaining_ > 0)
            --shock_hold_ticks_remaining_;
    }

    bool shock_active_() const { return shock_hold_ticks_remaining_ > 0; }

    bool is_quasi_static_candidate_(double command_norm, const MotionMetrics& metrics) const {
        return command_norm <= quasi_static_speed_threshold_
            && std::abs(metrics.pitch_rate) <= quasi_static_angle_rate_threshold_
            && std::abs(metrics.roll_rate) <= quasi_static_angle_rate_threshold_
            && metrics.max_joint_rate <= quasi_static_joint_rate_threshold_
            && (!metrics.residual_valid || metrics.mean_residual <= quasi_static_residual_threshold_);
    }

    void update_quasi_static_window_(bool quasi_static_candidate) {
        if (quasi_static_candidate) {
            ++quasi_static_ticks_;
            return;
        }
        quasi_static_ticks_ = 0;
    }

    bool quasi_static_ready_() const {
        return quasi_static_ticks_ > 0
            && static_cast<double>(quasi_static_ticks_) * dt_ >= quasi_static_hold_time_;
    }

    void update_support_reference_models_(
        const std::array<double, 4>& support_proxy,
        const std::array<ReferenceFeatureVector, 4>& reference_features,
        const std::array<double, 4>& load_share, const std::array<bool, 4>& observer_available,
        bool reference_learning_enabled) {
        if (!reference_learning_enabled)
            return;

        for (size_t i = 0; i < kJointCount; ++i) {
            if (!observer_available[i] || !std::isfinite(support_proxy[i]))
                continue;
            if (!std::isfinite(load_share[i]) || load_share[i] < reference_update_load_share_threshold_)
                continue;
            update_support_reference_model_(i, support_proxy[i], reference_features[i]);
        }
    }

    void update_motion_snapshot_(
        double pitch, double roll, const std::array<double, 4>& physical_angles,
        const std::array<double, 4>& support_proxy, const std::array<bool, 4>& observer_available) {
        last_pitch_ = pitch;
        last_roll_ = roll;
        for (size_t i = 0; i < kJointCount; ++i) {
            last_physical_angle_[i] = std::isfinite(physical_angles[i]) ? physical_angles[i] : nan_;
            last_support_proxy_[i] =
                (observer_available[i] && std::isfinite(support_proxy[i])) ? support_proxy[i] : nan_;
        }
    }

    double compute_global_support_score_(
        const std::array<double, 4>& support_proxy,
        const std::array<double, 4>& predicted_support_reference,
        const std::array<double, 4>& legacy_support_score,
        const std::array<bool, 4>& observer_available) const {
        double total_proxy = 0.0;
        double total_reference = 0.0;
        size_t observed_count = 0;

        for (size_t i = 0; i < kJointCount; ++i) {
            if (!observer_available[i] || !std::isfinite(support_proxy[i])
                || !std::isfinite(predicted_support_reference[i])) {
                continue;
            }
            total_proxy += support_proxy[i];
            total_reference += predicted_support_reference[i];
            ++observed_count;
        }

        if (observed_count == 0 || total_reference <= reference_floor_()) {
            double legacy_sum = 0.0;
            size_t legacy_count = 0;
            for (double value : legacy_support_score) {
                if (!std::isfinite(value))
                    continue;
                legacy_sum += value;
                ++legacy_count;
            }
            return legacy_count == 0 ? 1.0 : legacy_sum / static_cast<double>(legacy_count);
        }

        return support_presence_score_(
            relative_support_ratio_(total_proxy, total_reference),
            global_support_low_ratio_, global_support_high_ratio_);
    }

    std::array<double, 4> normalize_load_share_(const std::array<double, 4>& support_estimate) const {
        auto safe_score = support_estimate;
        double sum = 0.0;
        for (double& value : safe_score) {
            if (!std::isfinite(value))
                value = load_share_floor_;
            value = std::max(value, load_share_floor_);
            sum += value;
        }
        if (!std::isfinite(sum) || sum <= 1e-9)
            return {0.25, 0.25, 0.25, 0.25};
        for (double& value : safe_score)
            value /= sum;
        return safe_score;
    }

    void publish_contact_estimate(
        const std::array<double, 4>& confidence, const std::array<double, 4>& residual,
        const std::array<double, 4>& load_share, const std::array<double, 4>& support_estimate,
        double total_support_estimate) {
        const auto safe_confidence =
            all_finite_(confidence) ? confidence : std::array<double, 4>{1.0, 1.0, 1.0, 1.0};
        const auto safe_residual =
            all_finite_(residual) ? residual : std::array<double, 4>{0.0, 0.0, 0.0, 0.0};
        const auto safe_load_share =
            all_finite_(load_share) ? load_share : std::array<double, 4>{0.25, 0.25, 0.25, 0.25};
        *left_front_contact_confidence_ = safe_confidence[0];
        *left_back_contact_confidence_ = safe_confidence[1];
        *right_back_contact_confidence_ = safe_confidence[2];
        *right_front_contact_confidence_ = safe_confidence[3];

        *left_front_contact_residual_ = safe_residual[0];
        *left_back_contact_residual_ = safe_residual[1];
        *right_back_contact_residual_ = safe_residual[2];
        *right_front_contact_residual_ = safe_residual[3];

        *left_front_contact_load_share_ = safe_load_share[0];
        *left_back_contact_load_share_ = safe_load_share[1];
        *right_back_contact_load_share_ = safe_load_share[2];
        *right_front_contact_load_share_ = safe_load_share[3];

        *left_front_contact_normal_force_estimate_ =
            std::isfinite(support_estimate[0]) ? support_estimate[0] : nan_;
        *left_back_contact_normal_force_estimate_ =
            std::isfinite(support_estimate[1]) ? support_estimate[1] : nan_;
        *right_back_contact_normal_force_estimate_ =
            std::isfinite(support_estimate[2]) ? support_estimate[2] : nan_;
        *right_front_contact_normal_force_estimate_ =
            std::isfinite(support_estimate[3]) ? support_estimate[3] : nan_;
        *contact_normal_force_total_ = std::isfinite(total_support_estimate) ? total_support_estimate : nan_;

        *contact_confidence_mean_ = mean_array_(safe_confidence);
    }

    void publish_neutral_confidence() {
        publish_contact_estimate(
            {1.0, 1.0, 1.0, 1.0}, {0.0, 0.0, 0.0, 0.0}, {0.25, 0.25, 0.25, 0.25},
            {nan_, nan_, nan_, nan_}, nan_);
    }

    // Chassis geometry for wheel-speed and support mapping.
    const double wheel_radius_;
    const double radius_base_;
    const double pivot_offset_;
    const double body_length_;
    const double body_width_;
    const double arm_length_;

    // Temporal filtering and confidence logic.
    const double confidence_alpha_;
    const double support_force_alpha_;
    const double support_reference_rise_alpha_;
    const double support_reference_fall_alpha_;
    const double moving_speed_threshold_;
    const double slip_ratio_gain_;

    // Legacy torque proxy fallback.
    const double wheel_torque_reference_;
    const double joint_torque_reference_;

    // Relative support thresholds.
    const double local_support_low_ratio_;
    const double local_support_high_ratio_;
    const double global_support_low_ratio_;
    const double global_support_high_ratio_;

    // Learning window and shock gating.
    const double shock_support_jump_ratio_;
    const double shock_angle_rate_threshold_;
    const double shock_joint_rate_threshold_;
    const double shock_residual_threshold_;
    const double shock_hold_time_;
    const double quasi_static_speed_threshold_;
    const double quasi_static_angle_rate_threshold_;
    const double quasi_static_joint_rate_threshold_;
    const double quasi_static_residual_threshold_;
    const double quasi_static_hold_time_;
    const double reference_update_load_share_threshold_;

    // Output floors.
    const double load_share_floor_;
    const double confidence_floor_;

    std::array<double, 4> last_confidence_ = {1.0, 1.0, 1.0, 1.0};
    std::array<double, 4> filtered_support_proxy_ = {nan_, nan_, nan_, nan_};

    std::array<ReferenceFeatureVector, 4> support_reference_weights_ = zero_reference_weights_array_();
    std::array<bool, 4> support_reference_initialized_ = {false, false, false, false};

    double last_pitch_ = nan_;
    double last_roll_ = nan_;
    std::array<double, 4> last_support_proxy_ = {nan_, nan_, nan_, nan_};
    std::array<double, 4> last_physical_angle_ = {nan_, nan_, nan_, nan_};

    std::uint64_t shock_hold_ticks_remaining_ = 0;
    std::uint64_t quasi_static_ticks_ = 0;

    InputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
    InputInterface<double> chassis_radius_;

    InputInterface<double> left_front_wheel_velocity_;
    InputInterface<double> left_back_wheel_velocity_;
    InputInterface<double> right_back_wheel_velocity_;
    InputInterface<double> right_front_wheel_velocity_;

    InputInterface<double> left_front_wheel_torque_;
    InputInterface<double> left_back_wheel_torque_;
    InputInterface<double> right_back_wheel_torque_;
    InputInterface<double> right_front_wheel_torque_;

    InputInterface<double> left_front_joint_torque_;
    InputInterface<double> left_back_joint_torque_;
    InputInterface<double> right_back_joint_torque_;
    InputInterface<double> right_front_joint_torque_;

    InputInterface<double> left_front_joint_physical_angle_;
    InputInterface<double> left_back_joint_physical_angle_;
    InputInterface<double> right_back_joint_physical_angle_;
    InputInterface<double> right_front_joint_physical_angle_;

    InputInterface<double> left_front_joint_support_observer_z3_;
    InputInterface<double> left_back_joint_support_observer_z3_;
    InputInterface<double> right_back_joint_support_observer_z3_;
    InputInterface<double> right_front_joint_support_observer_z3_;

    InputInterface<double> chassis_imu_pitch_;
    InputInterface<double> chassis_imu_roll_;

    OutputInterface<double> left_front_contact_confidence_;
    OutputInterface<double> left_back_contact_confidence_;
    OutputInterface<double> right_back_contact_confidence_;
    OutputInterface<double> right_front_contact_confidence_;

    OutputInterface<double> left_front_contact_residual_;
    OutputInterface<double> left_back_contact_residual_;
    OutputInterface<double> right_back_contact_residual_;
    OutputInterface<double> right_front_contact_residual_;

    OutputInterface<double> left_front_contact_load_share_;
    OutputInterface<double> left_back_contact_load_share_;
    OutputInterface<double> right_back_contact_load_share_;
    OutputInterface<double> right_front_contact_load_share_;

    OutputInterface<double> left_front_contact_normal_force_estimate_;
    OutputInterface<double> left_back_contact_normal_force_estimate_;
    OutputInterface<double> right_back_contact_normal_force_estimate_;
    OutputInterface<double> right_front_contact_normal_force_estimate_;
    OutputInterface<double> contact_normal_force_total_;

    OutputInterface<double> contact_confidence_mean_;
};

} // namespace rmcs_core::controller::active_suspension

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::active_suspension::AdaptiveOmniContactEstimator,
    rmcs_executor::Component)
