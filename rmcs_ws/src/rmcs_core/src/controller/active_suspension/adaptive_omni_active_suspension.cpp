#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::active_suspension {
namespace {

template <size_t N>
std::array<double, N> read_double_array_parameter(
    const rclcpp::Node& node, const char* name, const std::array<double, N>& defaults) {
    const auto values = node.get_parameter_or(name, std::vector<double>{defaults.begin(), defaults.end()});
    if (values.size() == 1) {
        std::array<double, N> result{};
        result.fill(values.front());
        return result;
    }
    if (values.size() == N) {
        std::array<double, N> result{};
        for (size_t i = 0; i < N; ++i)
            result[i] = values[i];
        return result;
    }
    return defaults;
}

template <size_t N>
std::array<double, N> deg_array_to_rad(const std::array<double, N>& values) {
    std::array<double, N> result{};
    for (size_t i = 0; i < N; ++i)
        result[i] = values[i] * std::numbers::pi / 180.0;
    return result;
}

enum class SuspensionControlMode : std::uint8_t {
    kAuto = 0,
    kDebug = 1,
};

SuspensionControlMode parse_control_mode(const std::string& mode) {
    return (mode == "debug" || mode == "manual") ? SuspensionControlMode::kDebug
                                                 : SuspensionControlMode::kAuto;
}

} // namespace

class AdaptiveOmniActiveSuspension
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

    AdaptiveOmniActiveSuspension()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , radius_base_(get_parameter_or("radius_base", 0.2341741))
        , pivot_offset_(get_parameter_or("pivot_offset", 0.17389))
        , body_length_(get_parameter_or(
              "body_length", std::numbers::sqrt2 * std::max(radius_base_ - pivot_offset_, 0.03)))
        , body_width_(get_parameter_or(
              "body_width", std::numbers::sqrt2 * std::max(radius_base_ - pivot_offset_, 0.03)))
        , arm_length_(get_parameter_or("arm_length", 0.155))
        , pivot_z_(get_parameter_or("pivot_z", 0.14))
        , wheel_bottom_offset_(get_parameter_or("wheel_bottom_offset", 0.055))
        , pitch_lever_arm_(get_parameter_or(
              "pitch_lever_arm", 0.5 * body_length_ + pivot_offset_ / std::numbers::sqrt2))
        , roll_lever_arm_(get_parameter_or(
              "roll_lever_arm", 0.5 * body_width_ + pivot_offset_ / std::numbers::sqrt2))
        , min_angle_rad_(deg_to_rad(get_parameter_or("min_angle", 20.0)))
        , max_angle_rad_(deg_to_rad(get_parameter_or("max_angle", 58.0)))
        , target_physical_velocity_limit_(
              deg_to_rad(get_parameter_or("target_physical_velocity_limit", 180.0)))
        , target_physical_acceleration_limit_(
              deg_to_rad(get_parameter_or("target_physical_acceleration_limit", 720.0)))
        , control_mode_(parse_control_mode(get_parameter_or("control_mode", std::string{"auto"})))
        , debug_min_angle_rad_(deg_to_rad(get_parameter_or("debug_min_angle_deg", 0.0)))
        , debug_max_angle_rad_(deg_to_rad(get_parameter_or("debug_max_angle_deg", 58.0)))
        , debug_target_physical_angle_rad_(deg_array_to_rad(read_double_array_parameter(
              *this, "debug_target_physical_angle_deg", std::array<double, kJointCount>{0.0, 0.0, 0.0, 0.0})))
        , heave_gain_(get_parameter_or("heave_gain", 0.6))
        , warp_gain_(get_parameter_or("warp_gain", 0.6))
        , unload_confidence_threshold_(get_parameter_or("unload_confidence_threshold", 0.55))
        , reload_confidence_threshold_(get_parameter_or("reload_confidence_threshold", 0.75))
        , unload_load_share_threshold_(get_parameter_or("unload_load_share_threshold", 0.18))
        , max_seek_ground_extension_(get_parameter_or("max_seek_ground_extension", 0.035))
        , seek_ground_velocity_(get_parameter_or("seek_ground_velocity", 0.25))
        , seek_ground_release_velocity_(get_parameter_or("seek_ground_release_velocity", 0.18))
        , contact_deadband_(get_parameter_or("contact_deadband", 0.05))
        , pitch_gain_(deg_to_rad(get_parameter_or("pitch_gain_deg_per_rad", 5.0)))
        , roll_gain_(deg_to_rad(get_parameter_or("roll_gain_deg_per_rad", 5.0)))
        , torque_limit_(get_parameter_or("torque_limit", get_parameter_or("steady_torque_limit", 35.0)))
        , low_confidence_torque_boost_(get_parameter_or("low_confidence_torque_boost", 30.0)) {

        register_input("/chassis/left_front_joint/base_target_physical_angle", lf_base_target_angle_);
        register_input("/chassis/left_back_joint/base_target_physical_angle", lb_base_target_angle_);
        register_input("/chassis/right_back_joint/base_target_physical_angle", rb_base_target_angle_);
        register_input("/chassis/right_front_joint/base_target_physical_angle", rf_base_target_angle_);

        register_input(
            "/chassis/left_front_joint/base_target_physical_velocity", lf_base_target_velocity_, false);
        register_input(
            "/chassis/left_back_joint/base_target_physical_velocity", lb_base_target_velocity_, false);
        register_input(
            "/chassis/right_back_joint/base_target_physical_velocity", rb_base_target_velocity_, false);
        register_input(
            "/chassis/right_front_joint/base_target_physical_velocity",
            rf_base_target_velocity_, false);

        register_input(
            "/chassis/left_front_joint/base_target_physical_acceleration",
            lf_base_target_acceleration_, false);
        register_input(
            "/chassis/left_back_joint/base_target_physical_acceleration",
            lb_base_target_acceleration_, false);
        register_input(
            "/chassis/right_back_joint/base_target_physical_acceleration",
            rb_base_target_acceleration_, false);
        register_input(
            "/chassis/right_front_joint/base_target_physical_acceleration",
            rf_base_target_acceleration_, false);

        register_input("/chassis/left_front_joint/physical_angle", lf_joint_angle_);
        register_input("/chassis/left_back_joint/physical_angle", lb_joint_angle_);
        register_input("/chassis/right_back_joint/physical_angle", rb_joint_angle_);
        register_input("/chassis/right_front_joint/physical_angle", rf_joint_angle_);

        register_input("/chassis/left_front_contact/confidence", lf_contact_confidence_);
        register_input("/chassis/left_back_contact/confidence", lb_contact_confidence_);
        register_input("/chassis/right_back_contact/confidence", rb_contact_confidence_);
        register_input("/chassis/right_front_contact/confidence", rf_contact_confidence_);

        register_input("/chassis/left_front_contact/load_share", lf_contact_load_share_, false);
        register_input("/chassis/left_back_contact/load_share", lb_contact_load_share_, false);
        register_input("/chassis/right_back_contact/load_share", rb_contact_load_share_, false);
        register_input("/chassis/right_front_contact/load_share", rf_contact_load_share_, false);

        register_input("/chassis/imu/pitch", pitch_angle_, false);
        register_input("/chassis/imu/roll", roll_angle_, false);

        register_output("/chassis/left_front_joint/target_angle", lf_target_angle_, nan_);
        register_output("/chassis/left_back_joint/target_angle", lb_target_angle_, nan_);
        register_output("/chassis/right_back_joint/target_angle", rb_target_angle_, nan_);
        register_output("/chassis/right_front_joint/target_angle", rf_target_angle_, nan_);

        register_output("/chassis/left_front_joint/target_physical_angle", lf_target_physical_angle_, nan_);
        register_output("/chassis/left_back_joint/target_physical_angle", lb_target_physical_angle_, nan_);
        register_output("/chassis/right_back_joint/target_physical_angle", rb_target_physical_angle_, nan_);
        register_output("/chassis/right_front_joint/target_physical_angle", rf_target_physical_angle_, nan_);

        register_output(
            "/chassis/left_front_joint/target_physical_velocity", lf_target_physical_velocity_, nan_);
        register_output(
            "/chassis/left_back_joint/target_physical_velocity", lb_target_physical_velocity_, nan_);
        register_output(
            "/chassis/right_back_joint/target_physical_velocity", rb_target_physical_velocity_, nan_);
        register_output(
            "/chassis/right_front_joint/target_physical_velocity", rf_target_physical_velocity_, nan_);

        register_output(
            "/chassis/left_front_joint/target_physical_acceleration",
            lf_target_physical_acceleration_, nan_);
        register_output(
            "/chassis/left_back_joint/target_physical_acceleration",
            lb_target_physical_acceleration_, nan_);
        register_output(
            "/chassis/right_back_joint/target_physical_acceleration",
            rb_target_physical_acceleration_, nan_);
        register_output(
            "/chassis/right_front_joint/target_physical_acceleration",
            rf_target_physical_acceleration_, nan_);

        register_output("/chassis/left_front_joint/torque_limit", lf_torque_limit_, torque_limit_);
        register_output("/chassis/left_back_joint/torque_limit", lb_torque_limit_, torque_limit_);
        register_output("/chassis/right_back_joint/torque_limit", rb_torque_limit_, torque_limit_);
        register_output("/chassis/right_front_joint/torque_limit", rf_torque_limit_, torque_limit_);

        register_output("/chassis/body/heave_reference", heave_reference_, nan_);
        register_output("/chassis/body/pitch_reference", pitch_reference_, nan_);
        register_output("/chassis/body/roll_reference", roll_reference_, nan_);
        register_output("/chassis/body/warp_reference", warp_reference_, nan_);

        register_output("/chassis/body/heave_estimate", heave_estimate_, nan_);
        register_output("/chassis/body/pitch_estimate", pitch_estimate_, nan_);
        register_output("/chassis/body/roll_estimate", roll_estimate_, nan_);
        register_output("/chassis/body/warp_estimate", warp_estimate_, nan_);
    }

    void before_updating() override {
        pitch_feedback_from_imu_ = pitch_angle_.ready();
        roll_feedback_from_imu_ = roll_angle_.ready();

        if (!pitch_feedback_from_imu_)
            pitch_angle_.make_and_bind_directly(0.0);
        if (!roll_feedback_from_imu_)
            roll_angle_.make_and_bind_directly(0.0);
    }

    void update() override {
        const auto joint_angle =
            read_required_(lf_joint_angle_, lb_joint_angle_, rb_joint_angle_, rf_joint_angle_);
        if (!all_finite_(joint_angle))
            return disable_outputs_();

        if (control_mode_ == SuspensionControlMode::kDebug)
            return update_debug_mode_(joint_angle);

        const auto base_target_angle = read_required_(
            lf_base_target_angle_, lb_base_target_angle_, rb_base_target_angle_, rf_base_target_angle_);
        if (!all_finite_(base_target_angle))
            return disable_outputs_();

        const auto raw_confidence = read_required_(
            lf_contact_confidence_, lb_contact_confidence_, rb_contact_confidence_,
            rf_contact_confidence_);
        const auto confidence =
            all_finite_(raw_confidence) ? raw_confidence : std::array<double, 4>{1.0, 1.0, 1.0, 1.0};
        const auto load_share = read_load_share_or_fallback_(confidence);

        const double pitch = std::isfinite(*pitch_angle_) ? *pitch_angle_ : 0.0;
        const double roll = std::isfinite(*roll_angle_) ? *roll_angle_ : 0.0;

        const auto base_target_clearance = physical_angles_to_clearances_(base_target_angle, 0.0, 0.0);
        const auto current_clearance = physical_angles_to_clearances_(joint_angle, pitch, roll);
        if (!all_finite_(base_target_clearance) || !all_finite_(current_clearance))
            return disable_outputs_();

        const double clearance_reference =
            *std::min_element(base_target_clearance.begin(), base_target_clearance.end());
        const double clearance_estimate =
            *std::min_element(current_clearance.begin(), current_clearance.end());
        const BodyModes reference_modes{
            .heave_radius = clearance_reference, .pitch_angle = 0.0, .roll_angle = 0.0, .warp_radius = 0.0};
        BodyModes estimated_modes = compute_modes_from_distances_(current_clearance);
        estimated_modes.heave_radius = clearance_estimate;
        if (pitch_feedback_from_imu_)
            estimated_modes.pitch_angle = pitch;
        if (roll_feedback_from_imu_)
            estimated_modes.roll_angle = roll;
        if (!all_finite_(reference_modes) || !all_finite_(estimated_modes))
            return disable_outputs_();
        publish_modes_(reference_modes, estimated_modes);
        update_seek_ground_states_(confidence, load_share);

        if (!trajectory_active_) {
            trajectory_physical_angle_ = joint_angle;
            trajectory_physical_velocity_.fill(0.0);
            trajectory_physical_acceleration_.fill(0.0);
            trajectory_active_ = true;
        }

        const auto desired =
            compute_desired_physical_angle_(reference_modes.heave_radius, estimated_modes);
        if (!all_finite_(desired))
            return disable_outputs_();

        step_trajectory_(desired);
        if (!trajectory_finite_())
            return disable_outputs_();
        if (!publish_targets_())
            return disable_outputs_();
        if (!update_torque_limits_(confidence))
            return disable_outputs_();
    }

private:
    struct SeekGroundState {
        double extension_radius = 0.0;
        bool active = false;
    };

    struct BodyModes {
        double heave_radius = nan_;
        double pitch_angle = nan_;
        double roll_angle = nan_;
        double warp_radius = nan_;
    };

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double joint_zero_physical_angle_rad_ = 62.5 * std::numbers::pi / 180.0;
    static constexpr double dt_ = 1e-3;
    static constexpr std::array<double, 4> ones_sign_ = {1.0, 1.0, 1.0, 1.0};
    static constexpr std::array<double, 4> pitch_sign_ = {-1.0, 1.0, 1.0, -1.0};
    static constexpr std::array<double, 4> roll_sign_ = {-1.0, -1.0, 1.0, 1.0};
    static constexpr std::array<double, 4> warp_sign_ = {1.0, -1.0, 1.0, -1.0};

    static double deg_to_rad(double deg) { return deg * std::numbers::pi / 180.0; }

    static double physical_to_motor_angle(double physical_angle_rad) {
        return joint_zero_physical_angle_rad_ - physical_angle_rad;
    }

    template <typename InterfaceT>
    static std::array<double, 4> read_required_(
        const InterfaceT& left_front, const InterfaceT& left_back, const InterfaceT& right_back,
        const InterfaceT& right_front) {
        return {*left_front, *left_back, *right_back, *right_front};
    }

    static bool all_finite_(const std::array<double, 4>& values) {
        return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
    }

    static bool all_finite_(const BodyModes& modes) {
        return std::isfinite(modes.heave_radius) && std::isfinite(modes.pitch_angle)
            && std::isfinite(modes.roll_angle) && std::isfinite(modes.warp_radius);
    }

    static void reset_seek_ground_state_(SeekGroundState& state) {
        state.extension_radius = 0.0;
        state.active = false;
    }

    void update_debug_mode_(const std::array<double, 4>& joint_angle) {
        const double pitch = std::isfinite(*pitch_angle_) ? *pitch_angle_ : 0.0;
        const double roll = std::isfinite(*roll_angle_) ? *roll_angle_ : 0.0;

        const auto current_clearance = physical_angles_to_clearances_(joint_angle, pitch, roll);
        if (!all_finite_(current_clearance))
            return disable_outputs_();

        BodyModes estimated_modes = compute_modes_from_distances_(current_clearance);
        estimated_modes.heave_radius =
            *std::min_element(current_clearance.begin(), current_clearance.end());
        if (pitch_feedback_from_imu_)
            estimated_modes.pitch_angle = pitch;
        if (roll_feedback_from_imu_)
            estimated_modes.roll_angle = roll;
        if (!all_finite_(estimated_modes))
            return disable_outputs_();

        publish_modes_(BodyModes{}, estimated_modes);

        if (!trajectory_active_) {
            trajectory_physical_angle_ = joint_angle;
            trajectory_physical_velocity_.fill(0.0);
            trajectory_physical_acceleration_.fill(0.0);
            trajectory_active_ = true;
        }

        const auto desired = clamp_debug_target_physical_angle_();
        step_trajectory_(desired);
        if (!trajectory_finite_())
            return disable_outputs_();
        if (!publish_targets_())
            return disable_outputs_();
        if (!update_torque_limits_(std::array<double, 4>{1.0, 1.0, 1.0, 1.0}))
            return disable_outputs_();
    }

    std::array<double, 4> clamp_debug_target_physical_angle_() const {
        std::array<double, 4> desired = debug_target_physical_angle_rad_;
        const double min_angle = std::min(debug_min_angle_rad_, debug_max_angle_rad_);
        const double max_angle = std::max(debug_min_angle_rad_, debug_max_angle_rad_);
        for (double& angle : desired)
            angle = std::clamp(angle, min_angle, max_angle);
        return desired;
    }

    std::array<double, 4> physical_angles_to_clearances_(
        const std::array<double, 4>& physical_angles, double pitch, double roll) const {
        std::array<double, 4> clearances{};
        for (size_t i = 0; i < kJointCount; ++i)
            clearances[i] = physical_angle_to_clearance_(i, physical_angles[i], pitch, roll);
        return clearances;
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

    Eigen::Vector3d pivot_position_(size_t index) const {
        return body_corner_(index) + pivot_offset_ * radial_direction_(index)
             + Eigen::Vector3d{0.0, 0.0, pivot_z_};
    }

    Eigen::Vector3d wheel_bottom_position_body_(size_t index, double physical_angle_rad) const {
        const Eigen::Vector3d radial = radial_direction_(index);
        const Eigen::Vector3d arm_horizontal = arm_length_ * radial;
        const Eigen::Vector3d arm_vertical{0.0, 0.0, -arm_length_};
        const Eigen::Vector3d wheel_bottom_offset{0.0, 0.0, -wheel_bottom_offset_};
        return pivot_position_(index) + std::cos(physical_angle_rad) * arm_horizontal
             + std::sin(physical_angle_rad) * arm_vertical + wheel_bottom_offset;
    }

    double physical_angle_to_clearance_(
        size_t index, double physical_angle_rad, double pitch, double roll) const {
        const Eigen::Vector3d wheel_bottom_body =
            wheel_bottom_position_body_(index, physical_angle_rad);
        return -(rotation_world_body_(pitch, roll) * wheel_bottom_body).z();
    }

    double clamp_clearance_to_joint_limits_(
        size_t index, double clearance, double pitch, double roll) const {
        const double clearance_at_min_angle =
            physical_angle_to_clearance_(index, min_angle_rad_, pitch, roll);
        const double clearance_at_max_angle =
            physical_angle_to_clearance_(index, max_angle_rad_, pitch, roll);
        if (!std::isfinite(clearance) || !std::isfinite(clearance_at_min_angle)
            || !std::isfinite(clearance_at_max_angle)) {
            return nan_;
        }

        return std::clamp(
            clearance, std::min(clearance_at_min_angle, clearance_at_max_angle),
            std::max(clearance_at_min_angle, clearance_at_max_angle));
    }

    double clearance_to_physical_angle_(
        size_t index, double clearance, double pitch, double roll) const {
        double low_angle = min_angle_rad_;
        double high_angle = max_angle_rad_;
        double low_clearance = physical_angle_to_clearance_(index, low_angle, pitch, roll);
        double high_clearance = physical_angle_to_clearance_(index, high_angle, pitch, roll);
        double target = clamp_clearance_to_joint_limits_(index, clearance, pitch, roll);
        if (!std::isfinite(low_clearance) || !std::isfinite(high_clearance) || !std::isfinite(target))
            return nan_;

        if (std::abs(low_clearance - target) <= 1e-9)
            return low_angle;
        if (std::abs(high_clearance - target) <= 1e-9)
            return high_angle;

        // The forward geometry is continuous on the joint travel range, so bisection gives a
        // pose-consistent inverse without assuming zero pitch/roll.
        for (int iteration = 0; iteration < 48; ++iteration) {
            const double mid_angle = 0.5 * (low_angle + high_angle);
            const double mid_clearance = physical_angle_to_clearance_(index, mid_angle, pitch, roll);
            if (!std::isfinite(mid_clearance))
                return nan_;
            if (std::abs(mid_clearance - target) <= 1e-7)
                return mid_angle;

            const double low_error = low_clearance - target;
            const double mid_error = mid_clearance - target;
            if ((low_error <= 0.0 && mid_error >= 0.0) || (low_error >= 0.0 && mid_error <= 0.0)) {
                high_angle = mid_angle;
                high_clearance = mid_clearance;
            } else {
                low_angle = mid_angle;
                low_clearance = mid_clearance;
            }
        }

        return std::abs(low_clearance - target) <= std::abs(high_clearance - target) ? low_angle
                                                                                      : high_angle;
    }

    double min_clearance_() const {
        return physical_angle_to_clearance_(kLeftFront, min_angle_rad_, 0.0, 0.0);
    }
    double max_clearance_() const {
        return physical_angle_to_clearance_(kLeftFront, max_angle_rad_, 0.0, 0.0);
    }

    static double modal_component_(
        const std::array<double, 4>& values, const std::array<double, 4>& signs) {
        double component = 0.0;
        for (size_t i = 0; i < kJointCount; ++i)
            component += 0.25 * signs[i] * values[i];
        return component;
    }

    BodyModes compute_modes_from_distances_(const std::array<double, 4>& corner_distance) const {
        const double pitch_radius = modal_component_(corner_distance, pitch_sign_);
        const double roll_radius = modal_component_(corner_distance, roll_sign_);

        return {
            .heave_radius = modal_component_(corner_distance, ones_sign_),
            .pitch_angle = pitch_radius / std::max(std::abs(pitch_lever_arm_), 1e-6),
            .roll_angle = roll_radius / std::max(std::abs(roll_lever_arm_), 1e-6),
            .warp_radius = modal_component_(corner_distance, warp_sign_),
        };
    }

    std::array<double, 4> reconstruct_clearances_from_modes_(const BodyModes& modes) const {
        const double pitch_radius = modes.pitch_angle * pitch_lever_arm_;
        const double roll_radius = modes.roll_angle * roll_lever_arm_;

        std::array<double, 4> corner_radius{};
        for (size_t i = 0; i < kJointCount; ++i) {
            corner_radius[i] = modes.heave_radius + pitch_sign_[i] * pitch_radius
                             + roll_sign_[i] * roll_radius + warp_sign_[i] * modes.warp_radius;
            corner_radius[i] = std::clamp(corner_radius[i], min_clearance_(), max_clearance_());
        }
        return corner_radius;
    }

    void publish_modes_(const BodyModes& reference_modes, const BodyModes& estimated_modes) {
        *heave_reference_ = reference_modes.heave_radius;
        *pitch_reference_ = reference_modes.pitch_angle;
        *roll_reference_ = reference_modes.roll_angle;
        *warp_reference_ = reference_modes.warp_radius;

        *heave_estimate_ = estimated_modes.heave_radius;
        *pitch_estimate_ = estimated_modes.pitch_angle;
        *roll_estimate_ = estimated_modes.roll_angle;
        *warp_estimate_ = estimated_modes.warp_radius;
    }

    std::array<double, 4> read_load_share_or_fallback_(
        const std::array<double, 4>& confidence) const {
        std::array<double, 4> load_share{
            load_share_or_fallback_(lf_contact_load_share_, confidence[kLeftFront]),
            load_share_or_fallback_(lb_contact_load_share_, confidence[kLeftBack]),
            load_share_or_fallback_(rb_contact_load_share_, confidence[kRightBack]),
            load_share_or_fallback_(rf_contact_load_share_, confidence[kRightFront])};
        if (!all_finite_(load_share))
            return confidence;

        const double sum = load_share[0] + load_share[1] + load_share[2] + load_share[3];
        if (!std::isfinite(sum) || sum <= 1e-6)
            return confidence;
        for (double& value : load_share)
            value = std::clamp(value / sum, 0.0, 1.0);
        return load_share;
    }

    static double load_share_or_fallback_(const InputInterface<double>& input, double fallback) {
        return (input.ready() && std::isfinite(*input)) ? *input : fallback;
    }

    void update_seek_ground_states_(
        const std::array<double, 4>& confidence, const std::array<double, 4>& load_share) {
        update_seek_ground_state_(lf_seek_ground_state_, confidence[kLeftFront], load_share[kLeftFront]);
        update_seek_ground_state_(lb_seek_ground_state_, confidence[kLeftBack], load_share[kLeftBack]);
        update_seek_ground_state_(rb_seek_ground_state_, confidence[kRightBack], load_share[kRightBack]);
        update_seek_ground_state_(rf_seek_ground_state_, confidence[kRightFront], load_share[kRightFront]);
    }

    void update_seek_ground_state_(
        SeekGroundState& state, double confidence, double load_share) const {
        const bool should_seek =
            confidence < unload_confidence_threshold_ || load_share < unload_load_share_threshold_;
        const bool should_release =
            confidence > reload_confidence_threshold_
            && load_share > unload_load_share_threshold_ + std::max(contact_deadband_, 1e-3);

        if (should_seek)
            state.active = true;
        else if (should_release)
            state.active = false;

        const double velocity = state.active ? seek_ground_velocity_ : -seek_ground_release_velocity_;
        state.extension_radius =
            std::clamp(state.extension_radius + velocity * dt_, 0.0, max_seek_ground_extension_);
    }

    std::array<double, 4> compute_desired_physical_angle_(
        double clearance_reference, const BodyModes& estimated_modes) const {
        const double pitch = estimated_modes.pitch_angle;
        const double roll = estimated_modes.roll_angle;
        const double anchor_target = estimated_modes.heave_radius
                                   + heave_gain_ * (clearance_reference - estimated_modes.heave_radius);
        const double pitch_radius = pitch_lever_arm_ * pitch_gain_ * estimated_modes.pitch_angle;
        const double roll_radius = roll_lever_arm_ * roll_gain_ * estimated_modes.roll_angle;
        const double warp_radius = warp_gain_ * estimated_modes.warp_radius;

        std::array<double, 4> desired_clearance{};
        const std::array<double, 4> seek_extension{
            lf_seek_ground_state_.extension_radius, lb_seek_ground_state_.extension_radius,
            rb_seek_ground_state_.extension_radius, rf_seek_ground_state_.extension_radius};

        for (size_t i = 0; i < kJointCount; ++i)
            desired_clearance[i] = anchor_target - pitch_sign_[i] * pitch_radius
                                 - roll_sign_[i] * roll_radius - warp_sign_[i] * warp_radius
                                 + seek_extension[i];

        const double min_corner_clearance =
            *std::min_element(desired_clearance.begin(), desired_clearance.end());
        const double anchor_shift = anchor_target - min_corner_clearance;
        for (size_t i = 0; i < kJointCount; ++i)
            desired_clearance[i] =
                clamp_clearance_to_joint_limits_(i, desired_clearance[i] + anchor_shift, pitch, roll);
        if (!all_finite_(desired_clearance))
            return {nan_, nan_, nan_, nan_};

        std::array<double, 4> desired{};
        for (size_t i = 0; i < kJointCount; ++i)
            desired[i] = clearance_to_physical_angle_(i, desired_clearance[i], pitch, roll);
        return desired;
    }

    void step_trajectory_(const std::array<double, 4>& desired) {
        for (size_t i = 0; i < kJointCount; ++i) {
            double& angle_state = trajectory_physical_angle_[i];
            double& velocity_state = trajectory_physical_velocity_[i];
            double& acceleration_state = trajectory_physical_acceleration_[i];
            const double target_angle = desired[i];

            const double position_error = target_angle - angle_state;
            const double stopping_distance =
                velocity_state * velocity_state / (2.0 * target_physical_acceleration_limit_);

            double desired_velocity = 0.0;
            if (std::abs(position_error) > 1e-6 && std::abs(position_error) > stopping_distance)
                desired_velocity = std::copysign(target_physical_velocity_limit_, position_error);

            const double velocity_error = desired_velocity - velocity_state;
            acceleration_state = std::clamp(
                velocity_error / dt_, -target_physical_acceleration_limit_,
                target_physical_acceleration_limit_);

            velocity_state += acceleration_state * dt_;
            velocity_state = std::clamp(
                velocity_state, -target_physical_velocity_limit_, target_physical_velocity_limit_);
            angle_state += velocity_state * dt_;

            const double next_error = target_angle - angle_state;
            if ((position_error > 0.0 && next_error < 0.0)
                || (position_error < 0.0 && next_error > 0.0)
                || (std::abs(next_error) < 1e-5 && std::abs(velocity_state) < 1e-3)) {
                angle_state = target_angle;
                velocity_state = 0.0;
                acceleration_state = 0.0;
            }
        }
    }

    bool trajectory_finite_() const {
        return all_finite_(trajectory_physical_angle_) && all_finite_(trajectory_physical_velocity_)
            && all_finite_(trajectory_physical_acceleration_);
    }

    bool publish_targets_() {
        const std::array<double, 4> target_angle{
            physical_to_motor_angle(trajectory_physical_angle_[kLeftFront]),
            physical_to_motor_angle(trajectory_physical_angle_[kLeftBack]),
            physical_to_motor_angle(trajectory_physical_angle_[kRightBack]),
            physical_to_motor_angle(trajectory_physical_angle_[kRightFront])};
        if (!all_finite_(target_angle))
            return false;

        *lf_target_physical_angle_ = trajectory_physical_angle_[kLeftFront];
        *lb_target_physical_angle_ = trajectory_physical_angle_[kLeftBack];
        *rb_target_physical_angle_ = trajectory_physical_angle_[kRightBack];
        *rf_target_physical_angle_ = trajectory_physical_angle_[kRightFront];

        *lf_target_physical_velocity_ = trajectory_physical_velocity_[kLeftFront];
        *lb_target_physical_velocity_ = trajectory_physical_velocity_[kLeftBack];
        *rb_target_physical_velocity_ = trajectory_physical_velocity_[kRightBack];
        *rf_target_physical_velocity_ = trajectory_physical_velocity_[kRightFront];

        *lf_target_physical_acceleration_ = trajectory_physical_acceleration_[kLeftFront];
        *lb_target_physical_acceleration_ = trajectory_physical_acceleration_[kLeftBack];
        *rb_target_physical_acceleration_ = trajectory_physical_acceleration_[kRightBack];
        *rf_target_physical_acceleration_ = trajectory_physical_acceleration_[kRightFront];

        *lf_target_angle_ = target_angle[kLeftFront];
        *lb_target_angle_ = target_angle[kLeftBack];
        *rb_target_angle_ = target_angle[kRightBack];
        *rf_target_angle_ = target_angle[kRightFront];
        return true;
    }

    bool update_torque_limits_(const std::array<double, 4>& confidence) {
        update_joint_torque_limit_(confidence[kLeftFront], *lf_torque_limit_);
        update_joint_torque_limit_(confidence[kLeftBack], *lb_torque_limit_);
        update_joint_torque_limit_(confidence[kRightBack], *rb_torque_limit_);
        update_joint_torque_limit_(confidence[kRightFront], *rf_torque_limit_);
        return all_finite_(
            std::array<double, 4>{*lf_torque_limit_, *lb_torque_limit_, *rb_torque_limit_, *rf_torque_limit_});
    }

    void update_joint_torque_limit_(double confidence, double& torque_limit) const {
        const double confidence_adjusted_limit =
            torque_limit_ + low_confidence_torque_boost_ * (1.0 - std::clamp(confidence, 0.0, 1.0));
        if (!std::isfinite(confidence_adjusted_limit)) {
            torque_limit = torque_limit_;
            return;
        }
        torque_limit = std::max(0.0, confidence_adjusted_limit);
    }

    void disable_outputs_() {
        trajectory_active_ = false;
        reset_seek_ground_state_(lf_seek_ground_state_);
        reset_seek_ground_state_(lb_seek_ground_state_);
        reset_seek_ground_state_(rb_seek_ground_state_);
        reset_seek_ground_state_(rf_seek_ground_state_);

        *lf_target_angle_ = nan_;
        *lb_target_angle_ = nan_;
        *rb_target_angle_ = nan_;
        *rf_target_angle_ = nan_;

        *lf_target_physical_angle_ = nan_;
        *lb_target_physical_angle_ = nan_;
        *rb_target_physical_angle_ = nan_;
        *rf_target_physical_angle_ = nan_;

        *lf_target_physical_velocity_ = nan_;
        *lb_target_physical_velocity_ = nan_;
        *rb_target_physical_velocity_ = nan_;
        *rf_target_physical_velocity_ = nan_;

        *lf_target_physical_acceleration_ = nan_;
        *lb_target_physical_acceleration_ = nan_;
        *rb_target_physical_acceleration_ = nan_;
        *rf_target_physical_acceleration_ = nan_;

        *lf_torque_limit_ = torque_limit_;
        *lb_torque_limit_ = torque_limit_;
        *rb_torque_limit_ = torque_limit_;
        *rf_torque_limit_ = torque_limit_;

        *heave_reference_ = nan_;
        *pitch_reference_ = nan_;
        *roll_reference_ = nan_;
        *warp_reference_ = nan_;

        *heave_estimate_ = nan_;
        *pitch_estimate_ = nan_;
        *roll_estimate_ = nan_;
        *warp_estimate_ = nan_;
    }

    // Geometry and body-to-wheel mapping.
    const double radius_base_;
    const double pivot_offset_;
    const double body_length_;
    const double body_width_;
    const double arm_length_;
    const double pivot_z_;
    const double wheel_bottom_offset_;
    const double pitch_lever_arm_;
    const double roll_lever_arm_;

    // Joint workspace and target trajectory limits.
    const double min_angle_rad_;
    const double max_angle_rad_;
    const double target_physical_velocity_limit_;
    const double target_physical_acceleration_limit_;
    const SuspensionControlMode control_mode_;
    const double debug_min_angle_rad_;
    const double debug_max_angle_rad_;
    const std::array<double, 4> debug_target_physical_angle_rad_;

    // Body-mode control gains.
    const double heave_gain_;
    const double warp_gain_;
    const double contact_deadband_;
    const double pitch_gain_;
    const double roll_gain_;

    // Seek-ground state-machine thresholds.
    const double unload_confidence_threshold_;
    const double reload_confidence_threshold_;
    const double unload_load_share_threshold_;
    const double max_seek_ground_extension_;
    const double seek_ground_velocity_;
    const double seek_ground_release_velocity_;

    // Joint torque-limit scheduling.
    const double torque_limit_;
    const double low_confidence_torque_boost_;

    bool trajectory_active_ = false;
    std::array<double, 4> trajectory_physical_angle_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> trajectory_physical_velocity_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> trajectory_physical_acceleration_ = {0.0, 0.0, 0.0, 0.0};

    SeekGroundState lf_seek_ground_state_;
    SeekGroundState lb_seek_ground_state_;
    SeekGroundState rb_seek_ground_state_;
    SeekGroundState rf_seek_ground_state_;

    bool pitch_feedback_from_imu_ = false;
    bool roll_feedback_from_imu_ = false;

    InputInterface<double> lf_base_target_angle_;
    InputInterface<double> lb_base_target_angle_;
    InputInterface<double> rb_base_target_angle_;
    InputInterface<double> rf_base_target_angle_;

    InputInterface<double> lf_base_target_velocity_;
    InputInterface<double> lb_base_target_velocity_;
    InputInterface<double> rb_base_target_velocity_;
    InputInterface<double> rf_base_target_velocity_;

    InputInterface<double> lf_base_target_acceleration_;
    InputInterface<double> lb_base_target_acceleration_;
    InputInterface<double> rb_base_target_acceleration_;
    InputInterface<double> rf_base_target_acceleration_;

    InputInterface<double> lf_joint_angle_;
    InputInterface<double> lb_joint_angle_;
    InputInterface<double> rb_joint_angle_;
    InputInterface<double> rf_joint_angle_;

    InputInterface<double> lf_contact_confidence_;
    InputInterface<double> lb_contact_confidence_;
    InputInterface<double> rb_contact_confidence_;
    InputInterface<double> rf_contact_confidence_;

    InputInterface<double> lf_contact_load_share_;
    InputInterface<double> lb_contact_load_share_;
    InputInterface<double> rb_contact_load_share_;
    InputInterface<double> rf_contact_load_share_;

    InputInterface<double> pitch_angle_;
    InputInterface<double> roll_angle_;

    OutputInterface<double> lf_target_angle_;
    OutputInterface<double> lb_target_angle_;
    OutputInterface<double> rb_target_angle_;
    OutputInterface<double> rf_target_angle_;

    OutputInterface<double> lf_target_physical_angle_;
    OutputInterface<double> lb_target_physical_angle_;
    OutputInterface<double> rb_target_physical_angle_;
    OutputInterface<double> rf_target_physical_angle_;

    OutputInterface<double> lf_target_physical_velocity_;
    OutputInterface<double> lb_target_physical_velocity_;
    OutputInterface<double> rb_target_physical_velocity_;
    OutputInterface<double> rf_target_physical_velocity_;

    OutputInterface<double> lf_target_physical_acceleration_;
    OutputInterface<double> lb_target_physical_acceleration_;
    OutputInterface<double> rb_target_physical_acceleration_;
    OutputInterface<double> rf_target_physical_acceleration_;

    OutputInterface<double> lf_torque_limit_;
    OutputInterface<double> lb_torque_limit_;
    OutputInterface<double> rb_torque_limit_;
    OutputInterface<double> rf_torque_limit_;

    OutputInterface<double> heave_reference_;
    OutputInterface<double> pitch_reference_;
    OutputInterface<double> roll_reference_;
    OutputInterface<double> warp_reference_;

    OutputInterface<double> heave_estimate_;
    OutputInterface<double> pitch_estimate_;
    OutputInterface<double> roll_estimate_;
    OutputInterface<double> warp_estimate_;
};

} // namespace rmcs_core::controller::active_suspension

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::active_suspension::AdaptiveOmniActiveSuspension,
    rmcs_executor::Component)
