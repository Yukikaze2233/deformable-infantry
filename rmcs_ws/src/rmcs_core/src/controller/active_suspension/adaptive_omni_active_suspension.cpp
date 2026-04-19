#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>

#include <rclcpp/node.hpp>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::active_suspension {

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
        , min_angle_rad_(deg_to_rad(get_parameter_or("min_angle", 20.0)))
        , max_angle_rad_(deg_to_rad(get_parameter_or("max_angle", 58.0)))
        , target_physical_velocity_limit_(
              deg_to_rad(get_parameter_or("target_physical_velocity_limit", 180.0)))
        , target_physical_acceleration_limit_(
              deg_to_rad(get_parameter_or("target_physical_acceleration_limit", 720.0)))
        , contact_rebalance_gain_(deg_to_rad(get_parameter_or("contact_rebalance_gain_deg", 12.0)))
        , contact_deadband_(get_parameter_or("contact_deadband", 0.05))
        , pitch_gain_(deg_to_rad(get_parameter_or("pitch_gain_deg_per_rad", 5.0)))
        , roll_gain_(deg_to_rad(get_parameter_or("roll_gain_deg_per_rad", 5.0)))
        , switch_torque_limit_(get_parameter_or("switch_torque_limit", 120.0))
        , steady_torque_limit_(get_parameter_or("steady_torque_limit", 35.0))
        , angle_error_torque_gain_(get_parameter_or("angle_error_torque_gain", 40.0))
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

        register_output("/chassis/left_front_joint/torque_limit", lf_torque_limit_, steady_torque_limit_);
        register_output("/chassis/left_back_joint/torque_limit", lb_torque_limit_, steady_torque_limit_);
        register_output("/chassis/right_back_joint/torque_limit", rb_torque_limit_, steady_torque_limit_);
        register_output("/chassis/right_front_joint/torque_limit", rf_torque_limit_, steady_torque_limit_);
    }

    void before_updating() override {
        if (!pitch_angle_.ready())
            pitch_angle_.make_and_bind_directly(0.0);
        if (!roll_angle_.ready())
            roll_angle_.make_and_bind_directly(0.0);
    }

    void update() override {
        const auto base_target_angle = read_required_(
            lf_base_target_angle_, lb_base_target_angle_, rb_base_target_angle_, rf_base_target_angle_);
        if (!all_finite_(base_target_angle))
            return disable_outputs_();

        const auto joint_angle =
            read_required_(lf_joint_angle_, lb_joint_angle_, rb_joint_angle_, rf_joint_angle_);
        if (!all_finite_(joint_angle))
            return disable_outputs_();

        const auto raw_confidence = read_required_(
            lf_contact_confidence_, lb_contact_confidence_, rb_contact_confidence_,
            rf_contact_confidence_);
        const auto confidence =
            all_finite_(raw_confidence) ? raw_confidence : std::array<double, 4>{1.0, 1.0, 1.0, 1.0};

        const double pitch = std::isfinite(*pitch_angle_) ? *pitch_angle_ : 0.0;
        const double roll = std::isfinite(*roll_angle_) ? *roll_angle_ : 0.0;

        if (!trajectory_active_) {
            trajectory_physical_angle_ = joint_angle;
            trajectory_physical_velocity_.fill(0.0);
            trajectory_physical_acceleration_.fill(0.0);
            trajectory_active_ = true;
        }

        const auto desired = compute_desired_physical_angle_(base_target_angle, confidence, pitch, roll);
        if (!all_finite_(desired))
            return disable_outputs_();

        step_trajectory_(desired);
        if (!trajectory_finite_())
            return disable_outputs_();
        if (!publish_targets_())
            return disable_outputs_();
        if (!update_torque_limits_(joint_angle, confidence))
            return disable_outputs_();
    }

private:
    struct SwitchState {
        double last_target = nan_;
        std::uint64_t ticks_since_switch = 0;
        bool switch_active = false;
    };

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double joint_zero_physical_angle_rad_ = 62.5 * std::numbers::pi / 180.0;
    static constexpr double dt_ = 1e-3;
    static constexpr std::uint64_t high_torque_hold_ticks_ = 500;
    static constexpr double torque_decay_rate_ = 8.0;

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

    static void reset_switch_state_(SwitchState& switch_state) {
        switch_state.last_target = nan_;
        switch_state.ticks_since_switch = 0;
        switch_state.switch_active = false;
    }

    std::array<double, 4> compute_desired_physical_angle_(
        const std::array<double, 4>& base_target, const std::array<double, 4>& confidence,
        double pitch, double roll) const {
        std::array<double, 4> desired = base_target;
        const double mean_confidence =
            (confidence[0] + confidence[1] + confidence[2] + confidence[3]) / 4.0;

        constexpr std::array<double, 4> pitch_sign = {-1.0, 1.0, 1.0, -1.0};
        constexpr std::array<double, 4> roll_sign = {-1.0, -1.0, 1.0, 1.0};

        for (size_t i = 0; i < kJointCount; ++i) {
            const double confidence_error = mean_confidence - confidence[i];
            double contact_bias = 0.0;
            if (std::abs(confidence_error) > contact_deadband_)
                contact_bias = -contact_rebalance_gain_ * confidence_error;

            desired[i] += pitch_sign[i] * pitch_gain_ * pitch;
            desired[i] += roll_sign[i] * roll_gain_ * roll;
            desired[i] += contact_bias;
            desired[i] = std::clamp(desired[i], min_angle_rad_, max_angle_rad_);
        }

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

    bool update_torque_limits_(
        const std::array<double, 4>& current_angle, const std::array<double, 4>& confidence) {
        const std::array<double, 4> target_angle = trajectory_physical_angle_;
        update_joint_torque_limit_(
            target_angle[kLeftFront], current_angle[kLeftFront], confidence[kLeftFront],
            lf_switch_state_, *lf_torque_limit_);
        update_joint_torque_limit_(
            target_angle[kLeftBack], current_angle[kLeftBack], confidence[kLeftBack], lb_switch_state_,
            *lb_torque_limit_);
        update_joint_torque_limit_(
            target_angle[kRightBack], current_angle[kRightBack], confidence[kRightBack],
            rb_switch_state_, *rb_torque_limit_);
        update_joint_torque_limit_(
            target_angle[kRightFront], current_angle[kRightFront], confidence[kRightFront],
            rf_switch_state_, *rf_torque_limit_);
        return all_finite_(
            std::array<double, 4>{*lf_torque_limit_, *lb_torque_limit_, *rb_torque_limit_, *rf_torque_limit_});
    }

    void update_joint_torque_limit_(
        double current_target_angle, double current_angle, double confidence, SwitchState& switch_state,
        double& torque_limit) const {
        const double angle_error = std::abs(current_target_angle - current_angle);
        const double steady_limit = std::clamp(
            steady_torque_limit_ + angle_error_torque_gain_ * angle_error
                + low_confidence_torque_boost_ * (1.0 - std::clamp(confidence, 0.0, 1.0)),
            0.0, switch_torque_limit_);
        if (!std::isfinite(steady_limit)) {
            torque_limit = steady_torque_limit_;
            return;
        }

        if (!std::isfinite(switch_state.last_target)
            || std::abs(current_target_angle - switch_state.last_target) > 1e-4) {
            switch_state.last_target = current_target_angle;
            switch_state.ticks_since_switch = 0;
            switch_state.switch_active = true;
        } else if (switch_state.switch_active) {
            ++switch_state.ticks_since_switch;
        }

        if (!switch_state.switch_active) {
            torque_limit = steady_limit;
            return;
        }

        if (switch_state.ticks_since_switch <= high_torque_hold_ticks_) {
            torque_limit = switch_torque_limit_;
            return;
        }

        const double decay_per_tick = std::exp(-torque_decay_rate_ * dt_);
        const double decay =
            std::pow(decay_per_tick, switch_state.ticks_since_switch - high_torque_hold_ticks_);
        torque_limit = std::max(
            steady_limit, steady_limit + (switch_torque_limit_ - steady_limit) * decay);
    }

    void disable_outputs_() {
        trajectory_active_ = false;
        reset_switch_state_(lf_switch_state_);
        reset_switch_state_(lb_switch_state_);
        reset_switch_state_(rb_switch_state_);
        reset_switch_state_(rf_switch_state_);

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

        *lf_torque_limit_ = steady_torque_limit_;
        *lb_torque_limit_ = steady_torque_limit_;
        *rb_torque_limit_ = steady_torque_limit_;
        *rf_torque_limit_ = steady_torque_limit_;
    }

    const double min_angle_rad_;
    const double max_angle_rad_;
    const double target_physical_velocity_limit_;
    const double target_physical_acceleration_limit_;
    const double contact_rebalance_gain_;
    const double contact_deadband_;
    const double pitch_gain_;
    const double roll_gain_;
    const double switch_torque_limit_;
    const double steady_torque_limit_;
    const double angle_error_torque_gain_;
    const double low_confidence_torque_boost_;

    bool trajectory_active_ = false;
    std::array<double, 4> trajectory_physical_angle_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> trajectory_physical_velocity_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> trajectory_physical_acceleration_ = {0.0, 0.0, 0.0, 0.0};

    SwitchState lf_switch_state_;
    SwitchState lb_switch_state_;
    SwitchState rb_switch_state_;
    SwitchState rf_switch_state_;

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
};

} // namespace rmcs_core::controller::active_suspension

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::active_suspension::AdaptiveOmniActiveSuspension,
    rmcs_executor::Component)
