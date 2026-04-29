#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace rmcs_core::hardware {

class DeformableInfantryOmniSim
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

    DeformableInfantryOmniSim()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , joint_names_(read_string_array_parameter_(
              "joint_names",
              {"left_front_joint", "left_back_joint", "right_back_joint", "right_front_joint"}))
        , wheel_names_(read_string_array_parameter_(
              "wheel_names",
              {"left_front_wheel", "left_back_wheel", "right_back_wheel", "right_front_wheel"}))
        , joint_command_topics_(read_string_array_parameter_(
              "joint_effort_command_topics",
              {
                  "/sim/chassis/left_front_joint/effort_cmd",
                  "/sim/chassis/left_back_joint/effort_cmd",
                  "/sim/chassis/right_back_joint/effort_cmd",
                  "/sim/chassis/right_front_joint/effort_cmd",
              }))
        , wheel_command_topics_(read_string_array_parameter_(
              "wheel_effort_command_topics",
              {
                  "/sim/chassis/left_front_wheel/effort_cmd",
                  "/sim/chassis/left_back_wheel/effort_cmd",
                  "/sim/chassis/right_back_wheel/effort_cmd",
                  "/sim/chassis/right_front_wheel/effort_cmd",
              }))
        , joint_max_torque_(read_double_array_parameter_(
              "joint_max_torque", {25.0, 25.0, 25.0, 25.0}))
        , wheel_max_torque_(read_double_array_parameter_(
              "wheel_max_torque", {3.436926, 3.436926, 3.436926, 3.436926}))
        , joint_zero_physical_angle_rad_(
              deg_to_rad(get_parameter_or("joint_zero_physical_angle_deg", 62.5)))
        , control_power_limit_(get_parameter_or("control_power_limit", 120.0))
        , radius_base_(get_parameter_or("radius_base", 0.2341741))
        , radius_arm_length_(get_parameter_or("radius_arm_length", 0.150)) {
        const auto default_base_target_deg = read_double_array_parameter_(
            "base_target_physical_angle_deg", {58.0, 58.0, 58.0, 58.0});
        for (size_t i = 0; i < kJointCount; ++i)
            state_.base_target_physical_angle[i] = deg_to_rad(default_base_target_deg[i]);

        register_output("/chassis/control_velocity", chassis_control_velocity_);
        register_output("/chassis/control_power_limit", chassis_control_power_limit_, control_power_limit_);
        register_output("/chassis/imu/pitch", chassis_imu_pitch_, 0.0);
        register_output("/chassis/imu/roll", chassis_imu_roll_, 0.0);
        register_output("/chassis/radius", chassis_radius_, radius_base_);

        register_joint_interfaces_();
        register_wheel_interfaces_();

        const auto joint_state_topic = get_parameter_or("joint_state_topic", std::string{"/joint_states"});
        if (!joint_state_topic.empty()) {
            joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
                joint_state_topic, rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::JointState::UniquePtr msg) {
                    joint_state_subscription_callback(std::move(msg));
                });
        }

        const auto imu_topic = get_parameter_or("imu_topic", std::string{"/imu"});
        if (!imu_topic.empty()) {
            imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
                imu_topic, rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::Imu::UniquePtr msg) {
                    imu_subscription_callback(std::move(msg));
                });
        }

        const auto cmd_vel_topic = get_parameter_or("cmd_vel_topic", std::string{"/cmd_vel"});
        if (!cmd_vel_topic.empty()) {
            cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
                cmd_vel_topic, rclcpp::QoS{10}.reliable(),
                [this](geometry_msgs::msg::Twist::UniquePtr msg) {
                    cmd_vel_subscription_callback(std::move(msg));
                });
        }

        const auto base_target_topic = get_parameter_or(
            "base_target_physical_angle_topic", std::string{"/sim/chassis/base_target_physical_angle"});
        if (!base_target_topic.empty()) {
            base_target_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
                base_target_topic, rclcpp::QoS{10}.reliable(),
                [this](std_msgs::msg::Float64MultiArray::UniquePtr msg) {
                    base_target_subscription_callback(std::move(msg));
                });
        }

        for (size_t i = 0; i < kJointCount; ++i) {
            if (!joint_command_topics_[i].empty()) {
                joint_command_publishers_[i] =
                    create_publisher<std_msgs::msg::Float64>(joint_command_topics_[i], rclcpp::QoS{10}.reliable());
            }
            if (!wheel_command_topics_[i].empty()) {
                wheel_command_publishers_[i] =
                    create_publisher<std_msgs::msg::Float64>(wheel_command_topics_[i], rclcpp::QoS{10}.reliable());
            }
        }
    }

    void update() override {
        const SimState state = snapshot_();

        chassis_control_velocity_->vector << state.cmd_vx, state.cmd_vy, state.cmd_wz;
        *chassis_control_power_limit_ = control_power_limit_;
        *chassis_imu_pitch_ = state.pitch;
        *chassis_imu_roll_ = state.roll;

        std::array<double, kJointCount> physical_angles{};
        bool all_joint_angles_finite = true;
        for (size_t i = 0; i < kJointCount; ++i) {
            const double joint_control_torque = read_control_input_(joint_control_torque_inputs_[i]);
            const double physical_angle = state.joints[i].position;
            const double physical_velocity = state.joints[i].velocity;
            const double measured_torque =
                std::isfinite(state.joints[i].effort) ? state.joints[i].effort : joint_control_torque;

            physical_angles[i] = physical_angle;
            all_joint_angles_finite = all_joint_angles_finite && std::isfinite(physical_angle);

            *joint_angle_outputs_[i] = std::isfinite(physical_angle)
                                         ? joint_zero_physical_angle_rad_ - physical_angle
                                         : nan_;
            *joint_velocity_outputs_[i] =
                std::isfinite(physical_velocity) ? -physical_velocity : nan_;
            *joint_torque_outputs_[i] = measured_torque;
            *joint_max_torque_outputs_[i] = joint_max_torque_[i];
            *joint_physical_angle_outputs_[i] = physical_angle;
            *joint_physical_velocity_outputs_[i] = physical_velocity;

            *base_target_physical_angle_outputs_[i] = state.base_target_physical_angle[i];
            *base_target_physical_velocity_outputs_[i] = 0.0;
            *base_target_physical_acceleration_outputs_[i] = 0.0;
        }

        *chassis_radius_ = all_joint_angles_finite ? calculate_radius_(physical_angles) : radius_base_;

        for (size_t i = 0; i < kJointCount; ++i) {
            const double wheel_control_torque = read_control_input_(wheel_control_torque_inputs_[i]);
            const double measured_torque =
                std::isfinite(state.wheels[i].effort) ? state.wheels[i].effort : wheel_control_torque;

            *wheel_velocity_outputs_[i] = state.wheels[i].velocity;
            *wheel_torque_outputs_[i] = measured_torque;
            *wheel_max_torque_outputs_[i] = wheel_max_torque_[i];
        }

        publish_effort_commands_();
    }

private:
    struct JointFeedback {
        double position = nan_;
        double velocity = nan_;
        double effort = nan_;
    };

    struct SimState {
        std::array<JointFeedback, kJointCount> joints{};
        std::array<JointFeedback, kJointCount> wheels{};
        std::array<double, kJointCount> base_target_physical_angle = {0.0, 0.0, 0.0, 0.0};
        double cmd_vx = 0.0;
        double cmd_vy = 0.0;
        double cmd_wz = 0.0;
        double pitch = 0.0;
        double roll = 0.0;
    };

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    static double deg_to_rad(double deg) { return deg * std::numbers::pi / 180.0; }

    std::array<std::string, kJointCount> read_string_array_parameter_(
        const char* name, const std::array<std::string, kJointCount>& defaults) const {
        const auto values = get_parameter_or(name, std::vector<std::string>{defaults.begin(), defaults.end()});
        if (values.size() == kJointCount) {
            return {values[0], values[1], values[2], values[3]};
        }

        RCLCPP_WARN(
            get_logger(), "Parameter '%s' expects exactly %zu values. Using defaults.",
            name, static_cast<size_t>(kJointCount));
        return defaults;
    }

    std::array<double, kJointCount> read_double_array_parameter_(
        const char* name, const std::array<double, kJointCount>& defaults) const {
        const auto values = get_parameter_or(name, std::vector<double>{defaults.begin(), defaults.end()});
        if (values.size() == 1) {
            return {values[0], values[0], values[0], values[0]};
        }
        if (values.size() == kJointCount) {
            return {values[0], values[1], values[2], values[3]};
        }

        RCLCPP_WARN(
            get_logger(), "Parameter '%s' expects 1 or %zu values. Using defaults.",
            name, static_cast<size_t>(kJointCount));
        return defaults;
    }

    void register_joint_interfaces_() {
        for (size_t i = 0; i < kJointCount; ++i) {
            const std::string& name = joint_names_[i];
            register_output("/chassis/" + name + "/angle", joint_angle_outputs_[i], nan_);
            register_output("/chassis/" + name + "/velocity", joint_velocity_outputs_[i], nan_);
            register_output("/chassis/" + name + "/torque", joint_torque_outputs_[i], 0.0);
            register_output("/chassis/" + name + "/max_torque", joint_max_torque_outputs_[i], joint_max_torque_[i]);
            register_output("/chassis/" + name + "/physical_angle", joint_physical_angle_outputs_[i], nan_);
            register_output(
                "/chassis/" + name + "/physical_velocity", joint_physical_velocity_outputs_[i], nan_);
            register_output(
                "/chassis/" + name + "/base_target_physical_angle",
                base_target_physical_angle_outputs_[i], state_.base_target_physical_angle[i]);
            register_output(
                "/chassis/" + name + "/base_target_physical_velocity",
                base_target_physical_velocity_outputs_[i], 0.0);
            register_output(
                "/chassis/" + name + "/base_target_physical_acceleration",
                base_target_physical_acceleration_outputs_[i], 0.0);

            register_input("/chassis/" + name + "/control_torque", joint_control_torque_inputs_[i], false);
        }
    }

    void register_wheel_interfaces_() {
        for (size_t i = 0; i < kJointCount; ++i) {
            const std::string& name = wheel_names_[i];
            register_output("/chassis/" + name + "/velocity", wheel_velocity_outputs_[i], nan_);
            register_output("/chassis/" + name + "/torque", wheel_torque_outputs_[i], 0.0);
            register_output("/chassis/" + name + "/max_torque", wheel_max_torque_outputs_[i], wheel_max_torque_[i]);
            register_input("/chassis/" + name + "/control_torque", wheel_control_torque_inputs_[i], false);
        }
    }

    SimState snapshot_() const {
        std::scoped_lock lock(state_mutex_);
        return state_;
    }

    double calculate_radius_(const std::array<double, kJointCount>& physical_angles) const {
        double radius_sum = 0.0;
        for (double angle : physical_angles)
            radius_sum += radius_base_ + radius_arm_length_ * std::cos(angle);
        return radius_sum / static_cast<double>(kJointCount);
    }

    static double read_control_input_(const InputInterface<double>& input) {
        return (input.ready() && std::isfinite(*input)) ? *input : 0.0;
    }

    void joint_state_subscription_callback(sensor_msgs::msg::JointState::UniquePtr msg) {
        std::scoped_lock lock(state_mutex_);
        for (size_t i = 0; i < msg->name.size(); ++i) {
            const double position = value_or_nan_(msg->position, i);
            const double velocity = value_or_nan_(msg->velocity, i);
            const double effort = value_or_nan_(msg->effort, i);

            if (const auto joint_index = find_index_(msg->name[i], joint_names_)) {
                state_.joints[*joint_index] = {position, velocity, effort};
                continue;
            }
            if (const auto wheel_index = find_index_(msg->name[i], wheel_names_))
                state_.wheels[*wheel_index] = {position, velocity, effort};
        }
    }

    void imu_subscription_callback(sensor_msgs::msg::Imu::UniquePtr msg) {
        const Eigen::Quaterniond q{
            msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z};
        if (!std::isfinite(q.w()) || !std::isfinite(q.x()) || !std::isfinite(q.y())
            || !std::isfinite(q.z()) || q.norm() < 1e-9) {
            return;
        }

        const Eigen::Quaterniond normalized = q.normalized();
        const Eigen::Vector3d body_x_world = normalized * Eigen::Vector3d::UnitX();
        const Eigen::Vector3d body_y_world = normalized * Eigen::Vector3d::UnitY();
        const Eigen::Vector3d body_z_world = normalized * Eigen::Vector3d::UnitZ();

        const double pitch =
            std::atan2(-body_x_world.z(), std::hypot(body_x_world.x(), body_x_world.y()));
        const double roll = std::atan2(body_y_world.z(), body_z_world.z());
        if (!std::isfinite(pitch) || !std::isfinite(roll))
            return;

        std::scoped_lock lock(state_mutex_);
        state_.pitch = pitch;
        state_.roll = roll;
    }

    void cmd_vel_subscription_callback(geometry_msgs::msg::Twist::UniquePtr msg) {
        std::scoped_lock lock(state_mutex_);
        state_.cmd_vx = msg->linear.x;
        state_.cmd_vy = msg->linear.y;
        state_.cmd_wz = msg->angular.z;
    }

    void base_target_subscription_callback(std_msgs::msg::Float64MultiArray::UniquePtr msg) {
        if (msg->data.size() != 1 && msg->data.size() != kJointCount) {
            RCLCPP_WARN(
                get_logger(),
                "Base target topic expects 1 or %zu values in radians. Received %zu.",
                static_cast<size_t>(kJointCount), msg->data.size());
            return;
        }

        std::scoped_lock lock(state_mutex_);
        if (msg->data.size() == 1) {
            state_.base_target_physical_angle.fill(msg->data.front());
            return;
        }

        for (size_t i = 0; i < kJointCount; ++i)
            state_.base_target_physical_angle[i] = msg->data[i];
    }

    void publish_effort_commands_() {
        std_msgs::msg::Float64 msg;
        for (size_t i = 0; i < kJointCount; ++i) {
            msg.data = read_control_input_(joint_control_torque_inputs_[i]);
            if (joint_command_publishers_[i])
                joint_command_publishers_[i]->publish(msg);

            msg.data = read_control_input_(wheel_control_torque_inputs_[i]);
            if (wheel_command_publishers_[i])
                wheel_command_publishers_[i]->publish(msg);
        }
    }

    static double value_or_nan_(const std::vector<double>& values, size_t index) {
        return index < values.size() ? values[index] : nan_;
    }

    static std::optional<size_t> find_index_(
        const std::string& name, const std::array<std::string, kJointCount>& names) {
        for (size_t i = 0; i < names.size(); ++i) {
            if (name == names[i])
                return i;
        }
        return std::nullopt;
    }

    const std::array<std::string, kJointCount> joint_names_;
    const std::array<std::string, kJointCount> wheel_names_;
    const std::array<std::string, kJointCount> joint_command_topics_;
    const std::array<std::string, kJointCount> wheel_command_topics_;
    const std::array<double, kJointCount> joint_max_torque_;
    const std::array<double, kJointCount> wheel_max_torque_;
    const double joint_zero_physical_angle_rad_;
    const double control_power_limit_;
    const double radius_base_;
    const double radius_arm_length_;

    mutable std::mutex state_mutex_;
    SimState state_{};

    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
    OutputInterface<double> chassis_control_power_limit_;
    OutputInterface<double> chassis_imu_pitch_;
    OutputInterface<double> chassis_imu_roll_;
    OutputInterface<double> chassis_radius_;

    std::array<OutputInterface<double>, kJointCount> joint_angle_outputs_{};
    std::array<OutputInterface<double>, kJointCount> joint_velocity_outputs_{};
    std::array<OutputInterface<double>, kJointCount> joint_torque_outputs_{};
    std::array<OutputInterface<double>, kJointCount> joint_max_torque_outputs_{};
    std::array<OutputInterface<double>, kJointCount> joint_physical_angle_outputs_{};
    std::array<OutputInterface<double>, kJointCount> joint_physical_velocity_outputs_{};
    std::array<OutputInterface<double>, kJointCount> base_target_physical_angle_outputs_{};
    std::array<OutputInterface<double>, kJointCount> base_target_physical_velocity_outputs_{};
    std::array<OutputInterface<double>, kJointCount> base_target_physical_acceleration_outputs_{};

    std::array<OutputInterface<double>, kJointCount> wheel_velocity_outputs_{};
    std::array<OutputInterface<double>, kJointCount> wheel_torque_outputs_{};
    std::array<OutputInterface<double>, kJointCount> wheel_max_torque_outputs_{};

    std::array<InputInterface<double>, kJointCount> joint_control_torque_inputs_{};
    std::array<InputInterface<double>, kJointCount> wheel_control_torque_inputs_{};

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr base_target_subscription_;

    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, kJointCount> joint_command_publishers_{};
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, kJointCount> wheel_command_publishers_{};
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DeformableInfantryOmniSim, rmcs_executor::Component)
