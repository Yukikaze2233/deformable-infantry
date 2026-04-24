#include <cmath>
#include <limits>

#include <fast_tf/fast_tf.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::active_suspension {

class AdaptiveOmniImuAttitude
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AdaptiveOmniImuAttitude()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/tf", tf_);

        register_output("/chassis/imu/pitch", pitch_angle_, nan_);
        register_output("/chassis/imu/roll", roll_angle_, nan_);
    }

    void update() override {
        const auto body_x_world = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::BaseLink::DirectionVector{Eigen::Vector3d::UnitX()}, *tf_);
        const auto body_y_world = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::BaseLink::DirectionVector{Eigen::Vector3d::UnitY()}, *tf_);
        const auto body_z_world = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::BaseLink::DirectionVector{Eigen::Vector3d::UnitZ()}, *tf_);

        if (!body_x_world->allFinite() || !body_y_world->allFinite() || !body_z_world->allFinite()) {
            *pitch_angle_ = nan_;
            *roll_angle_ = nan_;
            return;
        }

        // Extract chassis pitch/roll from the body axes expressed in the odom/IMU frame. The
        // formulas are invariant to yaw, so they remain valid even when the TF tree carries a
        // non-zero heading.
        const double pitch =
            std::atan2(-body_x_world->z(), std::hypot(body_x_world->x(), body_x_world->y()));
        const double roll = std::atan2(body_y_world->z(), body_z_world->z());

        *pitch_angle_ = std::isfinite(pitch) ? pitch : nan_;
        *roll_angle_ = std::isfinite(roll) ? roll : nan_;
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<rmcs_description::Tf> tf_;

    OutputInterface<double> pitch_angle_;
    OutputInterface<double> roll_angle_;
};

} // namespace rmcs_core::controller::active_suspension

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::active_suspension::AdaptiveOmniImuAttitude,
    rmcs_executor::Component)
