#pragma once

#include <cstddef>
#include <cstdint>

namespace rmcs_core::controller::chassis {

enum JointIndex : size_t {
    kLeftFront  = 0,
    kLeftBack   = 1,
    kRightBack  = 2,
    kRightFront = 3,
    kJointCount = 4,
};

enum class SuspensionPhase : uint8_t { kInactive, kArming, kActive, kReleasing };

class JointState {
public:
    struct Config {
        double entry_offset               = 0;
        double ride_height_offset         = 0;
        double hold_travel                = 0;
        double velocity_threshold         = 0;
        double min_arming_time            = 0.02;
        double deploy_angle               = 0;
        double max_angle                  = 0;
    };

    struct PerLegInput {
        double physical_angle     = 0;
        double physical_velocity  = 0;
        double motor_angle         = 0;
        bool   deploy_requested    = false;
        bool   contact_ready       = false;
        double dt                  = 0.001;
    };

    struct PerLegState {
        SuspensionPhase phase        = SuspensionPhase::kInactive;
        bool active                  = false;
        double phase_elapsed         = 0;
        bool contact_latched         = false;
        bool requested_deploy        = false;
    };

    void configure(const Config& config);
    void reset(size_t index);
    void update(size_t index, const PerLegInput& input, PerLegState& state);

private:
    Config config_;
};

} // namespace rmcs_core::controller::chassis
