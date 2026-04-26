#include "joint_state.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rmcs_core::controller::chassis {

void JointState::configure(const Config& config) { config_ = config; }

void JointState::reset(size_t index) {
    (void)index;
}

void JointState::update(size_t index, const PerLegInput& input, PerLegState& state) {
    if (state.requested_deploy != input.deploy_requested) {
        state.phase_elapsed = 0;
    } else {
        state.phase_elapsed += input.dt;
    }
    state.requested_deploy = input.deploy_requested;

    if (!input.deploy_requested
        || !std::isfinite(input.physical_angle)
        || !std::isfinite(input.physical_velocity)) {
        state.phase           = SuspensionPhase::kInactive;
        state.active          = false;
        state.phase_elapsed   = 0;
        state.contact_latched = false;
        return;
    }

    const bool entry_ready  = input.physical_angle <= (config_.deploy_angle + config_.entry_offset);
    const bool velocity_ready = std::abs(input.physical_velocity) <= config_.velocity_threshold;
    const bool contact_ready  = input.contact_ready;

    switch (state.phase) {
    case SuspensionPhase::kInactive:
        state.active = false;
        if (entry_ready) {
            state.phase = SuspensionPhase::kArming;
            state.phase_elapsed = 0;
        }
        break;

    case SuspensionPhase::kArming:
        state.active = false;
        if (!entry_ready) {
            state.phase = SuspensionPhase::kInactive;
            state.phase_elapsed = 0;
            break;
        }
        if (velocity_ready
            && std::isfinite(input.motor_angle)
            && (contact_ready || state.phase_elapsed >= config_.min_arming_time)) {
            state.phase         = SuspensionPhase::kActive;
            state.active        = true;
            state.phase_elapsed = 0;
        }
        break;

    case SuspensionPhase::kActive: {
        const double release_angle =
            config_.deploy_angle + config_.ride_height_offset + config_.hold_travel;
        if (input.physical_angle > release_angle
            || (!contact_ready && state.phase_elapsed >= config_.min_arming_time)) {
            state.phase  = SuspensionPhase::kReleasing;
            state.active = false;
            state.phase_elapsed = 0;
            break;
        }
        state.active = true;
        break;
    }

    case SuspensionPhase::kReleasing:
        state.active = false;
        if (!entry_ready || state.phase_elapsed >= config_.min_arming_time) {
            state.phase = SuspensionPhase::kInactive;
            state.phase_elapsed = 0;
        }
        break;
    }
}

} // namespace rmcs_core::controller::chassis
