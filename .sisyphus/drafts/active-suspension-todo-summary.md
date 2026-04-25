# Active Suspension TODO Summary

## Goals
- Keep all 4 wheels in contact on uneven terrain.
- Maintain more even wheel normal force for traction.
- Keep the chassis level using IMU feedback.

## What Changed
- `deformable_chassis.cpp`
  - Replaced the old `apply_imu_attitude_correction_()`-driven main path with `update_active_suspension_(joint_feedback)` inside `run_joint_intent_pipeline_()`.
  - Added explicit suspension-oriented data types:
    - `SuspensionPhase`
    - `LegFeedback`
    - `AttitudeBias`
    - `LegControlState`
  - Added suspension control helpers:
    - `leg_feedback_at_()`
    - `compute_attitude_force_bias_()`
    - `compute_leg_support_force_()`
    - `leg_force_to_joint_torque_()`
    - `update_leg_suspension_state_()`
    - `update_active_suspension_()`
  - Changed IMU leveling role from direct joint-angle correction to suspension force bias.
  - Restored per-leg suspension intent generation:
    - activation check per leg
    - support force per leg
    - suspension torque per leg
    - ride-height target per leg
- `deformable-infantry-omni.yaml`
  - Restored suspension geometry/dynamics parameters:
    - `active_suspension_mass`
    - `active_suspension_rod_length`
    - `active_suspension_Kz`
    - `active_suspension_D_leg`
    - `active_suspension_torque_limit`
    - `active_suspension_gravity_comp_gain`
    - `active_suspension_control_acceleration_limit`
    - `active_suspension_preload_angle_deg`
    - `active_suspension_entry_offset_deg`
    - `active_suspension_ride_height_offset_deg`
    - `active_suspension_hold_travel_deg`
    - `active_suspension_activation_velocity_threshold_deg`
  - Kept IMU PID parameters, but they now conceptually act as suspension force-bias gains.
  - Added `eso_z2_output` and `eso_z3_output` wiring for all 4 `DeformableJointController`s.

## Current State
- Mainline is no longer only IMU angle leveling.
- Mainline now has a first restored suspension-first path:
  - deploy target request
  - per-leg activation
  - support force calculation
  - torque output
  - ride-height hold target
- IMU is now used as a support-force bias instead of replacing suspension intent.

## Known Risks / Follow-up Fixes
- `update_active_suspension_()` currently starts with `clear_suspension_outputs()` each cycle.
  - This may be too aggressive because it resets `joint_suspension_active_` and per-leg phase before re-arming in the same cycle.
  - Likely next fix: separate “clear published outputs” from “reset internal suspension state”.
- `LegControlState` exists but is only partially used.
  - Next step: make phase transitions persistent and meaningful across cycles.
- Contact estimation is not implemented yet.
  - `eso_z2/z3` are now exposed so contact-confidence logic can be added next.
- Normal-force equalization is not implemented yet.
  - Current control is a first suspension restoration, not the final full optimizer.

## Next Steps
1. Fix per-cycle reset behavior in suspension state/output handling.
2. Make `SuspensionPhase` persistent and use it for explicit inactive/arming/active/releasing transitions.
3. Add minimal contact-confidence estimation using:
   - `eso_z3`
   - joint torque
   - joint physical velocity
4. Add contact rebalance bias so lightly loaded / near-airborne legs get more support.
5. Validate that IMU force bias improves leveling without overriding suspension behavior.
6. Later: implement proper normal-force distribution / optimization.

## Suggested TODO Conversion
- [ ] Restore persistent per-leg suspension state machine
- [ ] Split output clearing from state reset in suspension path
- [ ] Add minimal contact confidence from ESO/joint feedback
- [ ] Add contact rebalance support bias
- [ ] Tune ride-height / preload / entry / release thresholds
- [ ] Validate IMU leveling as force bias instead of angle override
- [ ] Implement normal-force distribution after contact logic is stable
