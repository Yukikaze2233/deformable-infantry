# Draft: Active Suspension Completion Plan

## Requirements (confirmed)
- 保证所有 4 个车轮在不平坦地形上保持地面接触
- 维持均匀的车轮法向力以获得最佳牵引力
- 通过 IMU 反馈保持底盘水平
- 仅针对 `deformable-infantry-omni`
- 验证以硬件联调优先，不以单元测试为主
- 当前效果很差：悬挂效果差、IMU 不调平、行程很短
- 必须从公式推导重新审视整个主动悬挂框架，不接受只做局部调参

## Codebase Analysis (from full exploration)

### What's Already Implemented (code-level verification)
1. **Per-leg persistent state machine**: `update_leg_suspension_state_()` (line 906-983) with proper kInactive→kArming→kActive→kReleasing transitions, phase_elapsed tracking, and contact_latched logic. State IS persistent across cycles.
2. **Output/state separation**: `clear_suspension_output_interfaces_()` (line 491) ONLY clears output values. `reset_suspension_internal_state_()` (line 503) is a SEPARATE function for internal state reset - NOT called per-cycle.
3. **ESO z3 → contact confidence**: `update_current_joint_feedback()` (line 673) populates eso_z3 from input interfaces. `estimate_contact_confidence_()` (line 769) uses z3 (disturbance=lower confidence), torque (higher), velocity (lower). Filtered + latched via `update_leg_contact_estimates_()` (line 803).
4. **ADRC dual-mode joint controller**: `DeformableJointController` has full TD/ESO/NLESF with smooth mode switching via `set_config()` (preserves state).
5. **IMU as force bias**: `compute_attitude_force_bias_()` (line 856) generates pitch/roll force biases via AttitudePidAxis, applied additively to per-leg support force.
6. **Support force model**: `compute_leg_support_force_()` (line 875) = gravity_per_wheel + Kz*(angle-support_zero) + D_leg*velocity + attitude_bias + (optional COM-height acceleration)

### What's NOT Done (the real gaps)
1. **Contact rebalance**: No redistribution mechanism. Lightly loaded legs don't receive extra support bias. Each leg independently computes force.
2. **Force distribution optimization**: No QP-based or iterative normal-force equalization.
3. **IMU force bias validation**: Not yet confirmed that pitch/roll force bias improves leveling without overriding suspension behavior.
4. **Suspension parameter tuning**: History shows preload/softness tradeoff unresolved (todo.md documents this extensively).

### DAG Connection Status
- ESO z3 outputs: JointController → `/chassis/{lf,lb,rb,rf}_joint/eso_z3` → Chassis inputs (wired, optional)
- suspension_mode/torque: Chassis → JointController (wired, active)
- IMU: Hardware → deferred calibration in chassis

## Git History Context
- Old "soft suspension" at commits: 802a786, 156db93, 40795e9
- Transitioned to IMU-based leveling at: 08509f6  
- Multiple parallel branches: dev/suspention, deformable-infantry-v2, upstream/Active_suspension
- v2 YAML uses different wheel controller (WheelDemoController vs DeformableOmniWheelController)

## Technical Decisions
- Contact confidence formula: 1.0 - min(|z3|/80, 0.5) + min(|torque|/20, 0.3) - min(|velocity|/10, 0.2), clamped [0,1]
- Arm into active: (velocity_ready AND contact_ready) OR (velocity_ready AND min_arming_time elapsed)
- Release from active: angle > release_angle OR (lost contact AND min_arming_time elapsed)
- Support zero angle = deploy_angle - preload_angle (preload creates positive force at min_angle)

## Open Questions
- Framework scope: keep current `DeformableJointController (ADRC) + DeformableChassis (high-level suspension intent)` split, or allow a deeper architecture rewrite if formula analysis shows the split itself is wrong?
