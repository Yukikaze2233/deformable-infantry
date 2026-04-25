# Active Suspension Formula Redesign

## TL;DR

> **Quick Summary**: Redesign the upper-layer active suspension formulas in `DeformableChassis` from first-principles kinematics — replacing the ad-hoc joint-angle-space spring-damper with a proper Cartesian-space virtual impedance model and a geometric IMU-to-leg-displacement mapping. The ADRC `DeformableJointController` is preserved unchanged.
>
> **Deliverables**:
> - Documented kinematic/static derivation (joint angles ↔ wheel positions ↔ chassis pose)
> - Cartesian-space virtual impedance force model replacing angle-space spring
> - Geometric IMU-to-leg-displacement mapping replacing ad-hoc PID force bias
> - Proper gravity compensation via Jacobian transpose
> - Updated YAML parameters for new linear stiffness/damping
> - Frequency-sweep-based parameter identification
> - On-vehicle verification procedure
>
> **Estimated Effort**: Medium
> **Parallel Execution**: YES — 4 waves
> **Critical Path**: Task 1 → Task 5 → Task 9 → Task 10 → Task 12 → Task 13 → Task 14

---

## Context

### Original Request
Re-derive the active suspension framework from first principles. Current implementation has:
- ADRC-based joint controller (working) — we keep this
- Per-leg support force in joint-angle space: `F = mg/4 + Kz*(α - α₀) + D*α_dot + IMU_bias` (INADEQUATE)
- IMU leveling as ad-hoc PID force bias (INADEQUATE)
- Poor grip, no leveling, very short perceived travel

### Interview Summary
**Key Discussions**:
- Architecture: Keep `DeformableJointController` ADRC kernel unchanged. Modify only `DeformableChassis` upper-layer formulas.
- Mechanics: Parallel four-bar linkage ≈ single rod of length L ≈ 0.15 m, revolute joint at chassis, wheel at rod end. Physical angle α measured from vertical down.
- Scope: Layer 1 (impedance/kinematics) + Layer 2 (IMU geometric mapping). Layer 3 (QP force optimization) deferred.
- Verification: Frequency sweep identification first, then on-vehicle iterative tuning.

**Research Findings**:
- `active_suspension_wheel_base_half_x_`, `active_suspension_wheel_base_half_y_`, `active_suspension_com_height_` already exist as member variables in `DeformableChassis`.
- `kPitchSigns_` and `kRollSigns_` static arrays already exist for distributing pitch/roll forces to legs.
- ESO z3 contact estimation and per-leg state machine are already implemented and working — no changes needed.
- `compute_attitude_force_bias_()` uses PID with IMU calibration offset.
- Current `compute_leg_support_force_()` and `leg_force_to_joint_torque_()` are the two functions that need replacement.

### Coordinate Convention (from code analysis)
- α = physical_angle = angle between rod and vertical-down
  - α = 0° → rod vertical (wheel directly under chassis joint)
  - α = 8° (min_angle) → nearly vertical, most extended (wheels farthest below chassis)
  - α = 58° (max_angle) → rod swung outward, most retracted
- Wheel vertical position relative to joint: `z_w = -L·cos(α)`
- Force arm for vertical force: `L·sin(α)` (confirmed by existing τ = F·L·sin(α))
- Jacobian: `∂z/∂α = L·sin(α)`

---

## Work Objectives

### Core Objective
Replace the joint-angle-space spring-damper and ad-hoc IMU force bias in `DeformableChassis` with a physically correct model based on: (1) proper kinematics mapping joint angles to wheel Cartesian positions, (2) virtual impedance in Cartesian space, (3) geometric IMU-to-leg-displacement mapping via Jacobian pseudoinverse.

### Concrete Deliverables
- Modified `deformable_chassis.cpp`: new or rewritten functions for kinematics, Cartesian impedance, IMU mapping
- Modified `deformable-infantry-omni.yaml`: new parameter names (linear K, D units instead of angular)
- Sweep data processing script (Python/MATLAB) for b0 identification

### Definition of Done
- [ ] Cartesian-space support force model implemented and compiles
- [ ] IMU-to-leg geometric mapping implemented (replaces PID force bias)
- [ ] Gravity compensation works: robot holds itself up on level ground with minimal static error
- [ ] On uneven terrain: all 4 wheels maintain contact, chassis stays level within ±5°
- [ ] Frequency sweep data confirms correct b0 for all 4 joints
- [ ] No regression in normal deploy/tracking mode (ADRC normal mode unaffected)

### Must Have
- Proper forward kinematics: 4 α_i → 4 wheel positions → chassis pose (z, θ, φ)
- Jacobian J (3×4) mapping Δα → Δpose
- Pseudoinverse J⁺ for IMU error → Δα correction
- Cartesian spring: F_i = K_lin * (z_i_ref - z_i) + D_lin * (ż_i_ref - ż_i) per leg
- Torque from force: τ_i = L·sin(α_i) · F_i (already correct, keep this part)
- Gravity compensation: τ_g = J^T · w where w = [mg/4, mg/4, mg/4, mg/4]
- Parameter identification from frequency sweep data

### Must NOT Have (Guardrails)
- Do NOT modify `DeformableJointController` — ADRC kernel, mode switching, ESO all untouched
- Do NOT modify the per-leg state machine (`update_leg_suspension_state_()`) — it's working
- Do NOT modify contact estimation (`estimate_contact_confidence_()`, `update_leg_contact_estimates_()`) — it's working
- Do NOT implement Layer 3 QP force optimization — deferred to future plan
- Do NOT introduce new external dependencies (no new ROS packages, libraries)
- Do NOT change the public DAG interface (input/output topic names stay the same)
- Do NOT remove existing member variables that other functions depend on (add new ones instead)
- No AI slop: no excessive comments, no premature abstraction, no unused helper functions

---

## Verification Strategy

> **ZERO HUMAN INTERVENTION** — ALL verification is agent-executed. No manual testing required.

### Test Decision
- **Infrastructure exists**: YES — `colcon test --packages-select rmcs_core` available, frequency sweep infrastructure exists
- **Automated tests**: None — hardware integration priority. Compile check + static analysis only for non-hardware verification.
- **Framework**: `clang++` compile, `colcon build`, sweep CSV analysis
- **Agent-Executed QA**: Primary verification method for all tasks

### QA Policy
Every implementation task includes agent-executed QA scenarios. Evidence saved to `.sisyphus/evidence/task-{N}-{scenario-slug}.{ext}`.

- **Build verification**: Bash (`colcon build --packages-select rmcs_core`) — verify compilation, check for warnings
- **Static analysis**: Bash (`clang-tidy` or grep for common issues) — verify no obvious formula errors
- **Sweep data analysis**: Bash/Python — verify sweep CSV contains expected signals, compute b0
- **On-vehicle**: Interactive bash (tmux) — `attach-remote -r`, verify log output, check sensor values

---

## Execution Strategy

### Parallel Execution Waves

> Maximize throughput by grouping independent tasks into parallel waves.
> Each wave completes before the next begins.

```
Wave 1 (Start Immediately — derivation + documentation, ALL PARALLEL):
├── Task 1: Document coordinate conventions & measurement baseline [writing]
├── Task 2: Derive forward kinematics (α → wheel pos → chassis pose) [writing]
├── Task 3: Derive Jacobian & pseudoinverse for IMU leveling [writing]
└── Task 4: Derive Cartesian virtual impedance & gravity compensation [writing]

Wave 2 (After Wave 1 — code changes, MAX PARALLEL):
├── Task 5: Implement new kinematics & Jacobian functions in DeformableChassis [deep]
├── Task 6: Implement Cartesian impedance support force function [deep]
├── Task 7: Implement geometric IMU-to-leg-displacement mapping [deep]
└── Task 8: Update YAML configuration parameters [quick]

Wave 3 (After Wave 2 — integration & compile):
├── Task 9: Refactor update_active_suspension_() to use new formulas [deep]
├── Task 10: Build, fix compile errors, static verification [quick]
└── Task 11: Frequency sweep parameter identification [unspecified-high]

Wave 4 (After Wave 3 — on-vehicle verification):
├── Task 12: Deploy & static loading test (gravity comp verification) [unspecified-high]
├── Task 13: Uneven terrain test (wheel contact & leveling verification) [unspecified-high]
└── Task 14: Iterative parameter tuning & documentation [unspecified-high]

Wave FINAL (After ALL tasks):
├── Task F1: Plan compliance audit [oracle]
├── Task F2: Code quality review [unspecified-high]
└── Task F3: Real manual QA on vehicle [unspecified-high]

Critical Path: Task 1 → Task 5 → Task 9 → Task 10 → Task 12 → Task 13 → Task 14
Parallel Speedup: ~60% faster than sequential
Max Concurrent: 4 (Waves 1, 2, 3)
```

### Dependency Matrix

| Task | Depends On | Blocks | Wave |
|------|-----------|--------|------|
| 1 | — | 5, 6, 7 | 1 |
| 2 | — | 5 | 1 |
| 3 | — | 6, 7 | 1 |
| 4 | — | 6, 9 | 1 |
| 5 | 1, 2 | 9 | 2 |
| 6 | 1, 3, 4 | 9 | 2 |
| 7 | 1, 3 | 9 | 2 |
| 8 | 1, 4 | 9, 10 | 2 |
| 9 | 5, 6, 7, 8 | 10 | 3 |
| 10 | 9 | 12 | 3 |
| 11 | 10 | 12 | 3 |
| 12 | 10, 11 | 13 | 4 |
| 13 | 12 | 14 | 4 |
| 14 | 13 | F1-F3 | 4 |

### Agent Dispatch Summary

- **Wave 1**: 4 — T1-T4 → `writing`
- **Wave 2**: 4 — T5-T7 → `deep`, T8 → `quick`
- **Wave 3**: 3 — T9 → `deep`, T10 → `quick`, T11 → `unspecified-high`
- **Wave 4**: 3 — T12-T14 → `unspecified-high`
- **FINAL**: 3 — F1 → `oracle`, F2 → `unspecified-high`, F3 → `unspecified-high`

---

## TODOs

- [ ] 1. Document coordinate conventions & measurement baseline

  **What to do**:
  - Read `deformable_chassis.cpp` lines 1-100 (member variables, enums), lines 1246-1252 (`physical_to_motor_angle`, `motor_to_physical_angle`), lines 758-767 (`leg_feedback_at_`)
  - Read the hardware component `deformable-infantry-omni.cpp` to confirm motor zero-point calibration and joint direction
  - Document in a code comment block at the top of `compute_leg_support_force_()`:
    - Physical angle α definition: α = 0 → rod vertical-down, α increases as rod swings outward
    - Wheel vertical position: `z_w = joint_z - L·cos(α)` (relative to chassis joint)
    - Force arm: `r_eff = L·sin(α)` (perpendicular distance from joint axis to vertical force line)
    - Jacobian: `∂z_w/∂α = L·sin(α)` and its inverse: `∂α/∂z_w = 1/(L·sin(α))`
    - Chassis wheel positions: LF(-a,+b), LB(-a,-b), RB(+a,-b), RF(+a,+b) where a = `wheel_base_half_x_`, b = `wheel_base_half_y_`
    - Joint index order: kLeftFront=0, kLeftBack=1, kRightBack=2, kRightFront=3
  - Verify unit consistency: all internal calculations in radians, input angles in degrees, convert at I/O boundaries

  **Must NOT do**:
  - Do NOT change any existing variable or function names at this stage
  - Do NOT add comments to functions outside the suspension path
  - Do NOT create a separate .md document — put documentation as code comments

  **Recommended Agent Profile**:
  - **Category**: `writing`
    - Reason: Documentation task — reading existing code and writing structured comments
  - **Skills**: `[]`
    - No special skills needed — pure code reading and comment writing

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 2, 3, 4)
  - **Blocks**: Tasks 5, 6, 7
  - **Blocked By**: None

  **References**:
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:1-100` — Enum definitions (SuspensionPhase, JointIndex), member variable declarations, struct definitions (JointFeedbackFrame, LegFeedback, AttitudeBias, LegControlState, LegCommand, AttitudePidAxis)
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:1246-1252` — `physical_to_motor_angle()` and `motor_to_physical_angle()` — the angle convention bridge
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:758-767` — `leg_feedback_at_()` — how leg feedback is extracted from joint frame
  - `rmcs_ws/src/rmcs_core/src/hardware/deformable-infantry-omni.cpp` — Hardware component: motor zero-point calibration, joint direction
  - `rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni.yaml:68-109` — Current chassis_controller parameters with geometry values (mass, rod_length, preload, etc.)

  **Acceptance Criteria**:
  - [ ] Code comment block added above `compute_leg_support_force_()` containing all 6 documentation items
  - [ ] Comment confirms that existing `τ = F·L·sin(α)` torque formula is mechanically correct (force arm for vertical force)
  - [ ] Comment notes that existing angle-space spring `Kz*(α-α₀)` is dimensionally wrong for Cartesian suspension intent

  **QA Scenarios**:

  ```
  Scenario: Documentation is accurate and complete
    Tool: Bash (grep)
    Preconditions: File has been modified with comment block
    Steps:
      1. grep -A 30 "Coordinate Convention" deformable_chassis.cpp
      2. Verify output contains: α definition, z_w formula, r_eff formula
      3. grep "Jacobian" deformable_chassis.cpp — verify present
      4. grep "wheel_base_half" deformable_chassis.cpp — verify chassis positions documented
    Expected Result: All 6 documentation items found in comment block
    Failure Indicators: Missing any of the 6 items in grep output
    Evidence: .sisyphus/evidence/task-1-doc-verification.txt
  ```

  **Commit**: YES (groups with Tasks 2-4 as a single Wave-1 commit)
  - Message: `docs(chassis): document suspension coordinate conventions and kinematics baseline`
  - Files: `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp`

- [ ] 2. Derive forward kinematics & chassis pose from leg angles

  **What to do**:
  - Derive the mapping from 4 joint angles α = [α₁,α₂,α₃,α₄] to wheel contact positions and chassis pose
  - For each leg i, wheel position in chassis frame:
    - Vertical: `z_i(α_i) = z_joint - L·cos(α_i)` where z_joint ≈ 0 (joint at chassis reference plane)
    - Small-perturbation linearization: `Δz_i ≈ L·sin(α_i_nom) · Δα_i` where α_i_nom is the operating point
  - For chassis pose (z, θ, φ), assuming all 4 wheels remain on a flat ground plane:
    - Chassis center height: `z_c = (z₁+z₂+z₃+z₄)/4` (assuming symmetric geometry)
    - Pitch: `θ ≈ (z_front_avg - z_back_avg) / (2·wheel_base_half_x_)`
    - Roll: `φ ≈ (z_right_avg - z_left_avg) / (2·wheel_base_half_y_)`
    - Where `z_front_avg = (z_LF+z_RF)/2`, `z_back_avg = (z_LB+z_RB)/2`, etc.
  - Write the derived formulas as a DOC comment block near the new kinematics function location (to be created in Task 5)
  - Note the 1-DOF redundancy: 4 legs for 3 pose DOF → nullspace available for preload

  **Must NOT do**:
  - Do NOT write code yet — this is pure formula derivation as documentation
  - Do NOT assume wheel horizontal positions are fixed — note that changing α also changes wheel x,y positions, but for vertical dynamics this is second-order
  - Do NOT derive Layer 3 (force optimization QP) formulas

  **Recommended Agent Profile**:
  - **Category**: `writing`
    - Reason: Mathematics derivation and documentation with geometric reasoning

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 1, 3, 4)
  - **Blocks**: Task 5
  - **Blocked By**: None

  **References**:
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:875-897` — `compute_leg_support_force_()` — current formula using `kPitchSigns_`, `kRollSigns_`, `active_suspension_wheel_base_half_x_`, `active_suspension_wheel_base_half_y_` — confirms pitch/roll moment arm convention
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:856-873` — `compute_attitude_force_bias_()` — current IMU PID convention: pitch error = `-corrected_pitch`, roll error = `corrected_roll`
  - `plan.md:16-24` — System DOF analysis: 4 actuators - 3 pose DOF = 1 redundant DOF
  - `plan.md:50-53` — Layer 2 target formula: `Δl_i = Δz + x_i·Δθ + y_i·Δφ` — the intended geometric mapping

  **Acceptance Criteria**:
  - [ ] Forward kinematics formula documented: α_i → z_i(α_i) for each leg
  - [ ] Chassis pose formula documented: z, θ, φ from z_i
  - [ ] Small-perturbation Jacobian form documented: `Δz_i = J_i · Δα_i` with `J_i = L·sin(α_i_nom)`
  - [ ] Redundancy noted: 4 inputs → 3 outputs → 1-DOF nullspace

  **QA Scenarios**:

  ```
  Scenario: Derived formulas are dimensionally consistent
    Tool: Bash (grep)
    Preconditions: Formula documentation added to code as comments
    Steps:
      1. grep "z_i" deformable_chassis.cpp — verify z_i = L·cos(α) or equivalent
      2. grep "Jacobian\|J_ij\|J =" deformable_chassis.cpp — verify Jacobian matrix form documented
      3. grep "pseudoinverse\|J+" deformable_chassis.cpp — verify inverse kinematics reference
      4. Manual check: all units consistent (radians for α, meters for z and L)
    Expected Result: All kinematic quantities documented with consistent units
    Evidence: .sisyphus/evidence/task-2-kinematics-doc.txt
  ```

  **Commit**: YES (groups with Tasks 1, 3, 4)
  - Message: `docs(chassis): document forward kinematics from joint angles to chassis pose`
  - Files: `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp`

- [ ] 3. Derive Jacobian & pseudoinverse for IMU-to-leg geometric mapping

  **What to do**:
  - Derive the 3×4 Jacobian matrix J mapping Δα → Δpose where pose = [z, θ, φ]^T
    ```
    J = [∂z/∂α₁   ∂z/∂α₂   ∂z/∂α₃   ∂z/∂α₄  ]
        [∂θ/∂α₁   ∂θ/∂α₂   ∂θ/∂α₃   ∂θ/∂α₄  ]
        [∂φ/∂α₁   ∂φ/∂α₂   ∂φ/∂α₃   ∂φ/∂α₄  ]
    ```
  - Each element:
    - `∂z/∂α_i = L·sin(α_i) / 4` (average contribution to chassis height)
    - `∂θ/∂α_i = sign_pitch_i · L·sin(α_i) / (2·wheel_base_half_x_)` (pitch moment arm)
    - `∂φ/∂α_i = sign_roll_i · L·sin(α_i) / (2·wheel_base_half_y_)` (roll moment arm)
  - For IMU leveling: given measured pose error `e = [Δz_des, Δθ_err, Δφ_err]^T`, compute required Δα correction:
    - `Δα_cmd = J⁺ · e` where J⁺ is the Moore-Penrose pseudoinverse
    - For a 3×4 matrix: `J⁺ = J^T · (J·J^T)⁻¹`
  - The nullspace component: `Δα_null = (I - J⁺·J) · Δα_arbitrary` — can be used to add preload without affecting pose
  - Document the derived formulas as comments near the new IMU mapping function location (to be created in Task 7)

  **Must NOT do**:
  - Do NOT write code yet
  - Do NOT derive for non-flat ground — assume ground is locally planar

  **Recommended Agent Profile**:
  - **Category**: `writing`
    - Reason: Linear algebra derivation and documentation

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 1, 2, 4)
  - **Blocks**: Tasks 6, 7
  - **Blocked By**: None

  **References**:
  - `plan.md:50-53` — Layer 2 formula: `Δl_i = Δz + x_i·Δtheta + y_i·Δphi` — the intended geometric mapping structure
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:856-873` — `compute_attitude_force_bias_()` — current PID approach to be replaced, confirms pitch/roll sign convention
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:875-897` — `compute_leg_support_force_()` — confirms `kPitchSigns_` and `kRollSigns_` usage pattern
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:626-671` — `update_chassis_imu_calibration_()` — IMU offset calibration logic (must preserve this)

  **Acceptance Criteria**:
  - [ ] 3×4 Jacobian J documented with all 12 elements expressed in terms of α_i, L, wheel_base_half
  - [ ] Pseudoinverse J⁺ formula documented
  - [ ] Nullspace projector documented: `N = I - J⁺·J`
  - [ ] Sign convention matches existing `kPitchSigns_` and `kRollSigns_`

  **QA Scenarios**:

  ```
  Scenario: Jacobian derivation is internally consistent
    Tool: Bash (grep)
    Preconditions: Formula documentation added to code
    Steps:
      1. grep "J =" or "Jacobian" deformable_chassis.cpp — verify 3×4 structure
      2. grep "sign_pitch\|kPitchSign" deformable_chassis.cpp — verify sign convention reuse
      3. grep "pseudoinverse\|J+" deformable_chassis.cpp — verify pseudoinverse formula
      4. grep "nullspace\|N =" deformable_chassis.cpp — verify nullspace projector
      5. Manual check: ∂z/∂α_i should all be positive (extending leg = chassis rises)
    Expected Result: All 4 verification items found
    Evidence: .sisyphus/evidence/task-3-jacobian-doc.txt
  ```

  **Commit**: YES (groups with Tasks 1, 2, 4)
  - Message: `docs(chassis): document Jacobian and pseudoinverse for IMU-leveling mapping`
  - Files: `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp`

- [ ] 4. Derive Cartesian virtual impedance & gravity compensation formulas

  **What to do**:
  - Derive the Cartesian-space virtual impedance model to replace the current angle-space spring
  - **Current formula (WRONG — angle-space)**:
    ```
    F_i = mg/4 + Kz*(α_i - α_support_zero) + D_leg*α_dot_i + bias
    ```
  - **New formula (Cartesian-space)**:
    ```
    F_i = F_gravity_i + K_lin*(z_ref - z_i(α_i)) + D_lin*(ż_ref - ż_i(α_i, α_dot_i))
    ```
    where:
    - `z_i(α_i) = -L·cos(α_i)` — wheel vertical position
    - `ż_i = L·sin(α_i)·α_dot_i` — wheel vertical velocity (chain rule: dz/dα · dα/dt)
    - `z_ref` — reference wheel position, typically `z_ref = -L·cos(deploy_angle - preload_angle)`
    - K_lin — linear stiffness [N/m] (replaces Kz [N/rad])
    - D_lin — linear damping [N·s/m] (replaces D_leg [N/rad/s])
  - **Gravity compensation in joint torque space**:
    ```
    τ_gravity = J^T · w_gravity
    ```
    where w_gravity = [mg/4, mg/4, mg/4, mg/4]^T (uniform weight distribution assumption)
    For a revolute joint: τ_gravity_i = (mg/4) · L·sin(α_i) (matches existing formula when α_i equal)
  - Torque output formula (unchanged structure):
    ```
    τ_i = clamp(F_i · L·sin(α_i), ±torque_limit)
    ```
  - Document these formulas as comments near the new support force function location (to be created in Task 6)

  **Must NOT do**:
  - Do NOT change the existing gravity_comp logic that uses `total_force / 4` (keep it as-is, it's already correct)
  - Do NOT write code yet

  **Recommended Agent Profile**:
  - **Category**: `writing`
    - Reason: Physics derivation with clear before/after comparison

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 1, 2, 3)
  - **Blocks**: Tasks 6, 9
  - **Blocked By**: None

  **References**:
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:875-897` — `compute_leg_support_force_()` — current force formula to be replaced
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:899-904` — `leg_force_to_joint_torque_()` — torque conversion formula (keep this, it's mechanically correct: τ = F·L·sin(α))
  - `plan.md:38-46` — Layer 1 impedance control: `τ_i = K_i*(l₀-l_i) + D_i*(-dl_i/dt) + τ_gravity` — the intended Cartesian-space impedance
  - `plan.md:96-113` — ADRC suspension mode parameter mapping: k1 ↔ virtual spring, k2 ↔ virtual damper — confirms intent to treat ADRC NLESF as the impedance executor

  **Acceptance Criteria**:
  - [ ] Cartesian impedance formula documented: F = F_grav + K_lin*(z_ref - z) + D_lin*(ż_ref - ż)
  - [ ] `z_i(α_i)` and `ż_i(α_i, α_dot_i)` formulas documented with chain rule derivation
  - [ ] Gravity compensation documented: τ_g = J^T·w and per-leg equivalent
  - [ ] Comparison table: old (angle-space) vs new (Cartesian-space) with units

  **QA Scenarios**:

  ```
  Scenario: Derived formulas are dimensionally consistent
    Tool: Bash (grep)
    Preconditions: Formula documentation added to code
    Steps:
      1. grep "z_i(α)\|z_ref" deformable_chassis.cpp — verify vertical position formula
      2. grep "ż_i\|dz/dt\|chain.rule" deformable_chassis.cpp — verify velocity formula
      3. grep "K_lin\|D_lin" deformable_chassis.cpp — verify new parameter names
      4. grep "J^T\|transpose.*gravity" deformable_chassis.cpp — verify gravity comp formula
      5. Manual check: K_lin in N/m, D_lin in N·s/m, τ in N·m — dimension check
    Expected Result: All formulas documented with consistent SI units
    Evidence: .sisyphus/evidence/task-4-impedance-doc.txt
  ```

  **Commit**: YES (groups with Tasks 1, 2, 3)
  - Message: `docs(chassis): document cartesian virtual impedance and gravity compensation`
  - Files: `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp`

- [ ] 5. Implement forward kinematics & Jacobian computation functions

  **What to do**:
  - Add a new private member function `compute_wheel_cartesian_state_()` to `DeformableChassis`:
    ```cpp
    struct WheelCartesianState {
        std::array<double, kJointCount> z;       // vertical position [m]
        std::array<double, kJointCount> z_dot;   // vertical velocity [m/s]
        double chassis_z;                         // chassis center height [m]
        double chassis_pitch;                     // rad
        double chassis_roll;                      // rad
    };
    WheelCartesianState compute_wheel_cartesian_state_(const JointFeedbackFrame& joint_feedback) const;
    ```
  - Implementation: for each leg i, compute `z_i = -active_suspension_rod_length_ * cos(α_i)`, `ż_i = active_suspension_rod_length_ * sin(α_i) * α_dot_i`, then chassis pose from average and moment arms
  - Add a new private member function `compute_suspension_jacobian_()`:
    ```cpp
    // Returns 3×4 matrix stored as std::array<std::array<double, 4>, 3>
    // Row 0: ∂z/∂α_i, Row 1: ∂θ/∂α_i, Row 2: ∂φ/∂α_i
    std::array<std::array<double, kJointCount>, 3> compute_suspension_jacobian_(
        const std::array<double, kJointCount>& physical_angles) const;
    ```
  - Add a new private member function `compute_jacobian_pseudoinverse_()`:
    ```cpp
    // Returns 4×3 matrix (J⁺) for mapping pose error → leg angle correction
    std::array<std::array<double, 3>, kJointCount> compute_jacobian_pseudoinverse_(
        const std::array<std::array<double, kJointCount>, 3>& J) const;
    ```
    - Implement 3×3 matrix inversion `(J·J^T)⁻¹` using explicit Cramer's rule (small fixed-size, no Eigen dependency needed for this)
    - Then compute J⁺ = J^T · (J·J^T)⁻¹
  - Guard against singularities: `sin(α_i) < 1e-6` → clamp to minimum (α near 0 has no vertical force transmission)

  **Must NOT do**:
  - Do NOT use Eigen or external matrix libraries — implement fixed-size 3×3 inversion manually
  - Do NOT modify the existing `kPitchSigns_` and `kRollSigns_` arrays — reuse them
  - Do NOT call these functions from anywhere yet — Task 9 wires them in

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: Requires careful implementation of kinematics math with correct sign conventions and singularity handling
  - **Skills**: `[]`

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 6, 7, 8)
  - **Blocks**: Task 9
  - **Blocked By**: Tasks 1, 2

  **References**:
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:758-767` — `leg_feedback_at_()` — pattern for extracting per-leg data from JointFeedbackFrame
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:1246-1252` — `physical_to_motor_angle()`, `motor_to_physical_angle()` — angle conversion utilities to use for consistency
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:875-897` — `compute_leg_support_force_()` — shows how `kPitchSigns_`, `kRollSigns_`, `wheel_base_half_x_`, `wheel_base_half_y_` are accessed
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:39-46` — `JointFeedbackFrame` struct — the input data structure
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:25-36` — `JointIndex` enum and `kJointCount`

  **Acceptance Criteria**:
  - [ ] `compute_wheel_cartesian_state_()` implemented: 4 z_i, 4 ż_i, chassis_z, chassis_pitch, chassis_roll from 4 α_i
  - [ ] `compute_suspension_jacobian_()` implemented: 3×4 matrix with correct signs matching `kPitchSigns_`/`kRollSigns_`
  - [ ] `compute_jacobian_pseudoinverse_()` implemented: correct 4×3 output, singularity guard for sin(α) ≈ 0
  - [ ] All new functions handle NaN inputs gracefully (return NaN-filled outputs)

  **QA Scenarios**:

  ```
  Scenario: Forward kinematics produces physically reasonable values
    Tool: Bash (compile then Python verification)
    Preconditions: Code compiles. Create a small test harness or use static verification
    Steps:
      1. For α = [8°, 8°, 8°, 8°] (all at min_angle), L=0.15m:
         - Expected z_i = -0.15*cos(8°) ≈ -0.1485m for all legs
         - chassis_z ≈ -0.1485m, pitch ≈ 0, roll ≈ 0
      2. For α = [8°, 58°, 8°, 58°] (front down, back up):
         - z_front = -0.1485m, z_back = -0.15*cos(58°) ≈ -0.0795m
         - pitch = (z_front - z_back)/(2*a) should be positive (nose down)
      3. Verify Jacobian signs: ∂z/∂α_i all positive (extending leg raises chassis)
    Expected Result: Computed values within 1% of expected
    Failure Indicators: Wrong sign on any Jacobian element, NaN propagation
    Evidence: .sisyphus/evidence/task-5-kinematics-verification.txt
  ```

  **Commit**: YES (groups with Tasks 6, 7, 8)
  - Message: `feat(chassis): implement forward kinematics and Jacobian for suspension`
  - Files: `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp`

- [ ] 6. Implement Cartesian impedance support force function

  **What to do**:
  - Add a new private member function `compute_cartesian_support_force_()`:
    ```cpp
    double compute_cartesian_support_force_(size_t index, double z_current, double z_dot_current,
        double z_ref, double z_dot_ref, const AttitudeBias& attitude_bias) const;
    ```
  - Implementation:
    ```cpp
    double compute_cartesian_support_force_(...) {
        // Gravity compensation per leg
        const double gravity_force = active_suspension_gravity_comp_gain_
                                   * active_suspension_mass_ * kGravity_
                                   / static_cast<double>(kJointCount);
        // Cartesian spring: force = K_lin * (z_ref - z_current)
        const double spring_force = active_suspension_Kz_linear_
                                  * (z_ref - z_current);
        // Cartesian damper: force = D_lin * (ż_ref - ż_current)  
        const double damping_force = active_suspension_D_leg_linear_
                                   * (z_dot_ref - z_dot_current);
        // IMU attitude bias (pitch/roll force redistribution)
        const double attitude_force = kPitchSigns_[index] * attitude_bias.pitch_force
                                    + kRollSigns_[index] * attitude_bias.roll_force;
        // Acceleration feedforward (keep existing logic but in Cartesian)
        double accel_force = 0.0;
        if (active_suspension_com_height_ > 0.0 && ...) {
            // existing acceleration bias formula — keep unchanged
            accel_force = ...;
        }
        double total_force = gravity_force + spring_force + damping_force
                           + attitude_force + accel_force;
        return std::max(total_force, 0.0);
    }
    ```
  - Add new member variables in the class declaration:
    ```cpp
    double active_suspension_Kz_linear_ = 0.0;     // [N/m] linear stiffness
    double active_suspension_D_leg_linear_ = 0.0;  // [N·s/m] linear damping
    double active_suspension_z_ref_ = 0.0;          // [m] reference wheel vertical position
    ```
  - Add parameter loading in `DeformableChassis` constructor (or wherever existing suspension params are loaded):
    ```cpp
    active_suspension_Kz_linear_ = load_parameter_or(*this, "active_suspension_Kz_linear", 0.0);
    active_suspension_D_leg_linear_ = load_parameter_or(*this, "active_suspension_D_leg_linear", 0.0);
    ```
  - Compute `z_ref` once per cycle: `z_ref = -active_suspension_rod_length_ * cos(deploy_angle - active_suspension_preload_angle_)`
  - Keep old `active_suspension_Kz_` and `active_suspension_D_leg_` member variables BUT mark them as deprecated with a comment — remove their usage from active code path

  **Must NOT do**:
  - Do NOT delete old `active_suspension_Kz_` or `active_suspension_D_leg_` member variables — just stop using them in the active path
  - Do NOT change the torque output formula (`τ = F·L·sin(α)`) — it's correct
  - Do NOT remove the attitude bias or acceleration feedforward terms — only change the spring-damper core from angle-space to Cartesian

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: Physics implementation with multiple interacting force terms and unit conversion care

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 5, 7, 8)
  - **Blocks**: Task 9
  - **Blocked By**: Tasks 1, 3, 4

  **References**:
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:875-897` — `compute_leg_support_force_()` — the function being replaced, shows current gravity, spring, damping, attitude, accel terms
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:899-904` — `leg_force_to_joint_torque_()` — torque conversion (keep this unchanged)
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:856-873` — `compute_attitude_force_bias_()` — shows pitch_force/roll_force generation and sign convention
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:985-1016` — `update_active_suspension_()` — shows where `support_zero_angle` is computed from `deploy_angle - preload_angle` (this becomes `z_ref` computation)
  - `rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni.yaml:68-87` — current suspension geometry params to map to new linear units

  **Acceptance Criteria**:
  - [ ] `compute_cartesian_support_force_()` implemented with all 5 force components
  - [ ] New member variables `active_suspension_Kz_linear_`, `active_suspension_D_leg_linear_` declared
  - [ ] Parameter loading for new linear parameters in constructor
  - [ ] `z_ref` computed correctly from deploy_angle and preload_angle using cosine
  - [ ] Old `active_suspension_Kz_` and `active_suspension_D_leg_` still loaded but commented as "deprecated — replaced by linear variants"

  **QA Scenarios**:

  ```
  Scenario: Cartesian force model produces correct static equilibrium
    Tool: Bash (compile check + static verification)
    Preconditions: Code compiles
    Steps:
      1. Set α = 8° (deploy_angle), preload = 0°, L = 0.15m:
         z_current = -0.15*cos(8°) ≈ -0.1485m
         z_ref = z_current (no preload)
         spring_force = K_lin * (z_ref - z_current) = 0 ✓
      2. Set α = 8°, preload = 8°, L = 0.15m:
         z_current = -0.15*cos(8°) ≈ -0.1485m
         z_ref = -0.15*cos(8°-8°) = -0.15*cos(0°) = -0.15m
         spring_force = K_lin * (-0.15 - (-0.1485)) = K_lin * (-0.0015) < 0
         → total force = gravity + negative spring → less support
         Wait, this means preload REDUCES force? That's the bug the user found!
         → The correct formula should be: z_ref = -L*cos(deploy - preload)
         → At deploy angle: z_current ≥ z_ref, so spring force ≥ 0 (proper preload!) ✓
      3. Verify: with K_lin=15000 N/m, preload=8°, at α=8°:
         spring_force = 15000 * (-0.15 - (-0.1485)) = 15000 * (-0.0015) = -22.5N
         Hmm, still negative. Let me reconsider...
         Actually: z_ref should be MORE negative than z_current for positive preload.
         If we want preload: z_ref < z_current, so (z_ref - z_current) < 0 → negative spring force.
         But we want positive preload! So the sign convention needs: spring_force = K_lin * (z_current - z_ref)
         Or: z_ref = z(deploy_angle) and z_current = z(deploy_angle - preload) — the wheel is pushed further down
         → This task must verify the force DIRECTION is correct for preload
    Expected Result: spring_force > 0 when wheel is pushed below reference (compression = positive support)
    Failure Indicators: Preload sign wrong → less support at min_angle instead of more
    Evidence: .sisyphus/evidence/task-6-cartesian-force-verification.txt
  ```

  **Commit**: YES (groups with Tasks 5, 7, 8)
  - Message: `feat(chassis): implement cartesian-space virtual impedance force model`
  - Files: `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp`

- [ ] 7. Implement geometric IMU-to-leg-displacement mapping

  **What to do**:
  - Replace the PID-based `compute_attitude_force_bias_()` with a geometric mapping approach
  - Add a new private member function `compute_pose_error_from_imu_()`:
    ```cpp
    struct PoseError {
        double z_error = 0.0;      // [m] desired height change (typically 0)
        double pitch_error = 0.0;  // [rad] negative of corrected_pitch (level = 0)
        double roll_error = 0.0;   // [rad] corrected_roll (level = 0)
    };
    PoseError compute_pose_error_from_imu_() const;
    ```
  - From IMU: `pitch_error = -(*chassis_imu_pitch_ - chassis_imu_pitch_offset_)`, `roll_error = *chassis_imu_roll_ - chassis_imu_roll_offset_`
  - Add a new private member function `compute_leg_angle_correction_from_pose_error_()`:
    ```cpp
    std::array<double, kJointCount> compute_leg_angle_correction_from_pose_error_(
        const PoseError& pose_error,
        const std::array<std::array<double, kJointCount>, 3>& J,
        const std::array<std::array<double, 3>, kJointCount>& J_pinv) const;
    ```
  - Implementation: `Δα_cmd = J_pinv * [z_error, pitch_error, roll_error]^T` (4×3 · 3×1 = 4×1)
  - Convert the Δα correction to a force bias using the Cartesian impedance: `ΔF_i = K_lin * (L·sin(α_i) * Δα_cmd_i)` — this approaches it as: the pose error needs a leg angle correction, and the spring force for that correction is K_lin * Δz = K_lin * J_i * Δα_i
  - Alternatively (simpler): keep `compute_attitude_force_bias_()` but change its output from an ad-hoc PID on pitch/roll to a geometric force redistribution:
    - Compute desired force redistribution to counteract pitch error: total torque about y-axis needed = K_attitude_pitch * pitch_error. The front-back force difference = torque / wheel_base
    - This is similar to the existing approach but with a geometric foundation instead of pure PID
  - Actually, the simplest correct approach: add attitude force bias to `compute_cartesian_support_force_()` using the geometric mapping. Keep the PID for now but change the PID gains to be geometrically meaningful:
    ```cpp
    // Geometric interpretation: pitch error θ_err → required torque τ_y = K_attitude * θ_err
    // Required force difference between front and back: ΔF_pitch = τ_y / (2 * wheel_base_half_x_)
    // Per-leg contribution: ±ΔF_pitch/2
    ```
  - Update `AttitudePidAxis` usage: the PID gains should now represent `K_attitude` [N·m/rad] (torque per radian of error), not arbitrary force units

  **Must NOT do**:
  - Do NOT remove the IMU calibration logic (`update_chassis_imu_calibration_()`) — it's separate and working
  - Do NOT change the PID structure unnecessarily — the change is in SEMANTICS (from arbitrary force PID to geometric torque PID), not structure

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: Requires bridging IMU pitch/roll measurements with geometric force redistribution

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 5, 6, 8)
  - **Blocks**: Task 9
  - **Blocked By**: Tasks 1, 3

  **References**:
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:856-873` — `compute_attitude_force_bias_()` — current PID approach (replace/add geometric interpretation)
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:626-671` — `update_chassis_imu_calibration_()` — IMU offset calibration (do NOT modify)
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:82-100` — `AttitudePidAxis` struct — the PID structure to keep
  - `rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni.yaml:89-103` — IMU PID parameters to update with geometric interpretation comments
  - `plan.md:48-54` — Layer 2 formula `Δl_i = Δz + x_i·Δθ + y_i·Δφ` — the intended geometric mapping

  **Acceptance Criteria**:
  - [ ] IMU pitch/roll errors converted to chassis-level torque demands (K_attitude_pitch · pitch_error = τ_y in N·m)
  - [ ] Torque demands mapped to per-leg force bias via moment arm (wheel_base_half)
  - [ ] Per-leg sign convention verified: pitch_nose_up → front legs need MORE force, rear legs need LESS
  - [ ] YAML comment updated: IMU gains documented as `[N·m/rad]` not arbitrary units

  **QA Scenarios**:

  ```
  Scenario: IMU leveling forces have correct geometric signs
    Tool: Bash (compile + static analysis)
    Preconditions: Code compiles
    Steps:
      1. Pitch > 0 (nose up): front wheels should get MORE force, rear LESS
         Verify: pitch_force_bias * kPitchSigns_[LF] > 0 (front left gets positive bias)
         Verify: pitch_force_bias * kPitchSigns_[LB] < 0 (back left gets negative bias)
      2. Roll > 0 (right side up): right wheels should get MORE force, left LESS
         Verify: roll_force_bias * kRollSigns_[RF] > 0
         Verify: roll_force_bias * kRollSigns_[LF] < 0
      3. Both pitch=roll=0: attitude_bias = 0 for all legs
    Expected Result: Sign convention matches physical intuition
    Failure Indicators: Wrong sign → IMU corrections amplify error instead of reducing it
    Evidence: .sisyphus/evidence/task-7-imu-mapping-signs.txt
  ```

  **Commit**: YES (groups with Tasks 5, 6, 8)
  - Message: `feat(chassis): implement geometric IMU-to-leg-force mapping`
  - Files: `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp`, `rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni.yaml`

- [ ] 8. Update YAML configuration parameters for new formula

  **What to do**:
  - In `deformable-infantry-omni.yaml`, under `chassis_controller`:
    - ADD new parameters:
      ```yaml
      # Cartesian suspension stiffness and damping (replaces angle-space Kz/D_leg)
      active_suspension_Kz_linear: 15000.0    # [N/m] linear stiffness in wheel vertical direction
      active_suspension_D_leg_linear: 200.0   # [N·s/m] linear damping in wheel vertical direction
      ```
    - KEEP old parameters but add deprecation comment:
      ```yaml
      # Deprecated — replaced by active_suspension_Kz_linear and D_leg_linear
      # active_suspension_Kz: 150.0
      # active_suspension_D_leg: 10.0
      ```
    - UPDATE IMU parameter comments:
      ```yaml
      # IMU attitude correction gains — interpreted as [N·m/rad] (torque per radian of chassis tilt)
      active_suspension_Kp: 8.0     # pitch torque proportional gain
      active_suspension_Kr: 8.0     # roll torque proportional gain
      ```
    - ADD geometry parameters if not already present:
      ```yaml
      active_suspension_wheel_base_x: 0.48   # [m] full wheel base in x (front-back)
      active_suspension_wheel_base_y: 0.42   # [m] full wheel base in y (left-right)
      ```
    - But check: `active_suspension_wheel_base_half_x_` and `wheel_base_half_y_` are already loaded — verify the parameter names in the constructor
  - Compute initial K_lin from old Kz for continuity:
    - At α ≈ 8° (typical operating point): sin(8°) ≈ 0.139
    - Old: F = Kz_angle * (α₂-α₁), τ = F * L * sin(α)
    - New: F = K_lin * (z₂-z₁) = K_lin * L * (cos(α₁)-cos(α₂))
    - For small Δα: K_lin ≈ Kz_angle / (L² * sin²(α)) — at α=8°: 150 / (0.0225 * 0.0193) ≈ 345,000 N/m (way too high)
    - Better: choose K_lin such that at α=8°, a 1° angle change (≈ 0.0175 rad) produces similar force
    - Old: ΔF = Kz * Δα = 150 * 0.0175 = 2.625 N → small (this is why suspension feels soft!)
    - New: ΔF = K_lin * L * sin(α) * Δα = K_lin * 0.15 * 0.139 * 0.0175 = K_lin * 3.65e-4
    - For same ΔF: K_lin = 2.625 / 3.65e-4 ≈ 7200 N/m
    - Initial estimate: K_lin ≈ 10000 N/m, D_lin ≈ 500 N·s/m (to be tuned)

  **Must NOT do**:
  - Do NOT remove old parameters — only comment them as deprecated
  - Do NOT change non-suspension parameters (wheel controller, gimbal, etc.)
  - Do NOT guess K_lin/D_lin final values — use initial estimates with comment "tune based on vehicle testing"

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: YAML editing with parameter calculation — straightforward once formulas are derived

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 5, 6, 7)
  - **Blocks**: Tasks 9, 10
  - **Blocked By**: Tasks 1, 4

  **References**:
  - `rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni.yaml:68-109` — chassis_controller parameters to modify
  - `rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni.yaml:207-255` — lf_joint_controller ADRC parameters (do NOT modify)
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:875-897` — `compute_leg_support_force_()` — confirms which member variables need YAML counterparts
  - `code-plan.md:275-323` — Step 3 YAML update reference — shows parameter naming convention

  **Acceptance Criteria**:
  - [ ] `active_suspension_Kz_linear` and `active_suspension_D_leg_linear` added with reasonable initial values
  - [ ] Old `active_suspension_Kz` and `active_suspension_D_leg` commented as deprecated
  - [ ] IMU PID parameter comments updated with [N·m/rad] unit annotation
  - [ ] YAML syntax valid (run `python3 -c "import yaml; yaml.safe_load(open('deformable-infantry-omni.yaml'))"`)

  **QA Scenarios**:

  ```
  Scenario: YAML loads correctly with new parameters
    Tool: Bash (python3 yaml)
    Preconditions: File modifications complete
    Steps:
      1. python3 -c "import yaml; d=yaml.safe_load(open('rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni.yaml')); cc=d['chassis_controller']['ros__parameters']; print('Kz_linear:', cc.get('active_suspension_Kz_linear')); print('D_leg_linear:', cc.get('active_suspension_D_leg_linear'))"
      2. Verify Kz_linear > 0 and D_leg_linear > 0
      3. Verify old parameters still present (backward compat)
    Expected Result: New parameters loadable, old parameters preserved
    Failure Indicators: YAML syntax error, missing keys, string where number expected
    Evidence: .sisyphus/evidence/task-8-yaml-verification.txt
  ```

  **Commit**: YES (groups with Tasks 5, 6, 7)
  - Message: `feat(config): add cartesian suspension parameters, deprecate angle-space params`
  - Files: `rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni.yaml`

- [ ] 9. Refactor update_active_suspension_() to use new formulas

  **What to do**:
  - The function at line 985-1016 orchestrates the full suspension cycle. Modify it to use the new Cartesian formulas:
  - Replace the old `compute_leg_support_intents_()` internal logic:
    - Before calling `compute_leg_support_intents_()`, call `compute_wheel_cartesian_state_()` to get z_i, ż_i for all 4 legs
    - Compute `z_ref = -L·cos(deploy_angle - preload_angle)` once
    - Compute Jacobian and pseudoinverse: call `compute_suspension_jacobian_()`, `compute_jacobian_pseudoinverse_()`
    - Compute pose error from IMU: `compute_pose_error_from_imu_()`
    - Compute leg angle correction: `compute_leg_angle_correction_from_pose_error_()`
    - Pass Cartesian state (z_i, ż_i, z_ref) to the new `compute_cartesian_support_force_()` instead of angle-space
  - Wire the new flow into `compute_leg_support_intents_()`:
    ```cpp
    // Inside compute_leg_support_intents_():
    for (size_t index = 0; index < kJointCount; ++index) {
        if (leg_state.phase != SuspensionPhase::kActive) continue;
        
        // Convert pose error Δα to a Cartesian force bias:
        // The pose error Δα_cmd from geometric IMU mapping represents desired leg movement
        // Convert to force via spring: ΔF_attitude = K_lin * (L*sin(α_i) * Δα_cmd_i)
        AttitudeBias attitude_force_bias = ...; // from geometric mapping
        
        leg_state.support_force = compute_cartesian_support_force_(
            index, cartesian_state.z[index], cartesian_state.z_dot[index],
            z_ref, 0.0, // z_dot_ref = 0 (we want wheel to settle)
            attitude_force_bias);
        
        leg_commands_[index].final_target_angle = ride_height_angle;
        leg_commands_[index].suspension_mode = true;
        leg_commands_[index].suspension_torque =
            leg_force_to_joint_torque_(leg_state.support_force, leg_feedback.physical_angle);
    }
    ```
  - Keep the existing `clear_suspension_output_interfaces_()` and `prepare_leg_commands_for_cycle_()` calls at the top (unchanged)
  - Keep the existing state machine update (`update_leg_states_()`) — it only manages phase transitions, doesn't care about the force formula
  - Keep the existing `publish_suspension_outputs_()` at the end

  **Must NOT do**:
  - Do NOT change the state machine (kInactive→kArming→kActive→kReleasing) — it's correct
  - Do NOT change the entry/release angle logic — only the FORCE computation within active state
  - Do NOT remove the `prone_override_requested_()` or `suspension_requested_by_input_()` checks

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: Complex integration task — wiring 4 new functions into an existing pipeline without breaking the state machine or output path
  - **Skills**: `[]`

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Wave 3 (sequential — depends on all Wave 2 tasks)
  - **Blocks**: Task 10
  - **Blocked By**: Tasks 5, 6, 7, 8

  **References**:
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:985-1016` — `update_active_suspension_()` — the function being refactored
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:832-848` — `compute_leg_support_intents_()` — the inner loop to modify
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:820-830` — `update_leg_states_()` — the state machine (do NOT modify)
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:491-517` — `clear_suspension_output_interfaces_()` and `prepare_leg_commands_for_cycle_()` — output clearing (do NOT modify)
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:803-818` — `update_leg_contact_estimates_()` — contact estimation (do NOT modify)

  **Acceptance Criteria**:
  - [ ] New Cartesian functions are wired into the active suspension pipeline
  - [ ] Call order: clear outputs → prepare commands → check input → compute Cartesian state → compute Jacobian → IMU pose error → angle correction → per-leg support force → torque → publish
  - [ ] State machine, contact estimation, output publishing all unchanged
  - [ ] Old `compute_leg_support_force_()` still present but no longer called from active path (keep for reference)

  **QA Scenarios**:

  ```
  Scenario: Refactored pipeline produces valid torque outputs
    Tool: Bash (colcon build)
    Preconditions: All Wave 2 tasks complete
    Steps:
      1. colcon build --packages-select rmcs_core 2>&1 | tee build.log
      2. grep -i "error" build.log — should be NONE
      3. grep -i "warning" build.log — review any new warnings
      4. grep "unused" build.log — verify no unused variable warnings from old code
    Expected Result: Clean compile with 0 errors, ≤ 2 new warnings
    Failure Indicators: Compile error, linker error, undefined reference to new functions
    Evidence: .sisyphus/evidence/task-9-build-output.log
  ```

  **Commit**: YES (groups with Tasks 10, 11)
  - Message: `feat(chassis): wire cartesian suspension formulas into active suspension pipeline`
  - Files: `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp`

- [ ] 10. Build, fix compile errors, static verification

  **What to do**:
  - Run `build-rmcs --packages-select rmcs_core` (or `colcon build --packages-select rmcs_core` if build-rmcs unavailable)
  - Fix any compile errors from the new code:
    - Missing includes (e.g., `<array>`, `<cmath>`, `<algorithm>` — likely already present)
    - Undefined member variables (add declarations in class body)
    - Type mismatches (double vs float, const vs non-const)
    - Lambda capture issues if any
    - Unused variable warnings from deprecated old params — add `[[maybe_unused]]` or comment them
  - Run `clang-tidy` on deformable_chassis.cpp if available, or manual review:
    - Check for: integer division where float expected, missing `std::` prefixes, uninitialized variables
    - Verify: all `sin()`, `cos()`, `clamp()` calls use `std::`
    - Verify: `nan_` used consistently for invalid/disabled values
  - Verify the new code path doesn't trigger undefined behavior:
    - Division by `wheel_base_half_x_` or `wheel_base_half_y_` — guard with `> 1e-6` check
    - `sin(α)` near zero — guard with `kMinForceArmSin_` (likely already exists)
    - Matrix inversion singularity — guard: `det(J·J^T) < 1e-12` → fall back to diagonal pseudoinverse
  - Update the YAML comment with recommended build command if different from standard

  **Must NOT do**:
  - Do NOT modify any file outside `deformable_chassis.cpp` and `deformable-infantry-omni.yaml` unless necessary for compilation
  - Do NOT suppress warnings with flags — fix the root cause

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Build-and-fix cycle — mechanical compile error resolution, straightforward once errors are identified

  **Parallelization**:
  - **Can Run In Parallel**: NO (depends on Task 9)
  - **Parallel Group**: Wave 3 (sequential after Task 9)
  - **Blocks**: Task 12
  - **Blocked By**: Task 9

  **References**:
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp` — the file being compiled
  - `rmcs_ws/src/rmcs_core/CMakeLists.txt` — build configuration, verify no new source files need registration
  - Build output from Task 9 if it failed

  **Acceptance Criteria**:
  - [ ] `colcon build --packages-select rmcs_core` succeeds with 0 errors
  - [ ] 0 new warnings (or all new warnings reviewed and documented as acceptable)
  - [ ] All guard conditions in place: wheel_base > 1e-6, sin(α) > kMinForceArmSin_, det > 1e-12

  **QA Scenarios**:

  ```
  Scenario: Clean build with static analysis
    Tool: Bash
    Preconditions: Task 9 code changes applied
    Steps:
      1. colcon build --packages-select rmcs_core 2>&1 | tee build-output.log
      2. Check exit code = 0
      3. grep -c "error:" build-output.log → 0
      4. grep -c "warning:" build-output.log → count and review
      5. grep -n "division by\|uninitialized\|unused" build-output.log
    Expected Result: Build success, 0 errors, ≤ 3 warnings (reviewed)
    Failure Indicators: Non-zero exit code, errors in output
    Evidence: .sisyphus/evidence/task-10-build-clean.log
  ```

  **Commit**: YES (groups with Tasks 9, 11) — amend the Task 9 commit if fixups are small
  - Message: `fix(chassis): resolve compile errors from suspension formula refactor`
  - Files: `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp`

- [ ] 11. Frequency sweep parameter identification

  **What to do**:
  - Run frequency sweep on all 4 joints using the existing sweep infrastructure (`DeformableJointSweepController` + `DeformableJointSweepRecorder`)
  - Procedure:
    1. Set robot on blocks (wheels off ground) to avoid ground interaction during sweep
    2. Configure sweep: chirp signal 0.5-30 Hz, amplitude ±5° around nominal operating angle (e.g., 30°)
    3. Run sweep for each joint individually (~30 seconds per joint)
    4. Collect CSV data from recorder (includes eso_z2, eso_z3 columns)
  - Create a Python script `scripts/identify_b0.py`:
    ```python
    # For each joint CSV:
    # 1. Read control_torque (u) and physical_angle (y) columns
    # 2. Compute angular acceleration: y_ddot = np.gradient(np.gradient(y, dt), dt)
    # 3. Remove sections where y_ddot < threshold (near-static segments have poor SNR)
    # 4. Simple estimate: b0 = median(y_ddot / u) during active sweep segments
    # 5. Frequency-domain refinement: compute transfer function H(jω) = Y(jω)/U(jω)
    #    b0 = 1/|H(jω_low)| where ω_low is the low-frequency gain plateau
    # 6. Output: b0 estimate ± confidence interval for each joint
    ```
  - Update YAML b0 values for each joint controller based on identified values
  - From b0, derive recommended suspension parameters:
    - `suspension_k1 ≈ K_desired * b0` where K_desired is desired virtual stiffness (N·m/rad) at the joint
    - From b0 and system bandwidth, verify `suspension_eso_w0` is appropriate (~3-5× system bandwidth)

  **Must NOT do**:
  - Do NOT change ADRC parameters other than b0 unless sweep data clearly justifies it
  - Do NOT run sweep with wheels on ground — ground contact corrupts the open-loop identification

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: Hardware operation + data processing + parameter derivation — requires physical robot access and numerical analysis
  - **Skills**: `[]`

  **Parallelization**:
  - **Can Run In Parallel**: NO (depends on Task 10 for compiled code; b0 is needed for final parameter tuning)
  - **Parallel Group**: Wave 3 (sequential after Task 10)
  - **Blocks**: Task 12
  - **Blocked By**: Task 10

  **References**:
  - `code-plan.md:338-378` — Step 4: sweep identification procedure with b0 estimation methods
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_joint_controller.cpp` — ADRC parameters to update (b0, suspension_k1, suspension_k2)
  - `rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni.yaml:207-255` — Joint controller YAML where b0 lives (currently -0.60)
  - `deformable_infantry_omni_active_suspension_flow.md:183-196` — Sweep controller and recorder architecture

  **Acceptance Criteria**:
  - [ ] Sweep data collected for all 4 joints (4 CSV files)
  - [ ] `identify_b0.py` script produces b0 estimate for each joint
  - [ ] b0 values updated in YAML if estimates differ significantly from current -0.60
  - [ ] Recommended suspension_k1/k2 values computed and documented

  **QA Scenarios**:

  ```
  Scenario: b0 identification produces consistent estimates across joints
    Tool: Bash (python3)
    Preconditions: Sweep CSV files available at known paths
    Steps:
      1. python3 scripts/identify_b0.py sweep_lf.csv sweep_lb.csv sweep_rb.csv sweep_rf.csv
      2. Verify all 4 b0 estimates have same sign (all negative or all positive)
      3. Verify coefficient of variation across 4 estimates < 30%
      4. Verify |b0| is within reasonable range (0.1 to 5.0 for rad/s² per N·m)
    Expected Result: 4 b0 estimates with consistent sign and reasonable magnitude
    Failure Indicators: Widely divergent estimates, sign mismatch between joints, |b0| < 0.01
    Evidence: .sisyphus/evidence/task-11-b0-estimates.txt
  ```

  **Commit**: YES (groups with Tasks 9, 10)
  - Message: `feat(tools): add b0 identification script and update sweep-estimated parameters`
  - Files: `scripts/identify_b0.py`, `rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni.yaml`

- [ ] 12. Deploy & static loading test (gravity compensation verification)

  **What to do**:
  - Build and deploy to MiniPC:
    ```bash
    build-rmcs && wait-sync && attach-remote -r
    ```
  - Static loading test on level ground:
    1. Place robot on flat ground
    2. Activate suspension (switch combination)
    3. Verify via `value_broadcaster` that all 4 `suspension_mode` topics are `true`
    4. Verify that all 4 `suspension_torque` values are non-zero and positive (providing upward support)
    5. Check `eso_z3` values: should be similar across all 4 joints (uniform loading)
    6. Check IMU: calibrated pitch/roll should be near 0° (±2°)
  - Gravity compensation verification:
    1. With suspension active, robot should hold itself at ride height
    2. Manually push down on chassis center: should feel spring resistance (K_lin effect)
    3. Release: chassis should return to ride height (no permanent sag)
    4. Check that `physical_angle` of all 4 legs stabilizes near `deploy_angle + ride_height_offset` (±3°)
  - Log all relevant topics during the test for later analysis

  **Must NOT do**:
  - Do NOT run on uneven terrain yet — verify basics on flat ground first
  - Do NOT skip the IMU calibration phase — make sure `chassis_imu_calibration_` completes

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: On-vehicle testing requires physical robot access, real-time log monitoring via `attach-remote`, and physical interaction (pushing chassis)

  **Parallelization**:
  - **Can Run In Parallel**: NO (depends on Task 11 for tuned parameters)
  - **Parallel Group**: Wave 4 (sequential after Task 11)
  - **Blocks**: Task 13
  - **Blocked By**: Tasks 10, 11

  **References**:
  - `rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni.yaml:34-53` — `value_broadcaster` forward_list — which topics are broadcast for monitoring
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:985-1016` — `update_active_suspension_()` — the function being tested
  - `README.md` — deploy and sync procedures (`sync-remote`, `attach-remote`)
  - `deformable_infantry_omni_active_suspension_flow.md:94-101` — Command flow: Chassis → JointController → BottomBoard → Motors

  **Acceptance Criteria**:
  - [ ] Robot holds itself at ride height on flat ground (no collapse)
  - [ ] All 4 `suspension_mode` = true when suspension active
  - [ ] All 4 `suspension_torque` values within ±80 N·m (within torque_limit)
  - [ ] IMU pitch/roll near 0° after calibration (within ±2°)
  - [ ] Physical push test: spring resistance felt, returns to ride height
  - [ ] eso_z3 values show relatively uniform loading (±30% variation)

  **QA Scenarios**:

  ```
  Scenario: Gravity compensation holds robot weight on flat ground
    Tool: interactive_bash (tmux via attach-remote -r)
    Preconditions: Robot on flat ground, IMU calibrated, suspension active
    Steps:
      1. Wait 5 seconds for suspension to stabilize
      2. Log: value_broadcaster output for /chassis/*/suspension_torque
      3. Check: all 4 torque values between 5 and 60 N·m (not zero, not saturated)
      4. Log: /chassis/*/eso_z3 for all 4 joints
      5. Check: eso_z3 values within ±30% of each other
      6. Log: /chassis/imu/pitch, /chassis/imu/roll
      7. Push down on center of chassis with hand (~5kg force)
      8. Check: physical_angles increase (legs compress), torque increases
      9. Release: physical_angles return to near original values
    Expected Result: Stable suspension, uniform loading, spring response to push
    Failure Indicators: Any leg collapsing, torque saturating, large IMU offset, no spring response
    Evidence: .sisyphus/evidence/task-12-static-test-log.txt
  ```

  **Commit**: NO (verification task — only commits if YAML parameters are tuned)
  - If YAML params changed: `tune(chassis): adjust suspension parameters from static loading test`

- [ ] 13. Uneven terrain test (wheel contact & leveling verification)

  **What to do**:
  - Test on uneven terrain:
    1. Place robot with one wheel on a ~3-5cm raised surface (simulate obstacle)
    2. Activate suspension
    3. Verify: the raised-leg physical_angle increases (leg compresses)
    4. Verify: the other 3 legs maintain contact (physical_angle decreases = legs extend to reach ground)
    5. Verify via eso_z3: all 4 legs show significant z3 values (all in contact, no free-spinning wheel)
    6. Verify IMU: chassis pitch/roll should adjust toward level (within ±5° of level)
  - Test on ramp:
    1. Place robot on ~10-15° incline
    2. Activate suspension
    3. Verify: uphill legs compress more (higher physical_angle), downhill legs extend (lower physical_angle)
    4. Verify: chassis remains roughly level (IMU pitch/roll compensates for ramp angle)
    5. Verify: all 4 wheels maintain ground contact (no wheel lifts off ground)
  - Log all relevant topics during each scenario

  **Must NOT do**:
  - Do NOT test on extreme terrain (>30° incline or >10cm obstacles) until basic uneven terrain works
  - Do NOT drive the robot during this test — static positioning only

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: Physical test requiring setting up terrain, observing robot behavior, interpreting sensor data

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Wave 4 (sequential after Task 12)
  - **Blocks**: Task 14
  - **Blocked By**: Task 12

  **References**:
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:803-818` — `update_leg_contact_estimates_()` — contact confidence from eso_z3 (used to verify contact)
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp:769-781` — `estimate_contact_confidence_()` — contact detection formula
  - `plan.md:9-10` — Core effect: "wheel floats when going over obstacle → joint makes wheel touch ground"
  - `deformable_infantry_omni_active_suspension_flow.md:103-133` — Active suspension subprocess flow

  **Acceptance Criteria**:
  - [ ] All 4 wheels maintain ground contact on 3-5cm obstacle (eso_z3 confirms loading)
  - [ ] Chassis pitch/roll within ±5° of level on 10-15° ramp
  - [ ] No leg hits max_angle or min_angle mechanical limits during test
  - [ ] Suspension torque not saturated (±80 N·m) during steady-state test conditions

  **QA Scenarios**:

  ```
  Scenario: All wheels maintain contact on uneven terrain
    Tool: interactive_bash (tmux via attach-remote)
    Preconditions: Robot on test obstacle (one wheel raised 3-5cm), suspension active
    Steps:
      1. Place 3cm block under left-front wheel
      2. Wait 5 seconds for suspension to adapt
      3. Log eso_z3 for all 4 joints
      4. Verify: all 4 |eso_z3| > 5.0 (all legs loaded, no free wheel)
      5. Log physical_angle for all 4 joints
      6. Verify: LF physical_angle > others (compressed leg)
      7. Verify: no angle at max_angle or min_angle limit
      8. Log IMU pitch/roll
      9. Verify: |pitch| < 5°, |roll| < 5° relative to calibrated zero
    Expected Result: All 4 legs loaded, LF compressed, no saturation, chassis roughly level
    Failure Indicators: Any eso_z3 ≈ 0 (wheel off ground), angle at limit (saturated), IMU > 10° off level
    Evidence: .sisyphus/evidence/task-13-uneven-terrain-log.txt
  ```

  **Commit**: NO (verification task)

- [ ] 14. Iterative parameter tuning & documentation

  **What to do**:
  - Based on static and uneven terrain test results, tune parameters:
  - **Too soft** (chassis sags, poor support):
    - Increase `active_suspension_Kz_linear` (more spring stiffness)
    - Increase `active_suspension_preload_angle_deg` (more pre-compression)
    - Or decrease `active_suspension_ride_height_offset_deg` (lower target = more extension)
  - **Too stiff** (no suspension feel, wheels bounce):
    - Decrease `active_suspension_Kz_linear`
    - Increase `active_suspension_D_leg_linear` (add damping to prevent oscillation)
  - **IMU not leveling**:
    - Increase `active_suspension_Kp` and `active_suspension_Kr` (stronger attitude correction torque)
    - Check sign convention: if error grows instead of shrinks, flip sign
    - Increase `active_suspension_pitch_ki` / `active_suspension_roll_ki` for steady-state error correction
  - **Travel too short** (barely any suspension movement):
    - Increase `active_suspension_hold_travel_deg` (wider release window)
    - Decrease `active_suspension_entry_offset_deg` (enter suspension earlier)
    - Note: the "travel too short" problem documented in todo.md is partially inherent to the "deploy first, then suspend" strategy — document this tradeoff
  - Update YAML with final tuned values and document the tuning rationale in comments
  - Update `plan.md` or `todo.md` with test results and next-step recommendations

  **Must NOT do**:
  - Do NOT blindly increase all gains — document which knob was turned and why
  - Do NOT change ADRC parameters during suspension formula tuning (keep them stable)
  - Do NOT remove the documentation of the "deploy-then-suspend" travel tradeoff

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: Iterative hardware tuning with physical intuition — requires understanding of coupled parameter effects

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Wave 4 (sequential after Task 13)
  - **Blocks**: Final Verification (F1-F3)
  - **Blocked By**: Task 13

  **References**:
  - `todo.md:164-226` — Previous parameter tuning attempts and their effects (preload/softness tradeoff)
  - `todo.md:233-262` — Next step recommendations (direction A/B/C for solving preload-travel paradox)
  - `rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni.yaml:68-109` — All tunable suspension parameters
  - `code-plan.md:96-113` — ADRC suspension mode parameter mapping (for reference, do NOT change these)

  **Acceptance Criteria**:
  - [ ] At least 3 tuning iterations documented (before → after parameters + observed effect)
  - [ ] Final parameters achieve: gravity comp verified, 4-wheel contact on 3cm obstacle, IMU within ±5°
  - [ ] Tuning rationale documented in YAML comments
  - [ ] Any remaining known issues documented (e.g., preload-travel tradeoff)

  **QA Scenarios**:

  ```
  Scenario: Final tuned parameters pass all acceptance tests
    Tool: interactive_bash
    Preconditions: Parameters tuned through at least 3 iterations
    Steps:
      1. Run static loading test (Task 12 scenarios) — all pass
      2. Run uneven terrain test (Task 13 scenarios) — all pass
      3. Record final parameters: dump all active_suspension_* parameters
      4. Record IMU calibration values
    Expected Result: All tests pass with final parameters
    Failure Indicators: Any scenario failing after 5+ tuning iterations → document as known limitation
    Evidence: .sisyphus/evidence/task-14-final-params.txt
  ```

  **Commit**: YES
  - Message: `tune(chassis): final suspension parameter tuning from vehicle testing`
  - Files: `rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni.yaml`, possibly `todo.md`

---

## Final Verification Wave

- [ ] F1. **Plan Compliance Audit** — `oracle`
  Read the plan end-to-end. Verify each "Must Have": check implementation exists. Verify each "Must NOT Have": search for forbidden patterns. Check evidence files exist in `.sisyphus/evidence/`. Compare deliverables against plan.
  Output: `Must Have [N/N] | Must NOT Have [N/N] | Tasks [N/N] | VERDICT: APPROVE/REJECT`

- [ ] F2. **Code Quality Review** — `unspecified-high`
  Run `colcon build --packages-select rmcs_core`. Run `clang-tidy` on changed files. Review for: unused variables, sign errors in formulas, unit inconsistencies (rad vs deg, N vs N·m), missing finite checks on new inputs. Check AI slop: excessive comments, over-abstraction.
  Output: `Build [PASS/FAIL] | Lint [N issues] | Formula audit [N issues] | VERDICT`

- [ ] F3. **Real Manual QA** — `unspecified-high`
  Deploy to vehicle. Execute gravity comp test (level ground, all wheels should support evenly). Execute uneven terrain test (ramp/obstacle): verify wheels maintain contact, verify chassis leveling within tolerance via IMU log. Execute normal mode regression (deploy/retract without suspension).
  Output: `Gravity comp [PASS/FAIL] | Contact [4/4] | Leveling [±N°] | Deploy regression [PASS/FAIL] | VERDICT`

---

## Commit Strategy

- **Task 5-9**: `feat(chassis): implement cartesian suspension kinematics and impedance model` — `deformable_chassis.cpp`
- **Task 8**: `feat(config): add new suspension formula parameters for omni` — `deformable-infantry-omni.yaml`
- **Task 10**: `fix(chassis): resolve compile errors from suspension formula refactor` — `deformable_chassis.cpp`
- **Task 11**: `feat(tools): add sweep data processing script for b0 identification` — `scripts/identify_b0.py`
- **Task 14**: `tune(chassis): final suspension parameter tuning for omni` — `deformable-infantry-omni.yaml`, `deformable_chassis.cpp`

---

## Success Criteria

### Verification Commands
```bash
# Compile check
build-rmcs --packages-select rmcs_core

# Sweep data analysis
python3 scripts/identify_b0.py sweep_data.csv

# Deploy and verify on vehicle
sync-remote && attach-remote -r
# Expected: "suspension active" log, IMU pitch/roll within ±5°
```

### Final Checklist
- [ ] Cartesian impedance model implemented (not angle-space spring)
- [ ] IMU-to-leg geometric mapping implemented (not ad-hoc PID bias)
- [ ] Gravity compensation verified on level ground
- [ ] 4-wheel contact maintained on uneven terrain
- [ ] Chassis level within ±5° during operation
- [ ] Normal deploy/tracking mode unaffected
- [ ] ADRC joint controller unchanged
- [ ] Frequency sweep data confirms correct b0
