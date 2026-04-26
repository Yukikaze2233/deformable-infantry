# Joint 模块重构：状态机驱动架构

## TL;DR

> **Quick Summary**: 把 `JointController` 拆为 `JointState`（纯状态逻辑）+ 运动学/力模型 + `JointController`（薄编排层）。状态机成为控制核心：leg 处于什么 phase → 执行什么行为。
>
> **Deliverables**: 5 个文件重构（`joint_state.hpp/cpp`, 精简 `joint_controller.hpp/cpp`, 重写 `joint_controller.cpp`）
>
> **Estimated Effort**: Medium
> **Parallel Execution**: NO — 次序依赖

---

## 当前问题

`JointController`（474 行）做了太多事：
- 状态机（Inactive→Arming→Active→Releasing）
- 接地置信度估计
- Cartesian 运动学（z = -L·cos(α), chassis pose）
- IMU 姿态力偏置
- 支撑力计算（F = mg/4 + K(z-zref) + Dż + bias）
- 关节力矩输出（τ = F·L·sin(α)）
- 目标轨迹生成（速度/加速度限制 ramp）

全部通过 `update()` 一个大方法串联。读者看不出**状态机是控制流的核心**。

## 目标架构

```
JointController (薄编排层)
  │
  ├── JointState (纯状态逻辑)
  │     input:  angle, velocity, deploy_requested, contact_ready, dt
  │     output: phase, active_flag, elapsed
  │
  ├── Kinematics (纯运动学 + 力模型)
  │     input:  α, α̇, z_ref, config
  │     output: z, ż, F, τ, chassis_pose
  │
  └── ContactEstimator (接地检测)
        input:  z₃, torque, velocity
        output: confidence, contact_latched
```

**控制流变为**：
```
1. ContactEstimator → contact_ready flag
2. JointState.update() → which legs are Active?
3. For active legs: JointKinematics.compute() → z, ż, F, τ
4. JointKinematics.compute_trajectory() → target angles
5. Package CycleOutput
```

---

## 模块设计

### JointState

```cpp
class JointState {
public:
    struct Config {
        double entry_offset;           // rad, from deploy_angle
        double ride_height_offset;     // rad
        double hold_travel;            // rad
        double velocity_threshold;     // rad/s
        double min_arming_time;        // s
    };

    struct PerLegInput {
        double physical_angle;
        double physical_velocity;
        double motor_angle;            // for sync on Active entry
        bool   deploy_requested;
        bool   contact_ready;
        double dt;
    };

    struct PerLegState {
        SuspensionPhase phase;
        bool active;                   // suspension output active?
        double phase_elapsed;
        bool contact_latched;
    };

    void configure(const Config& config);
    void update(size_t index, const PerLegInput& input, PerLegState& state);
    void reset(size_t index);
};
```

- 纯粹的状态转换逻辑，不涉及物理量
- 每个 leg 独立 update，可以单元测试
- `contact_latched` 归状态机管（latch 是状态的一部分）

### JointKinematics

```cpp
struct WheelCartesianState {
    std::array<double, 4> z, z_dot;
    double chassis_z, chassis_pitch, chassis_roll;
};

class JointKinematics {
public:
    struct Config {
        double rod_length, wheel_base_half_x, wheel_base_half_y;
    };

    WheelCartesianState compute(const std::array<double,4>& angles,
                                 const std::array<double,4>& velocities) const;
};
```

- 纯函数，无状态。运动学计算从 α → z。
- 支撑力模型也在这里：`compute_support_force(index, z, z_dot, z_ref, config, bias)`

### JointController（精简后）

```cpp
class JointController {
public:
    // 公开接口不变：Config, CycleInput, CycleOutput, configure(), update()

private:
    JointState state_machine_;
    // 运动学和力模型作为私有方法，或独立类

    Config config_;
    std::array<JointState::PerLegState, 4> leg_states_;
    // trajectory state...
};
```

---

## TODOs

- [ ] 1. 创建 `joint_state.hpp` — 声明 `JointState` 类

  **What to do**: 从 `joint_controller.hpp` 中提取状态机相关类型和接口到新文件。
  - `SuspensionPhase` enum 移到 `joint_state.hpp`
  - 声明 `JointState` 类（Config / PerLegInput / PerLegState / configure / update / reset）
  - `update()` 接受单个 leg 的输入和输出引用

  **Files**: 新建 `controller/joint/joint_state.hpp`
  **Commit**: `feat(joint): add JointState class declaration`

- [ ] 2. 创建 `joint_state.cpp` — 实现状态转换逻辑

  **What to do**: 从 `joint_controller.cpp` 中搬走 `update_leg_states()` 和 `contact_ready()` 逻辑。
  - 状态转换逻辑（Inactive→Arming→Active→Releasing）
  - `contact_latched` 管理
  - `phase_elapsed` 计时
  - 纯逻辑，不涉及力/运动学

  **Files**: 新建 `controller/joint/joint_state.cpp`
  **Commit**: `feat(joint): implement JointState state transitions`

- [ ] 3. 重写 `joint_controller.hpp` — 引用 `JointState`

  **What to do**: 
  - `#include "joint_state.hpp"`
  - 移除 `SuspensionPhase` enum（已在 state_machine.hpp）
  - `JointController` 添加成员 `JointState state_machine_`
  - 添加成员 `std::array<JointState::PerLegState, 4> leg_states_`
  - 移除 `LegControlState` 内部 struct（用 `JointState::PerLegState` 替代）
  - API 不变（`Config`, `CycleInput`, `CycleOutput`, `configure`, `update`）

  **Files**: 修改 `controller/joint/joint_controller.hpp`
  **Commit**: `refactor(joint): integrate JointState into JointController`

- [ ] 4. 重写 `joint_controller.cpp` — 用状态机驱动控制流

  **What to do**:
  - `update()` 方法改为：
    1. 接触估计 → `contact_ready` flag per leg
    2. `state_machine_.update(index, input, state)` per leg
    3. 遍历 active legs → kinematics → support forces → torques
    4. 轨迹生成
    5. 打包输出
  - 移除已迁移到 `JointState` 的方法（`update_leg_states`, `contact_ready`）
  - 保持 `compute_wheel_cartesian`, `compute_attitude_bias`, `compute_support_forces` 等方法不变

  **Files**: 修改 `controller/joint/joint_controller.cpp`
  **Commit**: `refactor(joint): rewrite JointController::update with state-machine-driven flow`

- [ ] 5. 清理 `joint_controller.hpp` — 删除不再需要的内部类型

  **What to do**: 移除 `LegControlState`, `LegCommand`（内联到 CycleOutput），`AttitudeBias`（改为局部类型），精简头文件。

  **Commit**: `refactor(joint): remove unused internal types from JointController`

---

## 验证

- [ ] `colcon build --packages-select rmcs_core` 零错误
- [ ] 控制流不变：相同输入 → 相同输出
- [ ] 状态机独立可测：`JointState::update()` 是纯函数，给定输入就输出确定的状态

---

## 文件布局（最终）

```
controller/joint/
├── joint_state.hpp      ← 新：纯状态机
├── joint_state.cpp      ← 新：状态转换实现
├── joint_controller.hpp         ← 精简：编排层
├── joint_controller.cpp         ← 重写：状态机驱动
├── deformable_joint_controller.cpp  ← 不变：ADRC 伺服
├── deformable_joint_sweep_controller.cpp
└── deformable_joint_sweep_recorder.cpp
```

## 两个计划的关系

- `chassis-cleanup.md` — Chassis 侧精简（死代码、中间存储）
- `joint-refactor.md` — Joint 侧重构（状态机拆分）

建议先执行 Chassis 清理（影响面小，6 步纯删减），再执行 Joint 重构（有文件创建和逻辑迁移）。
