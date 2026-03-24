# test_virtual.cpp 使用说明（含非仿真实机规范）

文件位置：`src/controller/chassis/test_virtual.cpp`

## 1. 组件定位

`rmcs_core::virtue::chassis::ChassisLiftController` 是一个**仿真调试组件**，核心作用是：
- 按 `TD -> ESO -> NLESF` 执行 ADRC 闭环。
- 在组件内部执行机构动力学与限位接触仿真（`simulate_step_()`）。
- 输出最小调试量（力矩、速度、机构角、目标角）。

该文件默认用于仿真，不建议直接用于实机闭环输出。

## 2. 接口定义（按源码）

### 输入接口
- 无 `register_input`。
- 目标角由参数系统解析（`resolve_target_theta_rad_()`），不是外部 topic 输入。

### 输出接口
- `/test/motor/torque` (`double`)
  - 最终力矩指令（ADRC 输出经缩放、限幅和接触保护后）。
- `/test/motor/velocity_rad_s` (`double`, rad/s)
  - 电机侧角速度（变量 `theta_dot_rad_`）。
- `/test/mechanism/angle_deg` (`double`, deg)
  - 机构侧角度（变量 `theta_rad_`，转角度输出）。
- `/test/target/angle_deg` (`double`, deg)
  - 当前生效目标角度。

## 3. 运行行为与关键约束

- 控制周期常量 `dt_ = 0.001`（1 kHz）。
- 推荐 `rmcs_executor.update_rate` 也配置为 `1000.0`，否则模型积分与控制离散时间不一致。
- 参数在 `load_parameters_()` 启动时加载一次；`target_mode=yaml` 时，`fixed_target_angle_deg` 在每次 `update()` 会尝试重新读取，支持运行时改参。
- 兼容旧参数名（建议新配置不要继续使用）：
  - `physical_min_angle_deg` 回退到 `min_angle`
  - `physical_max_angle_deg` 回退到 `max_angle`
  - `fixed_target_angle_deg` 回退到 `fixed_target_angle`

## 4. 参数与约束（与代码一致）

### 4.1 角度边界与初始化
- `physical_min_angle_deg`（默认回退 `15.0`）
- `physical_max_angle_deg`（默认回退 `55.0`）
- `initial_angle_deg`（默认 `min_angle`，会 clamp 到 `[min, max]`）

### 4.2 仿真对象参数
- `inertia`（默认 `0.04`）
- `viscous_damping`（默认 `0.02`）
- `reduction_ratio`（默认 `36.0`）
- `gravity_gain`（默认 `11.2`）
- `torque_limit`（默认 `28.0`，最终输出限幅）

### 4.3 扰动与限位接触参数
- `disturbance_min` / `disturbance_max`（默认 `0.0/0.0`，若 `min > max` 会自动交换）
- `limit_restitution`（默认 `0.0`，强制 clamp `[0,1]`）
- `limit_smooth_zone_deg`（默认 `3.0`，下限 `0`）
- `limit_inward_torque_scale_min`（默认 `1.0`，强制 clamp `[0,1]`）
- `limit_approach_damping`（默认 `1.2`，下限 `0`）
- `limit_contact_guard_enable`（默认 `true`）
- `limit_contact_hold_margin`（默认 `0.8`，下限 `0`）
- `limit_release_speed_deg_s`（默认 `0.5`，下限 `0`，内部转 rad/s）

### 4.4 ADRC 参数
- `adrc_b0`（默认 `0.694`）
- `kt`（默认 `1.0`，ADRC 输出缩放）
- `adrc_td_h`（默认 `0.003`）
- `adrc_td_r`（默认 `300.0`）
- `adrc_eso_h`（默认 `0.001`）
- `adrc_eso_w0`（默认 `120.0`）
- `adrc_eso_auto_beta`（默认 `true`）
- `adrc_nlesf_k1`（默认 `20.0`）
- `adrc_nlesf_k2`（默认 `2.0`）
- `adrc_nlesf_alpha1`（默认 `0.75`）
- `adrc_nlesf_alpha2`（默认 `1.25`）
- `adrc_nlesf_delta`（默认 `0.02`）
- `adrc_nlesf_u_min`（默认 `-torque_limit`）
- `adrc_nlesf_u_max`（默认 `torque_limit`）

### 4.5 目标与日志参数
- `target_mode`（默认 `yaml`，内部转小写）
- `fixed_target_angle_deg`（默认回退 `55.0`，并 clamp 到 `[min, max]`）
- `code_target_center_deg`（默认 `fixed_target_angle_deg`）
- `code_target_amplitude_deg`（默认 `0.0`，下限 `0`）
- `code_target_period_s`（默认 `6.0`，下限 `0`）
- `log_every_n`（默认 `100`，最小 `1`）

## 5. 目标角使用规范

1. `target_mode: yaml`
- 目标来自 `fixed_target_angle_deg`（每周期读取，支持 `ros2 param set` 动态修改）。

2. `target_mode: code`
- 目标来自 `code_target_angle_rad_(time)`。
- 当前实现为正弦轨迹：`center + amplitude * sin(2*pi*t/period)`。

3. 运行时改参注意
- `target_mode=code` 时，修改 `fixed_target_angle_deg` 不生效。
- 任意目标都会被 clamp 到 `[physical_min_angle_deg, physical_max_angle_deg]`。

## 6. 非仿真（实机）代码使用规范

以下是把该控制逻辑迁移到真实执行器时的最小规范：

1. I/O 规范
- 增加输入：机构角度（必需）、速度（建议）等真实反馈 `register_input(...)`。
- 保留输出：控制力矩 `register_output(...)`，输出到真实驱动链路。

2. 控制链顺序固定
- 保持 `TD -> ESO -> NLESF -> kt 缩放 -> torque_limit 限幅` 顺序不变。
- `ESO` 的第二输入继续使用上一周期实际下发控制量 `last_torque_cmd_`。

3. 仿真逻辑替换
- 删除 `simulate_step_()` 在实机路径中的调用。
- `theta_rad_`、`theta_dot_rad_` 必须由真实传感器更新，不能再由模型积分得到。

4. 单位与侧别统一
- 角度统一 rad 参与控制，外部显示可用 deg。
- 若速度传感器给的是机构侧速度，且 `theta_dot_rad_` 定义为电机侧，则需按 `reduction_ratio` 做换算。

5. 安全规范
- 反馈非有限值（NaN/Inf）时不应继续输出控制，建议降为 0 或进入上层保护。
- 目标角和输出力矩必须双重限幅（控制器内 + 驱动层）。
- 限位接触保护如继续使用，必须改为基于真实反馈/限位信号判定，不能依赖仿真内部 `latched_limit_side_` 语义。

### 实机化伪代码（示意）

```cpp
void update() override {
    const double target_theta_rad = resolve_target_theta_rad_();

    // real feedback
    const double measured_angle_deg = *mechanism_angle_input_;
    if (!std::isfinite(measured_angle_deg)) {
        *motor_torque_output_ = 0.0;
        return;
    }
    theta_rad_ = clamp_angle_rad_(measured_angle_deg * kDegToRad);
    theta_dot_rad_ = *motor_velocity_input_;  // 若为机构侧速度需先换算

    const auto td_out = td_.update(target_theta_rad);
    const auto eso_out = eso_.update(theta_rad_, last_torque_cmd_);
    const auto nlesf_out = nlesf_.compute(td_out.x1 - eso_out.z1, td_out.x2 - eso_out.z2, eso_out.z3, adrc_b0_);

    double torque_cmd = std::clamp(adrc_output_scale_ * nlesf_out.u, -torque_limit_, torque_limit_);
    // 可选：替换为真实限位判定版本
    // torque_cmd = apply_contact_torque_guard_real_(torque_cmd);

    *motor_torque_output_ = torque_cmd;
    last_torque_cmd_ = torque_cmd;
}
```

## 7. 非仿真（实机）YAML 参数使用规范

### 7.1 必配（建议保留）
- `physical_min_angle_deg`
- `physical_max_angle_deg`
- `torque_limit`
- `adrc_b0`, `kt`
- `adrc_td_h`, `adrc_td_r`
- `adrc_eso_h`, `adrc_eso_w0`, `adrc_eso_auto_beta`
- `adrc_nlesf_k1`, `adrc_nlesf_k2`, `adrc_nlesf_alpha1`, `adrc_nlesf_alpha2`, `adrc_nlesf_delta`
- `adrc_nlesf_u_min`, `adrc_nlesf_u_max`
- `target_mode` 与其对应目标参数（`fixed_target_angle_deg` 或 `code_target_*`）

### 7.2 建议禁用或移除（纯仿真参数）
- `inertia`
- `disturbance_min`, `disturbance_max`
- `limit_restitution`
- `limit_smooth_zone_deg`
- `limit_inward_torque_scale_min`
- `limit_approach_damping`
- `limit_release_speed_deg_s`
- `initial_angle_deg`

### 7.3 条件保留（仅当你在实机实现了对应逻辑）
- `reduction_ratio`
- `viscous_damping`
- `gravity_gain`
- `limit_contact_guard_enable`
- `limit_contact_hold_margin`

### 7.4 实机 YAML 最小示例

```yaml
real_chassis_lift_controller:
  ros__parameters:
    physical_min_angle_deg: 15.0
    physical_max_angle_deg: 55.0
    torque_limit: 12.0

    adrc_b0: 0.694
    kt: 1.0
    adrc_td_h: 0.001
    adrc_td_r: 300.0
    adrc_eso_h: 0.001
    adrc_eso_w0: 120.0
    adrc_eso_auto_beta: true
    adrc_nlesf_k1: 20.0
    adrc_nlesf_k2: 2.0
    adrc_nlesf_alpha1: 0.75
    adrc_nlesf_alpha2: 1.25
    adrc_nlesf_delta: 0.02
    adrc_nlesf_u_min: -12.0
    adrc_nlesf_u_max: 12.0

    target_mode: yaml
    fixed_target_angle_deg: 20.0
    log_every_n: 200

    # 实机默认建议关闭仿真接触守护；如实现了真实限位判定可再开启
    limit_contact_guard_enable: false
```

## 8. 仿真快速验证（当前文件）

1. 启动：
```bash
ros2 launch rmcs_bringup rmcs.launch.py robot:=virtue_test
```

2. 查看输出：
```bash
ros2 topic echo /test/motor/torque
ros2 topic echo /test/motor/velocity_rad_s
ros2 topic echo /test/mechanism/angle_deg
ros2 topic echo /test/target/angle_deg
```

3. 动态改目标（仅 `target_mode: yaml` 生效）：
```bash
ros2 param set /virtual_chassis_lift_controller fixed_target_angle_deg 40.0
```
