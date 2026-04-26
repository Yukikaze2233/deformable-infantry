# Body-Space 主动悬挂统一重构方案

## 1. 问题分析

### 1.1 当前架构

```
DeformableChassis (意图层)
  → DeformableSuspensionController / JointController (悬挂协调)
    → 4× DeformableJointController (ADRC 双模伺服)
      → DeformableOmniWheelController (QCP 轮力优化)
```

当前 `JointController::compute_support_forces()` 对每条腿独立计算：

```
F_i = mg/4 + Kz·(z_i - z_ref) + D·ż_i + attitude_bias_i + accel_ff_i
```

### 1.2 存在的问题

| 问题 | 根因 |
|------|------|
| 斜面上姿态和阻抗冲突 | per-leg 弹簧隐式控制姿态（等高度），IMU PID 显式控制姿态（水平），两者目标不同 |
| 无跨腿协调 | 各腿独立，一腿悬空时其余三腿不补偿 |
| 无力均衡 | 目标是等高度而非等力，不平地形上力分布不均 |
| QCP 不知道实际法向力 | 用静态 mg/4 作摩擦约束，忽略载荷转移 |
| 冗余 DOF 浪费 | 4 执行器 3 DOF，多出的 1 DOF 未利用 |

### 1.3 设计决策

- **架构**: 统一 body-space 重构（非逐步补丁）
- **Null-space 优化**: 牵引力优先（traction-weighted），QCP 反馈驱动
- **QCP 接口**: 双向 — F_i → QCP + QCP 饱和度 η_i → 悬挂
- **加速度源**: 先用指令前馈，后续再加 IMU 线加速度融合

---

## 2. 数学框架

### 2.1 坐标系与运动学

单连杆悬挂，关节角 α（physical_angle），连杆长度 L = 0.150m。

**Per-leg 笛卡尔坐标**（已有 `compute_wheel_cartesian`）：
```
z_i     = -L·cos(α_i)           ← 轮触地点相对关节枢轴的垂直位移（负值）
ż_i     =  L·sin(α_i)·α̇_i      ← 垂直速度
```

α = 8°（min_angle, 最大伸展）: z = -0.1485m
α = 58°（max_angle, 最大收缩）: z = -0.0795m

**底盘 body 状态**：
```
z_chassis = (z_LF + z_LB + z_RB + z_RF) / 4
θ_kin     = (z_back - z_front) / (2·Lx)      ← 运动学俯仰
φ_kin     = (z_right - z_left) / (2·Ly)       ← 运动学横滚
```

其中 Lx = Ly = chassis_radius / √2 ≈ 0.1656m（45° 全向轮布局）。

**关键区分**：
- `θ_kin, φ_kin` 反映腿几何，在斜面上 ≠ 0（底盘跟着地形倾斜）
- `θ_imu, φ_imu` 反映真实姿态，在斜面上 ≠ 0（底盘相对重力方向的倾斜）
- 平地上两者一致；不平地形上分离

### 2.2 Body Wrench

4 个执行器控制 3 个 DOF + 1 个冗余 DOF：

```
W = [F_total, M_pitch, M_roll]^T
```

**高度控制**（运动学反馈）：
```
F_total = m·g + Kz_body·(z_chassis - z_ref) + Dz_body·ż_chassis
```

- z_ref = -L·cos(α_deploy - α_preload) = -L·cos(0°) = -L = -0.15m
- Kz_body 控制底盘整体刚度
- 只关心平均高度，不关心各腿是否等高

**俯仰控制**（IMU 反馈 + 加速度前馈）：
```
M_pitch = Kp·(0 - θ_imu) + Dp·(0 - θ̇_imu) + Ki_p·∫(0 - θ_imu)dt + m·ax·h_com
```

**横滚控制**（IMU 反馈 + 加速度前馈）：
```
M_roll = Kr·(0 - φ_imu) + Dr·(0 - φ̇_imu) + Ki_r·∫(0 - φ_imu)dt + m·ay·h_com
```

**为什么高度用运动学、姿态用 IMU**：
- 高度目标是"维持骑行高度"——这是几何量，用运动学测量
- 姿态目标是"保持水平"——这是相对重力的量，用 IMU 测量
- 当前系统混合使用导致冲突：per-leg 弹簧隐式地用运动学控制姿态（等高度 = 跟着地形倾斜），IMU PID 显式地要求水平。新方案彻底分离

### 2.3 力分配矩阵

**分配矩阵 A (3×4)**，列序 [LF, LB, RB, RF]：

```
A = [ 1      1      1      1    ]   ← Σ F_i = F_total
    [-Lx     Lx     Lx    -Lx   ]   ← Σ pitch_sign_i·F_i·Lx = M_pitch
    [ Ly     Ly    -Ly    -Ly   ]   ← Σ roll_sign_i·F_i·Ly = M_roll
```

符号约定（与代码一致）：
```
pitch_signs = [-1, +1, +1, -1]    (前腿负，后腿正)
roll_signs  = [+1, +1, -1, -1]    (左腿正，右腿负)
```

**伪逆 B = A⁺**（对称布局下的解析形式）：

```
B = [1/4,  -1/(4·Lx),   1/(4·Ly)]   ← LF
    [1/4,   1/(4·Lx),   1/(4·Ly)]   ← LB
    [1/4,   1/(4·Lx),  -1/(4·Ly)]   ← RB
    [1/4,  -1/(4·Lx),  -1/(4·Ly)]   ← RF
```

即：
```
F_nominal_i = F_total/4 + pitch_sign_i · M_pitch/(4·Lx) + roll_sign_i · M_roll/(4·Ly)
```

**零空间 N**（A·N = 0 的非零解）：

```
N = [1, -1, 1, -1]^T / 2
```

验证：A·N = [1-1+1-1, -Lx+Lx(-1)+Lx-Lx(-1), Ly+Ly(-1)-Ly-Ly(-1)] = [0, 0, 0] ✓

**完整力分配**：
```
F_i = F_nominal_i + λ · N_i
```

λ 的物理意义：在对角轮对之间转移力
- λ > 0 → LF 和 RB 增加，LB 和 RF 减少
- λ < 0 → 反之

### 2.4 Traction-Weighted Null-Space 优化

QCP 每周期输出每轮的牵引利用率：
```
η_i = |τ_wheel_i| / (μ · F_i · r_wheel)    ∈ [0, 1]
```

η_i → 1 表示该轮接近摩擦极限，需要更多法向力。

**优化目标**：将法向力向高牵引需求的轮倾斜：

```
F_target_i = F_total · (1 + k_t·η_i) / Σ_j(1 + k_t·η_j)
```

其中 k_t 是牵引权重增益（可调参数，建议初始值 2.0）。

当 η_i 全部相等时，F_target_i = F_total/4（退化为力均衡）。

**加权最小二乘求解 λ**：

```
minimize  J = Σ w_i · (F_nominal_i + λ·N_i - F_target_i)²

w_i = 1 + k_t · η_i²    (高牵引需求的轮权重更大)
```

对 λ 求导令 dJ/dλ = 0：

```
Σ w_i · (F_nominal_i + λ·N_i - F_target_i) · N_i = 0

λ* = -Σ w_i·(F_nominal_i - F_target_i)·N_i / Σ w_i·N_i²
```

**闭式解**，无需 QP 求解器。计算量 = O(4) 乘加。

**约束处理**：clamp λ* 使 F_i ≥ 0：
```
λ_max = min_i { -F_nominal_i / N_i  |  N_i < 0 }
λ_min = max_i { -F_nominal_i / N_i  |  N_i > 0 }
λ* = clamp(λ*, λ_min, λ_max)
```

### 2.5 接地恢复（Contact Recovery）

**检测**：沿用现有 ESO z3 + 力矩 + 速度的接地置信度估计。

**1 腿悬空**：3 执行器 3 DOF，精确解（无冗余）：

```
A_reduced = A[:, grounded_indices]    (3×3)
det(A_reduced) = ±2·Lx·Ly ≠ 0        (矩形布局，任去 1 列均可逆)
F_grounded = A_reduced⁻¹ · W
F_airborne = F_extend                  (小常数，如 5N，温和伸展)
```

物理机制：
1. 悬空腿的 mg/4 份额分配给其余 3 腿
2. 3 腿总力增加 → 底盘下压
3. 底盘下压 → 悬空腿的关节角减小 → 腿伸展
4. ADRC 悬挂模式低增益允许平滑伸展
5. 轮触地 → ESO z3 增大 → 接地置信度恢复 → 切回 4 腿模式

**2 腿悬空**：2 执行器 3 DOF，欠定。降级策略：
- 优先级：高度 > 俯仰 > 横滚
- 用 2×2 子矩阵求解高度 + 主要姿态轴

**A_reduced⁻¹ 预计算**：4 种单腿悬空情况的逆矩阵可以在 `configure()` 时预计算，运行时只需查表 + 矩阵乘法。

### 2.6 加速度前馈

载荷转移公式（已在 body wrench 中）：

```
ΔM_pitch = m · ax · h_com    (前后载荷转移)
ΔM_roll  = m · ay · h_com    (左右载荷转移)
```

分配到每腿后：
```
ΔF_i = pitch_sign_i · m·ax·h_com/(4·Lx) + roll_sign_i · m·ay·h_com/(4·Ly)
```

当前用指令加速度 `a_cmd`（无延迟前馈）。后续可加 IMU 融合：
```
a_blended = α·a_cmd + (1-α)·a_measured,  α ≈ 0.7
```

**关于离心力**：曲线运动时的离心加速度 `a_centripetal = ω²·R` 体现为侧向加速度 ay，已被 M_roll 前馈覆盖。Spin-in-place 时 COM 不移动，无净侧向加速度，主要靠 IMU 姿态反馈处理。

---

## 3. 双向 QCP 集成

### 3.1 数据流

```
Cycle N (1kHz):
  ┌─────────────────────────────────────────────────────────┐
  │ Suspension Controller                                    │
  │   读取 η_i[N-1] (上周期 QCP 输出)                        │
  │   计算 body wrench W                                     │
  │   分配 F_i = B·W + λ*·N (traction-weighted)             │
  │   输出 F_i[N] → normal_force_estimate                   │
  │   输出 τ_ff_i = F_i·L·sin(α_i) → suspension_torque     │
  ├─────────────────────────────────────────────────────────┤
  │ 4× ADRC Joint Controller                                │
  │   读取 suspension_torque 作为前馈                         │
  │   悬挂模式低增益提供柔顺性                                 │
  ├─────────────────────────────────────────────────────────┤
  │ QCP Wheel Controller                                     │
  │   读取 F_i[N] → 替代静态 mg/4 作为摩擦约束               │
  │   求解轮力矩 τ_wheel_i                                   │
  │   计算 η_i[N] = |τ_wheel_i| / (μ·F_i·r)                │
  │   输出 η_i[N] → traction_utilization                    │
  └─────────────────────────────────────────────────────────┘
```

1 周期延迟 = 1ms。牵引利用率变化时间尺度 ~10-100ms，延迟可忽略。

### 3.2 新增接口

**Suspension → QCP**（OutputInterface<double>）：
```
/chassis/left_front_wheel/normal_force_estimate
/chassis/left_back_wheel/normal_force_estimate
/chassis/right_back_wheel/normal_force_estimate
/chassis/right_front_wheel/normal_force_estimate
```

**QCP → Suspension**（OutputInterface<double>）：
```
/chassis/left_front_wheel/traction_utilization
/chassis/left_back_wheel/traction_utilization
/chassis/right_back_wheel/traction_utilization
/chassis/right_front_wheel/traction_utilization
```

### 3.3 QCP 约束改动

当前 QCP 用菱形约束（所有轮共享同一摩擦上限）：
```cpp
const double rhombus_top = (μ * mass * g * r_wheel) / 4;
```

改为 per-wheel 约束。两种方案：

**方案 A：保守菱形**（改动最小）：
```cpp
// 用 4 轮中最小法向力作为统一约束
double min_normal = *std::min_element(normal_forces.begin(), normal_forces.end());
const double rhombus_top = μ * min_normal * r_wheel;
```

**方案 B：Per-wheel half-plane**（更精确，已有框架）：
```cpp
// 现有 com_height half-plane 框架可以直接复用
// 将 F_i 代入 half-plane 约束的 rhs
for (int i = 0; i < 4; ++i) {
    half_planes[2*i].rhs   = μ * normal_forces[i] * r_wheel;
    half_planes[2*i+1].rhs = μ * normal_forces[i] * r_wheel;
}
```

建议先用方案 A（1 行改动验证概念），后续切方案 B。

---

## 4. 与 ADRC 的关系

### 4.1 两层职责分离

```
Body-space 控制器（本方案）:
  ├── 宏观力分配: 重力补偿、姿态校正、载荷转移、牵引优化
  ├── 输出: per-leg 支撑力 F_i → 转换为力矩前馈 τ_ff_i
  └── 频率: 低频主导（< 10Hz 的姿态/载荷变化）

ADRC 悬挂模式（不改动）:
  ├── 微观柔顺: 冲击吸收、高频扰动抑制
  ├── ESO z3 自动补偿未建模扰动
  ├── 低增益 (k1=6, k2=3, w0=80) 提供弹簧-阻尼特性
  └── 频率: 高频主导（> 10Hz 的地面冲击）
```

### 4.2 为什么不需要改 ADRC

ADRC 的控制律：
```
u = (k1·fal(e1) + k2·fal(e2) - z3) / b0 + τ_ff / b0
```

其中 τ_ff 就是 suspension_torque 前馈。Body-space 改进的是 τ_ff 的计算方式（从 per-leg 独立弹簧 → body-space 分配），ADRC 只是接收不同的前馈值，内部逻辑完全不变。

---

## 5. 数学等价性

### 5.1 当前系统的隐式 body-space 行为

当前 per-leg 弹簧 `F_i = mg/4 + Kz·(z_i - z_ref) + D·ż_i` 隐式产生：

**体高度刚度**：
```
F_total = Σ F_i = mg + Kz·Σ(z_i - z_ref) + D·Σż_i
        = mg + 4·Kz·(z_chassis - z_ref) + 4·D·ż_chassis
```
→ 等效 Kz_body = 4·Kz = 60000 N/m, Dz_body = 4·D = 800 N·s/m

**俯仰刚度**（从弹簧几何）：
```
M_pitch = Σ pitch_sign_i · F_i · Lx
        = Kz · Lx · Σ pitch_sign_i · z_i + D · Lx · Σ pitch_sign_i · ż_i
        = 4·Kz·Lx² · θ_kin + 4·D·Lx² · θ̇_kin
```
→ 等效俯仰刚度 = 4·Kz·Lx² ≈ 4×15000×0.0274 ≈ 1644 N·m/rad

**加上 IMU PID**：
```
M_pitch_total = 4·Kz·Lx²·θ_kin + 4·D·Lx²·θ̇_kin + Kp·(0-θ_imu) + Dp·(0-θ̇_imu)
```

在平地上 θ_kin ≈ θ_imu，两项协同。在斜面上 θ_kin ≠ θ_imu，两项冲突。

### 5.2 新系统的显式 body-space 行为

```
F_total = mg + Kz_body·(z_chassis - z_ref) + Dz_body·ż_chassis
M_pitch = Kp·(0 - θ_imu) + Dp·(0 - θ̇_imu) + Ki_p·∫θ_imu + m·ax·h_com
```

**高度**只用运动学 z_chassis → 不产生隐式姿态刚度
**姿态**只用 IMU → 不受地形影响

初始参数设为等价值确保平地行为一致：
```
Kz_body = 60000 N/m
Dz_body = 800 N·s/m
Kp = 8.0, Ki_p = 0.35, Dp = 0.28  (保持 YAML 现有值)
```

---

## 6. 实现计划

### 6.1 Phase 1: JointController body-space 重构

**修改文件**：
- `joint_controller.hpp` — Config 新增 body-space 参数，新增 traction 输入
- `joint_controller.cpp` — 重写力计算管线

**新增方法**：
```cpp
struct BodyWrench { double F_total, M_pitch, M_roll; };

BodyWrench compute_body_wrench(const CycleInput& input, const WheelCartesianState& ws);
void allocate_leg_forces(std::array<double, 4>& out, const BodyWrench& W,
                         const std::array<double, 4>& traction_util);
void handle_contact_loss(std::array<double, 4>& forces, const BodyWrench& W);
```

**删除**：
- `AttitudeBias` 结构体
- `compute_attitude_bias()` 方法
- 旧 `compute_support_forces()` 签名

**Config 新增字段**：
```cpp
double Kz_body = 60000.0;
double Dz_body = 800.0;
double k_traction = 2.0;           // traction weight gain
double contact_loss_extend_force = 5.0;  // N, gentle extension
```

**CycleInput 新增**：
```cpp
std::array<double, kJointCount> traction_utilization{};  // η_i from QCP
```

### 6.2 Phase 2: 双向 QCP 接口

**修改文件**：
- `deformable_suspension_controller.hpp/cpp` — 新增 8 个接口（4 normal_force out, 4 traction_util in）
- `deformable_omni_wheel_controller.cpp` — 新增 8 个接口（4 normal_force in, 4 traction_util out），改 QCP 约束

### 6.3 Phase 3: YAML 配置

**修改文件**：
- `deformable-infantry-omni.yaml`

**新增参数**：
```yaml
suspension_controller:
  ros__parameters:
    # Body-space impedance (replaces per-leg Kz/D)
    active_suspension_Kz_body: 60000.0
    active_suspension_Dz_body: 800.0
    # Traction optimization
    active_suspension_k_traction: 2.0
    active_suspension_contact_loss_extend_force: 5.0
```

---

## 7. 验证计划

| 测试 | 预期行为 | 观测量 |
|------|----------|--------|
| 平地静态 | F_i ≈ mg/4 ≈ 55N, η_i ≈ 0 | ValueBroadcaster 输出 |
| 倾斜平台 | 姿态校正，力重分配 | IMU pitch/roll → 0, F_i 不等但 Σ = mg |
| 加减速 | 载荷转移补偿 | 前轮/后轮力差 ∝ ax |
| 单轮抬高 | 悬空腿伸展，3 腿重分配 | contact_confidence 变化，F_airborne → F_extend |
| Spin 模式 | 牵引优化 | η_i 均匀化，外侧轮 F_i 增加 |
| 过坎 | 柔顺接地 + 快速恢复 | 无轮离地，姿态扰动 < 5° |

编译验证：`build-rmcs --packages-select rmcs_core`
