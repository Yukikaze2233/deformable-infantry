# deformable_infantry_docs 草案

> 说明：仓库中未找到 `rmcs_notebook` 或 `rmcs_note`，本草案按当前 RMCS 根目录现有专题 Markdown 风格整理，适合直接复制到根目录 `deformable_infantry_docs/` 下。

## 建议目录结构

```text
deformable_infantry_docs/
├── README.md
├── v1_screw_kinematics_and_control.md
├── v2_direct_drive_control.md
├── active_suspension.md
├── omni_kinematics.md
└── v2_steering_kinematics.md
```

---

## 1. README.md

```md
# Deformable Infantry 文档索引

本文档目录用于整理 deformable infantry 系列底盘的机构、解算、控制和主动悬挂相关说明，便于后续维护和新人理解。

当前包含以下几个部分：

- [Deformable Infantry V1：丝杆解算与控制](./v1_screw_kinematics_and_control.md)
- [Deformable Infantry V2：直驱电机关节控制](./v2_direct_drive_control.md)
- [Deformable Infantry：主动悬挂](./active_suspension.md)
- [Deformable Infantry Omni：底盘解算](./omni_kinematics.md)
- [Deformable Infantry V2：舵轮解算](./v2_steering_kinematics.md)

## 文档目的

这些文档主要回答以下问题：

1. deformable infantry 的机构自由度和控制量分别是什么
2. 不同版本（v1 / v2 / omni）的执行器形式和解算方式有何区别
3. 底盘控制、关节控制、轮系控制、主动悬挂分别由哪些组件负责
4. 当前代码实现与理想控制架构之间还有哪些差距

## 推荐阅读顺序

1. V1 丝杆解算与控制
2. V2 直驱电机关节控制
3. Omni 底盘解算
4. V2 舵轮解算
5. 主动悬挂

## 关键源码入口

- `rmcs_ws/src/rmcs_core/src/hardware/deformable-infantry-omni.cpp`
- `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp`
- `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_joint_controller.cpp`
- `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_omni_wheel_controller.cpp`
- `rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni.yaml`

## 相关现有文档

- `plan.md`
- `code-plan.md`
- `todo.md`
- `deformable_infantry_omni_active_suspension_flow.md`
```

---

## 2. v1_screw_kinematics_and_control.md

```md
# Deformable Infantry V1：丝杆解算与控制

本文档说明 deformable infantry v1 的机械传动形式、几何关系、丝杆解算思路，以及控制链路在 RMCS 中应该如何理解。

## 1. 机构特点

deformable infantry v1 的关节执行器不是直驱关节电机，而是通过丝杆机构驱动悬挂或轮组相对底盘运动。与 v2 的直驱方案相比，v1 的核心特征是：

- 电机输出首先转化为丝杆的轴向位移
- 丝杆位移再通过连杆机构转化为轮组或关节角度变化
- 控制量和被控量之间不是简单的一阶角度映射，而是“电机角 -> 丝杆位移 -> 机构几何角度”的链式关系

因此，v1 的控制核心不只是电机闭环，还包括一层几何解算。

## 2. 控制目标

v1 丝杆系统的控制通常包含两个层次：

1. **几何层**：把目标轮距、目标关节角、目标底盘高度等转换为丝杆目标长度
2. **执行层**：把丝杆目标长度转换为电机目标位置、速度或力矩

如果用更形式化的话说：

```text
目标底盘形态 / 目标关节角
    -> 机构几何解算
    -> 丝杆目标长度
    -> 电机目标位置 / 速度 / 力矩
```

## 3. 丝杆解算的基本思路

v1 的丝杆解算可以拆成三步：

### 3.1 电机角度到丝杆位移

若丝杆导程为 `p`，电机转角为 `theta_motor`，则轴向位移可写成：

```text
x = p / (2π) * theta_motor
```

若中间还存在减速器，则需要进一步乘以减速比或其倒数，取决于编码器测得的是电机侧还是输出侧角度。

### 3.2 丝杆位移到机构几何量

丝杆位移 `x` 不直接等于车轮高度或关节角，而是通过三角形或四连杆几何关系转换。通常需要根据机构尺寸建立：

```text
f(x, link_lengths, installation_offsets) = joint_angle
```

或反向：

```text
g(joint_angle, link_lengths, installation_offsets) = screw_length
```

实际实现时一般更推荐使用“目标几何量 -> 反解 -> 丝杆长度”，因为控制器最终更容易围绕统一的目标几何量进行设计。

### 3.3 丝杆长度到电机控制量

在控制实现上，可以有三种常见方式：

- 位置控制：直接跟踪丝杆目标长度
- 速度控制：围绕长度误差生成速度指令
- 力矩控制：围绕长度误差或受力状态生成推力/力矩

在 deformable infantry 语境下，最有工程意义的是：

- 平时做位置或速度跟踪
- 接地、悬挂或受扰时，需要保留一定柔顺性，避免丝杆机构被硬顶

## 4. v1 丝杆控制的工程问题

与直驱方案相比，丝杆方案通常有以下问题：

- 传动链更长，摩擦、间隙、反向死区更明显
- 电机角误差和机构角误差之间存在非线性映射
- 丝杆机构天然更适合准静态定位，不如直驱方案容易做快速柔顺控制
- 若丝杆效率低或背驱困难，则被动柔顺性较差

因此，v1 的控制重点通常不是高带宽动态悬挂，而是：

- 保证目标几何量可达
- 保证解算正确
- 保证极限位和机械约束安全

## 5. 推荐文档内容补充

如果后续补全 v1 文档，建议把以下内容补进去：

1. v1 机构示意图
2. 丝杆导程、减速比、安装尺寸
3. 正解与反解公式
4. 行程上下限
5. 电机侧零点与机构侧零点定义
6. 当前控制器实现文件和参数配置位置

## 6. 与 v2 的区别

v1 与 v2 的本质区别在于：

- **v1**：电机不直接控制关节角，必须经过丝杆和机构解算
- **v2**：电机更接近直接控制关节本体，控制链更短，适合做更高带宽的关节控制和主动悬挂

因此，这两套系统的“解算复杂度”和“控制可达带宽”是不同的，不能简单共用同一套控制假设。
```

---

## 3. v2_direct_drive_control.md

```md
# Deformable Infantry V2：直驱电机关节控制

本文档说明 deformable infantry v2 中直驱关节的控制逻辑，以及它与 v1 丝杆方案在控制实现上的差异。

## 1. v2 的核心变化

与 v1 相比，v2 的关键变化是：

- 关节执行器更接近直接驱动关节本体
- 电机角度与关节物理角之间的映射更直接
- 机构传动链更短
- 更适合做高带宽角度控制、速度控制，以及后续主动悬挂控制

这意味着 v2 的控制设计可以更多围绕“关节角”和“关节角速度”本身来进行，而不必像 v1 那样首先处理复杂的丝杆几何反解。

## 2. 控制分层

在 RMCS 中，v2 的关节控制更适合按以下分层理解：

### 2.1 高层控制器

高层控制器负责生成关节目标：

- 目标关节角
- 目标关节角速度
- 可选的关节力矩前馈

在当前 deformable infantry 代码中，这一层主要体现在：

- `DeformableChassis`

### 2.2 关节局部执行器

局部执行器负责：

- 读取当前关节反馈
- 跟踪上游的目标角和目标速度
- 根据模式（普通 / 悬挂）切换参数
- 输出最终控制力矩

在当前代码中，这一层主要体现在：

- `DeformableJointController`

## 3. 关节物理量定义

v2 中需要特别注意以下几个量的定义：

- `motor_angle`：电机侧角度
- `physical_angle`：机构实际物理角
- `physical_velocity`：机构物理角速度
- `control_torque`：最终输出给关节电机的控制力矩

当前代码中，物理角与电机角之间采用的是类似如下关系：

```text
physical_angle = const - motor_angle
physical_velocity = -motor_velocity
```

这意味着在调控制器符号时，必须非常清楚“测量量”和“输出量”的方向定义，否则会出现：

- 在上下限之间来回跳
- 越调越发散
- 明明目标正确，但控制方向反了

## 4. 控制模式

v2 关节控制至少有两种模式：

### 普通模式

普通模式的目标是“快速、稳定地到达目标角度”。

适用于：

- 机械展开 / 收起
- 静态姿态切换
- 非悬挂工况

### 悬挂模式

悬挂模式的目标不是刚性锁定，而是：

- 保持一定目标角附近的可压缩工作区
- 接收上游 suspension torque 前馈
- 允许更柔顺的动态响应

当前项目中，这一模式已经通过 `DeformableJointController` 中的双模式参数结构体现出来。

## 5. ADRC 方案的意义

当前 deformable infantry v2 关节控制逐步转向 ADRC 方案，其优势主要在于：

- 统一 normal / suspension 两套行为，但共享同一执行器框架
- ESO 可输出 `z2 / z3`，方便观察速度估计和扰动估计
- `z3` 可以作为后续接地检测或负载变化的辅助观测量

因此，v2 的关节控制不仅是“把角度控住”，还是主动悬挂观测链的重要基础。

## 6. 当前代码入口

- `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_joint_controller.cpp`
- `rmcs_ws/src/rmcs_bringup/config/deformable-infantry-v2.yaml`
- `rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni.yaml`

## 7. 需要继续补全的内容

如果后续要把 v2 直驱文档写完整，建议补充：

1. 关节电机型号与减速结构
2. 电机零点、物理零点、安装方向定义
3. 普通模式和悬挂模式的参数对照表
4. `measurement_angle / setpoint_angle / mode_input / suspension_torque` 的接口定义
5. ESO z2/z3 的物理含义与典型调试方法
```

---

## 4. active_suspension.md

```md
# Deformable Infantry：主动悬挂

本文档描述 deformable infantry 主动悬挂的目标、控制架构、当前实现状态，以及下一步演进方向。

## 1. 主动悬挂的目标

deformable infantry 的主动悬挂不是单纯“让底盘更平”，而是同时追求以下三个目标：

1. 保证 4 个车轮在不平坦地形上尽可能持续接地
2. 保持更合理的轮压 / 法向力分布
3. 通过 IMU 反馈维持底盘姿态稳定

其中最重要的是第一点：如果轮子都不能持续接地，那么单纯姿态调平并不能真正提高通过性和抓地能力。

## 2. 自由度与控制含义

四个关节执行器至少对应以下底盘自由度：

- `z`：底盘高度模态
- `pitch`：前后俯仰模态
- `roll`：左右横滚模态
- 冗余 1 自由度：可用于轮压分配或性能优化

这意味着 deformable infantry 的关节系统天然不仅能“抬腿/降腿”，还可以作为一个 4 执行器、3 主姿态模态 + 1 冗余模态的底盘悬挂系统。

## 3. 理想控制架构

完整目标架构可以分三层：

### Layer 1：每条腿独立柔顺支撑

目标：

- 每条腿单独对地形起伏做出支撑反应
- 保持柔顺接地

这一层可近似看作虚拟弹簧-阻尼器：

```text
tau_i = K_i * (l0 - l_i) + D_i * (-dl_i/dt) + tau_gravity
```

### Layer 2：IMU 姿态控制

目标：

- 用 IMU 的 pitch / roll 误差生成姿态修正
- 通过前后、左右腿支撑差异维持底盘水平

### Layer 3：轮压优化

目标：

- 让四个车轮的法向力分布更合理
- 利用冗余自由度提高 traction 和稳定性

## 4. 当前主线实现状态

当前主线已经从“纯 IMU 调平”演进到“suspension-first 的第一版恢复”，其特点是：

- `DeformableChassis` 负责高层 joint intent
- 每条腿单独判断是否进入 suspension
- 每条腿单独计算支撑力和 `suspension_torque`
- IMU 不再直接改目标角，而是作为支撑力偏置
- `DeformableJointController` 负责局部执行和力矩输出

## 5. 当前支撑力模型

当前恢复后的第一版支撑力模型可概括为：

```text
leg_force = gravity_force_per_wheel
         + Kz * (alpha - support_zero_angle)
         + D_leg * alpha_dot
         + pitch / roll bias
         + acceleration bias
```

再通过力臂映射成关节力矩：

```text
tau = leg_force * rod_length * sin(alpha)
```

这已经比单纯的目标角修正更接近真正的悬挂，但还不是完整版本。

## 6. 当前未完成的部分

当前仍未完成：

- 真正跨周期持久的 suspension phase 状态机仍需继续加强
- contact confidence 已接入雏形，但还未深度参与 phase 决策
- `eso_z3` 仍处于“辅助观测信号”阶段
- 轮压均衡和正式的法向力分配还未实现

## 7. 当前关键源码入口

- `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp`
- `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_joint_controller.cpp`
- `rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni.yaml`
- `plan.md`
- `code-plan.md`
- `todo.md`

## 8. 后续建议补充内容

建议后续把以下内容加入正式文档：

1. 悬挂 phase 图
2. deploy / support_zero / ride_height 三种参考角的关系图
3. IMU bias、contact confidence、leg force 的数据流图
4. 典型调参顺序
5. 当前失败模式和调试经验总结
```
