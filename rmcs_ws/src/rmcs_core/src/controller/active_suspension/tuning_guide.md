# 主动悬挂调参指南

## 适用范围

本指南对应当前这套测试链路：

- `rmcs_bringup/config/deformable-infantry-omni-active-suspension-test.yaml`
- `rmcs_core/src/controller/active_suspension/adaptive_omni_deformable_chassis.cpp`
- `rmcs_core/src/controller/active_suspension/adaptive_omni_contact_estimator.cpp`
- `rmcs_core/src/controller/active_suspension/adaptive_omni_active_suspension.cpp`
- `rmcs_core/src/controller/active_suspension/adaptive_deformable_omni_wheel_controller.cpp`

它的实际架构不是“全自动地形规划悬挂”，而是：

1. 底盘层先给出一个预设基础姿态。
2. 接地估计器根据轮速、轮扭矩、关节扭矩估计四轮接地置信度。
3. 主动悬挂层再用 `pitch`、`roll` 和接地置信度对四个关节目标角做二次修正。
4. 四个 ADRC 把修正后的目标角闭环到实际关节。
5. 轮子控制器再利用接地置信度做扭矩重分配。

## 先理解每层在做什么

### 1. 基础姿态层

`AdaptiveOmniDeformableChassis` 负责生成：

- `/chassis/control_velocity`
- `/chassis/*_joint/base_target_angle`
- `/chassis/*_joint/base_target_physical_angle`

这层主要由遥控器、键盘、旋钮触发预设高度或特殊姿态。

关键参数：

- `min_angle`
- `max_angle`
- `target_physical_velocity_limit`
- `target_physical_acceleration_limit`

### 2. 接地估计层

`AdaptiveOmniContactEstimator` 输出：

- `/chassis/*_contact/confidence`
- `/chassis/*_contact/residual`
- `/chassis/contact/confidence_mean`

它不是直接测法向力，而是启发式估计：

- 期望轮速和实际轮速偏差越大，接地越差。
- 轮扭矩和关节扭矩越能“挂住负载”，接地越可信。

关键参数：

- `wheel_radius`
- `confidence_alpha`
- `moving_speed_threshold`
- `slip_ratio_gain`
- `wheel_torque_reference`
- `joint_torque_reference`
- `confidence_floor`

### 3. 主动悬挂修正层

`AdaptiveOmniActiveSuspension` 输入：

- 四个基础目标角
- 四个实际关节物理角
- 四个接地置信度
- `/chassis/imu/pitch`
- `/chassis/imu/roll`

输出：

- `/chassis/*_joint/target_angle`
- `/chassis/*_joint/target_physical_angle`
- `/chassis/*_joint/torque_limit`

关键参数：

- `contact_rebalance_gain_deg`
- `contact_deadband`
- `pitch_gain_deg_per_rad`
- `roll_gain_deg_per_rad`
- `switch_torque_limit`
- `steady_torque_limit`
- `angle_error_torque_gain`
- `low_confidence_torque_boost`

### 4. 轮子控制层

`AdaptiveDeformableOmniWheelController` 会同时用接地置信度：

- 估计车体速度
- 对轮扭矩做加权分配

关键参数：

- `mess`
- `moment_of_inertia`
- `wheel_radius`
- `friction_coefficient`
- `k1`
- `k2`
- `no_load_power`
- `min_contact_weight`

## 调参前提

在动主动悬挂参数前，先确认下面几项成立：

1. 四个关节零点正确，关节实际角度方向和机构几何含义一致。
2. 只给基础目标角时，ADRC 能稳定闭到目标，不持续抖动、不明显超调。
3. `/chassis/imu/pitch`、`/chassis/imu/roll` 数值正常，且正负方向符合底盘定义。
4. 平地静止时，四个接地置信度接近且稳定。
5. 主动修正关闭时，底盘能正常行驶。

如果前提不满足，不要直接调主动悬挂增益。

## 建议观测量

推荐先通过 `ValueBroadcaster` 转发并观察这些量：

- `/chassis/imu/pitch`
- `/chassis/imu/roll`
- `/chassis/left_front_contact/confidence`
- `/chassis/left_back_contact/confidence`
- `/chassis/right_back_contact/confidence`
- `/chassis/right_front_contact/confidence`
- `/chassis/contact/confidence_mean`
- `/chassis/left_front_joint/control_torque`
- `/chassis/left_back_joint/control_torque`
- `/chassis/right_back_joint/control_torque`
- `/chassis/right_front_joint/control_torque`

如果要更快定位问题，建议额外转发：

- `/chassis/*_joint/physical_angle`
- `/chassis/*_joint/target_physical_angle`
- `/chassis/*_contact/residual`

## 调参顺序

### 第 1 步：先把主动修正关掉，只调基础姿态

先把下面三个参数设成 0：

- `contact_rebalance_gain_deg`
- `pitch_gain_deg_per_rad`
- `roll_gain_deg_per_rad`

此时系统应退化为“只有预设基础姿态”的版本。

验证目标：

- 预设姿态切换方向正确。
- 四个关节都能跟上基础目标。
- 切换时不会出现明显打角、卡顿、严重超调。

如果这一层不稳定，后面所有主动修正都会被掩盖。

### 第 2 步：确认 IMU 姿态方向

把 `pitch_gain_deg_per_rad`、`roll_gain_deg_per_rad` 保持很小，例如 `1.0` 到 `2.0`。

做两个简单动作：

1. 让车头抬高或压低，观察四个关节目标的变化方向。
2. 让左侧或右侧抬高，观察左右关节目标的变化方向。

如果修正方向反了，不要先怀疑算法，直接改增益符号：

- `pitch_gain_deg_per_rad` 可取负值。
- `roll_gain_deg_per_rad` 可取负值。

先把方向调对，再谈大小。

### 第 3 步：单独调接地估计

继续保持 `contact_rebalance_gain_deg = 0`，只看置信度。

建议做三组工况：

1. 平地静止
2. 平地匀速
3. 单轮压块、单轮悬空、低附路面打滑

目标现象：

- 平地静止时四轮 `confidence` 接近。
- 平地匀速时四轮 `confidence` 仍然接近，不应持续偏一侧。
- 某个轮子明显更容易失地或打滑时，该轮 `confidence` 要能明显偏离其余三轮。

参数建议：

- `wheel_radius`
  必须先准。这个值错了，轮速残差会整体失真。
- `confidence_alpha`
  大则响应快但更抖，小则更稳但更钝。建议从 `0.1` 到 `0.2` 开始。
- `moving_speed_threshold`
  过低会让低速挪车也进入“滑移判别”模式，过高会让慢速越障时估计不敏感。
- `slip_ratio_gain`
  过小则打滑不敏感，过大则普通速度误差也会被当成低接地。
- `wheel_torque_reference`
  过大则轮扭矩贡献太弱，过小则稍微有扭矩就被判成高接地。
- `joint_torque_reference`
  逻辑同上。
- `confidence_floor`
  建议不要太高。太高会压缩高低轮之间的差异。

### 第 4 步：再打开接地重分配

把 `contact_rebalance_gain_deg` 从小值开始加，例如：

- `3` 到 `5`
- 再到 `8` 到 `12`

同时把 `pitch_gain_deg_per_rad`、`roll_gain_deg_per_rad` 先保持较小。

目标现象：

- 单轮失地或低附时，四个关节目标角出现合理的差分修正。
- 修正量有明显方向性，而不是四个关节一起乱动。

如果某个轮子低接地时修正方向和机构预期相反，可以直接改：

- `contact_rebalance_gain_deg` 为负值

因为当前实现里低接地轮的修正方向最终由这个增益符号决定。

### 第 5 步：调姿态补偿幅度

当接地重分配方向已经对了，再调：

- `pitch_gain_deg_per_rad`
- `roll_gain_deg_per_rad`

建议方法：

1. 先只调 `pitch_gain_deg_per_rad`
2. 再只调 `roll_gain_deg_per_rad`
3. 最后一起细调

经验上先从小值开始更稳妥：

- `2` 到 `4`：通常只提供轻微修正
- `5` 到 `8`：开始能明显看到姿态补偿
- `8` 以上：要密切注意抖动和耦合

如果姿态反馈一有变化就频繁修正，先别急着减增益，优先看：

- `confidence_alpha` 是否太大
- `contact_deadband` 是否太小
- 关节 ADRC 是否本身过于敏感

### 第 6 步：最后调动作感和速度感

真正决定“修正看起来灵不灵”的，不只是增益，还包括轨迹限速：

- `target_physical_velocity_limit`
- `target_physical_acceleration_limit`

这两个参数在基础姿态层和主动悬挂层里都存在。

建议：

- 两层先保持一致
- 若主动修正想更柔和，先减主动层
- 若预设姿态切换太慢，先加基础层

不要一边让基础层很激进，一边让主动层很保守，否则叠层后的动作会显得别扭。

## 现象和对应处理

### 现象：平地上四轮置信度长期差很多

优先检查：

- 轮半径是否正确
- 某个轮速方向是否反了
- 关节零点是否错了
- 接地估计参数是否过敏

优先改：

- `wheel_radius`
- `slip_ratio_gain`
- `wheel_torque_reference`
- `joint_torque_reference`

### 现象：越障时置信度几乎不变化

说明估计器太钝。

优先改：

- 增大 `slip_ratio_gain`
- 降低 `wheel_torque_reference`
- 降低 `joint_torque_reference`
- 降低 `moving_speed_threshold`

### 现象：关节有修正，但幅度太小

优先改：

- 增大 `contact_rebalance_gain_deg`
- 增大 `pitch_gain_deg_per_rad`
- 增大 `roll_gain_deg_per_rad`
- 增大 `target_physical_velocity_limit`
- 增大 `target_physical_acceleration_limit`

同时确认：

- ADRC 的实际闭环能力够不够
- 关节目标角是否已经被 `min_angle` / `max_angle` 夹死

### 现象：主动修正很明显，但整车发抖

优先改：

- 减小 `contact_rebalance_gain_deg`
- 减小 `pitch_gain_deg_per_rad`
- 减小 `roll_gain_deg_per_rad`
- 增大 `contact_deadband`
- 减小 `confidence_alpha`
- 降低主动层 `target_physical_acceleration_limit`

### 现象：接地差的一侧一直在大力顶，但没有实际效果

优先检查：

- 关节是否已经打到角度上限或下限
- ADRC 是否已经持续饱和
- 机构几何下该方向修正是否本来就不利于重新接地

如果从力矩日志看经常顶满，就需要回头检查关节闭环和机械行程，而不是只继续加主动悬挂增益。

## 当前实现里需要特别注意的点

### 1. 基础层输出的速度和加速度当前没有被主动层直接利用

当前主动层真正用来生成二次轨迹的是：

- 基础目标角
- 主动层自己的 `target_physical_velocity_limit`
- 主动层自己的 `target_physical_acceleration_limit`

所以如果你改了基础层的目标速度和加速度，但主动层自己的限速更严，最终关节动作还是会被主动层卡住。

### 2. 接地置信度同时影响两条链

同一个 `confidence` 不只影响主动悬挂角度修正，也影响轮子扭矩分配。

这意味着：

- 把置信度调得过于敏感，可能悬挂和轮控一起变得神经质。
- 把置信度调得过于迟钝，可能悬挂和轮控一起反应迟缓。

不要把它当成只服务于悬挂的一组参数。

### 3. `switch_torque_limit` 和 `steady_torque_limit` 更适合按“相对效果”理解

在当前链路里，它们的作用是给 ADRC 一个动态约束或激进度提示。

如果你们底层实际力矩映射和这里的数值不是一一对应关系，就按“切换时更猛一些、稳定后更缓一些”的相对效果去调，不要执着于字面物理值。

## 推荐的现场调参流程

1. 先关掉全部主动修正，只调基础姿态和 ADRC。
2. 验证 `pitch`、`roll` 正负方向。
3. 单独调接地估计，先让 `confidence` 有可用差异。
4. 打开 `contact_rebalance_gain_deg`，先把接地方向调对。
5. 再慢慢加 `pitch_gain_deg_per_rad`、`roll_gain_deg_per_rad`。
6. 最后用轨迹限速和死区把“动作感”收顺。

## 最后的判断标准

可以认为主动悬挂“已经调起来了”，至少要满足下面四条：

1. 平地行驶时四轮置信度基本均衡，不持续偏一侧。
2. 单轮失地、压块、打滑时，置信度会出现稳定且可解释的差异。
3. 关节目标角会随 `pitch`、`roll` 和低接地轮位置发生合理差分变化。
4. 修正后的目标角能够被 ADRC 稳定闭过去，不靠持续大幅振荡来实现。
