# 主动悬挂说明与调参指南

## 1. 当前目标

当前 `active_suspension` 目录下的实现，已经按照新的需求重构为以下目标：

1. 让底盘尽量保持水平。
2. 让底盘与四条腿中“轮底最高”的那条腿保持目标距离。
3. 当某个轮子轻载或悬空时，快速向下探地，使其尽快重新接地。

这里“轮底最高”在当前实现中等价为：

`四个角点中，车体到轮底距离最小的那一个角`

因为该角对应的轮底最接近底盘本体，也就是最高的那个轮底。

## 2. 参与文件

- `adaptive_omni_deformable_chassis.cpp`
- `adaptive_omni_contact_estimator.cpp`
- `adaptive_omni_active_suspension.cpp`
- `adaptive_deformable_omni_wheel_controller.cpp`
- `rmcs_bringup/config/deformable-infantry-omni-active-suspension-test.yaml`

本次改造主要落在：

- `adaptive_omni_contact_estimator.cpp`
- `adaptive_omni_active_suspension.cpp`

## 3. 现在这套算法在做什么

### 3.1 上层姿态层

`adaptive_omni_deformable_chassis.cpp` 仍然负责根据遥控器、键盘、旋钮生成基础关节目标：

- `/chassis/*_joint/base_target_physical_angle`

但在当前需求下，这些基础目标的作用已经不再是“最终姿态”，而更接近：

- 提供机械工作区间内的基础站姿
- 提供“目标离地间隙”的参考

主动悬挂层会从这 4 个基础目标里取：

```text
clearance_reference = min(base_target_clearance_i)
```

也就是：

`把四个基础腿长中最短的那个，作为底盘相对最高轮底的目标距离`

### 3.2 接地估计层

`adaptive_omni_contact_estimator.cpp` 现在输出两类量：

- `confidence`
- `load_share`

其中：

- `confidence`：更偏向“这个角当前是不是接地可靠”
- `load_share`：更偏向“四个角当前谁更轻、谁更重”

新增接口：

- `/chassis/left_front_contact/load_share`
- `/chassis/left_back_contact/load_share`
- `/chassis/right_back_contact/load_share`
- `/chassis/right_front_contact/load_share`

当前并没有把关节扭矩直接解释成高精度牛顿值，而是先做相对量。这是因为扭矩返回当前只被认为“与真实扭矩成比例”，不是高精度绝对值。

当前接地估计主路径读取的是独立支撑观测器输出：

- `/chassis/left_front_joint/support_observer_z3`
- `/chassis/left_back_joint/support_observer_z3`
- `/chassis/right_back_joint/support_observer_z3`
- `/chassis/right_front_joint/support_observer_z3`

这样接地估计不再直接依赖关节 ADRC 的 `eso_z3`，可以避免在执行器依赖图里形成
`active_suspension -> ADRC -> contact_estimator -> active_suspension` 的闭环。

另外：

- `load_share` 由相对支撑代理量归一化得到，不是由 `confidence` / `support_score` 归一化得到
- `/chassis/*_contact/normal_force_estimate`
- `/chassis/contact/normal_force_total`

以上两个输出表示相对支撑估计本身，不是 `[0, 1]` 的接地分数。

### 3.3 主动悬挂层

`adaptive_omni_active_suspension.cpp` 现在不再使用“平均高度 + pitch/roll 叠加”的解释。

当前实现使用的核心逻辑是：

1. 将四个关节物理角换算成四个角点的真实竖直距离 `d_i(alpha_i, pitch, roll)`。
2. 取四个角点距离中的最小值作为当前“最高轮底锚点”距离。
3. 以这个最小值去跟踪基础目标中的最小值。
4. 同时用 IMU 的 `pitch / roll` 做水平校正。
5. 对轻载或悬空角启动探地状态机，给该角附加向下伸腿量。
6. 最后将目标竖直距离反解回关节物理角，再发给现有关节 ADRC。

## 4. 核心物理关系

### 4.1 关节角到轮底竖直距离

当前实现使用：

```text
d_i = - (R_world_body(pitch, roll) * p_wheel_bottom_body(alpha_i)).z
```

其中：

- `d_i`：第 `i` 个角轮底到底盘的世界竖直距离
- `R_world_body`：由 `pitch / roll` 生成的车体姿态旋转
- `p_wheel_bottom_body(alpha_i)`：该角轮底在车体系中的位置
- `alpha_i`：关节物理角

代码中对应参数：

- `radius_base`
- `rod_length`
- `body_length`
- `body_width`
- `pivot_offset`
- `arm_length`
- `pivot_z`
- `wheel_bottom_offset`

在车体目标保持水平时，上式退化为：

```text
d_i = arm_length * sin(alpha_i) - pivot_z + wheel_bottom_offset
```

因此目标角反解使用：

```text
alpha_i = asin((d_i + pivot_z - wheel_bottom_offset) / arm_length)
```

### 4.2 水平控制

当前使用 IMU 的：

- `/chassis/imu/pitch`
- `/chassis/imu/roll`

目标固定为：

```text
pitch_reference = 0
roll_reference = 0
```

然后根据左右、前后符号对四条腿做差分修正。

### 4.3 最高轮底锚定

当前不是跟踪平均距离，而是跟踪：

```text
clearance_reference = min(base_target_clearance_i)
clearance_estimate  = min(current_clearance_i)
```

再通过：

```text
anchor_target = clearance_estimate
              + heave_gain * (clearance_reference - clearance_estimate)
```

把最小角点距离拉回目标值。

随后对四个角的目标做统一平移，使得：

```text
min(desired_clearance_i) = clearance_reference
```

这样做的结果是：

`底盘的最低离地约束，始终跟最高轮底那条腿绑定`

### 4.4 探地状态机

对每个角，若满足任一条件：

- `confidence < unload_confidence_threshold`
- `load_share < unload_load_share_threshold`

则进入探地状态。

探地状态下，该角会按固定速度增加探地伸长量：

```text
extension_radius += seek_ground_velocity * dt
```

当满足：

- `confidence > reload_confidence_threshold`
- `load_share > unload_load_share_threshold`

时退出探地状态，并按释放速度逐步收回：

```text
extension_radius -= seek_ground_release_velocity * dt
```

探地量被限制在：

```text
0 <= extension_radius <= max_seek_ground_extension
```

## 5. 当前新增/变更的接口

### 5.1 接地估计新增

- `/chassis/left_front_contact/load_share`
- `/chassis/left_back_contact/load_share`
- `/chassis/right_back_contact/load_share`
- `/chassis/right_front_contact/load_share`

### 5.2 主动悬挂调试输出

- `/chassis/body/heave_reference`
- `/chassis/body/heave_estimate`
- `/chassis/body/pitch_reference`
- `/chassis/body/pitch_estimate`
- `/chassis/body/roll_reference`
- `/chassis/body/roll_estimate`
- `/chassis/body/warp_reference`
- `/chassis/body/warp_estimate`

这些量已经加进：

- `deformable-infantry-omni-active-suspension-test.yaml`

对应的 `ValueBroadcaster` 转发列表中。

## 6. 参数说明

### 6.1 `adaptive_contact_estimator`

- `wheel_radius`
  轮半径，用于轮速运动学换算。

- `confidence_alpha`
  `confidence` 低通滤波系数。

- `moving_speed_threshold`
  超过该速度后，接地估计更多依赖滑移残差。

- `slip_ratio_gain`
  滑移误差惩罚强度。

- `wheel_torque_reference`
  轮扭矩归一化参考值。

- `joint_torque_reference`
  关节扭矩归一化参考值。

- `load_share_floor`
  `load_share` 的最小底值，避免某个角被归一化成 0 导致数值不稳定。

- `confidence_floor`
  接地置信度下限。

### 6.2 `adaptive_active_suspension`

- `radius_base`
  旧几何代理常数，当前主要用于默认值推导，不再直接作为控制量。

- `rod_length`
  旧代理模型长度，当前主要用于兼容旧参数，不再直接定义真实竖直距离。

- `body_length`
  车体前后方向几何尺寸，用于竖直距离投影。

- `body_width`
  车体左右方向几何尺寸，用于竖直距离投影。

- `pivot_offset`
  角点到腿摆杆连接点的水平偏置。

- `arm_length`
  摆杆等效长度。

- `pivot_z`
  摆杆连接点相对车体参考平面的高度。

- `wheel_bottom_offset`
  从腿部几何末端到轮底最低点的固定垂向偏置。

- `pitch_lever_arm`
  `pitch` 转换到角点距离修正时使用的等效前后力臂。

- `roll_lever_arm`
  `roll` 转换到角点距离修正时使用的等效左右力臂。

- `heave_gain`
  最高轮底锚点距离回正增益。

- `pitch_gain_deg_per_rad`
  `pitch` 水平校正增益。

- `roll_gain_deg_per_rad`
  `roll` 水平校正增益。

- `warp_gain`
  对对角扭转误差的抑制强度。

- `unload_confidence_threshold`
  小于该值时，认为该角可能轻载或悬空。

- `reload_confidence_threshold`
  大于该值时，认为该角重新恢复接地。

- `unload_load_share_threshold`
  该角载荷分配低于此阈值时，也触发探地。

- `max_seek_ground_extension`
  单个角允许附加的最大探地伸长量。

- `seek_ground_velocity`
  探地时的附加伸长速度。

- `seek_ground_release_velocity`
  恢复接地后，附加伸长量的回收速度。

- `switch_torque_limit`
- `steady_torque_limit`
- `angle_error_torque_gain`
- `low_confidence_torque_boost`

这四个仍用于关节力矩限幅调度。

## 7. 调参顺序

### 第 1 步：先确认几何与方向

必须先确认：

- 四个关节零点正确
- `physical_angle` 正方向正确
- `pitch`、`roll` 正方向正确
- `body_length`、`body_width`、`pivot_offset`、`arm_length`、`pivot_z`、`wheel_bottom_offset` 至少量级正确

否则后面所有调参都没有意义。

### 第 2 步：先只调水平，不开探地

建议临时设置：

- `max_seek_ground_extension: 0.0`

此时系统退化为：

`最高轮底锚定 + 水平控制`

先调：

- `heave_gain`
- `pitch_gain_deg_per_rad`
- `roll_gain_deg_per_rad`
- `warp_gain`

观察：

- `/chassis/body/heave_reference`
- `/chassis/body/heave_estimate`
- `/chassis/body/pitch_estimate`
- `/chassis/body/roll_estimate`

目标是：

- 平地静止时 `pitch_estimate`、`roll_estimate` 接近 0
- `heave_estimate` 能跟住 `heave_reference`
- 切换基础姿态时不抖、不明显打角

### 第 3 步：再调接地估计

观察：

- `/chassis/*_contact/confidence`
- `/chassis/*_contact/load_share`

目标是：

- 平地静止时，四个 `load_share` 接近静态真实分布
- 某一角被垫高或悬空时，该角 `load_share` 明显变小
- 低速越障时 `confidence` 和 `load_share` 的变化方向符合实际

优先调：

- `joint_torque_reference`
- `wheel_torque_reference`
- `confidence_alpha`
- `load_share_floor`

### 第 4 步：最后打开探地

恢复：

- `max_seek_ground_extension > 0`

建议从小值开始：

- `max_seek_ground_extension: 0.015 ~ 0.025`
- `seek_ground_velocity: 0.12 ~ 0.20`
- `seek_ground_release_velocity: 0.10 ~ 0.18`

目标是：

- 某角轻载后，目标角会快速向下探
- 接地恢复后，不会长时间卡在最大探地量
- 不会因为噪声反复进出探地状态

优先调：

- `unload_confidence_threshold`
- `reload_confidence_threshold`
- `unload_load_share_threshold`
- `seek_ground_velocity`
- `seek_ground_release_velocity`

## 8. 推荐测试工况

### 8.1 平地静止

检查：

- `pitch/roll` 是否压平
- `heave_estimate` 是否贴近 `heave_reference`
- 四个 `load_share` 是否稳定

### 8.2 单角垫块

检查：

- 被垫高角是否成为“最高轮底锚点”
- 底盘是否仍尽量保持水平
- 其余轻载轮是否出现适度探地

### 8.3 单角悬空

检查：

- 对应角 `confidence` 或 `load_share` 是否快速下降
- 探地附加伸长是否迅速增长
- 落地后是否能及时释放探地量

### 8.4 低速越障

检查：

- 底盘是否明显比原先更平
- 高地侧是否作为锚点稳定车体高度
- 低地侧是否主动补腿

## 9. 现阶段限制

当前实现已经满足新的控制方向，但仍有明确边界：

- 它依赖 `confidence + load_share` 判定轻载，不是高精度绝对法向载荷闭环。
- “任意路段都绝对水平”在机械行程和力矩受限时不可能完全保证，只能在当前执行器能力范围内尽量做到。
- 若地形突变超出悬挂总行程，或速度过高导致估计明显滞后，仍然可能出现短时失地。
- 轮控层目前仍主要使用 `confidence`，还没完全升级到按 `load_share` 或绝对 `Fz` 做牵引约束。

## 10. 当前结论

现在这版程序已经不再是“单纯 IMU 调平”。

它的实际行为是：

- 以最高轮底对应的角作为离地锚点
- 用 IMU 持续压 `pitch/roll` 到水平
- 对轻载或悬空角快速探地下压
- 通过现有轨迹器和 ADRC 将目标稳定落到关节执行层

如果后续还要继续升级，最自然的下一步是：

1. 把 `load_share` 升级成更可信的相对法向载荷估计
2. 让轮控层也显式使用 `load_share`
3. 再考虑更强的地形记忆或绝对法向载荷观测
