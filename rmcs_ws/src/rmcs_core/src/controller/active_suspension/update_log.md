# 主动悬挂更新日志

后续所有针对 `active_suspension` 目录的更新说明，统一追加到本文件中。

## v0.3.3

日期：2026-04-23

### 本次更新

#### 1. 将接地观测主路径从 ADRC `eso_z3` 解耦为独立支撑观测器

问题：

- `adaptive_omni_contact_estimator.cpp` 原先直接读取关节 ADRC 输出的：
  - `/chassis/*_joint/eso_z3`
- 而 ADRC 本身又依赖主动悬挂输出的：
  - `/chassis/*_joint/target_angle`
  - `/chassis/*_joint/torque_limit`
- 在 active-suspension 测试配置里，这会形成执行器依赖环：

```text
AdaptiveOmniActiveSuspension
  -> ADRC
  -> AdaptiveOmniContactEstimator
  -> AdaptiveOmniActiveSuspension
```

影响：

- `rmcs_executor::Executor` 拓扑排序时会直接判定为循环依赖
- 配置无法正常启动

修复：

- 新增独立组件：
  - `adaptive_omni_joint_support_observer.cpp`
- 该组件只读取硬件侧无环路信号：
  - `/chassis/*_joint/physical_angle`
  - `/chassis/*_joint/torque`
- 输出新的独立观测 topic：
  - `/chassis/*_joint/support_observer_z3`
- `adaptive_omni_contact_estimator.cpp` 改为读取这些新 topic

结果：

- 接地估计不再依赖 ADRC 闭环输出
- 主动悬挂测试配置的依赖图恢复为 DAG

#### 2. 修正 `load_share` 和 `normal_force_estimate` 的输出语义

问题：

- `load_share` 之前实际是由 `support_score` 归一化得到
- `/chassis/*_contact/normal_force_estimate` 之前实际发布的也是 `support_score`
- 这会把“相对支撑大小”和“接地置信分数”混成一个量

修复：

- `load_share` 改为由 `support_proxy` 主路径归一化得到
- 当独立 observer 暂不可用时，回退到 legacy 支撑代理量
- `/chassis/*_contact/normal_force_estimate`
- `/chassis/contact/normal_force_total`

现在发布的是相对支撑估计本身，而不是 `[0, 1]` 的分数

涉及文件：

- `adaptive_omni_joint_support_observer.cpp`
- `adaptive_omni_contact_estimator.cpp`
- `plugins.xml`
- `deformable-infantry-omni-active-suspension-test.yaml`
- `tuning_guide.md`

## v0.3.2

日期：2026-04-22

### 本次更新

#### 1. 将接地基线升级为“姿态相关参考 + shock gating + 准静态学习窗口”

问题：

- 上一版 `adaptive_omni_contact_estimator.cpp` 已经去掉了 `mass / joint_b0` 对主判据的依赖
- 但仍然存在两个明显短板：
  - 单腿参考仍然是标量基线，无法区分“正常上坡后的静态载荷转移”和“异常失载”
  - 基线会持续学习，如果在突变扭矩、越障冲击、快速姿态变化时更新，容易把瞬态异常学成正常状态

本次思路：

1. 把单腿参考从常数升级为：

```text
Ref_i(alpha_i, pitch, roll)
```

- 这里不引入质量、惯量、质心位置这类非长度机械参数
- 参考模型只使用：
  - 关节物理角 `alpha_i`
  - IMU `pitch`
  - IMU `roll`
- 参考值通过在线低阶模型学习，用来表达：
  - 同一条腿在不同摆杆角度下的正常支撑变化
  - 上坡、横坡、车体静态变形时的正常载荷转移

2. 引入 `shock gating`

- 当出现以下任一情况时，冻结参考学习：
  - `support_proxy` 突跳
  - `pitch / roll` 变化过快
  - 关节物理角变化过快
  - 轮速残差显著突增
- 这样做的目的不是停用接地估计
- 而是避免把瞬态冲击当成新的正常支撑分布写进参考模型

3. 只在准静态窗口内更新参考

- 只有在以下条件同时满足时，才允许学习参考模型：
  - 底盘控制速度较低
  - `pitch / roll` 变化率较低
  - 关节角速度较低
  - 轮速残差较低
  - 且该状态持续一段最小时间
- 这样参考模型只在“稳定、可信”的窗口里学习

更新：

- `adaptive_omni_contact_estimator.cpp`
  - 新增姿态相关参考模型 `Ref_i(alpha_i, pitch, roll)`
  - 全车总支撑参考改为由四条腿的姿态参考求和得到
  - 新增 `shock gating` 状态
  - 新增准静态窗口计时器
  - 参考模型只在 `quasi-static && !shock_active` 时更新
- `deformable-infantry-omni-active-suspension-test.yaml`
  - 新增参数：
    - `shock_support_jump_ratio`
    - `shock_angle_rate_threshold_deg`
    - `shock_joint_rate_threshold_deg`
    - `shock_residual_threshold`
    - `shock_hold_time`
    - `quasi_static_speed_threshold`
    - `quasi_static_angle_rate_threshold_deg`
    - `quasi_static_joint_rate_threshold_deg`
    - `quasi_static_residual_threshold`
    - `quasi_static_hold_time`
    - `reference_update_load_share_threshold`
  - 同时重新调整了相对支撑比阈值：
    - `local_support_low_ratio`
    - `local_support_high_ratio`
    - `global_support_low_ratio`
    - `global_support_high_ratio`

效果：

- 上坡或静态变形后，接地参考可以逐步适应新的正常载荷分布
- 但越障冲击、扭矩突变、快速姿态变化不会立刻污染基线
- 直接改善的是：
  - 四条腿失载判定
  - 探地触发时机
  - 复杂姿态下的接地估计鲁棒性
- 不直接改变：
  - 最高轮底锚点高度控制
  - 底盘水平控制主公式

涉及文件：

- `adaptive_omni_contact_estimator.cpp`
- `deformable-infantry-omni-active-suspension-test.yaml`

## v0.3.1

日期：2026-04-22

### 本次更新

#### 1. 将接地估计从“绝对力标定”改为“相对支撑基线”

背景：

- 当前主动悬挂的真实控制目标不是“精确闭环绝对法向力”
- 而是：
  - 在机械允许范围内尽量让轮子重新接地
  - 让最高轮底对应的锚点高度保持在目标范围
  - 同时保持底盘水平

原实现的问题：

- `adaptive_omni_contact_estimator.cpp` 中接地判据依赖：
  - `mass`
  - `joint_b0`
  - 一组按 `m g` 比例定义的阈值
- 这意味着：
  - 接地估计是否稳定，部分取决于整车质量标定是否正确
  - `z3` 是否能映射成“看起来合理的支撑力”，部分取决于 `b0` 是否正确
- 这和当前控制目标并不完全一致，因为主动悬挂真正需要的是：
  - 哪条腿相对更轻
  - 全车是否整体失去支撑
  - 哪条腿应该进入探地

如果只做简单删除，会出现什么问题：

- 如果只是把 `mass` 删掉，保留 `load_share` 作为唯一接地依据
- 那么当四条腿同时变轻时：
  - 四个 `load_share` 仍然可能接近 `0.25`
  - 系统看不到“整车整体支撑变差”
  - 这会伤到“尽量让轮子接地”的目标

本次思路：

- 不再把 `z3` 解释成依赖 `b0` 标定的绝对支撑力
- 改为使用：

```text
support_proxy_i = |z3_i| / jacobian_i
```

- 这里仍然保留几何长度信息，因为局部雅可比本质上来自悬挂几何
- 但不再使用质量、输入增益这类非长度机械参数去做绝对量标定

为了保持控制目标可实现，增加了两层相对判据：

1. 每条腿自己的支撑基线 `support_reference_i`
   - 用来判断该腿当前是“相对正常支撑”还是“相对失载”
2. 全车总支撑基线 `total_support_reference`
   - 用来判断四条腿是否整体同时变轻

这样做的目的：

- 保留“谁更轻、谁该探地”的判别能力
- 同时补上“全车一起失支撑”的检测能力
- 避免退化成只看 `load_share`，从而漏检整体卸载

更新：

- 删除 `adaptive_contact_estimator` 中对以下参数的主路径依赖：
  - `mass`
  - `joint_b0`
  - `support_force_deadband`
  - `min_contact_force_fraction`
  - `full_contact_force_fraction`
  - `total_support_low_ratio`
  - `total_support_high_ratio`
  - `legacy_force_blend`
  - `max_support_force_ratio`
- 接地估计主路径改为：
  - `z3 + 几何雅可比 -> support_proxy`
  - `support_proxy / support_reference -> local support score`
  - `sum(support_proxy) / total_support_reference -> global support score`
  - `local/global support score + load_share + slip residual -> confidence`
- 保留旧轮/关节扭矩路径作为 fallback，但只作为相对分数，不再映射成绝对力

新增参数：

- `support_reference_rise_alpha`
  基线在支撑变强时的跟踪速度。
- `support_reference_fall_alpha`
  基线在支撑变弱时的跟踪速度。
- `local_support_low_ratio`
  单腿相对支撑比的弱接地阈值。
- `local_support_high_ratio`
  单腿相对支撑比的强接地阈值。
- `global_support_low_ratio`
  全车相对总支撑比的低支撑阈值。
- `global_support_high_ratio`
  全车相对总支撑比的正常支撑阈值。

输出语义变化：

- `/chassis/*_contact/normal_force_estimate`
- `/chassis/contact/normal_force_total`

以上两个输出保留原 topic 名以维持接口兼容，但它们现在表示：

- 相对支撑估计
- 不再承诺是高精度绝对牛顿值

这样是否影响控制目标：

- 不影响以下目标：
  - 最高轮底锚点高度控制
  - 车体水平控制
  - 轻载腿探地
- 原因是这些目标本质依赖：
  - 几何关系
  - 哪条腿相对轻载
  - 全车是否整体失支撑
- 它们并不要求绝对质量标定

涉及文件：

- `adaptive_omni_contact_estimator.cpp`
- `deformable-infantry-omni-active-suspension-test.yaml`

## v0.3.0

日期：2026-04-22

### 本次更新

#### 1. 将接地估计主路径切换为 `z3 -> 支撑力 -> load_share / confidence`

问题：

- 旧版 `adaptive_omni_contact_estimator.cpp` 主要依赖：
  - 轮扭矩归一化
  - 关节扭矩归一化
  - 轮速残差
- 这种方法本质是经验打分：
  - `support_proxy` 不是物理支撑力
  - `load_share` 只是代理量归一化
  - 对 `wheel_torque_reference / joint_torque_reference` 非常敏感

更新：

- 新增从关节 ADRC `ESO z3` 读取扰动估计的主路径
- 利用悬挂几何的局部雅可比，把 `z3 / b0` 映射为每条腿的等效竖直支撑力估计
- 新增输出：
  - `/chassis/*_contact/normal_force_estimate`
  - `/chassis/contact/normal_force_total`
- `load_share` 改为直接由支撑力归一化得到
- `confidence` 改为：
  - 低速时更依赖支撑力存在度
  - 运动时由支撑力存在度和轮速残差共同决定

兼容策略：

- 保留旧版轮/关节扭矩代理量路径作为 fallback
- 当 `z3` 或关节物理角不可用时，仍可退回经验估计，不会直接失效

涉及文件：

- `adaptive_omni_contact_estimator.cpp`
- `deformable-infantry-omni-active-suspension-test.yaml`

#### 2. 为通用 ADRC 控制器补充可选 ESO 观测输出

问题：

- 当前接地估计需要读取各关节内环的 `z3`
- 但 `rmcs_core::controller::adrc::AdrcController` 原本只输出 `control`

更新：

- 通用 ADRC 控制器新增可选输出参数：
  - `eso_z2_output`
  - `eso_z3_output`
- 在 active-suspension 测试配置中，把四个关节 ADRC 的 `eso_z3_output` 接到了：
  - `/chassis/left_front_joint/eso_z3`
  - `/chassis/left_back_joint/eso_z3`
  - `/chassis/right_back_joint/eso_z3`
  - `/chassis/right_front_joint/eso_z3`

涉及文件：

- `adrc_controller.cpp`
- `deformable-infantry-omni-active-suspension-test.yaml`

#### 3. 清理未参与计算的保留参数

更新：

- 从 `AdaptiveOmniActiveSuspension` 中删除未参与计算的参数：
  - `rod_length`
  - `contact_rebalance_gain_deg`
- 从 `AdaptiveOmniContactEstimator` 中删除未参与计算的参数：
  - `pivot_z`
  - `wheel_bottom_offset`

说明：

- `pivot_offset` 仍然保留，因为它还参与 `body_length / body_width` 的默认值推导
- `pivot_z` 与 `wheel_bottom_offset` 在主动悬挂主控里仍然有效，只是在新的接地估计器里当前没有进入公式

涉及文件：

- `adaptive_omni_active_suspension.cpp`
- `adaptive_omni_contact_estimator.cpp`
- `deformable-infantry-omni-active-suspension-test.yaml`

#### 4. 按功能分块整理参数声明

更新：

- 将 `AdaptiveOmniActiveSuspension` 与 `AdaptiveOmniContactEstimator` 底部的参数声明按功能分组：
  - 几何参数
  - 轨迹/增益参数
  - 状态机阈值
  - 限扭调度
  - 接地估计阈值与滤波参数

目的：

- 便于维护
- 便于核对“参数是否真的进入当前计算链”
- 避免所有 `const double` 连成一整段难以辨认

### 当前参数说明

以下说明按“当前仍生效的参数名”整理。

#### 1. `adaptive_active_suspension`

- `radius_base`
  底盘基准半径。当前主要用于给几何默认值提供参考。
- `pivot_offset`
  车体角点到关节安装点沿径向的偏移，用于确定每条腿的关节位置。
- `body_length`
  车体前后等效长度，用于定义四个角点的前后位置。
- `body_width`
  车体左右等效宽度，用于定义四个角点的左右位置。
- `arm_length`
  关节到腿末端的主力臂长度，参与角度到轮底 clearance 的正解与反解。
- `pivot_z`
  关节安装点在车体系中的竖直高度。
- `wheel_bottom_offset`
  从腿末端到轮底接地点的竖直偏移。
- `pitch_lever_arm`
  将四角 clearance 的前后差分换算成 `pitch` 时使用的等效力臂。
- `roll_lever_arm`
  将四角 clearance 的左右差分换算成 `roll` 时使用的等效力臂。
- `min_angle`
  关节物理角下限。
- `max_angle`
  关节物理角上限。
- `target_physical_velocity_limit`
  关节目标轨迹的物理角速度上限。
- `target_physical_acceleration_limit`
  关节目标轨迹的物理角加速度上限。
- `heave_gain`
  锚点高度回到基础参考高度的增益。
- `warp_gain`
  抑制四角扭曲模态 `warp` 的增益。
- `unload_confidence_threshold`
  接地置信度低于该值时，认为该腿可能轻载/离地，触发探地。
- `reload_confidence_threshold`
  接地置信度高于该值时，允许退出探地。
- `unload_load_share_threshold`
  承载占比低于该值时，也会触发探地。
- `max_seek_ground_extension`
  探地状态下额外向下伸的最大 clearance。
- `seek_ground_velocity`
  探地时增加伸腿量的速度。
- `seek_ground_release_velocity`
  退出探地后回收伸腿量的速度。
- `contact_deadband`
  探地状态机的迟滞宽度，用于避免在阈值附近来回抖动。
- `pitch_gain_deg_per_rad`
  用 IMU `pitch` 做水平修正时的增益。
- `roll_gain_deg_per_rad`
  用 IMU `roll` 做水平修正时的增益。
- `switch_torque_limit`
  目标角切换后的瞬时高限扭上限。
- `steady_torque_limit`
  稳态限扭基线。
- `angle_error_torque_gain`
  角度误差越大，允许的稳态限扭越高。
- `low_confidence_torque_boost`
  接地置信度越低，允许的稳态限扭越高，方便探地。

已删除：

- `rod_length`
  当前算法未使用。
- `contact_rebalance_gain_deg`
  当前算法未使用。

#### 2. `adaptive_contact_estimator`

- `wheel_radius`
  由底盘速度推期望轮速时使用的轮半径。
- `radius_base`
  底盘基准半径。当前主要作为默认几何链条的参考量。
- `pivot_offset`
  车体角点到关节安装点的径向偏移。当前保留，主要用于默认几何推导。
- `body_length`
  车体前后等效长度，用于定义四角位置和径向方向。
- `body_width`
  车体左右等效宽度，用于定义四角位置和径向方向。
- `arm_length`
  参与 `z3 -> support_proxy` 映射的局部雅可比计算。
- `confidence_alpha`
  `confidence` 的一阶低通滤波系数。
- `support_force_alpha`
  `support_proxy` 的一阶低通滤波系数。
- `support_reference_rise_alpha`
  相对支撑基线在支撑增强时的跟踪速度。
- `support_reference_fall_alpha`
  相对支撑基线在支撑减弱时的跟踪速度。
- `moving_speed_threshold`
  底盘速度高于该值时，`confidence` 计算中更强调轮速残差。
- `slip_ratio_gain`
  轮速残差对 `slip_score` 的惩罚强度。
- `wheel_torque_reference`
  旧版轮扭矩代理量的归一化参考值。当前主要用于 fallback。
- `joint_torque_reference`
  旧版关节扭矩代理量的归一化参考值。当前主要用于 fallback。
- `local_support_low_ratio`
  单腿相对支撑比偏低时的阈值。
- `local_support_high_ratio`
  单腿相对支撑比恢复正常时的阈值。
- `global_support_low_ratio`
  全车相对总支撑比偏低时的阈值。
- `global_support_high_ratio`
  全车相对总支撑比恢复正常时的阈值。
- `load_share_floor`
  `load_share` 归一化底值，防止数值退化到 0。
- `confidence_floor`
  `confidence` 下限，防止状态机过于激进。

已删除：

- `mass`
  当前 estimator 主路径不再使用整车质量做绝对力标定。
- `joint_b0`
  当前 estimator 主路径不再使用 ADRC 名义输入增益做绝对力标定。
- `pivot_z`
  当前 estimator 版本未参与计算。
- `wheel_bottom_offset`
  当前 estimator 版本未参与计算。
- `support_force_deadband`
  已不再使用绝对力死区。
- `min_contact_force_fraction`
  已由 `local_support_low_ratio` 取代。
- `full_contact_force_fraction`
  已由 `local_support_high_ratio` 取代。
- `total_support_low_ratio`
  已由 `global_support_low_ratio` 取代。
- `total_support_high_ratio`
  已由 `global_support_high_ratio` 取代。
- `legacy_force_blend`
  当前 estimator 主路径不再混合绝对力域路径。
- `max_support_force_ratio`
  当前 estimator 主路径不再做绝对力上限限幅。

#### 3. 与本次链路相关的 ADRC 额外参数

- `eso_z2_output`
  可选，把关节 ADRC 的速度观测量 `z2` 输出到指定 topic。
- `eso_z3_output`
  可选，把关节 ADRC 的总扰动观测量 `z3` 输出到指定 topic。

当前 active-suspension 接地估计主路径只依赖：

- `eso_z3_output`

### 相关文件

- `adaptive_omni_active_suspension.cpp`
- `adaptive_omni_contact_estimator.cpp`
- `adrc_controller.cpp`
- `deformable-infantry-omni-active-suspension-test.yaml`

## v0.2.0

日期：2026-04-19

### 本次修复

#### 1. 修复插件导出缺失导致的组件加载失败

问题：

- 测试配置中已经引用：
  - `rmcs_core::controller::active_suspension::AdaptiveOmniDeformableChassis`
  - `rmcs_core::controller::active_suspension::AdaptiveOmniContactEstimator`
  - `rmcs_core::controller::active_suspension::AdaptiveOmniActiveSuspension`
  - `rmcs_core::controller::active_suspension::AdaptiveDeformableOmniWheelController`
- 但这些类没有出现在 `rmcs_core/plugins.xml`
- 执行器通过 `pluginlib::ClassLoader` 按类型名加载时会直接失败

修复：

- 在 `plugins.xml` 中补充导出上述 4 个类

涉及文件：

- `plugins.xml`

#### 2. 修复 `heave_gain` 被公共平移抵消的问题

问题本质：

- `adaptive_omni_active_suspension.cpp` 这个模块本来应同时承担两类功能：
  1. 控制“底盘相对最高轮底的目标竖直距离”
  2. 控制 `pitch / roll / warp` 这些差分模态
- 原实现先计算：

```text
anchor_target = estimated + heave_gain * (reference - estimated)
```

- 这一步本来是在构造公共高度目标
- 但后面又做：

```text
anchor_shift = clearance_reference - min(desired_clearance)
desired_clearance_i += anchor_shift
```

- 结果会把所有腿整体平移，使得最小值重新等于 `clearance_reference`
- 这样前面的公共高度项被整体抵消，最终只剩相对差分形状

造成的现象：

- `heave_gain` 参数存在
- 文档和配置都认为它在控制锚点高度
- 但在未饱和区间里，它几乎不影响输出

修复：

- 将整体平移改成：

```text
anchor_shift = anchor_target - min(desired_clearance)
```

- 这样最小腿长会对齐到 `anchor_target`
- `anchor_target` 才真正成为主动悬挂的公共高度目标
- `heave_gain` 因而真正参与控制

修复后的模块职责：

- `AdaptiveOmniActiveSuspension`
  负责把“最高轮底锚点高度目标”和“水平控制目标”合成为最终四腿目标

涉及文件：

- `adaptive_omni_active_suspension.cpp`

#### 3. 修复探地状态机对 `load_share` 没有迟滞的问题

问题本质：

- `adaptive_omni_contact_estimator.cpp` 负责输出连续的接地指标：
  - `confidence`
  - `load_share`
- `adaptive_omni_active_suspension.cpp` 负责根据这些量判断“轻载/悬空”，再决定是否进入探地状态
- 原实现中：

```text
进入条件：load_share < unload_load_share_threshold
退出条件：load_share > unload_load_share_threshold
```

- 进入和退出围绕同一个阈值，没有迟滞区

造成的现象：

- 当 `load_share` 在阈值附近噪声波动时
- 探地状态机会在每几个周期内反复切换
- 腿会出现“探地 / 回收 / 再探地 / 再回收”的抖动

这本来应该由哪个模块负责：

- `AdaptiveOmniContactEstimator`
  负责提供方向可信但带噪声的连续接地指标
- `AdaptiveOmniActiveSuspension`
  负责把连续指标变成稳定的事件判定

也就是说：

- 接地估计器可以有噪声
- 主动悬挂状态机不能把噪声当作开关沿

修复：

- 使用现有 `contact_deadband` 作为 `load_share` 的退出迟滞宽度
- 改成：

```text
进入：load_share < unload_load_share_threshold
退出：load_share > unload_load_share_threshold + contact_deadband
```

效果：

- 阈值附近的小幅波动不会立即退出探地
- 探地状态更稳定

修复后的模块职责：

- `AdaptiveOmniActiveSuspension`
  负责对 `confidence / load_share` 做带迟滞的探地状态判定

涉及文件：

- `adaptive_omni_active_suspension.cpp`

### 需求说明

以下行为变化经确认属于需求变更，不计为 bug：

- 上游原有“前高后低 / 前低后高 / uphill”这类非对称基础姿态模式，不再要求保留

当前主动悬挂层以新的需求为准：

- 以最高轮底对应的最小竖直距离作为锚点
- 以 `pitch = 0`、`roll = 0` 为水平目标
- 对轻载或悬空腿执行探地下压

### 相关文件

- `plugins.xml`
- `adaptive_omni_active_suspension.cpp`
- `adaptive_omni_contact_estimator.cpp`
- `deformable-infantry-omni-active-suspension-test.yaml`
