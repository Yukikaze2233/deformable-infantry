# 悬挂接地置信度组件方案 v1.2

## 0. 文档目的

本文档按当前目录中的实际实现重写，描述的是已经落地的 `GroundContactConfidenceEstimator` 行为，而不是早期草案。

当前相关代码文件：

- `ground_contact_confidence_estimator.cpp`
- `contact_confidence/types.hpp`
- `contact_confidence/motion_model.hpp`
- `contact_confidence/reference_model.hpp`
- `contact_confidence/scoring.hpp`

当前下游直接使用方：

- `load_weight_allocator.cpp`

## 1. 组件定位

`GroundContactConfidenceEstimator` 是主动悬挂下的单关节接地置信度估计组件。

它负责：

- 读取单条悬挂腿的关节测量值、位置设定值、控制输出和 `eso_z3`
- 在线学习该关节自由运动时的参考基线
- 输出单条腿的接地置信度及调试评分

它不负责：

- 控制悬挂关节
- 计算整车支撑模式
- 计算支撑平面
- 直接输出离散接地状态机

当前实现是“每条腿一个实例”的结构，不是四腿集中式融合器。

## 2. 当前实现结构

代码已经拆成 4 个小模块加 1 个总装文件：

- `types.hpp`
  - 公共数据结构、默认常量和配置定义
- `motion_model.hpp`
  - 运动观测、命令活跃判定、自由态判定
- `reference_model.hpp`
  - 自由态样本积累、基线学习、`Warmup -> Ready`
- `scoring.hpp`
  - 四个子分数、入态/持态融合、置信度滤波
- `ground_contact_confidence_estimator.cpp`
  - 参数加载、输入输出注册、总流程调度

## 3. 输入输出接口

### 3.1 输入

当前实现要求以下 6 个输入全部就绪且为有限值：

- `measurement_angle`
  - 默认话题：`/chassis/joint/physical_angle`
- `measurement_velocity`
  - 默认话题：`/chassis/joint/physical_velocity`
- `setpoint_angle`
  - 默认话题：`/chassis/joint/target_physical_angle`
- `control_torque`
  - 默认话题：`/chassis/joint/control_torque`
- `joint_torque`
  - 默认话题：`/chassis/joint/torque`
- `eso_z3`
  - 默认话题：`/chassis/joint/eso_z3`

另外保留 1 个可选兼容输入：

- `setpoint_velocity`
  - 默认话题：`/chassis/joint/target_physical_velocity`

说明：

- 当前版本没有把 `joint_torque` 作为可选输入，它是必选输入。
- 当前版本保留了 `setpoint_velocity` 接口位置，但它不再参与运动观测、自由态学习或评分逻辑，只用于兼容旧拓扑和旧参数配置。
- 默认话题名是占位模板，部署时应给每条腿实例化并改成各自关节话题。

### 3.2 输出

当前实现输出 5 个连续量：

- `ground_contact_confidence`
  - 默认话题：`/chassis/joint/ground_contact_confidence`
- `ground_contact_z3_score`
  - 默认话题：`/chassis/joint/ground_contact_z3_score`
- `ground_contact_block_score`
  - 默认话题：`/chassis/joint/ground_contact_block_score`
- `ground_contact_tracking_score`
  - 默认话题：`/chassis/joint/ground_contact_tracking_score`
- `ground_contact_torque_score`
  - 默认话题：`/chassis/joint/ground_contact_torque_score`

当前没有实现离散输出，如 `Airborne` 或 `Loaded`。

### 3.3 无效输入和预热阶段的输出

如果输入未就绪或存在非有限值：

- 组件直接输出 `NaN`
- 置信度滤波器复位
- 入态/持态逻辑复位

如果 `z3` 参考模型尚未准备完成：

- 同样输出 `NaN`
- 同样复位滤波器和相位逻辑

因此当前实现的语义是：

- `NaN` 表示“当前不可用”
- 不是“当前确定未接地”

## 4. 总体流程

每个 `update()` 周期执行如下流程：

1. 读取 6 个必选输入，并在可用时附带读取可选的 `setpoint_velocity`。
2. 由 `MotionModel` 生成当前运动观测。
3. 用当前观测更新 `ReferenceLearner`。
4. 若 `z3` 参考模型未 `Ready`，则输出 `NaN` 并返回。
5. 计算 `z3 / block / tracking / torque` 四个子分数。
6. 计算 `entry_confidence` 和 `hold_confidence`。
7. 通过 `ConfidencePhaseLogic` 组合成 `confidence_raw`。
8. 通过 `ConfidenceFilter` 做上升/下降速率不同的滤波。
9. 发布最终置信度和 4 个调试分数。

当前结构可以概括为：

`输入 -> 运动观测 -> 自由态参考学习 -> 子分数 -> 入态/持态融合 -> 时间滤波 -> 输出`

## 5. 运动观测层

`MotionModel` 先从输入构造 4 个基础观测量：

- `command_torque = abs(control_torque)`
- `measured_velocity = abs(measurement_velocity)`
- `tracking_error = abs(setpoint_angle - measurement_angle)`
- `progress_velocity = sign(setpoint_angle - measurement_angle) * measurement_velocity`

其中：

- `progress_velocity > 0`
  - 表示当前运动方向有利于减小位置误差
- `progress_velocity < 0`
  - 表示当前运动方向在增大位置误差

### 5.1 命令活跃判定

当前代码中，以下任一条件满足就认为命令活跃：

- `command_torque >= min_command_torque`
- `tracking_error >= min_tracking_error`

默认阈值：

- `min_command_torque = 1.0`
- `min_tracking_error = 0.01`

### 5.2 block_score

当前实现先计算：

- `tracking_score = clamp(tracking_error / min_tracking_error, 0, 1)`
- `progress_score = clamp(max(progress_velocity, 0) / min_progress_velocity, 0, 1)`

然后在命令活跃时计算：

`block_score = clamp(tracking_score * (1 - progress_score), 0, 1)`

含义是：

- 已经存在明显位置误差
- 但关节没有朝目标方向推进，或推进速度不足
- 位置误差越大、推进越弱
- 得分越高表示受阻越明显

默认阈值：

- `min_progress_velocity = 0.05`

如果当前没有活跃命令，则 `block_score` 记为 `0`。

### 5.3 tracking_score

当前实现：

`tracking_score = clamp(tracking_error / min_tracking_error, 0, 1)`

前提是命令活跃；否则记为 `0`。

含义是：

- 跟踪误差一旦达到 `min_tracking_error`
- 分数就会进入饱和区间

因此它更像“误差是否已经明显存在”的指示量，而不是高精度误差度量。

## 6. 自由态参考学习

当前实现的核心不是使用固定阈值，而是先在线学习“该关节自由运动时通常是什么样子”。

### 6.1 参考模型内部状态

`ReferenceModel` 当前保存：

- `free_state_z3_samples`
- `free_state_torque_samples`
- `z3_baseline`
- `torque_baseline`
- `z3_residual_scale`
- `state`

其中样本容器是有界窗口，默认长度：

- `free_state_reference_window_size = 400`

### 6.2 likely_free_state 判据

只有当当前样本被认为“很像自由态”时，才允许拿去更新参考值。

当前判据同时要求：

- 运动表现接近自由运动
- `tracking_error <= free_state_tracking_error_threshold`
- `raw_z3_delta <= free_state_raw_z3_delta_threshold`
- `block_score <= free_state_block_score_threshold`
- `tracking_score <= free_state_block_score_threshold`

其中“运动表现接近自由运动”定义为以下任一成立：

- 当前没有活跃命令
- `progress_velocity >= min_progress_velocity`

默认阈值：

- `min_progress_velocity = 0.05`
- `free_state_tracking_error_threshold = 0.005`
- `free_state_block_score_threshold = 0.2`
- `free_state_raw_z3_delta_threshold = 0.5`

### 6.3 grounded_z3_reference 的作用

代码里保留了一个可选约束：

- `grounded_z3_reference`
- `grounded_z3_reject_margin`

如果配置了 `grounded_z3_reference`，并且当前 `eso_z3` 落在其附近，则该样本不会被当作自由态参考样本使用。

默认值：

- `grounded_z3_reference = NaN`
- `grounded_z3_reject_margin = 0.5`

默认 `NaN` 表示此功能关闭。

### 6.4 样本收集逻辑

当前代码分两步收样：

1. 只要满足 `likely_free_state`，就把当前 `eso_z3` 加入 `free_state_z3_samples`。
2. 只有满足 `free_motion_state` 时，才把 `abs(joint_torque)` 加入 `free_state_torque_samples`。

`free_motion_state` 比 `likely_free_state` 更严格，额外要求：

- 命令活跃
- `measured_velocity >= min_free_motion_velocity`
- 同时仍满足 `likely_free_state`
- 同时远离 `grounded_z3_reference`

这表示：

- `z3` 基线允许在更宽的自由态条件下学习
- `joint_torque` 基线只在“确实存在主动控制且实际运动速度不小”时学习

### 6.5 Ready 条件

当前实现中，组件开始输出结果只要求 `z3` 参考进入 `Ready`，对应条件为：

- `free_state_z3_samples >= min_free_samples_for_ready`
- `z3_baseline` 有效
- `z3_residual_scale` 有效
- `z3_residual_scale >= min_z3_residual_scale`

`torque_ready()` 仍然单独要求：

- `free_state_torque_samples >= min_free_motion_samples_for_torque_ready`
- `torque_baseline` 有效

默认阈值：

- `min_free_samples_for_ready = 200`
- `min_free_motion_samples_for_torque_ready = 100`
- `free_state_z3_residual_quantile = 0.9`
- `min_z3_residual_scale = 0.25`

这意味着当前代码路径中：

- `z3` 参考一旦可用，组件就会开始输出结果
- `joint_torque` 参考可以稍后再补齐
- `hold_confidence` 在早期可能暂时退化为只依赖 `z3_score`

### 6.6 参考值计算方式

当前实现采用：

- `z3_baseline`
  - `free_state_z3_samples` 的均值
- `torque_baseline`
  - `free_state_torque_samples` 的中位数
  - 只有在 `torque_ready()` 后才有效
- `z3_residual_scale`
  - `abs(z3_sample - z3_baseline)` 的 `0.9` 分位数
  - 再与 `min_z3_residual_scale` 取较大值

这意味着 `z3` 的判断不是看绝对值，而是看：

`当前 z3 偏离自由态基线的程度，相对自由态波动尺度是否足够大`

## 7. 子分数定义

### 7.1 z3_score

先计算：

- `raw_z3_delta = abs(eso_z3 - z3_baseline)`
- `normalized_strength = raw_z3_delta / z3_residual_scale`

然后分段映射为 `z3_score`：

- 若 `normalized_strength < normalized_z3_strength_presence_threshold`
  - `z3_score = 0`
- 否则先给一个存在性分值
  - 默认 `normalized_z3_strength_presence_score = 0.7`
- 再向满分阈值线性爬升到 `1.0`

默认参数：

- `normalized_z3_strength_presence_threshold = 1.0`
- `normalized_z3_strength_presence_score = 0.7`
- `normalized_z3_strength_full_scale_threshold = 3.0`

按默认配置理解：

- `z3` 偏离达到 1 倍自由态尺度后，直接给到 `0.7`
- 偏离达到 3 倍自由态尺度后，给到 `1.0`

### 7.2 block_score

直接复用 `MotionModel` 的 `block_score`，描述“有明显位置误差，但没有朝目标方向推进”的程度。

### 7.3 tracking_score

直接复用 `MotionModel` 的 `tracking_score`，描述“位置误差是否已经明显存在”。

### 7.4 torque_score

当前实现不是用绝对力矩，而是用相对自由态基线的超额量：

- `torque_delta = max(0, abs(joint_torque) - torque_baseline)`
- `torque_score = clamp(torque_delta / torque_activation_threshold, 0, 1)`

只有在以下条件下才计算：

- `torque_ready() == true`
- `command_active == true`

默认参数：

- `torque_activation_threshold = 1.0`

因此 `joint_torque` 在当前实现中的角色是：

- 只做辅助加分
- 不做绝对载荷反演

## 8. 置信度融合

当前代码没有把四个分数一次性做总加权平均，而是分成两类：

- `entry_confidence`
  - 接触建立证据
- `hold_confidence`
  - 接触保持证据

### 8.1 entry_confidence

由下面两项组成：

- `block_score`
- `tracking_score`

计算方式是归一化加权平均：

`entry = (block_weight * block + tracking_weight * tracking) / (block_weight + tracking_weight)`

默认权重：

- `block_weight = 0.30`
- `tracking_weight = 0.20`

### 8.2 hold_confidence

由下面两项组成：

- `z3_score`
- `torque_score`

计算方式同样是归一化加权平均：

`hold = (z3_weight * z3 + torque_weight * torque) / (z3_weight + torque_weight)`

默认权重：

- `z3_weight = 0.40`
- `torque_weight = 0.10`

代码里保留了“当其中一个参考未就绪时，把另一项权重顶上去”的兼容分支，因此当前主流程下通常是：

- `z3` 参考就绪后即可开始计算并发布结果
- 若 `torque` 参考尚未就绪，则 `torque_weight` 会临时转移给 `z3_score`
- 等 `torque` 参考就绪后，再恢复 `z3 + torque` 的联合保持证据

### 8.3 相位逻辑

`ConfidencePhaseLogic` 用于处理“是否进入持态”。

当前规则：

- 初始为未持态
- 若 `entry_confidence >= entry_activate_threshold`
  - 进入持态
- 或者若 `hold_confidence >= hold_activate_threshold`
  - 也进入持态
- 已持态时，如果 `hold_confidence < hold_deactivate_threshold`
  - 退出持态

默认阈值：

- `entry_activate_threshold = 0.55`
- `hold_activate_threshold = 0.55`
- `hold_deactivate_threshold = 0.30`

最终输出规则：

- 未持态时：输出 `entry_confidence`
- 已持态时：输出 `max(entry_confidence, hold_confidence)`

这一设计的意图是：

- 接触建立时，更依赖“受阻”和“跟踪误差”
- 接触建立后，更依赖 `z3` / `torque` 的持续支撑证据

## 9. 时间滤波

`ConfidenceFilter` 对原始置信度再做一次一阶滤波，并区分上升和下降速度。

当前实现：

- 上升时使用 `confidence_rise_rate`
- 下降时使用 `confidence_fall_rate`
- `alpha = clamp(rate * dt, 0, 1)`
- `filtered = filtered + alpha * (raw - filtered)`

默认参数：

- `dt = 0.001`
- `confidence_rise_rate = 15.0`
- `confidence_fall_rate = 6.0`

因此默认行为是：

- 接地置信度上升更快
- 脱地置信度下降更慢

输入失效或回到 `Warmup` 时，滤波器会直接复位为 `NaN`。

## 10. 当前实现的典型行为

### 10.1 自由运动阶段

如果关节在自由空间中运动，通常会表现为：

- 若存在活跃命令，则关节运动方向有利于减小位置误差
- `progress_velocity` 为正，且通常不小于 `min_progress_velocity`
- 跟踪误差较小
- `z3` 偏离不大
- `joint_torque` 维持在自由态基线附近

此时组件主要在做两件事：

- 积累自由态参考样本
- 输出较低或不可用的接地置信度

### 10.2 接触建立阶段

当关节开始碰地或受外界约束时，通常先出现：

- 存在位置误差
- `progress_velocity` 下降，甚至变成负值
- `block_score` 升高
- `tracking_score` 升高

这两项优先拉高 `entry_confidence`，用于触发入态。

### 10.3 承载保持阶段

一旦进入持态，若接地约束持续存在，通常会看到：

- `z3_score` 维持较高
- `torque_score` 作为辅助维持项

此时最终置信度更多由 `hold_confidence` 支撑。

### 10.4 脱地阶段

当支撑约束消失后：

- `hold_confidence` 会下降
- 低于 `hold_deactivate_threshold` 后退出持态
- 最终置信度按下降滤波速率衰减

## 11. 当前参数清单

### 11.1 参考学习参数

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `min_command_torque` | `1.0` | 命令活跃判定的最小控制力矩 |
| `min_tracking_error` | `0.01` | 命令活跃和 `tracking_score` 的最小误差阈值 |
| `min_progress_velocity` | `0.05` | 判定“正在朝目标推进”的最小推进速度 |
| `min_free_motion_velocity` | `0.05` | 收集自由运动 `joint_torque` 参考的最小实际速度 |
| `free_state_tracking_error_threshold` | `0.005` | 自由态允许的最大跟踪误差 |
| `free_state_block_score_threshold` | `0.2` | 自由态允许的最大 `block_score` 和 `tracking_score` |
| `free_state_raw_z3_delta_threshold` | `0.5` | 自由态允许的最大原始 `z3` 偏移 |
| `grounded_z3_reference` | `NaN` | 已知接地 `z3` 参考中心，默认关闭 |
| `grounded_z3_reject_margin` | `0.5` | 已知接地 `z3` 参考的排除半宽 |
| `min_free_samples_for_ready` | `200` | `z3` 基线最少样本数 |
| `min_free_motion_samples_for_torque_ready` | `100` | `joint_torque` 基线最少样本数 |
| `free_state_reference_window_size` | `400` | 自由态滑动窗口长度 |
| `free_state_z3_residual_quantile` | `0.9` | `z3` 残差尺度分位数 |
| `min_z3_residual_scale` | `0.25` | `z3` 残差尺度下限 |

兼容说明：

- 如果只配置了旧参数 `min_command_velocity` 而没有配置 `min_progress_velocity`，当前实现会把前者当作后者的回退值使用。

### 11.2 融合参数

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `z3_weight` | `0.40` | `hold_confidence` 中 `z3_score` 权重 |
| `block_weight` | `0.30` | `entry_confidence` 中 `block_score` 权重 |
| `tracking_weight` | `0.20` | `entry_confidence` 中 `tracking_score` 权重 |
| `torque_weight` | `0.10` | `hold_confidence` 中 `torque_score` 权重 |
| `entry_activate_threshold` | `0.55` | 入持态阈值 |
| `hold_activate_threshold` | `0.55` | 用保持证据直接入持态的阈值 |
| `hold_deactivate_threshold` | `0.30` | 退持态阈值 |
| `normalized_z3_strength_full_scale_threshold` | `3.0` | `z3_score` 满分阈值 |
| `normalized_z3_strength_presence_threshold` | `1.0` | `z3_score` 起评分阈值 |
| `normalized_z3_strength_presence_score` | `0.7` | `z3_score` 起评分值 |
| `torque_activation_threshold` | `1.0` | `torque_score` 的满分超额阈值 |

### 11.3 滤波参数

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `dt` | `0.001` | 更新周期，用于计算滤波系数 |
| `confidence_rise_rate` | `15.0` | 置信度上升速度 |
| `confidence_fall_rate` | `6.0` | 置信度下降速度 |

## 12. 与下游模块的关系

当前目录里，`load_weight_allocator.cpp` 已经直接消费以下三个输出：

- `ground_contact_confidence`
- `ground_contact_z3_score`
- `ground_contact_torque_score`

并把它们融合为四条腿的 `load_weight`。

这说明当前接地置信度组件的角色已经比较明确：

- 它是主动悬挂中的腿级观测器
- 它的输出会进一步参与载荷分配
- 但它本身仍不负责整车级几何和模式判断

## 13. 当前实现的限制

### 13.1 上电后不是立即可用

必须先经过在线学习阶段，收集够自由态样本，组件才会开始输出有效置信度。

在此之前输出为 `NaN`，不是 `0`。

### 13.2 torque 参考可能晚于 z3 参考就绪

当前实现不再要求 `torque` 参考与 `z3` 参考同时就绪。

这带来的结果是：

- 组件会更早开始输出有效置信度
- 但在早期阶段，`hold_confidence` 可能暂时只依赖 `z3_score`
- 等 `torque_baseline` 学出来后，`torque_score` 才会参与保持证据

### 13.3 没有离散状态输出

当前只输出连续置信度和 4 个子分数，没有单独输出：

- `Airborne`
- `Touching`
- `Loaded`
- `Jammed`

### 13.4 没有多腿级融合判据

当前实现完全按单腿独立工作，不使用：

- 四腿相对排序
- IMU
- 车体姿态
- 支撑几何

### 13.5 输入掉线会直接复位

一旦输入无效：

- 当前置信度直接变成 `NaN`
- 滤波状态和持态逻辑都会清空

当前没有掉线保持策略。

### 13.6 `joint_name` 当前未参与逻辑

`joint_name` 参数已经被读取，但当前实现里没有参与任何计算或自动话题拼接，只是保留了接口位置。

### 13.7 当前没有直接输出 `progress_velocity`

当前实现内部已经使用 `progress_velocity` 做运动判据，但还没有把它作为调试输出发布。

这意味着在线调参时，需要通过：

- `tracking_score`
- `block_score`
- 原始 `measurement_velocity`
- 原始 `setpoint_angle - measurement_angle`

联合间接判断推进判据是否合理。

## 14. 部署建议

如果按当前实现上线，建议至少做到：

1. 为四条腿各实例化一个 `GroundContactConfidenceEstimator`。
2. 把 6 个必选输入正确映射到对应关节的话题；若仍保留旧链路，也可以继续接 `setpoint_velocity`，但它当前不参与核心计算。
3. 确认上游 ADRC 已经导出每条腿的 `eso_z3`。
4. 给系统预留一段自由运动预热时间，用于收集参考样本。
5. 调试时优先联看 `tracking_score`、`block_score` 和原始误差方向，再看最终 `ground_contact_confidence`。

## 15. 文档结论

当前 `v1.2` 实现的核心思路可以概括为：

`先学习自由态参考，再用位置误差与推进受阻证据完成入态，用 z3 与相对 torque 证据维持持态，最后输出经过相位逻辑与时间滤波的连续接地置信度。`

与旧版草案相比，当前实现的关键特点是：

- 已经从“目标速度驱动判据”切换为“位置误差 + 力矩 + 推进方向”判据
- 已经拆成独立子模块
- 已经具备在线参考学习
- 已经明确区分 `entry_confidence` 和 `hold_confidence`
- 已经允许 `z3` 参考先于 `torque` 参考单独就绪
- 已经接入下游 `load_weight_allocator`
- 仍然保留预热依赖、单腿独立和无离散状态输出这些限制
