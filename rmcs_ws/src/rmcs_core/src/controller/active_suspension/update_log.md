# 主动悬挂更新日志

后续所有针对 `active_suspension` 目录的更新说明，统一追加到本文件中。

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
