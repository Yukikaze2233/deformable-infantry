#import "/rmcs_notebook/template/template.typ": *

== 文档说明

本文档用于整理 deformable infantry 系列底盘的机构、解算与控制。其目标不是重复源码注释，而是以统一的数学符号把以下问题讲清楚：

- v1 丝杆机构中，电机角、丝杆位移、关节角、轮组高度之间如何映射；
- v2 直驱关节中，电机角、物理角、局部伺服与主动悬挂之间如何分层；
- deformable infantry omni 与 v2 舵轮底盘中，底盘自由度与轮系约束如何解算；
- 主动悬挂中，柔顺接地、姿态控制与轮压优化三层控制之间如何衔接。

== 阅读建议

若第一次阅读 deformable infantry 文档，建议顺序为：

1. v1 丝杆机构与解算；
2. v2 直驱关节控制；
3. 主动悬挂；
4. omni 底盘解算；
5. v2 舵轮解算。

== 与现有 RMCS 文档的关系

本文档与以下资料互补：

- `plan.md`：主动悬挂控制目标与分层架构；
- `code-plan.md`：ADRC 方案与实现路径；
- `todo.md`：当前实现进度、调参结果与未解决问题；
- `deformable_infantry_omni_active_suspension_flow.md`：当前 omni 配置下的实际控制流。

本文档不会复述所有工程细节，而是优先解释“为什么这么建模”和“公式从哪里来”。
