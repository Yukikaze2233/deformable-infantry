# Draft: Referee Protocol Match Analysis

## Requirements (confirmed)
- 分析两个 RoboMaster 2026 通信协议 PDF 中与裁判系统串口相关的协议内容。
- 对照当前代码库中的裁判系统串口实现，判断是否匹配。
- 重点关注：帧结构、CRC、命令字/数据 ID、上下行数据定义、交互数据与可能的版本差异。
- 用户补充：主要关注“包内数据帧不一样的地方”，不要把纯解释性文字增补误判为协议差异。

## Technical Decisions
- 先分别提取两份 PDF 中与裁判系统串口直接相关的协议内容，再与代码中的解析/编码实现对照。
- 代码侧主要关注 `rmcs_core` 及相关包中的 referee serial parse/encode 路径。
- 在代码结果返回前，先给出一版“仅基于两份 PDF 差异的文档侧预判”。
- 预判时只把真实 wire format / 帧内字段变化算作协议差异；解释性补充单独标注。

## Research Findings
- 已启动代码库探索代理，正在收集 referee serial 相关实现位置与协议细节。
- 代码探索代理因 30 分钟无活动超时失败，尚未拿到代码侧 referee serial 的结构化结果。
- `look_at` 无法直接提取 PDF 内容，需改用文件读取方式获取 PDF 文本。
- 已成功通过本地只读方式提取两份 PDF 文本。
- 从 V1.0.0 提取到：自定义客户端协议基于 MQTT + Protobuf，并包含 `RemoteControl`、`CustomByteBlock(机器人端对应 0x0310)`、CRC8/CRC16 示例、机器人/选手端 ID 说明等。
- 从 V1.3.0 提取到：自定义客户端协议拆分为 `KeyboardMouseControl` + `CustomControl(通过图传链路以 0x0311 命令字发送给机器人)`，并出现更多状态同步/控制命令定义，CRC8/CRC16 附录仍存在。
- 初步观察到 V1.0.0 与 V1.3.0 存在协议层变化：RemoteControl 拆分、CustomByteBlock 容量变化、若干 message 名称/字段/枚举语义有调整。

## Open Questions
- 用户当前未限定输出形式：可提供“口头分析结论”或进一步整理成正式 work plan。

## Scope Boundaries
- INCLUDE: 协议文档与当前代码实现的一致性分析。
- EXCLUDE: 直接修改代码或实现适配。
