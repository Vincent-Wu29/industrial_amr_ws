# 验收证据公开范围

源码仓库保留以下既有验收结果，便于查看项目结论、失败记录和实验范围：

- 本目录的 `final_acceptance_v10_report.md/.json`：最终全量汇总，包含预检和故障演练；正式性能对照仅为两组各30次。
- 本目录的 `history_m4_*_report.md/.json`：v5–v9独立历史报告，包括失败、中断和负预检，不计入v10正式统计。
- [正式实验报告](../experiments/results.md)及其 [figures目录](../experiments/figures/)：60行样本CSV、统计汇总、审计JSON、完整候选参数和三张图。
- [动态障碍修复](../obstacle_repair.md)及其轨迹对比图。

本目录报告从本地 `artifacts/validation/` 原样复制，未重新采样或修改统计。报告中的原始目录或机器路径仅用于说明来源，不是源码运行所需路径。完整日志、逐帧真值、原始代价地图、旧源码快照和诊断输出仍保存在本地，未随源码上传；因此本仓库中的精选报告不能替代完整原始证据审计。

复现步骤见 [README](../../README.md)和[实验结果](../experiments/results.md)。重新运行会在本地生成新的 `artifacts/`；不同批次使用新目录，保留失败样本，不覆盖既有结果。
