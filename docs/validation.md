# 验证记录

更新：2026-10-29。Ubuntu24.04 / ROS2 Jazzy / Gazebo Sim8.11.0 / Cyclone DDS；版本见environment.md。M0–M4本轮验收通过，最终审计complete_and_consistent=true、overall_acceptance_passed=true、issues=[]。这是本项目仿真范围内的结果。

## 已完成结果

| 范围 | 实际结果 |
|---|---|
| 构建与自动化 | 六个ROS包编译/链接/安装通过；colcon **43 tests、0 errors、0 failures、0 skipped** |
| M1固定图 | 15/15目标、14069真值样本、0记录接触 |
| M1实际SLAM | 重采图5082真值样本、0记录接触，保存PGM/YAML及配套站点 |
| M1保存图重启 | 新图重启AMCL后3/3目标、2901真值样本、0记录接触 |
| M2 | 正常批前10/10运输闭环通过 |
| M3正常 | 独立seed42–71，30/30运输、60/60导航，全部EMPTY/真值/实际停车有效、0接触 |
| M3场景矩阵 | 十五场景各5次，75/75语义断言通过、0记录接触、日志完整性问题0 |
| M4系统策略回归 | 15/15通过，仓库请求candidate时effective_profile=baseline，验证范围隔离及既有功能 |
| M4固定窄道对照 | 两组各30，基线30绕行、候选30穿越，均成功/0接触/真值/停车有效；仿真P95 33.092→19.414s，下降41.33% |

最新自动化日志为artifacts/runner_logs/20261029T020135Z/unit.log，正式根含unit.log/test_result.log。43项汇总包含Core32内部用例、Registry8内部检查（各归并一个CTest）、12项实际ROS适配器/CLI用例、原生BT延迟确认、原生collision_monitor及guard、日志、生命周期、资源、统计和新增策略范围测试；汇总计数不等于把所有内部断言简单相加。

仓库保留[汇总报告](evidence/final_acceptance_v10_report.md)、[审计JSON](experiments/figures/v10_acceptance_audit.json)和[证据范围说明](evidence/README.md)。完整本地证据位于 `artifacts/validation/final_acceptance_v10/`，未纳入源码仓库。M4图表、60行CSV、参数、负结果及复现方法见[实验结果](experiments/results.md)。

## 统计与范围

正常独立30单目标RTF1：sim均值84.9262s/P95 85.504s；wall均值88.832269s/P95 94.510426s。前十单同时覆盖M2；不与故障套件正常参考样本合成35条后再宣称独立30单结论。

M3矩阵accepted75：COMPLETED20、CANCELED20、FAILED25、INTERRUPTED10，Nav尝试110；这是故障演练分布，断言通过不等于运输成功。60项真值/停车有效；guard/nav退出与clock_reset共15项明确保持false。20次取消延迟0.199797–1.201378wall秒，均在5s预算内。

M4两项候选数值只用于固定narrow_aisle导航路径（含staging）：global cost_scaling3→40、global marking range5→1m。local仍5m观测，全局障碍层仍启用，物理足迹/余量/速度/监控不变。缩短全局动态标记提前量是已知权衡；数值候选在warehouse货架转角曾失败，因此正式运输仍为baseline。15项新回归证明策略隔离，不用来证明40/1参数在仓库通用。

正式M4只统计m4中的30+30条，预检两条排除；总体报告narrow为31+31是全量保留的结果。目标RTF3，实际均值baseline2.212/candidate2.179；sim、wall、controller CPU总秒分别报告。两组路径均值8.297/6.018m，候选通道内部最小物理侧隙0.209611m。

既有基线生产代码和有效参数未变，M1/M2/M3证据复用。全源码版本2/公共版本1；只允许注册candidate片段、精确harness坐标/策略解析/元数据和资源测试追加约束，旧快照及旧新SHA均保留并逐字符核对。全新复跑若全部为当前单一源码，则无需旧版本快照。当前交付源码、实际world/mode/目标、种子、headless/倍率/RMW和UTC总跨度均已审计。

## 关键修复

- 中文空格路径触发rosidl解析错误：独立带归属标记的Linux构建镜像，不改~/ros_ws。
- 单个原生spawner成组激活joint_state_broadcaster与diff_drive；异步生命周期coordinator超时后重读真实状态，避免重复configure。
- Nav2 BT确认20→1000ms，原生120ms延迟测试覆盖原失败与修复；导航180sim和清理5wall预算未放宽。
- 未分类NavAbort/意外guard失权锁定HOLD，空载也不能靠货物核对解除；晚接受目标仍定向取消，提交日志先flush后发布结果。
- 原动态夹具接触经独立ECM同快照/SAT追踪后增加原生collision_monitor：近障0.80×0.64m停止区、至少2点、完整足迹1.5s预测、scan超时0.4s。旧接触原始数据与修复图保留，见obstacle_repair.md。
- FastDDS两次初始化RPC异常保留，不延长业务预算凑成功；同环境Cyclone对照后正式重采正常30和M3矩阵。具体底层根因未证明，见dds_stability.md。
- M4发现远距全局标记形成虚假屏障，以及通用候选在仓库转角不可用；将数值候选限定固定窄道，记录requested/effective配置并拒绝手动误用，见实验结果及各版预登记。

## 旧失败与已知限制

历史记录独立保留：m1_first_attempt首次采图漂移接触1、m1_mapping_passed修复后实采图；m3_30_happy_pass旧无monitor正常30；m3_dynamic_contact_failure旧动态接触；fastdds_initial_rpc_failures 24成功/26；overnight_interrupted_acceptance、controller_discovery_failure、bt_acknowledgement_failure及高倍率故障批均不进入当前通过率。M4 v5–v9的中断、负预检、入口停机、虚假屏障和仓库边界原始数据已收为history_m4_*，逐组链接见实验结果。

v7首次CLI缺状态用例曾rc1而非预期rc2，原stdout未保留，unit_attempt_1保存失败日志。随后完整测试多次通过，5次独立诊断均UNKNOWN/rc2；raw为artifacts/diagnostics/cli_missing_state_v10，正式根只收summary。曾观测到FakeSystem健康/时钟发布停顿引发预期清理，但原首次根因仍未确定，不声称已彻底修复；测试临时文件现持久化并启用showlocals。

- clock_pause恢复后留制动时间再验证实际静止。time_only重置会移除本机动态AMR，记录CLOCK_RESET并有界失败/退出；退出后truth和停车不可验证，不能把世界消失当作停车证明。合并本轮全部场景共有18条此类上下文失效标记。
- L2假Nav2用例不算物理导航；诊断夹具在活动任务SIGINT结束时曾记录rclcpp结果发布异常，此时不承诺结果交付，以日志/UNKNOWN处理。
- 接受任务缺终态计INTERRUPTED；失败、取消、中断保留分母。没有通过重跑同种子替换失败。
- 几何检测适用于既定平面矩形世界和模型包络。磁盘写满未物理注入，已测不可写目录和IO失败门控。没有真机或安全认证结论。
- M5视觉精停、数据库、HTTP和真机移植为可选扩展，未纳入本轮。
