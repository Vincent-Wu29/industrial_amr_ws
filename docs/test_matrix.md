# 测试矩阵与证据层级

领域用例在 `src/amr_mission/test/core_tests.cpp`，实际 ROS2 测试在同包的 `test_ros_integration.py` 和 `test_motion_guard.py`。物理场景由 `scenario_runner` 驱动，每轮生成原始产物。具体已跑数量见 validation.md；存在测试代码不代表所有物理验收已通过。

| 设计用例 | 自动化实现 | 层级与边界 |
|---|---|---|
| T01 正常运输 | 领域主流程、ROS Action/独立确认、happy | L1/L2/L3 |
| T02 动态/临时障碍 | crossing / temporary，Gazebo实体+碰撞体+真值；原生激光collision_monitor停车/减速/失效闭锁 | L2原生监控+L3；物理结果见validation.md |
| T03 永久封闭 | permanent，四面实体围住 B02 | L3正式5次通过预期FAILED/LOADED/HOLD断言 |
| T04 非法站点/配置 | C++ registry_tests、非法 ID/类型测试 | L1 + 启动校验 |
| T05 去重 | 活动/终态重复与查询；duplicate 场景 | L1/L2/L3 |
| T06 冲突/互斥 | ID 冲突和新 ID BUSY | L1；串行 executor 原子占槽 |
| T07 各阶段取消 | 领域取消、L2 等待窗、四个 cancel_* 场景 | L1/L2/L3 |
| T08 晚接受/发送超时 | 领域晚接受和超时保留清理、L2 延迟接受 | L1/L2 |
| T09 成功与取消竞争 | 领域固定事件次序 | L1，确定性验证两种排序 |
| T10 取消拒绝/结果缺失/odom 过期 | 领域清理超时、L2 拒绝取消/无终态、停发 odom | L1/L2 |
| T11 装卸异常 | 领域超时/重复/冲突/失败；failure、silent 设备 | L1 + L3 device_failure/load_timeout |
| T12 旧会话/交互 | 领域精确关联校验 | L1 |
| T13 Nav2 未就绪/拒绝/退出 | readiness 门控、L2 goal 拒绝、nav_exit | L1/L2/L3 |
| T14 Mission/guard 退出 | guard 心跳集成测试、guard_exit fail-fast | L2/L3；世界退出后不宣称继续观测物理停止 |
| T15 时钟/传感器过期 | 领域回拨/墙钟预算、guard 新鲜度、readiness | L1/L2/L3；回拨后世界失效边界单独记录 |
| T16 重启 UNKNOWN/HOLD | 新 Core 不继承历史，默认节点 HOLD | L1；真实 ROS 节点默认参数遵循此规则 |
| T17 日志不可写 | 领域故障锁定；节点启动/每次 flush 检查并 fail-fast | L1/L2；真实不可写目录测试，未模拟磁盘写满 |
| T18 有序轨迹/单终态 | 领域终态唯一；metrics 检查重复接受、重复终态、序号与坏 JSON | L1/离线；中断按 INTERRUPTED 计入 |

几何监测使用 Gazebo 独立位姿与完整矩形包络作分离轴检测，排除轮地正常接触；只适用于本项目既定平面几何。传感器/地图资源、BT 恢复上限、QoS/stamped 参数和统计分母另有 pytest。

目标门槛：M1 15 段导航 + 保存地图后 3 段复跑；M2 10/10成功；M3正常至少30单、成功率≥95%、可验证轨迹零接触，各故障至少5次且语义断言全部通过；M4每配置至少30个同种子样本并满足预登记安全与性能约束。正常批失败保留分母，仅补齐缺失种子，不重跑替换失败。未达到数量或未运行物理专项时必须保留为待验收。


2026-10-29本轮实际通过：43项自动化、M1 15+3目标及实采地图、M2前10正常样本、M3正常30/30与场景75/75；M4正式30+30全过，simP95降低41.33%，记录接触0。新增15项系统策略回归全部通过，仓库请求candidate时实际为baseline；40/1m数值候选仅用于固定窄道导航，不宣称仓库通用。范围单元测试覆盖全部仓库场景，参数按effective_profile审计。完整数据和边界见[验证记录](validation.md)与[实验结果](experiments/results.md)。
