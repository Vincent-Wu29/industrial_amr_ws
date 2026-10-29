# M4 预登记实验 v10：限定固定窄道导航的参数策略

2026-10-29，v10正式采样前登记。v1–v9全部归档。主要指标、15%门槛和安全断言保持，旧失败不删除、不用重跑替换。

v9的窄道功能预检通过，但将40/1m参数用于warehouse正常运输时在货架角落触发NearStop并FAILED203/LOADED/HOLD，0接触。因此数值候选不具备仓库通用性；完整失败保留于acceptance_20261029_v9。v10增加明确适用范围，禁止把窄道单项改进包装为通用导航改进。

| 场景与模式 | 请求策略 | 实际参数 |
|---|---|---|
| 固定narrow_aisle导航实验 | baseline | global衰减3、全局标记5m |
| 固定narrow_aisle导航实验 | candidate | global衰减40、全局标记1m |
| warehouse运输及故障场景 | baseline或candidate | 已验证基线3/5m |

scenario_runner通过纯函数解析有效策略，并在metrics/manifest同时记录profile（请求）和effective_profile（实际）。新增单元测试覆盖全部15项仓库场景，防止候选泄漏；手动生成工具要求candidate必须显式指定narrow_aisle/navigation，否则拒绝。候选只针对本节固定路径及staging验证，不承诺同一地图任意目标或仓库任务通用。

仓库15项系统回归验证策略隔离和原功能，不作为40/1m数值在仓库可用的证据；该数值在仓库已知失败。默认及正式运输仍为baseline。完整说明此范围后，再验证窄道两组各30次。

## 机制与已保留证据

- v8基线30/30，candidate40为3/4；seed45改绕行后NearStop停车，0接触。旧数据acceptance_20261029_v8完整保留。
- 独立原生诊断同seed45三次：factor40/global标记5m时1成功/2失败；失败快照15–16列通道最小代价>=253、物理空旷中央带5/9个254格，当前AMCL XY误差约.025/.034m。清图后闭合格消失，支持观测层遗留错误障碍解释，不能笼统称定位发散。
- 同factor40、仅global标记缩到1m的诊断3/3实际穿越，0接触/0闭合列/0中央带致命格；AMCL XY误差没有同步改善。诊断根narrow_plan_probe_20261029、narrow_nearfield_probe_20261029保留plan/costmap/amcl/truth；analyze_navigation_probe.py可复算。诊断不计正式样本。
- 两项全局差异共同构成候选：静态非致命代价衰减减少绕行，缩短远距离标记抑制虚假屏障。不能归因单参数。全局对动态障碍的标记提前量缩短，是明确权衡。
- 全局障碍层仍enabled、raytrace6m；local仍5m观测、全量/scan输入原生监控，真实足迹.40×.30/padding.12、inflation_radius.55、速度.35/.8、加速度.4/1.0、DWB预测1.7、NearStop半.40/.32/min2、Approach1.5s/step.05、scan超时.4均相同。未缩小防护、删除实体或放宽停止条件。

## 冻结采样计划

- 世界narrow_aisle：通道x8–12、y12–12.8、净宽.8m，存在上方绕行。共同staging HOME→(6.5,12.4)→(7,12.4)，计时到(13,12.4)，朝向0，端面各留1m转向空间。实际目标由navigation_smoke核对。
- 原Navfn+DWB，BT确认1000ms、每段180sim、停止清理5wall、两轮清图/Wait均不改；统一Cyclone DDS0.10.5/rmw2.2.4。
- 先两组seed45各1预检，候选必须实际穿越；再请求candidate策略的仓库15项系统回归RTF1（有效配置必须baseline），全部断言通过才正式采样。诊断、预检、回归均不计性能样本。
- 正式每组30，配对seed42–71，同机headless串行独立新世界、目标RTF3，先Baseline后Candidate；本轮全部重新采样，不复用旧baseline30。Gazebo seed不控制AMCL和线程调度全部差异，报告实际RTF、controller CPU秒与wall耗时。
- primary为目标仿真耗时P95下降>=15%；成功率不降，所有样本断言/0接触/真值/实际停车验证通过。P95第ceil(.95N)项，另报N/均值/中位数。失败保留分母，不重跑替换；缺终态计中断。任一门槛不满足则报告候选不成立。
- 真值x8.5–11.5每.1m分箱完整覆盖且y12–12.8才为corridor，否则bypass；绕行能到达也不能说穿越。记录路径长度和完整物理底盘到两侧墙的最小间隙。性能统计只读m4，排除m4_preflight。
- 新根acceptance_20261029_v10。基线生产运行及有效配置未变，既有M1/M2/M3证据可保留；源码例外仅candidate片段、harness精确窄道坐标+策略解析/有效配置元数据、资源测试新增全局range与策略范围断言。旧harness_v6.py/resources_v8.py和旧新SHA保留；审计精确文本替换后必须等于交付源码，其余源码一版本。参数按effective_profile分组，baseline一份、narrow candidate一份且仅两项global差异。
- 新增策略范围测试后完整单元至少43项全过。登记JSON在首个预检前冻结，正式后不改候选/指标；最终完整性与15%改善分开审计，明示仓库回归的实际配置，不省略v9通用性失败。
