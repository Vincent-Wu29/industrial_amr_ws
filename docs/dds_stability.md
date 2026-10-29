# 初始化RPC通信排查

2026-10-28，本机Fast DDS 2.14.6 / rmw_fastrtps_cpp 8.4.3的正常运输批在26个固定样本中取得24成功、2失败。即使完成余下4单，也达不到设计规定的30单至少95%，因此停止该批，未将失败重跑替换。完整记录在`artifacts/validation/fastdds_initial_rpc_failures`。

seed60/domain31：运输已接受，发Nav2目标后未取得接受回应；Nav2没有开始导航日志，车留在出生点。guard按5s首命令预算撤权，业务按5s响应和5s清理预算返回FAILED400/EMPTY/HOLD，禁止继续准入。独立真值接触计数0；下游目标终结未确认，停车验证标记保持false。seed67/domain65：客户端提交业务Action等待15s超时；Mission仍持续READY，日志无admission，车未动，真值接触0。

两种请求分别跨越Python→C++和C++→Nav2。接收端没有相应业务处理日志、其他状态消息持续到达，表明应优先排查请求/响应的发现与匹配。源代码未显示阻塞整个Mission executor的证据；这不等于已经证明请求在DDS哪一层丢失。

第一方仓库有[服务发现响应超时报告](https://github.com/ros2/rmw_fastrtps/issues/842)和[同版本端点匹配报告](https://github.com/ros2/rmw_fastrtps/issues/941)。这些是相关线索，不能直接认定为本项目的根因。官方[2.14传输环境变量说明](https://fast-rtps.docs.eprosima.com/en/2.14.x/fastdds/env_vars/env_vars.html)也指出传输选择依赖builtin配置，因此没有未经验证就将UDP变量当作修复。

对照采用Cyclone DDS，业务代码、安全参数和全部超时保持。依赖安装只新增8包，0升级、0移除；实际中间件为rmw_cyclonedds_cpp 2.2.4 / Cyclone DDS 0.10.5。每次进程启动显式传入`RMW_IMPLEMENTATION`，客户端、Nav2和Gazebo内ROS插件采用一致环境；不修改用户bashrc或其他工作空间。

同一源码在Cyclone下完整42项自动化及真实Gazebo障碍运输10/10、导航20/20均通过，接触0、真值/空载/静止验证全部有效。Windows证据在`artifacts/validation/cyclone_transport_probe`。随后将项目推荐环境设为Cyclone，并在全新目录进行正式全量验收；当前结果不作为一般性的DDS性能或稳定性优劣结论，最终验收以validation.md完整数据为准。

`tools/runtime_env.sh`为已构建的专属WSL镜像准备一致环境，显式已有RMW设置会被保留。`run_acceptance.ps1 -Middleware ...`用于受控选择。测试补样和最终审计会拒绝混合中间件；更换通信环境需要新的验收根目录，不与旧失败批拼接。运输任务本身不会因为切换DDS而恢复或自动重发。
