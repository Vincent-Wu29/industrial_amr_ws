# 接口与状态语义

`amr_interfaces` 下的 `.action` / `.msg` / `.srv` 为唯一字段定义；`TaskSnapshot` 中包含状态与货物常量。

| 入口 | 类型 | 用途 |
|---|---|---|
| `/mission/execute_transport` | ExecuteTransport Action | 提交、反馈、异步结果与定向取消 |
| `/mission/get_transport` | GetTransport | 当前会话内查询活动/历史任务及业务 goal UUID |
| `/stations/get_station` | GetStation | 查询冻结的站点姿态与地图版本 |
| `/mission/confirm_operation` | ConfirmOperation | 匹配 run/task/station/interaction/operation 的装卸成功或失败 |
| `/mission/reconcile_load` | ReconcileLoad | 无活动任务且停车/清理/就绪通过时核对货物，要求 run_id 与说明 |
| `/mission/state` | TaskSnapshot | reliable / transient_local / depth=1，5 Hz 与事件时发布 |
| `/mission/events` | MissionEvent | reliable / volatile / depth=100，有序实时事件 |
| `/motion_guard/set_enabled` | SetBool | Mission 控制运动许可 |
| `/motion_guard/enabled` | Bool | guard 实际许可状态，每 50 ms 发布 |
| `/system/ready` | Bool | 时钟、传感器、AMCL/地图、TF、lifecycle 与控制器物理就绪 |
| `/ground_truth/pose` | PoseStamped | world 坐标系下的独立 Gazebo ECM 位姿，仅供评估与夹具触发 |
| `/diagnostics` | DiagnosticArray | 物理就绪原因与业务 READY/HOLD |

动作拒绝没有自定义 Result；客户端从 admission 事件获取拒绝码，使用业务查询定位原任务。`task_cli cancel` 先查询 UUID，再调用标准 CancelGoal，stamp=0、UUID 非零，仅取消该业务目标。

| 数值 | 名称 |
|---:|---|
| 0 | OK |
| 100 / 101 | INVALID_REQUEST / UNKNOWN_STATION |
| 102 / 103 / 104 / 105 | DUPLICATE_TASK / TASK_ID_CONFLICT / BUSY / CACHE_FULL |
| 200 / 201 / 202 | NOT_READY / NAV_UNAVAILABLE / NAV_GOAL_REJECTED |
| 203 / 204 / 205 | NAV_FAILED / NAV_TIMEOUT / NO_PROGRESS（预留；Nav2 进展失败保留其原始原因） |
| 300 / 301 / 302 | DOCK_CHECK_FAILED / LOAD_TIMEOUT / UNLOAD_TIMEOUT |
| 303 / 304 | DEVICE_FAILED / CONFIRM_CONFLICT |
| 400 / 401 / 402 | CANCEL_NOT_CONFIRMED / STOP_NOT_CONFIRMED / CLOCK_RESET |
| 403 / 404 | INTERRUPTED / LOG_IO_ERROR |
| 500 | USER_CANCELED |

仿真时间控制导航、装卸与总任务预算；单调墙钟控制响应、清理和进程保护。停车要求新鲜 odom 中速度低于 0.02 m/s 和 0.05 rad/s 持续 0.5 s；到站使用 map→base_footprint，位置/角度容差 0.18 m / 0.18 rad，Nav2 goal checker 为 0.15 m / 0.15 rad。

一次装卸确认只应用一次，相同结果重发返回已应用，相反结果为冲突。等待窗口内失败、超时或取消后货物 UNKNOWN；带货导航取消保留 LOADED。所有异步普通回调按导航代次过滤，但晚接受的悬而未决目标仍保留取消清理义务。

CancelGoal 成功不表示下游已经终结。下游缺终态或停车证据过期时，5 s 墙钟清理上限后返回 FAILED，并保持 HOLD。若卸货成功先提交，后到取消拒绝；若取消先接受，后到成功确认拒绝。

时钟回拨记录 `CLOCK_RESET` 并锁定准入，任务在原清理预算内失败结束，随后 Mission 退出并触发主 launch 全栈退出。场景失效后的物理停车没有有效观测时明确不验证；客户端不得将 FAILED 或世界消失等同于 CANCELED。CLI 对业务结果关联和状态心跳作独立检查，丢失服务时报告 UNKNOWN。
