# 基于 ROS2 的工业 AMR 自主导航与任务执行系统

单台差速 AMR 的仓储运输仿真：Gazebo Harmonic → ros2_control → AMCL / Nav2 → C++ 运输状态机 → 独立装卸确认。依据 [项目设计方案](AMR_ROS2_项目设计方案.md) 完成M0–M4，六个ROS包位于 `src/`。实测结果与限制见 [验证记录](docs/validation.md)，展示命令和代码入口见 [演示指南](docs/demo.md)。

本轮六包构建及43项自动化检查通过；固定图导航15/15、保存图重启3/3，正常运输30/30，故障矩阵75/75断言、系统策略回归15/15通过。窄道正式A/B各30次均成功、记录接触0，仿真耗时P95 **33.092→19.414秒，下降41.33%**。候选只用于固定窄道实验路径，仓库任务使用已验证基线。

[完整实验报告与60行数据](docs/experiments/results.md) · [最终审计](docs/experiments/figures/v10_acceptance_audit.json) · [动态障碍修复证据](docs/obstacle_repair.md)。

## 环境与构建

Ubuntu 24.04、ROS2 Jazzy、Gazebo Harmonic（Sim 8）、Nav2、gz_ros2_control、ros2_controllers、slam_toolbox、yaml-cpp、rmw_cyclonedds_cpp。Windows 通过 WSL2 运行。已经安装的版本见 [环境清单](docs/environment.md)，通信环境选择依据见 [DDS排查记录](docs/dds_stability.md)。

Windows 源码路径含空格时，rosidl 在该路径直接构建会失败。请在 **Ubuntu-24.04 终端**克隆项目后执行；已有源码时进入对应目录即可：

```bash
git clone https://github.com/Vincent-Wu29/industrial_amr_ws.git ~/industrial_amr_source
cd ~/industrial_amr_source
bash tools/wsl_build.sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source tools/runtime_env.sh
cd ~/industrial_amr_ws
```

构建脚本将 `src/` 复制到独立的 `~/industrial_amr_ws`，并拒绝覆盖未带项目标记的已有目录；不改动 `~/ros_ws`。上述示例的源码目录是 `~/industrial_amr_source`，与构建镜像分开。始终在克隆的源码目录中编辑，然后再次运行脚本同步、构建。脚本不删除文件；若主动删除源码文件，需要在 Linux 镜像中同步删除相应文件，或指定新的 `AMR_BUILD_WS`。

`build/`、`install/`、`log/` 和原始验收产物由本地构建及运行生成，不纳入源码仓库。精选验收报告与完整证据范围见 [证据说明](docs/evidence/README.md)。

其他 Ubuntu 主机可直接在没有空格的工作空间构建：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --base-paths src --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

构建后，以下命令会独立创建世界、完成一笔带两次装卸确认的运输、检查实际碰撞和停车，随后关闭该世界，并保存日志：

```bash
ros2 run amr_bringup scenario_runner --cases happy --headless --seed 42
```

## 运行一笔完整运输

每个新Ubuntu终端都先使用相同中间件并source工作空间；WSL默认镜像示例：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/industrial_amr_ws/install/setup.bash
cd ~/industrial_amr_ws
```

终端一启动新的仿真世界。默认人工装卸；每次主 launch 创建全新空载世界，并显式允许首次就绪后核对为空载。

```bash
ros2 launch amr_bringup system.launch.py mode:=mission gui:=false rviz:=false device_mode:=manual
```

终端二在 source 同一工作空间后执行：

```bash
ros2 run amr_bringup readiness_check --timeout 120
ros2 run amr_mission task_cli submit --task-id TASK-001 --source A01 --target B02
```

终端三查询状态，并在对应等待阶段分别确认装货、卸货：

```bash
ros2 run amr_mission task_cli status --task-id TASK-001
ros2 run amr_mission task_cli confirm-current --task-id TASK-001 --operation LOAD --outcome SUCCESS
ros2 run amr_mission task_cli confirm-current --task-id TASK-001 --operation UNLOAD --outcome SUCCESS
```

也可使用原生 Action：

```bash
ros2 action send_goal /mission/execute_transport amr_interfaces/action/ExecuteTransport \
  "{task_id: 'TASK-002', source_station: 'A01', target_station: 'B02'}" --feedback
ros2 run amr_mission task_cli cancel --task-id TASK-002
```

取消命令只表示请求是否被接受；最终状态需查询或观察原提交客户端。取消会关闭运动许可、定向取消导航，并等待导航终态与新鲜里程计持续静止证据。超出清理预算返回 FAILED + HOLD。有货或货物未知时，需要核实后执行 `task_cli reconcile --load EMPTY --note '已现场确认空载'`。残留导航或系统故障锁定不能靠核对货物解除，需要重建整套仿真。

CLI 会核对业务 goal UUID，并监测 Mission 状态心跳；进程失联或会话变化时报 `UNKNOWN`，不自动重发旧任务。时钟回拨会记录 `CLOCK_RESET`，在任务清理结束后触发全栈退出；本机 Gazebo 的重置操作会移除动态生成的机器人，需要重新启动世界。

Nav2未分类中止（错误码0或缺失结果）同样锁定上下文并要求重建全栈，因为内部可能仍有迟到的子目标；此时空载也不能通过货物核对解除锁定。

自动演示请显式选择 `device_mode:=scripted_success`。该模式由独立设备节点在两个等待窗口内延时确认，导航到达本身不会完成装卸。

## 导航与建图

普通导航直接使用 stamped 控制链，不启动 Mission/guard。请先停止业务系统：

```bash
ros2 launch amr_bringup system.launch.py mode:=navigation world:=mini_warehouse gui:=false
# 另一个终端
ros2 run amr_navigation navigation_smoke --stations A01 B02 HOME --repeat 5
```

建图为互斥启动模式；自动采图路线只适用于刚重建的默认世界和初始位姿：

```bash
ros2 launch amr_bringup mapping.launch.py gui:=false
# 另一个终端：使用 SLAM 位姿，带 scan/odom/TF 新鲜度与周向障碍检查
ros2 run amr_navigation mapping_drive --save artifacts/maps/mapped_warehouse_v1
```

采图工具保存 PGM、地图 YAML 与对应站点 YAML，并依据已知出生坐标将 SLAM 的初始原点平移到 world/map 约定。它不使用运动真值校正定位。停止建图，重新启动：

```bash
ros2 launch amr_bringup system.launch.py mode:=navigation \
  map:=$PWD/artifacts/maps/mapped_warehouse_v1.yaml \
  stations:=$PWD/artifacts/maps/mapped_warehouse_v1_stations.yaml
# 另一个已 source 的终端
ros2 run amr_navigation navigation_smoke --repeat 1 \
  --stations-file artifacts/maps/mapped_warehouse_v1_stations.yaml
```

Mission 启动时检查地图版本、站点格式、有限数值和整个碰撞包络是否位于已知自由区。保存地图后也必须完成复跑验证。

本次实际采图并复跑通过的 `mapped_warehouse_v1.pgm/.yaml` 已保存在 `amr_navigation/maps`，对应站点为 `amr_mission/config/mapped_warehouse_v1_stations.yaml`。重建安装后可以直接加载该地图；其来源和哈希保存在同目录的 provenance JSON。

## 测试与实验

以下tools命令在源码目录执行。`bash tools/validate.sh unit`构建并运行常规测试；`bash tools/validate.sh all`串行执行完整采样。正式业务验收默认RTF1，M4内部固定目标RTF3；其他倍率诊断使用独立输出目录。

Windows项目终端可运行 `./tools/run_acceptance.ps1 -Stages unit,m3,navigation,preflight,experiments`，默认统一Cyclone DDS，临时防止空闲休眠并在结束释放。完整复跑使用全新OutputDirectory/EvidenceName，示例见实验报告。M4先预检、再15项策略回归，全部通过才采正式A/B。`bash tools/validate.sh startup`单独复测五次启动。

M3正常批固定seed42–71，成功率门槛≥95%，失败保留分母。fill_normal_samples只补缺失种子，每个新样本创建新世界，不重试失败。修改运行源码或倍率后使用新目录。最终在已source的Ubuntu源码终端执行 `python3 tools/audit_acceptance.py --input <Linux验收根目录> --require-performance`，要求完整性和性能目标同时通过；不加该标志时仅完整性决定退出码。重复导出只接受同源证据，不覆盖其他实验。

```bash
colcon test --packages-select amr_mission amr_navigation amr_simulation amr_bringup \
  --event-handlers console_direct+
colcon test-result --verbose

ros2 run amr_bringup scenario_runner --suite m2_happy_path --repeat 10 --seed 42 --headless
ros2 run amr_bringup scenario_runner --suite m3_regression --repeat 5 --seed 42 --headless
ros2 run amr_bringup metrics_report --input artifacts --output artifacts/report.md

# M4：专属目录避免把排障样本混入正式实验
ros2 run amr_bringup scenario_runner --suite narrow_aisle --profile baseline --repeat 30 --seed 42 --headless --real-time-factor 3 --output artifacts/my_m4
ros2 run amr_bringup scenario_runner --suite narrow_aisle --profile candidate --repeat 30 --seed 42 --headless --real-time-factor 3 --output artifacts/my_m4
ros2 run amr_bringup metrics_report --input artifacts/my_m4 --compare baseline candidate --metric sim --output artifacts/my_m4_comparison.md
```

实验协议见 [protocol.md](docs/experiments/protocol.md)：候选调整全局代价衰减与标记距离，局部观测、物理足迹、安全余量和速度约束一致。候选只针对narrow_aisle的固定实验路径（含staging）；仓库运输策略解析为已验证基线，manifest同时记录请求profile与effective_profile。原始真值区分实际穿越与绕行，并记录路径长度与通道间隙。`candidate.yaml` 是参数片段，不能单独作为完整Nav2参数文件；生成工具会拒绝在仓库或mission模式误用候选。

手动使用候选时，在源码目录生成完整参数文件，再在同一Ubuntu终端启动：

```bash
python3 tools/materialize_profile.py --profile candidate --world narrow_aisle --output ~/industrial_amr_ws/artifacts/candidate_nav2.yaml
ros2 launch amr_bringup system.launch.py mode:=navigation world:=narrow_aisle \
  params_file:="$HOME/industrial_amr_ws/artifacts/candidate_nav2.yaml" gui:=false
```

L1 纯 C++ 测试独立于 ROS，也可通过根目录 CMake 构建。L2 使用真实 ROS2 Action/Service 和可控假 Nav2，验证正常装卸、查询/幂等、晚接受目标取消及装卸窗口取消。L3 每个案例独立 ROS_DOMAIN_ID、GZ_PARTITION、进程组与产物目录，结束时仅清理自身进程。完整物理回归可能耗时较长。

默认仿真目标实时因子为 1。批量场景可显式传入 `--real-time-factor 3`（主 launch 对应 `real_time_factor:=3.0`）；场景保留实际 world 文件、目标倍率及测得的仿真/墙钟比值。对照实验必须使用相同倍率；目标倍率不代表本机一定能达到该速度。

事件日志为 `artifacts/<run_id>/events.jsonl`，每条事件含有序序号、任务/会话/goal UUID、导航代次、装卸交互、仿真和墙钟耗时。日志无法提交时节点退出，避免发布未提交的成功结果。场景目录另含配置哈希、软件版本、系统日志、真值轨迹、物理障碍轨迹与 `metrics.json`。

碰撞由独立 Gazebo 真值通道与有向 footprint 几何监测计算，不把定位估计冒充真值；真值无效时场景验收失败。该检测适用于本项目的平面长方体障碍与已知底盘包络。报告的接受任务分母包括失败、取消和中断；故障场景的断言通过率单独列出。

## 结构与关键契约

| 包 | 内容 |
|---|---|
| `amr_description` | Xacro 差速模型、惯量、轮子、LiDAR、IMU、ros2_control 接口 |
| `amr_simulation` | 三个世界、控制器、单向桥、物理障碍夹具、ECM 真值插件/适配器与碰撞监测 |
| `amr_navigation` | 固定地图、AMCL/Navfn/DWB、有限恢复 BT、SLAM、自动导航/采图 |
| `amr_interfaces` | ExecuteTransport、状态、事件、站点/任务查询、装卸确认、货物核对 |
| `amr_mission` | 纯 C++ MissionCore、StationRegistry、ROS2 适配器、motion_guard、CLI 与设备模拟 |
| `amr_bringup` | 主启动、物理就绪检查、隔离场景回归、统计报告 |

```text
PENDING → 取货导航 → 到站/停车检查 → 等待装货确认
        → 送货导航 → 到站/停车检查 → 等待卸货确认 → COMPLETED
任何活动阶段 → STOPPING → CANCELED / FAILED
```

运动链：`Nav2 TwistStamped → /cmd_vel_plan → collision_monitor → /cmd_vel_nav → motion_guard → /diff_drive_controller/cmd_vel`。原生 Nav2 collision_monitor 使用实时激光，近障停止区域为0.80×0.64m，并按完整带余量足迹预测1.5s碰撞时间进行减速；激光源超过0.4s失效时停车。guard 另检查心跳、时钟推进、scan、odom、命令年龄和速度上限，保留非零命令的原始时间戳，不重放旧命令。导航模式同样经过 collision_monitor；建图使用专用低速采图流程。差速控制器仅由 Gazebo 插件创建的 controller manager 管理。

同一会话只允许一笔任务，最多保留 10,000 个任务 ID，达到上限拒绝新单。重启生成新 run_id、货物 UNKNOWN，单独启动 Mission 默认 HOLD；不跨重启承诺任务去重或自动续跑。运输完成点是卸货确认，返回 HOME 是独立导航操作。

更多见 [接口与错误码](docs/interfaces.md)、[测试矩阵](docs/test_matrix.md)、[第三方来源](THIRD_PARTY_NOTICES.md)。M5 数据库、视觉精停、HTTP 和真机移植仍为可选扩展。
