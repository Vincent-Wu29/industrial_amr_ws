# 本机环境审计

2026-10-27 在 Ubuntu-24.04 中读取实际安装信息；不修改已有 `~/ros_ws`。

| 项目 | 实测 |
|---|---|
| OS | Ubuntu 24.04.4 LTS，WSL2 |
| Kernel | 6.6.87.2-microsoft-standard-WSL2 |
| 内存 / Swap | 7.8 GiB / 2 GiB |
| Linux 根盘可用空间（审计时） | 933 GiB |
| ROS | Jazzy |
| Gazebo Sim | 8.11.0 |
| Nav2 | 1.3.12；nav2_bringup 1.3.13 |
| gz_ros2_control | 1.2.19 |
| ros2_controllers | 4.40.1 |
| ros_gz | 1.0.22 |
| slam_toolbox | 2.8.5 |

原 `~/ros_ws/src/robot_description` 包含机械臂、夹爪和相机模型；本项目使用自建轻量几何模型以保持依赖和来源清晰。初次审计时已有主要ROS软件包，未安装或升级；随后仅按下述记录新增Cyclone依赖。

2026-10-28为初始化RPC对照新增Cyclone DDS：`rmw_cyclonedds_cpp 2.2.4`、`cyclonedds 0.10.5`。apt使用`--no-upgrade --no-install-recommends`，只新增8个包及其依赖，0升级/0移除（下载2.03MB、新增8.65MB）。不修改bashrc或其他项目的RMW环境。当前项目推荐Cyclone，所有相关终端显式设置相同`RMW_IMPLEMENTATION`；默认FastDDS 2.14.6 / rmw_fastrtps_cpp 8.4.3的两个初始Action超时原样保留，见dds_stability.md。

Windows 当前路径中的空格触发 rosidl CMake 路径列表解析错误。`tools/wsl_build.sh` 使用带归属标记的新 Linux 工作空间，仅复制本项目 src；初次默认位置为 `/home/cheng/industrial_amr_ws`。源码以 Windows 项目为准。

请勿同时启动两套未隔离的仿真。自动场景设置独立 ROS_DOMAIN_ID 与 GZ_PARTITION。手动调试可自行设置两者；所有相关终端必须一致。GPU LiDAR 的 headless 模式仍需要 Ogre/EGL 渲染支持，实际验证见 validation.md。

批量物理场景自动选取domain30–89，避免本机Linux临时UDP端口范围；ROS单元测试另用215–226与232，分开各类夹具。`--domain-id`用于指定隔离诊断域，例如复测domain100的控制器启动。

Windows长测可使用 `tools/run_acceptance.ps1`。它仅在运行线程上申请防空闲休眠，结束或异常时释放；不修改电源方案或显示器设置。阶段日志保存在 `artifacts/runner_logs/<UTC时间>/`。手动休眠、系统暂停和通信故障仍会触发既有安全门控。
