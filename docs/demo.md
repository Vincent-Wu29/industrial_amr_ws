# 演示与复现

先按README构建，在每个Ubuntu终端设置`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`并source `~/industrial_amr_ws/install/setup.bash`，工作目录进入`~/industrial_amr_ws`。以下场景驱动为每个场景创建独立新世界，结束后关闭，原始日志与manifest保留在输出目录。

## 一次展示三个核心行为

```bash
ros2 run amr_bringup scenario_runner \
  --cases happy crossing cancel_loaded --seed 42 --headless \
  --output artifacts/demo
ros2 run amr_bringup metrics_report \
  --input artifacts/demo --output artifacts/demo_report.md
```

运行前关闭其他仿真。驱动自动隔离ROS domain与Gazebo partition；无需在外部终端重复发送目标。WSLg图形环境可用时，省略`--headless`可启动Gazebo界面；正式性能与故障证据使用headless配置。界面显示方式不改变业务结果的验证规则。

| 场景 | 展示重点 | 预期结果 |
|---|---|---|
| happy | 取货导航→停靠检查→独立装货确认→送货→独立卸货确认 | COMPLETED、EMPTY、无碰撞、终态静止 |
| crossing | 实体箱体横穿、实际LiDAR观测、近障减速/停车后恢复 | COMPLETED、EMPTY；实际箱体观测与碰撞计数可查 |
| cancel_loaded | 装货后取消送货导航，关闭运动许可并等待下游结束及停车 | CANCELED、LOADED、需要人工核对；不能假称自动卸货 |

正常运输完成点为B02卸货成功。回HOME是单独导航操作，不属于运输成功条件。设备模拟器是独立ROS节点，按当前任务和交互ID提交确认；导航成功不会自动等同于装卸成功。

## 查看证据

每个场景文件夹中，`metrics.json`记录断言、业务结果、碰撞及真值/停车验证，`manifest.yaml`记录版本、源代码/配置哈希、种子、启动命令和目标倍率。子目录的`events.jsonl`保留业务事件；`system.log`含Nav2与collision_monitor日志。`truth.jsonl`与`obstacle_truth.jsonl`为实际Gazebo ECM轨迹，`obstacle_lidar_evidence.jsonl`为箱体激光匹配证据。

建议按以下顺序讲解：先展示正常流程及两次独立确认，再展示动态障碍保护，随后展示带货取消与HOLD，最后打开测试矩阵和实验报告。失败场景的断言通过率与运输成功率分别解释。已有修复图见`obstacle_repair.md`；M4配对结论只采用`experiments/protocol.md`所登记的正式样本。

M4正式结果为两组各30次全过，固定窄道simP95 33.092→19.414秒，降低41.33%，详见[实验报告及60行CSV](experiments/results.md)。展示配对图与真实路线图，说明仓库仍使用基线，候选仅适用固定窄道；诊断单例不当作统计，v9仓库数值候选失败也保留。

## 核心代码讲解入口

| 代码 | 可讲解内容 |
|---|---|
| `src/amr_mission/src/mission_core.cpp` | 领域事件、单槽互斥、终态不变、取消/确认竞争、清理超时与故障锁 |
| `src/amr_mission/src/mission_node.cpp` | 异步Action适配、晚接受取消、导航代次、先提交日志再发布 |
| `src/amr_mission/src/station_registry.cpp` | YAML校验、地图关联、旋转完整足迹与占用栅格相交检测 |
| `src/amr_mission/src/motion_guard.cpp` | 独立运动许可、心跳/传感器/时钟新鲜度、不重放旧非零命令 |
| `src/amr_navigation/config/nav2.yaml` | Navfn/DWB、完整足迹、原生激光近障停止与预测减速 |
| `src/amr_bringup/scripts/scenario_runner` | 真正物理故障注入、断言、原始数据、进程组清理与超时 |

人工交互式提交、确认、查询和定向取消的命令见README；不要对自动场景的内部任务另行发送控制命令。
