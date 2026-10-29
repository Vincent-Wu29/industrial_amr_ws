# 第三方组件与来源

本仓库自己的模型、世界、几何地图、任务逻辑与工具采用 MIT License。运行依赖由 ROS/Gazebo 软件包独立分发，保留其各自许可证。

| 组件 | 用途 | 上游 |
|---|---|---|
| ROS2 / rclcpp / rclpy | 节点、通信、Action、launch | https://github.com/ros2 |
| Navigation2 | AMCL、地图、Navfn、DWB、BT、collision_monitor | https://github.com/ros-navigation/navigation2 |
| Cyclone DDS / rmw_cyclonedds | ROS2通信中间件 | https://github.com/eclipse-cyclonedds/cyclonedds / https://github.com/ros2/rmw_cyclonedds |
| ros2_control / ros2_controllers | 轮反馈、差速控制 | https://github.com/ros-controls |
| gz_ros2_control | Gazebo 硬件接口 | https://github.com/ros-controls/gz_ros2_control |
| Gazebo Sim / ros_gz | 物理世界、传感器和消息桥 | https://github.com/gazebosim |
| slam_toolbox | 激光建图 | https://github.com/SteveMacenski/slam_toolbox |
| yaml-cpp / PyYAML | 配置解析 | https://github.com/jbeder/yaml-cpp / https://github.com/yaml/pyyaml |

Nav2参数采用本机Jazzy插件/API契约；项目参数针对本底盘设定。BT仅组合标准Nav2节点，两轮清图/等待恢复；模型、纹理和世界未复制第三方资产。几何地图由generate_assets.py生成，另有实际SLAM地图及来源哈希。M4交付为固定场景参数策略与定量验证，复用原生规划/控制插件；实测范围、失败和数据见docs/experiments/results.md，不包装为自研规划器或通用导航结论。
