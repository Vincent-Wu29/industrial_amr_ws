# AMR 实测报告

已接受任务：0；终态分布：`{}`。
任务成功率：None；导航尝试：102，成功率：0.9901960784313726。

缺少终态的已接受任务计为 INTERRUPTED；取消和故障仍计入总分母。成功任务耗时单独统计；故障演练通过不等于运输成功。

| 场景 / 配置 | N | 断言通过 | 运输/导航成功 | 碰撞 | 真值未验证 | 仿真 P95 / s | 墙钟 P95 / s |
|---|---:|---:|---:|---:|---:|---:|---:|
| narrow/baseline | 30 | 30 | 30 | 0 | 0 | 29.799999999999997 | 16.124361237000016 |
| narrow/candidate | 4 | 3 | 3 | 0 | 1 | 20.013999999999996 | 8.644933306999974 |

失败样本：
```json
[
  {
    "scenario": "narrow",
    "profile": "candidate",
    "seed": 45,
    "passed": false,
    "accepted": true,
    "success": false,
    "error": "RuntimeError: narrow navigation failed",
    "collision_count": 0,
    "truth_valid": false,
    "physical_stop_verified": false,
    "elapsed_wall_s": null,
    "elapsed_sim_s": null,
    "diagnostics": {
      "system/readiness": "READY",
      "controller_manager: Controllers Activity": "\nHigh execution jitter or mean error : [ diff_drive_controller joint_state_broadcaster ]",
      "controller_manager: Hardware Components Activity": "All hardware components are active",
      "controller_manager: Controller Manager Activity": "Controller Manager is running"
    },
    "truth_samples": 2094,
    "box_scan_hits": 0,
    "observed_real_time_factor": null
  }
]
```
