# AMR 实测报告

已接受任务：0；终态分布：`{}`。
任务成功率：None；导航尝试：6，成功率：0.8333333333333334。

缺少终态的已接受任务计为 INTERRUPTED；取消和故障仍计入总分母。成功任务耗时单独统计；故障演练通过不等于运输成功。

| 场景 / 配置 | N | 断言通过 | 运输/导航成功 | 碰撞 | 真值未验证 | 仿真 P95 / s | 墙钟 P95 / s |
|---|---:|---:|---:|---:|---:|---:|---:|
| narrow/baseline | 2 | 1 | 1 | 0 | 1 | 29.598 | 14.376617875000022 |

失败样本：
```json
[
  {
    "scenario": "narrow",
    "profile": "baseline",
    "seed": 43,
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
    "truth_samples": 2138,
    "box_scan_hits": 0,
    "observed_real_time_factor": null
  }
]
```
