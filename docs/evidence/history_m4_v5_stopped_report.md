# AMR 实测报告

已接受任务：0；终态分布：`{}`。
任务成功率：None；导航尝试：12，成功率：1.0。

缺少终态的已接受任务计为 INTERRUPTED；取消和故障仍计入总分母。成功任务耗时单独统计；故障演练通过不等于运输成功。

| 场景 / 配置 | N | 断言通过 | 运输/导航成功 | 碰撞 | 真值未验证 | 仿真 P95 / s | 墙钟 P95 / s |
|---|---:|---:|---:|---:|---:|---:|---:|
| narrow/baseline | 5 | 4 | 4 | 0 | 1 | 32.41 | 15.439942773999064 |

失败样本：
```json
[
  {
    "scenario": "narrow",
    "profile": "baseline",
    "seed": 46,
    "passed": false,
    "accepted": false,
    "success": false,
    "error": "",
    "collision_count": null,
    "truth_valid": false,
    "physical_stop_verified": false,
    "elapsed_wall_s": null,
    "elapsed_sim_s": null,
    "box_scan_hits": 0,
    "observed_real_time_factor": null
  }
]
```
