# AMR 实测报告

已接受任务：1；终态分布：`{'FAILED': 1}`。
任务成功率：0.0；导航尝试：2，成功率：0.5。

缺少终态的已接受任务计为 INTERRUPTED；取消和故障仍计入总分母。成功任务耗时单独统计；故障演练通过不等于运输成功。

| 场景 / 配置 | N | 断言通过 | 运输/导航成功 | 碰撞 | 真值未验证 | 仿真 P95 / s | 墙钟 P95 / s |
|---|---:|---:|---:|---:|---:|---:|---:|
| happy/candidate | 1 | 0 | 0 | 0 | 1 | None | None |

失败样本：
```json
[
  {
    "scenario": "happy",
    "profile": "candidate",
    "seed": 42,
    "passed": false,
    "accepted": true,
    "success": false,
    "error": "AssertionError: expected 9, got amr_interfaces.action.ExecuteTransport_Result(run_id='18e2dcb11784a286-7f64f13b', task_id='TASK-cbb6e13733', final_state=10, error_code=203, message='code=6 nav_error=105 ', load_state=1, requires_operator=True, elapsed_sim_s=85.96, elapsed_wall_s=92.617798129)",
    "collision_count": 0,
    "truth_valid": false,
    "physical_stop_verified": false,
    "elapsed_wall_s": 92.617798129,
    "elapsed_sim_s": 85.96,
    "final_state": 10,
    "error_code": 203,
    "load_state": 1,
    "requires_operator": true,
    "diagnostics": {
      "mission/readiness": "HOLD",
      "system/readiness": "READY",
      "controller_manager: Controllers Activity": "\nHigh execution jitter or mean error : [ diff_drive_controller joint_state_broadcaster ]",
      "controller_manager: Hardware Components Activity": "All hardware components are active",
      "controller_manager: Controller Manager Activity": "Controller Manager is running"
    },
    "truth_samples": 1942,
    "box_scan_hits": 0,
    "observed_real_time_factor": 0.9281153486317297
  }
]
```
