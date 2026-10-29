
配置列profile为请求策略。仓库mission请求candidate时实际使用baseline，窄道navigation使用candidate数值；逐样本effective_profile和configuration_resolution记录实际配置。仓库回归不作为候选数值在仓库通用的证据。

# AMR 实测报告

已接受任务：120；终态分布：`{'COMPLETED': 54, 'CANCELED': 24, 'FAILED': 30, 'INTERRUPTED': 12}`。
任务成功率：0.45；导航尝试：396，成功率：0.8939393939393939。

缺少终态的已接受任务计为 INTERRUPTED；取消和故障仍计入总分母。成功任务耗时单独统计；故障演练通过不等于运输成功。

| 场景 / 配置 | N | 断言通过 | 运输/导航成功 | 碰撞 | 真值未验证 | 仿真 P95 / s | 墙钟 P95 / s |
|---|---:|---:|---:|---:|---:|---:|---:|
| cancel_load_window/baseline | 5 | 5 | 0 | 0 | 0 | None | None |
| cancel_load_window/candidate | 1 | 1 | 0 | 0 | 0 | None | None |
| cancel_loaded/baseline | 5 | 5 | 0 | 0 | 0 | None | None |
| cancel_loaded/candidate | 1 | 1 | 0 | 0 | 0 | None | None |
| cancel_pickup/baseline | 5 | 5 | 0 | 0 | 0 | None | None |
| cancel_pickup/candidate | 1 | 1 | 0 | 0 | 0 | None | None |
| cancel_unload_window/baseline | 5 | 5 | 0 | 0 | 0 | None | None |
| cancel_unload_window/candidate | 1 | 1 | 0 | 0 | 0 | None | None |
| clock_pause/baseline | 5 | 5 | 0 | 0 | 0 | None | None |
| clock_pause/candidate | 1 | 1 | 0 | 0 | 0 | None | None |
| clock_reset/baseline | 5 | 5 | 0 | 0 | 5 | None | None |
| clock_reset/candidate | 1 | 1 | 0 | 0 | 1 | None | None |
| crossing/baseline | 5 | 5 | 5 | 0 | 0 | 90.91799999999999 | 93.76539409199995 |
| crossing/candidate | 1 | 1 | 1 | 0 | 0 | 90.67399999999999 | 94.49887626000003 |
| device_failure/baseline | 5 | 5 | 0 | 0 | 0 | None | None |
| device_failure/candidate | 1 | 1 | 0 | 0 | 0 | None | None |
| duplicate/baseline | 5 | 5 | 5 | 0 | 0 | 85.28200000000001 | 86.77391568200028 |
| duplicate/candidate | 1 | 1 | 1 | 0 | 0 | 84.64399999999999 | 91.16166492000002 |
| guard_exit/baseline | 5 | 5 | 0 | 0 | 5 | None | None |
| guard_exit/candidate | 1 | 1 | 0 | 0 | 1 | None | None |
| happy/baseline | 35 | 35 | 35 | 0 | 0 | 85.50399999999999 | 94.510425672 |
| happy/candidate | 1 | 1 | 1 | 0 | 0 | 84.734 | 88.809133239 |
| load_timeout/baseline | 5 | 5 | 0 | 0 | 0 | None | None |
| load_timeout/candidate | 1 | 1 | 0 | 0 | 0 | None | None |
| narrow/baseline | 31 | 31 | 31 | 0 | 0 | 33.092 | 14.706667039000195 |
| narrow/candidate | 31 | 31 | 31 | 0 | 0 | 19.414 | 9.716082260999883 |
| nav_exit/baseline | 5 | 5 | 0 | 0 | 5 | None | None |
| nav_exit/candidate | 1 | 1 | 0 | 0 | 1 | None | None |
| permanent/baseline | 5 | 5 | 0 | 0 | 0 | None | None |
| permanent/candidate | 1 | 1 | 0 | 0 | 0 | None | None |
| temporary/baseline | 5 | 5 | 5 | 0 | 0 | 90.392 | 98.31068375799987 |
| temporary/candidate | 1 | 1 | 1 | 0 | 0 | 87.92 | 95.81191159100001 |
