# 测试记录 26090722：prepare 五次干净重启验收（schedule_safety_margin_m=1.0）

> 配置：`schedule_safety_margin_m: 1.0`（commit `4b8cb26`），即排程 d_ij = 1.0 + 1.0 = **2.0m**。
> 方法同 implementation_plan_26090106 §8.4：每轮从干净状态启动 sim→onboard→GCS→标定→takeoff→
> `task prepare 150`→cleanup，失败即记、不重试覆盖。排程日志/START deviation/机载事件全程记录。

## 1. 环境

- workspace: /home/ub20tg/catkin_swarm6-2（PX4 SITL 15 机，无头）
- tcp_to_ros commit: `4b8cb26`
- d_ij（排程预测安全距离）: 2.0m
- runtime accept_timeout: 5s

## 2. 结果汇总

| 轮 | 时间 | 结果 | 详情 |
|---|---|---|---|
| 1 | 2026-09-07 22:37 | **SUCCESS** | SCHEDULE_OK(compute=0.168s,var=0.377,non_global=False), tau A03=2.197s/A04=0, START deviation=0, ACCEPTED<5s, task SUCCESS ticks=1638(~82s) 12/12, failed=[] |
| 2 | 2026-09-07 22:4x | **SUCCESS** | SCHEDULE_OK(compute=0.177s,var=0.390,non_global=False), START deviation=0, task SUCCESS ticks=1617(~81s) 12/12, failed=[] |
| 3 | 2026-09-07 22:5x | **SUCCESS** | SCHEDULE_OK(compute=0.178s,var=0.379,non_global=False), START deviation=0, task SUCCESS ticks=1658(~83s) 12/12, failed=[] |
| 4 | 2026-09-07 23:0x | **SUCCESS** | SCHEDULE_OK(compute=0.173s,var=0.382,non_global=False), START deviation=0, task SUCCESS ticks=1688(~84s) 12/12, failed=[] |
| 5 | 2026-09-07 23:0x | **FAIL** | SCHEDULE_OK(compute=0.164s,var=0.374,non_global=False), START deviation=0, 任务 ticks=600(~30s) 时 A08 level=EMERGENCY(peer=UAV9)/A09 INSUFFICIENT_DATA → FAILED。机载监督行为，非排程器错误 |

## 3. 结论

**结果：4/5 通过（1 FAIL）**。

- 第 1-4 轮 **SUCCESS**：每轮 SCHEDULE_OK（d_ij=2.0m）、compute 0.16-0.18s、non_global=False、
  START deviation=0、group ACCEPTED < 5s；任务 12/12 COMPLETED（ticks 1617-1688，约 81-84s）。
- 第 5 轮 **FAIL**：排程/START 同样全部正确，执行约 30s 时 A08 level=EMERGENCY（primary_peer=UAV9）、
  A09 INSUFFICIENT_DATA → 机载避碰监督触发。属现有 15 机 SITL 机载 intent/监督机制行为
  （与 schedule_safety_margin_m=0.5 时第 3 轮同类失败模式一致），非排程器错误。
- 对比：margin=0.5（d_ij=1.5m）时五次验收 0/5（test_record_26090106）；margin=1.0（d_ij=2.0m）
  时 4/5，排程把机间错开更宽，执行阶段机载 EMERGENCY 明显减少，但 intent 链路偶发问题仍存在。
- 每轮干净重启、失败即记、不重试覆盖；每轮 `cleanup.sh` 后进程 0、端口 clean。
- **整链路 4/5，不能视为 100% 通过；剩余 FAIL 为机载意图链路/监督偶发，非本次配置改动引入。**
