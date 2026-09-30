# 采样与预算

候选覆盖物体表面上的接触位置、夹爪旋转角和接触深度。进入物理验证前，先筛除碰撞、姿态不合要求和重复的候选；成功结果按实际闭合姿态去重。

## 常用设置

在 `configs/parameters.yaml` 修改公共参数，或在运行配置中覆盖：

| 参数 | 控制内容 |
| --- | --- |
| `sampling.surface_samples` | 每轮表面采样数 |
| `sampling.surface_rounds` | 最大采样轮数 |
| `sampling.rolls_per_contact` | 每个接触点尝试的旋转角数量 |
| `sampling.clearance_m` | 几何安全间隙 |
| `posture.bottom_clearance_m` | 夹爪相对物体底面的最小间隙 |
| `candidate_budget` | 每个组合最多进入物理验证的候选数 |
| `time_budget_s` | 每个组合的生成时间预算，不含资产准备 |

达到成功目标或任一预算限制后停止。时间预算在批次之间检查，因此实际耗时可能超过设定值。

## 查看结果

在组合目录的 `manifest.json` 中查看 `successes`、`attempted` 和 `stop_reason`。
若 `attempted` 为零，先检查姿态、碰撞和底面间隙，单纯增加物理验证预算没有作用。

中断后使用相同配置和输出目录运行 `generate --resume`。不要手动修改运行分片或准备缓存。
