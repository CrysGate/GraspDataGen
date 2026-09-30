# 配置

所有路径相对于仓库根目录。

| 配置 | 用途 |
| --- | --- |
| `configs/runs/production.yaml` | 生产任务的物体、夹爪、GPU、预算和输出位置 |
| `configs/runs/gui-test.yaml` | 单物体 GUI 调试 |
| `configs/parameters.yaml` | 公共采样、姿态筛选和物理验证参数 |
| `configs/calibration.yaml` | 夹爪标定参数 |
| `configs/objects/*.yaml` | 物体 USD 路径、朝上方向、质量和材质 |
| `configs/robots/*.yaml` | 机器人 USD 路径、TCP 和关节名称 |
| `configs/grippers/*.yaml` | 夹爪提取范围、坐标轴、接触区域及标定文件 |
| `configs/audit.yaml` | 物理反例和敏感性检查 |
| `configs/experiments/*.yaml` | 专项实验参数，运行前检查资产和标注路径 |

## 调整任务

复制一份运行配置并修改 `output`。只在任务文件中覆盖与公共设置不同的值，例如：

```yaml
parameters: configs/parameters.yaml
manifest: configs/objects/our_assets.yaml
objects: [shampoo_000]
sampling:
  rolls_per_contact: 16
```

以上是配置片段，完整示例见 `configs/runs/gui-test.yaml`。

- 省略 `objects` 时处理整份清单；指定时按名称选择物体。
- `sampling`、`posture`、`validation` 可省略，或覆盖该节已有字段。
- `target_successes`、`candidate_budget`、`time_budget_s` 按物体／夹爪组合计算。
- 省略 `grasp_regions` 时采样全表面；指定时目录必须存在。没有对应标注的物体仍采样全表面。

## 添加资产

物体的质量和材质在 `physics` 中设置，`source` 写明来源或覆盖原因。`up_axis` 是物体原始坐标系的朝上单位向量。

夹爪的 `wrist_up_axis_base` 是基座坐标系的安装侧朝上单位向量；驱动和关节限制来自 USD。添加夹爪的步骤见[夹爪适配](grippers.md)。

恢复任务时保持输入和代码不变；修改后使用新输出目录。
