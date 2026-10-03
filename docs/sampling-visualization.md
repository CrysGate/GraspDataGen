# 采样可视化

在实验 YAML 配置中添加 `sampling_inspection: true`，开启采样记录。
默认关闭。开启后跳过自动 YAML 导出。

实验结束后，在仓库根目录执行：
    .venv/bin/graspdatagen sampling-inspect --run outputs/实验目录/夹爪--物体
    .venv/bin/python diagnostics/inspect_sampling_groups.py --run outputs/实验目录/夹爪--物体

没有采样记录的旧实验无法补画这些图。

## 数据含义

- all：经过采样筛选、送入验证的候选。
- passed：验证通过，但尚未去重及按目标数量截断的候选。
- accepted：最终保存在抓取数据集中的候选。
- base：基础采样。
- yaw：从成功的基础抓取生成的绕物体轴旋转候选，属于条件采样。

位置表示物体坐标系中的目标 TCP 位置，不是表面接触点，
也不是闭合后的实际 TCP 位置。

## 输出图像

- sampling-groups-counts.png：位置分箱数量，共用数量色标。
- sampling-groups-position-fractions.png：各来源、各阶段内部归一化的位置分布，共用比例色标。
- sampling-groups-distributions.png：高度、方位角和合并的接近方向分布。
- sampling-groups-directions.png：分来源、分阶段的接近方向分布。
- sampling-groups-acceptance.png：passed/all 验证通过率，以及 accepted/passed 保留率。
- sampling-groups-projections.png：XY、XZ、YZ 投影；浅色为未保留，深色为保留。
- sampling-groups.json：数量、分箱及统计结果。

比例图中的灰色表示分母没有样本。高比例应结合该分箱的样本数量阅读。
方向按等角度分箱，不是等立体角分箱。
不同实验的高度分箱分别计算，跨实验比较时需要核对坐标范围。
distributions 图中的合并方向面板使用各自的色标。

脚本检查 accepted 是否属于 passed，并检查其数量和 ID 是否与保存的数据集一致。

## 已验证实验

| 夹爪与物体 | 候选 | 验证通过 | 最终保留 |
| --- | ---: | ---: | ---: |
| ARX-X5 与 800g 奶茶杯 | 9224 | 1124 | 645 |
| Franka Panda 与套娃 00020 | 20530 | 5615 | 1024 |
| Franka Panda 与 800g 奶茶杯 | 6814 | 0 | 0 |

套娃实验正常完成，达到 1024 个目标。
零成功的奶茶杯实验也正常出图，覆盖了 accepted 和 yaw 为空的情况。
以上验证说明记录和绘图在这些资产上能够运行，不代表采样均匀或真机抓取性能。
