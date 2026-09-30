# GraspDataGen

基于 Isaac Sim 6.0.1、PhysX 和 Warp，为刚体 USD/USDZ 物体生成并验证平行夹爪抓取。支持 Piper 和 ARX-X5。

## 快速开始

需要 Linux x86_64、Python 3.12 和 [uv](https://docs.astral.sh/uv/)，物理仿真需要 NVIDIA GPU。按[安装说明](docs/installation.md)准备资产后，在仓库根目录运行：

```bash
uv sync --locked
uv run --locked graspdatagen inspect --manifest configs/objects/our_assets.yaml --output outputs/inspect.json
uv run --locked graspdatagen generate --config configs/runs/production.yaml
uv run --locked python -m graspdatagen.web
```

打开 http://127.0.0.1:8080 查看结果。资产检查和 Web 查看使用 CPU；GPU 命令在沙箱外的宿主机执行。

## 配置与输出

- 在 `configs/runs/production.yaml` 选择物体、夹爪、GPU 和运行预算。
- 在 `configs/parameters.yaml` 调整公共采样和验证参数。
- 运行数据与准备缓存写入 `outputs/`。
- 抓取结果自动导出到物体 USD 旁的 `grasps-<robot>.yaml`。
- 配置不变时使用 `generate --resume` 恢复；修改输入或代码后使用新输出目录。

## 文档

- [安装与命令](docs/installation.md)
- [配置](docs/configuration.md)
- [Web 查看器](docs/web-viewer.md)
- [添加夹爪](docs/grippers.md)
- [采样与预算](docs/sampling.md)
- [已知限制](docs/status.md)
