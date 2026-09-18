# GraspDataGen 中文说明

GraspDataGen 基于 Isaac Sim 6.0.1、PhysX 和 Warp，从刚体 USD/USDZ 资产生成并验证平行夹爪抓取数据集。

## 快速开始

```bash
uv sync --locked
export OMNI_KIT_ACCEPT_EULA=YES
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=8
uv run graspdatagen generate --config configs/runs/production.yaml
```

常用命令：

```bash
uv run graspdatagen export --run outputs/production/piper--bottle
uv run graspdatagen view --grasps outputs/production/piper--bottle/grasps.yaml
uv run python -m graspdatagen.web
```

Web 查看器地址为 http://127.0.0.1:8080。恢复任务使用 `--resume`，重放任务使用 `graspdatagen replay`。涉及 GPU 的命令必须在宿主机运行，源资产放在 `Assets/`，生成内容写入 `outputs/`。

详细说明：

- [安装与 CLI](docs/installation.md)
- [数据与坐标约定](docs/contracts.md)
- [Web 查看器](docs/web-viewer.md)
- [夹爪适配](docs/grippers.md)
- [状态与待办](docs/refactor-plan.md)
