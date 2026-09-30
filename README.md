# GraspDataGen

Generate and validate parallel-jaw grasps for rigid USD/USDZ objects using Isaac Sim 6.0.1, PhysX, and Warp. Supported grippers: Piper and ARX-X5.

## Quick start

Requires Linux x86_64, Python 3.12, [uv](https://docs.astral.sh/uv/), and an NVIDIA GPU for simulation. Set up the assets listed in the [installation guide](docs/installation.md), then run from the repository root:

```bash
uv sync --locked
uv run --locked graspdatagen inspect --manifest configs/objects/our_assets.yaml --output outputs/inspect.json
uv run --locked graspdatagen generate --config configs/runs/production.yaml
uv run --locked python -m graspdatagen.web
```

Open http://127.0.0.1:8080 to view results. Inspection and Web viewing use the CPU.
Run GPU commands on the host, outside the agent sandbox.

## Configuration and results

- Choose objects, grippers, GPU and budgets in `configs/runs/production.yaml`.
- Adjust shared sampling and validation settings in `configs/parameters.yaml`.
- Run data and prepared assets are stored in `outputs/`.
- Generation exports `grasps-<robot>.yaml` beside each source object USD.
- Resume an unchanged run with `generate --resume`; use a new output directory after changing inputs or code.

## Documentation

- [Installation and commands](docs/installation.md)
- [Configuration](docs/configuration.md)
- [Web viewer](docs/web-viewer.md)
- [Adding a gripper](docs/grippers.md)
- [Sampling and budgets](docs/sampling.md)
- [Known limitations](docs/status.md)

[中文说明](README.zh-CN.md)
