# Installation and commands

## Setup

Use Linux x86_64, Python 3.12, [uv](https://docs.astral.sh/uv/), and an NVIDIA GPU/driver for simulation. Run commands from the repository root; GPU commands must run outside the agent sandbox.

```bash
uv sync --locked
uv run --locked graspdatagen --help
```

Prepare the assets referenced by `configs/robots/` and `configs/objects/our_assets.yaml`, including their textures and dependencies. Update the YAML paths if needed. Example paths:

```text
Assets/Robots/piper/Piper.usd
Assets/Robots/x5/ARX.usd
Data/our_Assets/shampoo/000/Aligned.usd
Data/Assets/Object/Rigid/bubble_tea_cup/300g/Aligned.usd
```

Check the objects before generation:

```bash
uv run --locked graspdatagen inspect --manifest configs/objects/our_assets.yaml --output outputs/inspect.json
```

Inspection uses the CPU. For configuration options, see [Configuration](configuration.md).

## Generate

```bash
uv run --locked graspdatagen generate --config configs/runs/production.yaml
```

Set GPU, output directory and budgets in the run YAML. The production preset uses 2,048 environments and targets 1,024 grasps per object/gripper pair. Generation prepares assets automatically; results are stored in `outputs/our-assets-production/`.

Resume with unchanged inputs and code:

```bash
uv run --locked graspdatagen generate --config configs/runs/production.yaml --resume
```

For two GPUs:

```bash
uv run --locked graspdatagen generate-parallel --config configs/runs/production.yaml --devices 0,1
```

Each GPU handles separate grippers and writes to `<output>-gpu<N>`. There must be at least one gripper per GPU. Resume with the same device order and `--resume`.

## Replay and audit

Keep the run directory and its prepared caches. Replay defaults to all saved grasps; `--grasp-id` selects one. For parallel generation, use the corresponding GPU output directory.

```bash
uv run --locked graspdatagen replay --run outputs/our-assets-production/piper--bubble_tea_cup_300g \
  --output outputs/replay/piper--bubble_tea_cup_300g.json
uv run --locked graspdatagen audit --run outputs/our-assets-production/piper--bubble_tea_cup_300g \
  --config configs/audit.yaml --output outputs/audit/piper--bubble_tea_cup_300g.json
```

Check `complete`, `all_targets_reached` and individual failure reports. A completed generation run may still fall short of its grasp target.

## View and export

Generation exports `grasps-<robot>.yaml` beside each object USD. Open them in the [Web viewer](web-viewer.md). To export again:

```bash
uv run --locked graspdatagen export --run outputs/our-assets-production/piper--bubble_tea_cup_300g
```

For a live simulation, add `--gui` to generation, replay or audit from a graphical desktop. Unset `CUDA_VISIBLE_DEVICES`; set `--renderer-gpu` to the machine's Vulkan GPU index if the default is unsuitable. The GUI preset selects CUDA device 1:

```bash
env -u CUDA_VISIBLE_DEVICES uv run --locked graspdatagen generate --config configs/runs/gui-test.yaml --gui
```

## Development

```bash
uv run --locked ruff check src/graspdatagen
uv run --locked mypy
```

Validate changes through real preparation, generation and replay. See [known limitations](status.md) for outstanding checks.
