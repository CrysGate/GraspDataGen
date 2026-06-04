# GraspDataGen 使用命令

这份文件只放常用运行命令和参数注意事项。完整流程是先用 `datagen.py`
生成候选抓取，再用 `grasp_sim.py` 做仿真验证；只有仿真通过的 grasp 才算可用。

## 参数规则

- `datagen.py` 生成候选抓取可以使用 `--max_num_envs 512`，包括 cola。
- 常规单独仿真可以先用 `--max_num_envs 256`。
- `cola.obj` 只有在 `grasp_sim.py` 仿真时必须把 `--max_num_envs` 降到 `64`，否则容易爆内存。
- `plate.obj` 对浅夹爪比较吃力，扰动力通常要降到 `0.2` 到 `0.6`。

## 批处理

`objects/datagen_example.json` 当前包含 `cola.obj`，批处理这里只做 datagen，默认使用
`--max_num_envs 512`。

```bash
uv run python scripts/graspgen/tools/batch_datagen_grippers.py \
  --object_scales_json objects/datagen_example.json \
  --object_root objects \
  --num_grasps 2048 \
  --max_num_envs 512 \
  --force_magnitude 1.0 \
  --overwrite_existing \
  --min_success_grasps 1
```

后续单独仿真 `cola.obj` 时仍使用 `--max_num_envs 64`。

## 单夹爪模板

### 生成候选抓取

```bash
uv run python scripts/graspgen/datagen.py \
  --gripper_config <gripper_config> \
  --object_scales_json objects/datagen_example.json \
  --object_root objects \
  --num_grasps 1024 \
  --max_num_envs 512 \
  --force_magnitude 1.0 \
  --overwrite_existing
```

### 常规仿真验证

```bash
uv run scripts/graspgen/grasp_sim.py \
  --grasp_file datagen_sim_data/<gripper_config>/<object>.yaml \
  --object_file objects/<object>.obj \
  --object_scale 1.0 \
  --max_num_envs 256 \
  --max_num_grasps 2048 \
  --force_headed
```

### Cola 仿真验证

`objects/datagen_example.json` 里的 cola scale 是 `0.1`，对应输出通常是
`cola.0.1.yaml`。如果使用 scale `1.0`，输出文件名通常是 `cola.yaml`。

```bash
uv run scripts/graspgen/grasp_sim.py \
  --grasp_file datagen_sim_data/<gripper_config>/cola.0.1.yaml \
  --object_file objects/cola.obj \
  --object_scale 0.1 \
  --max_num_envs 64 \
  --max_num_grasps 2048 \
  --force_headed
```

## Robotiq 2F 85

```bash
uv run python scripts/graspgen/datagen.py \
  --gripper_config robotiq_2f_85 \
  --object_scales_json objects/datagen_example.json \
  --object_root objects \
  --num_grasps 1024 \
  --max_num_envs 512 \
  --force_magnitude 1.0 \
  --overwrite_existing
```

Plate 专项：

```bash
uv run python scripts/graspgen/datagen.py \
  --gripper_config robotiq_2f_85 \
  --object_scales_json objects/plate.json \
  --object_root objects \
  --num_grasps 1024 \
  --max_num_envs 512 \
  --force_magnitude 0.5 \
  --overwrite_existing

uv run scripts/graspgen/grasp_sim.py \
  --grasp_file datagen_sim_data/robotiq_2f_85/plate.yaml \
  --object_file objects/plate.obj \
  --object_scale 1.0 \
  --max_num_envs 256 \
  --max_num_grasps 2048 \
  --force_headed \
  --force_magnitude 0.6
```

## OnRobot RG6

```bash
uv run python scripts/graspgen/datagen.py \
  --gripper_config onrobot_rg6 \
  --object_scales_json objects/datagen_example.json \
  --object_root objects \
  --num_grasps 1024 \
  --max_num_envs 512 \
  --force_magnitude 1.0 \
  --overwrite_existing

uv run scripts/graspgen/grasp_sim.py \
  --grasp_file datagen_sim_data/onrobot_rg6/banana.0.75.yaml \
  --object_file objects/banana.obj \
  --object_scale 0.75 \
  --max_num_envs 256 \
  --max_num_grasps 2048 \
  --force_headed

uv run scripts/graspgen/grasp_sim.py \
  --grasp_file datagen_sim_data/onrobot_rg6/handwheel.yaml \
  --object_file objects/handwheel.obj \
  --object_scale 1.0 \
  --max_num_envs 256 \
  --max_num_grasps 2048 \
  --force_headed
```

## Franka Panda

```bash
uv run python scripts/graspgen/datagen.py \
  --gripper_config franka_panda \
  --object_scales_json objects/datagen_example.json \
  --object_root objects \
  --num_grasps 1024 \
  --max_num_envs 512 \
  --force_magnitude 1.0 \
  --overwrite_existing
```

Plate 专项：

```bash
uv run python scripts/graspgen/datagen.py \
  --gripper_config franka_panda \
  --object_scales_json objects/plate.json \
  --object_root objects \
  --num_grasps 2048 \
  --max_num_envs 512 \
  --force_magnitude 0.5 \
  --overwrite_existing

uv run scripts/graspgen/grasp_sim.py \
  --grasp_file datagen_sim_data/franka_panda/plate.yaml \
  --object_file objects/plate.obj \
  --object_scale 1.0 \
  --max_num_envs 256 \
  --max_num_grasps 2048 \
  --force_headed \
  --force_magnitude 0.2
```

## Piper V2 Gripper

```bash
uv run python scripts/graspgen/datagen.py \
  --gripper_config piper_v2_gripper \
  --object_scales_json objects/datagen_example.json \
  --object_root objects \
  --num_grasps 1024 \
  --max_num_envs 512 \
  --force_magnitude 1.0 \
  --overwrite_existing

uv run scripts/graspgen/grasp_sim.py \
  --grasp_file datagen_sim_data/piper_v2_gripper/banana.0.75.yaml \
  --object_file objects/banana.obj \
  --object_scale 0.75 \
  --max_num_envs 256 \
  --max_num_grasps 2048 \
  --force_headed
```

## Piper H V1 Gripper

```bash
uv run python scripts/graspgen/datagen.py \
  --gripper_config piper_h_v1_gripper \
  --object_scales_json objects/datagen_example.json \
  --object_root objects \
  --num_grasps 1024 \
  --max_num_envs 512 \
  --force_magnitude 1.0 \
  --overwrite_existing

uv run scripts/graspgen/grasp_sim.py \
  --grasp_file datagen_sim_data/piper_h_v1_gripper/handwheel.yaml \
  --object_file objects/handwheel.obj \
  --object_scale 1.0 \
  --max_num_envs 256 \
  --max_num_grasps 2048 \
  --force_headed
```

## Piper L V1 Gripper

```bash
uv run python scripts/graspgen/datagen.py \
  --gripper_config piper_l_v1_gripper \
  --object_scales_json objects/datagen_example.json \
  --object_root objects \
  --num_grasps 1024 \
  --max_num_envs 512 \
  --force_magnitude 1.0 \
  --overwrite_existing

uv run scripts/graspgen/grasp_sim.py \
  --grasp_file datagen_sim_data/piper_l_v1_gripper/mug.yaml \
  --object_file objects/mug.obj \
  --object_scale 1.0 \
  --max_num_envs 256 \
  --max_num_grasps 2048 \
  --force_headed
```

## Piper X V1 Gripper

```bash
uv run python scripts/graspgen/datagen.py \
  --gripper_config piper_x_v1_gripper \
  --object_scales_json objects/datagen_example.json \
  --object_root objects \
  --num_grasps 1024 \
  --max_num_envs 512 \
  --force_magnitude 1.0 \
  --overwrite_existing

uv run scripts/graspgen/grasp_sim.py \
  --grasp_file datagen_sim_data/piper_x_v1_gripper/plate.yaml \
  --object_file objects/plate.obj \
  --object_scale 1.0 \
  --max_num_envs 256 \
  --max_num_grasps 2048 \
  --force_headed
```

## ChangingTek AG2F90

```bash
uv run python scripts/graspgen/datagen.py \
  --gripper_config changingtek_ag2f90 \
  --object_scales_json objects/datagen_example.json \
  --object_root objects \
  --num_grasps 1024 \
  --max_num_envs 512 \
  --force_magnitude 1.0 \
  --overwrite_existing

uv run scripts/graspgen/grasp_sim.py \
  --grasp_file datagen_sim_data/changingtek_ag2f90/banana.0.75.yaml \
  --object_file objects/banana.obj \
  --object_scale 0.75 \
  --max_num_envs 256 \
  --max_num_grasps 2048 \
  --force_headed
```

## OmniPicker

```bash
uv run python scripts/graspgen/datagen.py \
  --gripper_config omnipicker \
  --object_scales_json objects/datagen_example.json \
  --object_root objects \
  --num_grasps 1024 \
  --max_num_envs 512 \
  --force_magnitude 1.0 \
  --overwrite_existing
```

Plate 专项：

```bash
uv run scripts/graspgen/grasp_sim.py \
  --grasp_file datagen_sim_data/omnipicker/plate.yaml \
  --object_file objects/plate.obj \
  --object_scale 1.0 \
  --max_num_envs 64 \
  --max_num_grasps 1024 \
  --force_headed \
  --force_magnitude 0.5 \
  --gravity_force_scale 3.0 \
  --headed_hold_seconds 1.0
```

Banana 验证：

```bash
uv run scripts/graspgen/grasp_sim.py \
  --grasp_file datagen_sim_data/omnipicker/banana.0.75.yaml \
  --object_file objects/banana.obj \
  --object_scale 0.75 \
  --max_num_envs 256 \
  --max_num_grasps 2048 \
  --force_headed
```

## G2 Right OmniPicker Gripper

```bash
uv run python scripts/graspgen/datagen.py \
  --gripper_config g2_right_omnipicker_gripper \
  --object_scales_json objects/datagen_example.json \
  --object_root objects \
  --num_grasps 1024 \
  --max_num_envs 512 \
  --force_magnitude 1.0 \
  --overwrite_existing

uv run scripts/graspgen/grasp_sim.py \
  --grasp_file datagen_sim_data/g2_right_omnipicker_gripper/banana.0.75.yaml \
  --object_file objects/banana.obj \
  --object_scale 0.75 \
  --max_num_envs 256 \
  --max_num_grasps 2048 \
  --force_headed
```
