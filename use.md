uv run scripts/graspgen/grasp_sim.py \
      --grasp_file datagen_sim_data/robotiq_2f_85/banana.0.75.yaml \
      --object_file objects/banana.obj \
      --object_scale 0.75 \
      --max_num_envs 256 \
      --max_num_grasps 2048 \
      --force_headed \
      --force_magnitude 2.0 \
      --headed_hold_seconds 3.0 \
      --gravity_force_scale 2.0

python scripts/graspgen/datagen.py \
    --gripper_config franka_panda \
    --object_scales_json objects/datagen_example.json \
    --object_root objects \
    --num_grasps 2048 \
    --debug_validation_stats \
    --max_num_envs 512

python scripts/graspgen/datagen.py \
    --gripper_config robotiq_2f_85 \
    --object_scales_json objects/datagen_example.json \
    --object_root objects \
    --num_grasps 1024 \
    --max_num_envs 256

uv run scripts/graspgen/grasp_sim.py \
      --grasp_file datagen_sim_data/robotiq_2f_85/banana.0.75.yaml \
      --object_file objects/banana.obj \
      --object_scale 0.75 \
      --max_num_envs 256 \
      --max_num_grasps 1024 \
      --force_headed \
      --force_magnitude 3.0 \
      --gravity_force_scale 3.0 \
      --headed_hold_seconds 1.0


Piper USD的碰撞 API 原来主要挂在 Xform 父节点，实际 Mesh 没有直接成为 collider，导致验证时 finger 接触经常为 0。
```bash
scripts/graspgen/tools/extract_piper_gripper_usd.py:665：生成 Piper USD 时把 PhysicsCollisionAPI / PhysicsMeshCollisionAPI 直接写到 collision Mesh prim 上。

scripts/graspgen/tools/extract_piper_gripper_usd.py:692：给 Piper finger 绑定高摩擦 physics material。

scripts/graspgen/tools/extract_piper_gripper_usd.py:995：提高 Piper 主动 prismatic joint drive 到 stiffness=4000 / damping=400 / maxForce=1000。已重新生成 bots/piper_v2_gripper.usd 和 bots/piper_v2_gripper.npz。

scripts/graspgen/gripper_configurations.py:72：piper_v2_gripper 默认关闭 reject_interpenetration，因为这个三角网格相交检查会把物理接触后的微小穿透全部过滤掉。

scripts/graspgen/grasp_sim.py:178：加了 --debug_validation_stats，方便以后看左右 finger 接触数、interpenetration 数 和最终 success 数。

scripts/graspgen/grasp_sim.py:686：顺手修了 --max_num_grasps 小于候选数量时的 slice bug。
```

1. P1: 默认开启的穿透检查会让 fresh clone 下的 Franka/Piper 抓取验证失败或依赖本地脏文件。
     default_reject_interpenetration=True 后，读取 grasp file 时会强制进入
     load_interpenetration_meshes_from_grasp_file()，但这里构造 GripperConfig 只用了 CLI 的 base_frame/
     bite/...，没有读取 grasp YAML 里保存的 gripper_frame_link，也没有传入新增的 approach_axis/open_axis/
     bite_mid_axis_position。 fresh clone 里 bots/*.npz 没被 commit 追踪，创建 Franka/Piper gripper 时会落到默
     认 base_frame="base_frame"，而 Franka 实际是 panda_hand，Piper 是 link6/Link6。相关位置：scripts/graspgen/
     grasp_sim.py:422, scripts/graspgen/grasp_sim.py:430, scripts/graspgen/grasp_sim.py:708, scripts/graspgen/
     grasp_sim.py:840。
2. P1: interpenetration 路径用 skip_config_validation=True 会静默接受陈旧/不匹配的 .npz。
    这个检查依赖 gripper mesh、finger_indices 和 bite_point 来把 finger0 mesh 从 bite frame 还原到 link
    frame；这些数据都和 finger_colliders/bite/open_axis/approach_axis 强相关。但这里跳过了 saved config 校验，
    导致本地已有旧 bots/<gripper>.npz 时不会重建，结果可能随本地未跟踪文件变化，产生错误的穿透拒绝/放行。
    scripts/graspgen/grasp_sim.py:397, scripts/graspgen/grasp_sim.py:439, scripts/graspgen/grasp_sim.py:457,
    scripts/graspgen/gripper.py:289。
3. P2: --headed_hold_seconds 有限等待后，force_headed 模式不会关闭 SimulationApp。
    新逻辑允许 headed 模式等固定秒数后跳出渲染循环，但 main() 仍然因为 args.force_headed 为真而跳过
    simulation_app.close()。这会让“有限 hold”语义不完整，可能留下 Isaac app/窗口资源不关闭。scripts/graspgen/
    grasp_sim.py:1465, scripts/graspgen/grasp_sim.py:1473, scripts/graspgen/grasp_sim.py:1526。
