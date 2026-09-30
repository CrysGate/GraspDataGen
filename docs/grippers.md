# Adding a gripper

Piper and ARX-X5 are supported. New grippers must use the same two-finger, single-command parallel-jaw model.

1. Add the source USD and a configuration under `configs/robots/`. Set the TCP pose, active joint and finger names. Use metres and `xyzw` quaternions; drives and limits come from USD.
2. Add `configs/grippers/<name>.yaml`. Set retained bodies and joints, finger colliders, coordinate axes, contact region, material and calibration file. Keep attachments that affect collision clearance.
3. Prepare the gripper with a real object:

   ```bash
   uv run --locked graspdatagen prepare --manifest configs/objects/our_assets.yaml \
     --gripper configs/grippers/<name>.yaml --output outputs/prepare-<name>.json
   ```

4. Open the gripper cache path reported in the output. Check `inspection.png` and `gripper.usdc`: the TCP, opening direction and open/middle/closed states must match the source robot.
5. Add the gripper to a run configuration, generate a small dataset, and replay it in a fresh process. Confirm stable bilateral contact and valid joint motion before increasing the budget.

After changing geometry, TCP, material or calibration, generate into a new output directory.
