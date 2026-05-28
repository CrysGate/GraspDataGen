import numpy as np

# 加载 npz(allow_pickle=True 因为里面有 dict 等 Python 对象)
data = np.load('bots/onrobot_rg6.npz', allow_pickle=True)

# 1. 看里面都有哪些 key(就是数组的名字)
print(list(data.keys()))
# ['config', 'open_limit', 'num_openings', 'bite_point', 'bite_points', 
#  'open_widths', 'body_names', 'joint_names', 'body_transforms', ...]

# 2. 看具体某个数组的形状和类型
for key in data.keys():
  arr = data[key]
  print(f"{key:30s} shape={arr.shape} dtype={arr.dtype}")

# 3. 看具体内容
print(data['open_widths'])           # 比如 [0.0, 0.02, 0.04, 0.06, ...]
print(data['body_names'])            # ['base_frame', 'left_finger', 'right_finger', ...]
print(data['approach_axis'])         # 比如 2 (z 轴)
print(data['num_openings'])          # 比如 8

# 4. 标量字段需要 .item() 取出来
print(data['num_openings'].item())   # int

# 5. 字典字段也要 .item()
config = data['config'].item()
print(config)
# {'gripper_file': 'bots/onrobot_rg6.usd', 'bite': 0.025, ...}

# 6. 4D 数组(body_transforms)
bt = data['body_transforms']  # shape: (num_bodies, num_openings, 4, 4)
print(f"num_bodies={bt.shape[0]}, num_openings={bt.shape[1]}")
print("第 0 个 body 在第 0 档开合下的位姿矩阵:")
print(bt[0, 0])

data.close()