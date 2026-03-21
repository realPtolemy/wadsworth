# Copyright 2026 Love Mitteregger
"""Interactive smoke test for SO101Env.

Runs the environment with random actions and opens a live viewer.
Press Ctrl+C to exit.

Usage:
	source .env
	uv run python3 python/test_env.py
"""

import sys
import time

sys.path.insert(0, 'python')

import numpy as np  # pylint: disable=wrong-import-position
from env import SO101Env  # pylint: disable=wrong-import-position

env = SO101Env(seed=0)
obs, _ = env.reset()
env.render()

episode = 0
step = 0

print(f"dt={env.dt*1000:.0f}ms/step  obs_dim={env.obs_dim}  act_dim={env.act_dim}")
print("Running — close the viewer window or press Ctrl+C to stop.\n")

try:
	while env.is_running():
		action = np.random.uniform(-0.3, 0.3, size=env.act_dim)
		obs, reward, terminated, truncated, _ = env.step(action)
		env.render()
		step += 1

		if terminated or truncated:
			ball_pos = obs[12:15]
			ee_pos = obs[18:21]
			print(f"ep={episode:3d}  steps={step:4d}  "
			      f"ball=({ball_pos[0]:.3f}, {ball_pos[1]:.3f}, {ball_pos[2]:.3f})  "
			      f"ee=({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f})  "
			      f"{'TERMINATED' if terminated else 'truncated'}")
			obs, _ = env.reset()
			episode += 1
			step = 0

		time.sleep(env.dt)

except KeyboardInterrupt:
	pass
finally:
	env.close()
	print("Done.")
