# Copyright 2026 Love Mitteregger
"""Joint-angle demo for the SO-ARM101 simulation.

Mirrors hardware_demo.cpp:
  Phase 1 — move to pose A, then sweep wrist roll from its minimum to its
             maximum actuator limit while holding all other joints.
  Phase 2 — reset to pose B.

Logs the MuJoCo EE position at each phase so the output can be compared
directly against hardware_demo output to assess the sim-to-real gap.

Usage:
	source .env
	uv run python3 python/demo.py

Joint order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
"""

import sys
import time

import numpy as np

sys.path.insert(0, 'python')

from env import SO101Env  # pylint: disable=wrong-import-position

# ---------------------------------------------------------------------------
# Poses — angles in radians, from MuJoCo (shoulder_pan=0 for both)
# wrist_roll in pose A is swept separately; placeholder None replaced below.
# ---------------------------------------------------------------------------

_POSE_A = [0.000, -1.050, 0.659, 1.140, None, 0.603]
_POSE_B = [0.000, -1.700, 1.350, 0.647, 0.000, 0.200]

_SETTLE_STEPS = 300  # sim steps to hold pose A / pose B before sampling
_ROLL_STEPS = 30  # increments in the wrist-roll sweep
_ROLL_STEP_HOLD = 8  # sim steps to hold each roll increment


def _step_n(env, action, n):
	obs = None
	for _ in range(n):
		obs, _, _, _, _ = env.step(action)
		env.render()
		time.sleep(env.dt)
	return obs


def run_demo():
	env = SO101Env(seed=0)
	env.reset()
	env.render()

	# pylint: disable=protected-access
	roll_min = env._model.actuator_ctrlrange[4, 0]
	roll_max = env._model.actuator_ctrlrange[4, 1]

	print("\nSO-ARM101 Joint Demo\n")
	print(f"{'Phase':<18} {'MuJoCo x(m)':>12} {'MuJoCo y(m)':>12} {'MuJoCo z(m)':>12}")
	print("-" * 60)

	# -----------------------------------------------------------------------
	# Phase 1a: move to pose A with wrist_roll at minimum
	# -----------------------------------------------------------------------

	action_a = np.array(_POSE_A, dtype=np.float64)
	action_a[4] = roll_min
	obs = _step_n(env, action_a, _SETTLE_STEPS)

	ee = obs[18:21]
	print(f"{'pose A':<18} {ee[0]:>12.4f} {ee[1]:>12.4f} {ee[2]:>12.4f}")

	# -----------------------------------------------------------------------
	# Phase 1b: sweep wrist roll from min to max
	# -----------------------------------------------------------------------

	for step in range(_ROLL_STEPS + 1):
		t = step / _ROLL_STEPS
		action_a[4] = roll_min + t * (roll_max - roll_min)
		obs = _step_n(env, action_a, _ROLL_STEP_HOLD)

	ee = obs[18:21]
	print(f"{'roll max':<18} {ee[0]:>12.4f} {ee[1]:>12.4f} {ee[2]:>12.4f}")

	# -----------------------------------------------------------------------
	# Phase 2: reset to pose B
	# -----------------------------------------------------------------------

	action_b = np.array(_POSE_B, dtype=np.float64)
	obs = _step_n(env, action_b, _SETTLE_STEPS)

	ee = obs[18:21]
	print(f"{'pose B (reset)':<18} {ee[0]:>12.4f} {ee[1]:>12.4f} {ee[2]:>12.4f}")

	env.close()
	print("\nCompare these EE values against hardware_demo output to assess sim-to-real gap.")


if __name__ == "__main__":
	run_demo()
