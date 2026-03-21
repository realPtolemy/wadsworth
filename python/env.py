# Copyright 2026 Love Mitteregger
"""SO-ARM101 MuJoCo simulation environment.

Observation space (21-dim, all in SI units):
	joint positions  [0:6]   rad      - shoulder_pan, shoulder_lift, elbow_flex,
	                                    wrist_flex, wrist_roll, gripper
	joint velocities [6:12]  rad/s
	ball position    [12:15] m        - world frame
	ball velocity    [15:18] m/s      - world frame (linear)
	end-effector pos [18:21] m        - gripperframe site, world frame

Action space (6-dim):
	Joint position targets in rad, clipped to each joint's ctrlrange.
"""

import os

import numpy as np
from mujoco import MjData, MjModel, mj_forward, mj_resetData, mj_step, viewer

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_SCENE_XML = os.path.join(os.path.dirname(__file__), '..', 'sim', 'scene.xml')

_N_JOINTS = 6
_OBS_DIM = 21  # 6 + 6 + 3 + 3 + 3
_ACT_DIM = 6

# Ball spawn region (x, y) on the table surface — within arm reach
_BALL_X_RANGE = (0.15, 0.25)
_BALL_Y_RANGE = (-0.05, 0.05)
_BALL_Z = 0.03  # m, must match geom radius in scene.xml

# Termination: ball considered fallen if z drops below this
_BALL_FALL_Z = -0.05

# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------


class SO101Env:
	"""Gymnasium-compatible environment for the SO-ARM101 with a ball.

	The reward function is intentionally left as a stub returning 0.0.
	Override _compute_reward() in a skill-specific subclass for each of
	the reach, grasp, and throw training phases.

	Args:
		scene_path: Path to the MuJoCo scene XML. Defaults to sim/scene.xml.
		frame_skip: Number of 2 ms sim steps per policy step.
		            Default 5 gives 10 ms per step (100 Hz policy).
		max_steps:  Episode length in policy steps before truncation.
		seed:       RNG seed for ball spawn randomisation.
	"""

	def __init__(
	    self,
	    scene_path: str = _SCENE_XML,
	    frame_skip: int = 5,
	    max_steps: int = 500,
	    seed: int | None = None,
	):
		self._model = MjModel.from_xml_path(os.path.abspath(scene_path))
		self._data = MjData(self._model)
		self.frame_skip = frame_skip
		self.max_steps = max_steps
		self._rng = np.random.default_rng(seed)
		self._step_count = 0
		self._viewer = None

		# Cache body / site ids
		self._ball_body_id = self._model.body('ball').id
		self._ee_site_id = self._model.site('gripperframe').id

		# Cache action bounds from model
		self._ctrl_low = self._model.actuator_ctrlrange[:_ACT_DIM, 0].copy()
		self._ctrl_high = self._model.actuator_ctrlrange[:_ACT_DIM, 1].copy()

	# -----------------------------------------------------------------------
	# Gymnasium interface
	# -----------------------------------------------------------------------

	def reset(self, *, seed: int | None = None):
		"""Reset the simulation and return (observation, info).

		Resets joint positions to zero, randomises ball position on the table.
		"""
		if seed is not None:
			self._rng = np.random.default_rng(seed)

		mj_resetData(self._model, self._data)
		self._step_count = 0

		# Randomise ball spawn position
		# qpos layout: arm joints [0:6] | ball pos xyz [6:9] | ball quat [9:13]
		self._data.qpos[6] = self._rng.uniform(*_BALL_X_RANGE)
		self._data.qpos[7] = self._rng.uniform(*_BALL_Y_RANGE)
		self._data.qpos[8] = _BALL_Z

		mj_forward(self._model, self._data)
		return self._get_obs(), {}

	def step(self, action: np.ndarray):
		"""Apply action, advance simulation, return Gymnasium 5-tuple.

		Returns:
			obs:        np.ndarray, shape (21,)
			reward:     float
			terminated: bool — ball fell off table
			truncated:  bool — max_steps reached
			info:       dict
		"""
		self._data.ctrl[:_ACT_DIM] = np.clip(action, self._ctrl_low, self._ctrl_high)

		for _ in range(self.frame_skip):
			mj_step(self._model, self._data)

		self._step_count += 1
		obs = self._get_obs()
		reward = self._compute_reward(obs)
		terminated = self._is_terminated(obs)
		truncated = self._step_count >= self.max_steps

		return obs, reward, terminated, truncated, {}

	def render(self):
		"""Open (or sync) a passive viewer window.

		Creates a viewer on the first call; subsequent calls sync the display.
		Non-blocking — the simulation loop continues while the window is open.
		"""
		if self._viewer is None:
			self._viewer = viewer.launch_passive(self._model, self._data)
		self._viewer.sync()

	def is_running(self) -> bool:
		"""Return True if the viewer window is open."""
		return self._viewer is not None and self._viewer.is_running()

	def close(self):
		"""Close the viewer if open."""
		if self._viewer is not None:
			self._viewer.close()
			self._viewer = None

	# -----------------------------------------------------------------------
	# Override in skill-specific subclasses
	# -----------------------------------------------------------------------

	def _compute_reward(self, obs: np.ndarray) -> float:  # pylint: disable=unused-argument
		"""Reward function stub. Override per skill (reach, grasp, throw)."""
		return 0.0

	def _is_terminated(self, obs: np.ndarray) -> bool:
		"""Terminate when the ball falls off the table."""
		ball_z = obs[14]  # obs[12:15] = ball_pos, index 14 = z
		return bool(ball_z < _BALL_FALL_Z)

	# -----------------------------------------------------------------------
	# Observation
	# -----------------------------------------------------------------------

	def _get_obs(self) -> np.ndarray:
		joint_pos = self._data.qpos[:_N_JOINTS].copy()
		joint_vel = self._data.qvel[:_N_JOINTS].copy()
		ball_pos = self._data.xpos[self._ball_body_id].copy()
		ball_vel = self._data.qvel[6:9].copy()  # freejoint linear vel
		ee_pos = self._data.site_xpos[self._ee_site_id].copy()
		return np.concatenate([joint_pos, joint_vel, ball_pos, ball_vel, ee_pos])

	# -----------------------------------------------------------------------
	# Properties
	# -----------------------------------------------------------------------

	@property
	def obs_dim(self) -> int:
		return _OBS_DIM

	@property
	def act_dim(self) -> int:
		return _ACT_DIM

	@property
	def dt(self) -> float:
		"""Wall-clock seconds per policy step."""
		return self._model.opt.timestep * self.frame_skip
