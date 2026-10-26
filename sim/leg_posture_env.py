"""Small fixed-base MuJoCo task for checking a training pipeline.

This controls the 12 leg actuators in the baseline candidate. It does not
represent Luka's retained x86 balance controller or a deployable robot policy.
"""

from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "build/luka_as_is.xml"


class LegPostureEnv:
    """Gym-shaped reset/step API; no Gym dependency or hardware transport."""

    def __init__(self, frame_skip: int = 10, horizon: int = 100):
        if frame_skip < 1 or horizon < 1:
            raise ValueError("frame_skip and horizon must be positive")
        self.model = mujoco.MjModel.from_xml_path(str(MODEL))
        if self.model.nu != 12 or self.model.nq != 28:
            raise ValueError("unexpected baseline model dimensions")
        self.data = mujoco.MjData(self.model)
        joint_ids = self.model.actuator_trnid[:, 0].astype(int)
        if len(set(joint_ids)) != 12:
            raise ValueError("expected 12 distinct actuated joints")
        self.qpos_ids = self.model.jnt_qposadr[joint_ids]
        self.qvel_ids = self.model.jnt_dofadr[joint_ids]
        self.joint_names = tuple(mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, int(i)) for i in joint_ids)
        self.force_limits = self.model.actuator_forcerange[:, 1].copy()
        if not np.all(self.force_limits > 0):
            raise ValueError("expected positive torque limits")
        self.joint_ranges = self.model.jnt_range[joint_ids].copy()
        self.frame_skip = frame_skip
        self.horizon = horizon
        self.steps = 0
        self.done = False
        self.target = np.zeros(12, dtype=np.float64)
        self.rng = np.random.default_rng(0)

    def observation(self) -> np.ndarray:
        return np.concatenate((
            self.data.qpos[self.qpos_ids],
            self.data.qvel[self.qvel_ids],
            self.target,
        )).copy()

    def reset(self, seed: int | None = None, target: np.ndarray | None = None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.model, self.data)
        target_source = "synthetic" if target is None else "supplied"
        if target is None:
            # Small synthetic task around XML zero pose; not a physical safe range.
            sampled = self.rng.uniform(-0.15, 0.15, size=12)
            self.target = np.clip(sampled, self.joint_ranges[:, 0], self.joint_ranges[:, 1])
        else:
            target = np.asarray(target, dtype=np.float64)
            if target.shape != (12,) or not np.all(np.isfinite(target)):
                raise ValueError("target must be 12 finite joint coordinates")
            if np.any(target < self.joint_ranges[:, 0]) or np.any(target > self.joint_ranges[:, 1]):
                raise ValueError("target outside the candidate XML ranges")
            self.target = target.copy()
        self.steps = 0
        self.done = False
        mujoco.mj_forward(self.model, self.data)
        return self.observation(), {"joint_names": self.joint_names, "target_source": target_source}

    def step(self, action):
        action = np.asarray(action, dtype=np.float64)
        if action.shape != (12,) or not np.all(np.isfinite(action)):
            raise ValueError("action must be 12 finite normalized torque commands")
        if self.done:
            raise RuntimeError("episode ended; call reset")
        bounded = np.clip(action, -1.0, 1.0)
        self.data.ctrl[:] = bounded * self.force_limits
        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)
        self.steps += 1
        obs = self.observation()
        finite = bool(np.all(np.isfinite(obs)))
        error = (obs[:12] - self.target) / (self.joint_ranges[:, 1] - self.joint_ranges[:, 0])
        reward = float(-np.mean(error**2) - 0.001 * np.mean(obs[12:24]**2) - 0.0001 * np.mean(bounded**2)) if finite else -1e6
        info = {
            "time_s": float(self.data.time),
            "error_rms_rad": float(np.sqrt(np.mean((obs[:12] - self.target)**2))) if finite else float("inf"),
            "finite": finite,
        }
        terminated, truncated = not finite, self.steps >= self.horizon
        self.done = terminated or truncated
        return obs, reward, terminated, truncated, info
