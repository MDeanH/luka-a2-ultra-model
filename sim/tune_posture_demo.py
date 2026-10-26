"""Deterministic tiny controller search exercising the fixed-base training API.

This tunes two PD gains for a synthetic posture task. It is an integration
example, not a learned walking policy or a proposed Luka controller.
"""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from leg_posture_env import LegPostureEnv, MODEL


def evaluate(env, kp: float, kd: float, seeds: range) -> float:
    returns = []
    for seed in seeds:
        obs, _ = env.reset(seed=seed)
        total = 0.0
        while True:
            q, velocity, target = obs[:12], obs[12:24], obs[24:36]
            torque = kp * (target - q) - kd * velocity
            action = np.clip(torque / env.force_limits, -1.0, 1.0)
            obs, reward, terminated, truncated, _ = env.step(action)
            total += reward
            if terminated or truncated:
                break
        returns.append(total)
    return float(np.mean(returns))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=2)
    parser.add_argument("--horizon", type=int, default=50)
    args = parser.parse_args()
    if args.episodes < 1:
        parser.error("--episodes must be positive")
    env = LegPostureEnv(horizon=args.horizon)
    candidates = [(0.0, 0.0), (0.1, 0.0), (0.5, 0.05),
                  (1.0, 0.1), (2.0, 0.2), (5.0, 0.5)]
    results = [
        {"kp_Nm_per_rad": kp, "kd_Nm_s_per_rad": kd,
         "mean_return": evaluate(env, kp, kd, range(args.episodes))}
        for kp, kd in candidates
    ]
    best = max(results, key=lambda result: result["mean_return"])
    print(json.dumps({
        "status": "synthetic fixed-base software demonstration only",
        "model": "build/luka_as_is.xml", "episodes_per_candidate": args.episodes,
        "model_sha256": hashlib.sha256(MODEL.read_bytes()).hexdigest(),
        "environment_sha256": hashlib.sha256((Path(__file__).resolve().parent / "leg_posture_env.py").read_bytes()).hexdigest(),
        "search_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "episode_seeds": list(range(args.episodes)),
        "horizon_steps": args.horizon, "frame_skip": env.frame_skip,
        "candidates": results, "best": best,
    }, indent=2))


if __name__ == "__main__":
    main()
