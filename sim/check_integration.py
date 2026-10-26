"""Run structural and behavioral checks against this repo's simulation output."""

import hashlib
import math
from pathlib import Path
import sys
import tempfile

import mujoco
import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
from build_contact_scene import OUTPUT, generate  # noqa: E402
from leg_posture_env import LegPostureEnv  # noqa: E402
from preview_ipad_pose import DATA, inspect_frames, validate_render_output, verified_recording  # noqa: E402


def check_scene() -> dict:
    assert OUTPUT.read_bytes() == generate(), "checked-in scene has drifted"
    baseline = mujoco.MjModel.from_xml_path(str(ROOT / "build/luka_as_is.xml"))
    scene = mujoco.MjModel.from_xml_path(str(OUTPUT))
    assert baseline.nq == 28 and baseline.nv == 28 and baseline.nu == 12
    assert scene.nq == 35 and scene.nv == 34 and scene.nu == 12
    floor = mujoco.mj_name2id(scene, mujoco.mjtObj.mjOBJ_GEOM, "training_floor")
    assert floor >= 0 and scene.geom_bodyid[floor] == 0
    data = mujoco.MjData(scene)
    mujoco.mj_forward(scene, data)
    initial_contacts = data.ncon
    initial_height = float(data.qpos[2])
    for _ in range(round(1.0 / scene.opt.timestep)):
        mujoco.mj_step(scene, data)
    assert np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))
    assert data.ncon > 0, "no floor contact after the unbalanced model falls"
    assert all(floor in (contact.geom1, contact.geom2) for contact in data.contact)
    assert float(data.qpos[2]) < initial_height, "zero-control model unexpectedly did not fall"
    return {"initial_contacts": initial_contacts, "contacts_after_1s": data.ncon,
            "base_z_start_m": initial_height, "base_z_after_1s_m": float(data.qpos[2])}


def check_vendor_inputs() -> int:
    vendor = ROOT / "vendor/agibot_a2"
    lines = (vendor / "SHA256SUMS").read_text().splitlines()
    for line in lines:
        expected, relative = line.split("  ", 1)
        actual = hashlib.sha256((vendor / relative).read_bytes()).hexdigest()
        assert actual == expected, f"vendor input changed: {relative}"
    return len(lines)


def check_recordings() -> dict:
    model = mujoco.MjModel.from_xml_path(str(ROOT / "build/luka_as_is.xml"))
    expected = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(26)}
    counts = {}
    for path in sorted((DATA / "recordings").glob("*.a2rec")):
        entry = verified_recording(path)
        count, sample = inspect_frames(path, expected, 0)
        assert count == entry["messages"]
        assert all(math.isfinite(value) for value in sample["j"].values())
        counts[path.name] = count
    assert len(counts) == 3
    return counts


def check_originals_cannot_be_render_targets() -> None:
    original = DATA / "recordings/luka-20261019-115249-legs.a2rec"
    with tempfile.TemporaryDirectory() as directory:
        existing = Path(directory) / "existing.png"
        existing.write_bytes(b"keep")
        for forbidden in (original, DATA / "derived.png", existing):
            try:
                validate_render_output(forbidden)
            except (ValueError, FileExistsError):
                pass
            else:
                raise AssertionError(f"render accepted protected path: {forbidden}")
        assert existing.read_bytes() == b"keep"
    verified_recording(original)


def check_training_api() -> dict:
    env = LegPostureEnv(horizon=3)
    a, info = env.reset(seed=42)
    b, _ = env.reset(seed=42)
    assert info["target_source"] == "synthetic" and np.array_equal(a, b)
    assert a.shape == (36,)
    for _ in range(3):
        observation, reward, terminated, truncated, state = env.step(np.zeros(12))
        assert observation.shape == (36,) and np.isfinite(reward) and state["finite"]
        assert not terminated
    assert truncated
    try:
        env.step(np.zeros(12))
    except RuntimeError:
        pass
    else:
        raise AssertionError("ended episode accepted another action")
    return {"observation_values": 36, "actions": 12, "horizon_steps": 3}


def main() -> None:
    import json
    check_originals_cannot_be_render_targets()
    print(json.dumps({"status": "software-only integration checks",
                      "vendor_files_checked": check_vendor_inputs(),
                      "scene": check_scene(),
                      "recordings": check_recordings(),
                      "fixed_base_task": check_training_api()}, indent=2))


if __name__ == "__main__":
    main()
