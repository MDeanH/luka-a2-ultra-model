"""Load a model in MuJoCo and report what MuJoCo compiled: DOFs, actuators, masses, joint ranges, stability.

Usage: .venv/bin/python -I tools/mj_inspect.py <file.mjcf|file.urdf> [--sim SECONDS] [--render out.png]
Read-only on the model files. Needs the project venv (requirements-sim.txt).
"""
import argparse
import os
import sys

import mujoco
import numpy as np


def load(path):
    return mujoco.MjModel.from_xml_path(path)


def summary(m):
    names = lambda kind, n: [mujoco.mj_id2name(m, kind, i) for i in range(n)]
    joints = names(mujoco.mjtObj.mjOBJ_JOINT, m.njnt)
    return dict(
        nbody=m.nbody, njnt=m.njnt, nq=m.nq, nv=m.nv, nu=m.nu, ngeom=m.ngeom, nmesh=m.nmesh,
        total_mass=float(m.body_mass.sum()),
        joints=joints,
        ranges={j: tuple(float(x) for x in m.jnt_range[i]) for i, j in enumerate(joints)},
        actuators=names(mujoco.mjtObj.mjOBJ_ACTUATOR, m.nu),
        timestep=float(m.opt.timestep),
        free_base=bool((m.jnt_type == mujoco.mjtJoint.mjJNT_FREE).any()),
    )


def passive_sim(m, seconds):
    """Zero control for `seconds`; returns (max |qvel| at the end, any NaN, final qpos)."""
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    for _ in range(int(seconds / m.opt.timestep)):
        mujoco.mj_step(m, d)
    return float(np.abs(d.qvel).max()) if m.nv else 0.0, bool(np.isnan(d.qpos).any()), d.qpos.copy()


def render(m, out, width=640, height=900):
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = m.stat.center
    cam.distance, cam.azimuth, cam.elevation = 1.9 * m.stat.extent, 150, -10
    with mujoco.Renderer(m, height, width) as r:
        r.update_scene(d, camera=cam)
        img = r.render()
    import struct
    import zlib
    raw = b"".join(b"\x00" + img[y].tobytes() for y in range(img.shape[0]))
    chunk = lambda t, b: struct.pack(">I", len(b)) + t + b + struct.pack(">I", zlib.crc32(t + b) & 0xFFFFFFFF)
    with open(out, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("model")
    ap.add_argument("--sim", type=float, default=0.0)
    ap.add_argument("--render")
    a = ap.parse_args(argv)
    m = load(a.model)
    s = summary(m)
    print(f"{os.path.basename(a.model)}: bodies {s['nbody']} joints {s['njnt']} nq {s['nq']} nv {s['nv']} "
          f"actuators {s['nu']} geoms {s['ngeom']} meshes {s['nmesh']} free_base {s['free_base']} "
          f"timestep {s['timestep']} total_mass {s['total_mass']:.3f} kg")
    for j in s["joints"]:
        lo, hi = s["ranges"][j]
        print(f"  {j:<28} range [{lo:+.4f}, {hi:+.4f}]")
    if a.sim:
        vmax, nan, _ = passive_sim(m, a.sim)
        print(f"passive sim {a.sim} s, zero control: max|qvel| at end {vmax:.4f}, NaN {nan}")
    if a.render:
        render(m, a.render)
        print(f"rendered {a.render}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
