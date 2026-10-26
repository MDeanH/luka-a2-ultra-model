"""Fill the ledger's missing link masses with flagged ESTIMATES (Michael's decision, 2026-10-22).

Neither vendor file gives the hands or wrist motor/rod links any mass. The estimate is MuJoCo's inertia of each link's
vendor VISUAL mesh treated as solid at 1000 kg/m^3 (collision hulls ignored, so nothing is counted twice). It is a
guess with a stated method, not vendor data. Weighing Luka replaces it.

Usage: .venv/bin/python -I tools/estimate_inertials.py      (merge: only acts on links whose mass is `missing`)
"""
import csv
import os
import sys

import mujoco
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "tools"))
import seed_ledger as sl  # noqa: E402

MJCF = os.path.join(REPO, "vendor", "agibot_a2", "mjcf", "agibot_a2_dof12.mjcf")
DENSITY = 1000.0
EVIDENCE = ("estimate: MuJoCo 3.17 inertia of the vendor visual mesh, solid at 1000 kg/m^3, collision hulls ignored "
            "(Michael's choice 2026-10-22)")
NOTE = "ESTIMATE - neither vendor file gives this link a mass; replace with a weighed value"


def visual_inertials():
    """{body: (mass, com[3], full inertia [ixx, iyy, izz, ixy, ixz, iyz])} from visual meshes only."""
    xml = open(MJCF).read().replace('meshdir="."', f'meshdir="{os.path.dirname(MJCF)}"', 1)
    spec = mujoco.MjSpec.from_string(xml)
    for g in spec.geoms:
        g.density = DENSITY if g.name.endswith("_visual") else 0.0
    m = spec.compile()
    out = {}
    for i in range(1, m.nbody):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i)
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, m.body_iquat[i])
        R = R.reshape(3, 3)
        full = R @ np.diag(m.body_inertia[i]) @ R.T
        out[name] = (float(m.body_mass[i]), [float(x) for x in m.body_ipos[i]],
                     [full[0, 0], full[1, 1], full[2, 2], full[0, 1], full[0, 2], full[1, 2]])
    return out


def fill(ledger_path=sl.LEDGER):
    rows = sl.read(ledger_path)
    est = visual_inertials()
    targets = [r["param"].split(".")[1] for r in rows
               if r["status"] == "missing" and r["param"].startswith("link.") and r["param"].endswith(".mass")]
    added = []
    for link in targets:
        mass, com, inertia = est[link]
        rows = [r for r in rows if not (r["status"] == "missing" and r["param"] == f"link.{link}.mass")]
        new = [(f"link.{link}.mass", mass, "kg")]
        new += [(f"link.{link}.com.{k}", v, "m") for k, v in zip("xyz", com)]
        new += [(f"link.{link}.inertia.{k}", v, "kg*m^2") for k, v in zip(("ixx", "iyy", "izz", "ixy", "ixz", "iyz"), inertia)]
        for param, value, unit in new:
            if any(r["param"] == param and r["status"] == "active" for r in rows):
                continue
            rows.append(dict(param=param, value=f"{value:.8g}", unit=unit, source="estimate", status="active",
                             date="2026-10-22", evidence=EVIDENCE, note=NOTE))
        added.append((link, round(mass, 4)))
    sl.write(ledger_path, sl.LEDGER_COLUMNS, rows)
    return added


if __name__ == "__main__":
    for link, mass in fill():
        print(f"estimated {link}: {mass} kg")
