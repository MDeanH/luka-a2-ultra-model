"""Build Luka's twin: vendor MJCF (topology, meshes, geoms, materials) + model/ledger.csv (every number).

Usage: .venv/bin/python -I model/build_twin.py [--out build/luka_as_is.xml]

Spine 2: each frame offset, axis, joint range, torque limit, mass, centre of mass and inertia in the output comes
from an ACTIVE ledger row. A required value with no active row stops the build: nothing is silently defaulted.
MuJoCo's own mass-from-geometry inference is switched off, and so is inertia balancing, so the compiled numbers are
the ledger's. What the ledger lacks (joint damping/friction/armature, arm/head torque limits) is listed in the
report as a gap, never filled. Geometry (mesh shapes) is the vendor's, sha-pinned in vendor/agibot_a2/SHA256SUMS.
"""
import argparse
import math
import os
import sys

import mujoco

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "tools"))
import inspect_urdf  # noqa: E402
import seed_ledger as sl  # noqa: E402

MJCF = os.path.join(REPO, "vendor", "agibot_a2", "mjcf", "agibot_a2_dof12.mjcf")
DEFAULT_OUT = os.path.join(REPO, "build", "luka_as_is.xml")
INERTIA_KEYS = ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
DYNAMICS = ("damping", "frictionloss", "armature")


class Ledger:
    """Active ledger values; records every param the build reads, so the report can say where each came from."""

    def __init__(self, path=sl.LEDGER):
        rows = sl.read(path)
        problems = sl.ledger_problems(rows)
        if problems:
            raise SystemExit("ledger has problems; fix them first:\n  " + "\n  ".join(problems[:10]))
        self.active = {r["param"]: r for r in rows if r["status"] == "active"}
        self.missing = {r["param"] for r in rows if r["status"] == "missing"}
        self.used = {}

    def has(self, param):
        return param in self.active

    def need(self, param):
        if param not in self.active:
            raise SystemExit(f"build stopped: no active ledger value for {param}")
        r = self.active[param]
        self.used[param] = r
        return float(r["value"])

    def maybe(self, param):
        return self.need(param) if param in self.active else None


def rpy_to_quat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                              math.cos(y / 2), math.sin(y / 2))
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy]


def build(ledger_path=sl.LEDGER, out=DEFAULT_OUT):
    out = os.path.abspath(out)
    led = Ledger(ledger_path)
    urdf = inspect_urdf.load(sl.URDF)  # topology only: which joint owns which child link
    joint_of = {j["child"]: j["name"] for j in urdf.joints}
    should_move = [j["name"] for j in urdf.joints if any(j["name"].startswith(p) for p in sl.SHOULD_MOVE)]
    with open(MJCF) as f:
        xml = f.read().replace('meshdir="."', f'meshdir="{os.path.dirname(MJCF)}"', 1)
    spec = mujoco.MjSpec.from_string(xml)
    spec.modelname = "luka_as_is"
    spec.compiler.balanceinertia = False
    spec.compiler.inertiafromgeom = mujoco.mjtInertiaFromGeom.mjINERTIAFROMGEOM_FALSE
    report = dict(gaps=[], unenforced=[], scene=[], added_joints=[], unactuated=[])

    for b in spec.bodies:
        name = b.name
        if name == "world":
            continue
        if name in joint_of:
            j = joint_of[name]
            b.pos = [led.need(f"joint.{j}.origin.{k}") for k in "xyz"]
            b.quat = rpy_to_quat(*(led.need(f"joint.{j}.origin.{k}") for k in ("roll", "pitch", "yaw")))
        else:
            report["scene"].append(f"{name} placed at {list(b.pos)} (vendor MJCF scene placement, fixed to the world)")
        if led.has(f"link.{name}.mass"):
            b.explicitinertial = True
            b.mass = led.need(f"link.{name}.mass")
            b.ipos = [led.need(f"link.{name}.com.{k}") for k in "xyz"]
            b.inertia, b.iquat = [0, 0, 0], [1, 0, 0, 0]  # clear the vendor diagonal so fullinertia applies
            b.fullinertia = [led.need(f"link.{name}.inertia.{k}") for k in INERTIA_KEYS]
        elif urdf.links.get(name, {}).get("meshes"):
            # A link with real geometry must have a mass: never silently make it massless (spine 2).
            led.need(f"link.{name}.mass")
        else:
            b.explicitinertial = True
            b.mass, b.ipos, b.inertia = 0.0, [0, 0, 0], [0, 0, 0]
            report["scene"].append(f"{name}: massless frame (no geometry, no inertial in either vendor file)")

    existing = {jn.name for jn in spec.joints}
    for jname in [j["name"] for j in urdf.joints_of_type("revolute")] + should_move:
        child = next(j["child"] for j in urdf.joints if j["name"] == jname)
        axis = [led.need(f"joint.{jname}.axis.{k}") for k in "xyz"]
        rng = [led.need(f"joint.{jname}.limit.lower"), led.need(f"joint.{jname}.limit.upper")]
        if jname in existing:
            jn = spec.joint(jname)
        else:
            jn = spec.body(child).add_joint(name=jname, type=mujoco.mjtJoint.mjJNT_HINGE)
            report["added_joints"].append(jname)
        jn.axis, jn.range, jn.limited = axis, rng, mujoco.mjtLimited.mjLIMITED_TRUE
        for k in DYNAMICS:
            v = led.maybe(f"joint.{jname}.dynamics.{k}")
            val = v if v is not None else 0.0
            # MuJoCo 3.17: damping is a 3-vector (linear term first); frictionloss/armature are scalars.
            setattr(jn, k, [val, 0.0, 0.0] if k == "damping" else val)
            if v is None:
                report["gaps"].append(f"joint.{jname}.dynamics.{k}")
        vel = led.maybe(f"joint.{jname}.limit.velocity")
        report["unenforced"].append(f"joint.{jname}.limit.velocity = {vel if vel is not None else 'MISSING'} "
                                    f"(MuJoCo has no joint velocity limit; a controller must enforce it)")

    actuated = set()
    for a in spec.actuators:
        e = led.need(f"joint.{a.target}.limit.effort")
        a.forcerange, a.forcelimited = [-e, e], mujoco.mjtLimited.mjLIMITED_TRUE
        actuated.add(a.target)
    for jname in should_move:
        if jname not in actuated:
            report["unactuated"].append(f"{jname}: no actuator — torque limit missing in the ledger "
                                        f"({'missing' if f'joint.{jname}.limit.effort' in led.missing else 'n/a'})")

    os.makedirs(os.path.dirname(out), exist_ok=True)
    model = spec.compile()  # raises on any inconsistency
    text = spec.to_xml()  # to_xml recompiles, so it still needs the absolute meshdir
    absdir, reldir = os.path.dirname(MJCF), os.path.relpath(os.path.dirname(MJCF), os.path.dirname(out))
    if text.count(f'meshdir="{absdir}"') != 1:
        raise SystemExit("build stopped: could not find the meshdir to make relative")
    text = text.replace(f'meshdir="{absdir}"', f'meshdir="{reldir}"')  # the file resolves from its own folder
    with open(out, "w") as f:
        f.write(text)
    # The XML is rounded by MuJoCo's writer (~6 significant figures); the binary model keeps every number exactly.
    mujoco.mj_saveModel(model, os.path.splitext(out)[0] + ".mjb", None)
    write_report(out, led, report, model)
    return model, led, report


def write_report(out, led, report, model):
    by_source = {}
    for r in led.used.values():
        by_source[r["source"]] = by_source.get(r["source"], 0) + 1
    estimates = sorted(p for p, r in led.used.items() if r["source"] == "estimate")
    total = float(model.body_mass.sum())
    lines = [f"# Twin build report — {os.path.basename(out)}", "",
             "Generated by `model/build_twin.py` from the vendor MJCF (structure, meshes) + `model/ledger.csv` (numbers).",
             "**Status: builds and simulates. NOT validated against Luka** (no Luka telemetry replayed yet; spine 5).", "",
             "Files: `.mjb` = exact (every number equals the ledger); `.xml` = readable, MuJoCo's writer rounds to ~6 "
             "significant figures (max relative error ~5e-6).", "",
             f"- Bodies {model.nbody}, joints {model.njnt} (DOF {model.nv}), actuators {model.nu}, total mass {total:.3f} kg "
             f"(nameplate says 69 kg — the twin carries the URDF masses + estimated hands/wrists; unresolved until Luka is weighed)",
             f"- Ledger values used: {len(led.used)} — by source: " + ", ".join(f"{k} {v}" for k, v in sorted(by_source.items())),
             f"- Joints added beyond the vendor's 12 (arms + head, vendor position limits, unverified for Luka): "
             f"{len(report['added_joints'])}", "",
             f"## Estimates used ({len(estimates)}) — flagged, replace with measurements", ""]
    lines += [f"- `{p}` = {led.used[p]['value']} {led.used[p]['unit']}" for p in estimates if p.endswith(".mass")]
    lines += ["", f"(plus their centres of mass and inertias: {len(estimates)} estimate values in total)", "",
              f"## Gaps — set to 0 in the model because nothing is known ({len(report['gaps'])})", "",
              "Joint damping / frictionloss / armature for every moving joint. Measure on Luka (plan P5).", ""]
    lines += [f"- {u}" for u in report["unactuated"]]
    lines += ["", "## Not enforced by MuJoCo", ""] + [f"- {u}" for u in report["unenforced"][:3]] + \
             [f"- … {len(report['unenforced']) - 3} more joints, same note"]
    lines += ["", "## Scene", ""] + [f"- {s}" for s in report["scene"]]
    with open(os.path.splitext(out)[0] + ".report.md", "w") as f:
        f.write("\n".join(lines) + "\n")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=DEFAULT_OUT)
    a = ap.parse_args(argv)
    model, led, report = build(out=os.path.abspath(a.out))
    print(f"built {a.out}: DOF {model.nv}, actuators {model.nu}, total mass {float(model.body_mass.sum()):.3f} kg, "
          f"{len(led.used)} ledger values used, {len(report['gaps'])} gaps, {len(report['unactuated'])} unactuated joints")
    return 0


if __name__ == "__main__":
    sys.exit(main())
