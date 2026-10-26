"""Seed model/ledger.csv and model/measurements.csv from the vendor URDF (spine 2: every twin number has a source).

Usage: python3 -I tools/seed_ledger.py [--fresh]

Merge-only: adds rows that don't exist yet and never changes or removes an existing row, so it's safe to re-run
after Luka measurements have been added. Gaps the twin needs filled (limits of the arm/head joints the vendor model
welds fixed, links with no mass) get explicit `missing` rows, so what's unknown shows up in the ledger instead of
being hidden.

Ledger rows: param, value, unit, source, status, date, evidence, note
  source  vendor | docs | estimate | measured-on-luka | verified-on-luka   (empty for a missing row;
          `estimate` = a stated-method guess, flagged in every build report, never treated as vendor data)
  status  active | deprecated | missing     (at most one active row per param; a Luka value deprecates the vendor one)
"""
import csv
import math
import os
import sys
import xml.etree.ElementTree as ET

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "tools"))
import inspect_urdf  # noqa: E402

URDF = os.path.join(REPO, "vendor", "agibot_a2", "urdf", "agibot_a2_dof12.urdf")
LEDGER = os.path.join(REPO, "model", "ledger.csv")
MEASUREMENTS = os.path.join(REPO, "model", "measurements.csv")
LEDGER_COLUMNS = ["param", "value", "unit", "source", "status", "date", "evidence", "note"]
MEASUREMENT_COLUMNS = ["id", "description", "unit", "vendor_value", "luka_left", "luka_right", "luka_value",
                       "date", "evidence"]
SOURCES = {"", "vendor", "docs", "estimate", "measured-on-luka", "verified-on-luka"}
STATUSES = {"active", "deprecated", "missing"}
VENDOR_EVIDENCE = "vendor/agibot_a2/urdf/agibot_a2_dof12.urdf @ Link-U-OS/rl_training 8bf8fd8"
SEED_DATE = "2026-10-22"
# Joints the vendor welds fixed but that move on the real A2 (AgiBot docs: 7 per arm, 2 head).
SHOULD_MOVE = [f"idx{n:02d}_" for n in range(13, 29)]


def vendor_rows(path=URDF):
    """Every number in the URDF as (param, value, unit), in file order."""
    root = ET.parse(path).getroot()
    out = []
    for link in root.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        if inertial is None:
            continue
        o = inertial.find("origin")
        out.append((f"link.{name}.mass", inertial.find("mass").get("value"), "kg"))
        if o is not None:
            for k, v in zip("xyz", o.get("xyz", "0 0 0").split()):
                out.append((f"link.{name}.com.{k}", v, "m"))
        i = inertial.find("inertia")
        for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"):
            out.append((f"link.{name}.inertia.{k}", i.get(k), "kg*m^2"))
    for j in root.findall("joint"):
        name = j.get("name")
        out.append((f"joint.{name}.type", j.get("type"), "-"))
        o = j.find("origin")
        if o is not None:
            for k, v in zip("xyz", o.get("xyz", "0 0 0").split()):
                out.append((f"joint.{name}.origin.{k}", v, "m"))
            for k, v in zip(("roll", "pitch", "yaw"), o.get("rpy", "0 0 0").split()):
                out.append((f"joint.{name}.origin.{k}", v, "rad"))
        ax = j.find("axis")
        if ax is not None:
            for k, v in zip("xyz", ax.get("xyz").split()):
                out.append((f"joint.{name}.axis.{k}", v, "-"))
        lim = j.find("limit")
        if lim is not None:
            for k, unit in (("lower", "rad"), ("upper", "rad"), ("effort", "N*m"), ("velocity", "rad/s")):
                if lim.get(k) is not None:
                    out.append((f"joint.{name}.limit.{k}", lim.get(k), unit))
    return out


def is_placeholder(param, value):
    """Vendor effort/velocity of 0 on a joint it welds fixed is a placeholder, not a real limit."""
    return (any(f"joint.{p}" in param for p in SHOULD_MOVE)
            and param.rsplit(".", 1)[-1] in ("effort", "velocity") and float(value) == 0.0)


def missing_rows(path=URDF):
    """Params the Luka twin needs that the vendor file doesn't provide (or provides only as a placeholder)."""
    model = inspect_urdf.load(path)
    out = []
    for j in model.joints:
        if any(j["name"].startswith(p) for p in SHOULD_MOVE):
            for k, unit in (("lower", "rad"), ("upper", "rad"), ("effort", "N*m"), ("velocity", "rad/s")):
                out.append((f"joint.{j['name']}.limit.{k}", unit,
                            "vendor welds this joint fixed; need AgiBot docs or a sweep on Luka"))
    for j in model.joints:
        if j["type"] == "revolute" or any(j["name"].startswith(p) for p in SHOULD_MOVE):
            for k, unit in (("damping", "N*m*s/rad"), ("frictionloss", "N*m"), ("armature", "kg*m^2")):
                out.append((f"joint.{j['name']}.dynamics.{k}", unit,
                            "not in either vendor file; measure on Luka (plan P5)"))
    root = ET.parse(path).getroot()
    for link in root.findall("link"):
        if link.find("inertial") is None and link.find("visual") is not None:
            out.append((f"link.{link.get('name')}.mass", "kg", "no inertial in the vendor file; real part has mass"))
    return out


def measurement_rows(path=URDF):
    """Plan P3 tape-measure table, vendor values computed from the URDF at zero pose (no hand-typed numbers)."""
    m = inspect_urdf.load(path)
    w = inspect_urdf.zero_pose(m)
    lo, hi, _ = inspect_urdf.mesh_extents(m, w)
    p = lambda n: w[n][1]
    d = lambda a, b: math.dist(p(a), p(b))
    r4 = lambda x: f"{x:.4f}"
    return [
        ("M01", "Top of head to sole, legs straight", "m", r4(hi[2] - lo[2])),
        ("M02", "Shoulder actuator centre, left to right", "m", r4(d("left_arm_link01", "right_arm_link01"))),
        ("M03", "Shoulder centre to elbow centre (upper arm), per side", "m", r4(d("right_arm_link01", "right_arm_link04"))),
        ("M04", "Elbow centre to wrist centre (forearm), per side", "m", r4(d("right_arm_link04", "right_arm_link06"))),
        ("M05", "Hip-pitch actuator centre, left to right", "m", r4(d("left_hip_pitch", "right_hip_pitch"))),
        ("M06", "Hip-pitch centre to knee centre (thigh), per side", "m", r4(d("right_hip_pitch", "right_tarsus"))),
        ("M07", "Knee centre to ankle centre (shin), per side", "m", r4(d("right_tarsus", "right_toe_pitch"))),
        ("M08", "Ankle centre to sole, per side", "m", r4(p("right_toe_pitch")[2] - lo[2])),
        ("M09", "Shoulder centre to sole", "m", r4(p("right_arm_link01")[2] - lo[2])),
        ("M10", "Widest point across, arms hanging", "m", r4(hi[1] - lo[1])),
        ("M11", "Weight (crane scale on the hoist)", "kg", f"{m.total_mass:.3f}"),
    ]


def ledger_problems(rows, urdf=URDF):
    """Spine-2 checks on the ledger: vocabulary, one active row per param, vendor rows still equal the vendor file."""
    problems = []
    vendor = {p: v for p, v, _ in vendor_rows(urdf)}
    active = {}
    for r in rows:
        p = r["param"]
        if r["source"] not in SOURCES or r["status"] not in STATUSES:
            problems.append(f"{p}: bad source/status {r['source']!r}/{r['status']!r}")
        if r["status"] == "missing" and (r["value"] or r["source"]):
            problems.append(f"{p}: a missing row must have no value and no source")
        if r["status"] != "missing" and (r["value"] == "" or not r["source"] or not r["evidence"]):
            problems.append(f"{p}: a {r['status']} row needs a value, a source and evidence")
        if r["status"] == "active":
            if p in active:
                problems.append(f"{p}: more than one active row")
            active[p] = r
        if r["source"] == "vendor" and vendor.get(p) != r["value"]:
            problems.append(f"{p}: vendor row {r['value']!r} != vendor file {vendor.get(p)!r}")
    for p in vendor:
        if not any(r["param"] == p and r["source"] == "vendor" for r in rows):
            problems.append(f"{p}: in the vendor file but not in the ledger")
    for r in rows:
        if r["status"] == "missing" and r["param"] in active:
            problems.append(f"{r['param']}: marked missing but also has an active row")
    return problems


def read(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def write(path, columns, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=columns)
        w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c, "") for c in columns})
    os.replace(tmp, path)


def seed(ledger_path=LEDGER, measurements_path=MEASUREMENTS, urdf=URDF, fresh=False):
    rows = read(ledger_path)
    if fresh:
        luka = [r for r in rows if r["source"] not in ("vendor", "")]
        if any(r["source"] in ("measured-on-luka", "verified-on-luka") for r in luka):
            raise SystemExit("refusing --fresh: the ledger holds Luka measurements")
        rows = []
    have = {r["param"] for r in rows}
    active = {r["param"] for r in rows if r["status"] == "active"}
    have_vendor = {r["param"] for r in rows if r["source"] == "vendor"}
    added = 0
    for param, value, unit in vendor_rows(urdf):
        if param in have_vendor:
            continue
        note = ""
        if param in active:
            status = "deprecated"  # a Luka value already holds this param; the vendor row arrives deprecated
        elif is_placeholder(param, value):
            status, note = "deprecated", "placeholder 0 on a joint the vendor welds fixed; not a real limit"
        else:
            status = "active"
            if any(f"joint.{p}" in param for p in SHOULD_MOVE) and ".limit." in param:
                note = "position limit on a joint the vendor welds fixed; unverified for Luka"
        rows.append(dict(param=param, value=value, unit=unit, source="vendor", status=status,
                         date=SEED_DATE, evidence=VENDOR_EVIDENCE, note=note))
        have.add(param)
        if status == "active":
            active.add(param)
        added += 1
    for param, unit, note in missing_rows(urdf):
        if param not in active and not any(r["param"] == param and r["status"] == "missing" for r in rows):
            rows.append(dict(param=param, value="", unit=unit, source="", status="missing", date=SEED_DATE,
                             evidence="", note=note))
            added += 1
    if "robot.mass_label" not in have:
        rows.append(dict(param="robot.mass_label", value="69", unit="kg", source="docs", status="active",
                         date=SEED_DATE, evidence="nameplate, photo IMG_5619 (2026-10-19)",
                         note="manufacturer's label; compare robot total of vendor link masses 69.344"))
        added += 1
    write(ledger_path, LEDGER_COLUMNS, rows)

    meas = read(measurements_path)
    by_id = {r["id"]: r for r in meas}
    madd = 0
    for mid, desc, unit, vendor_value in measurement_rows(urdf):
        if mid in by_id:
            if by_id[mid]["vendor_value"] != vendor_value:
                raise SystemExit(f"{mid}: vendor value changed {by_id[mid]['vendor_value']} -> {vendor_value} (vendor edited?)")
            continue
        meas.append(dict(id=mid, description=desc, unit=unit, vendor_value=vendor_value))
        madd += 1
    write(measurements_path, MEASUREMENT_COLUMNS, meas)
    return added, madd, len(rows), len(meas)


if __name__ == "__main__":
    # --fresh rebuilds from scratch, and refuses if any Luka measurement is in the ledger.
    added, madd, total, mtotal = seed(fresh="--fresh" in sys.argv[1:])
    print(f"ledger: +{added} rows ({total} total) · measurements: +{madd} rows ({mtotal} total)")
