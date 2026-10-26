"""Read-only inspection of a URDF: joints, masses, zero-pose geometry, mesh extents.

Usage: python3 -I tools/inspect_urdf.py <file.urdf>
Importable: load(path) -> Model; zero_pose(model); mesh_extents(model, world).
Stdlib only. Never writes anything.
"""
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

IDENTITY = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


@dataclass
class Model:
    path: str
    name: str
    links: dict = field(default_factory=dict)
    joints: list = field(default_factory=list)
    children: dict = field(default_factory=dict)

    @property
    def roots(self):
        child_links = {j["child"] for j in self.joints}
        return [n for n in self.links if n not in child_links]

    @property
    def total_mass(self):
        return sum(l["mass"] for l in self.links.values())

    def joints_of_type(self, kind):
        return [j for j in self.joints if j["type"] == kind]


def _vec(s, n=3):
    return [float(x) for x in (s or " ".join(["0"] * n)).split()]


def _rpy_to_m(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr]]


def _mm(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mv(a, v):
    return [sum(a[i][k]*v[k] for k in range(3)) for i in range(3)]


def load(path):
    root = ET.parse(path).getroot()
    model = Model(path=path, name=root.get("name"))
    for l in root.findall("link"):
        inert = l.find("inertial")
        mass_el = inert.find("mass") if inert is not None else None
        model.links[l.get("name")] = dict(
            mass=float(mass_el.get("value")) if mass_el is not None else 0.0,
            meshes=[m.get("filename") for m in l.findall("visual/geometry/mesh")],
            cmeshes=[m.get("filename") for m in l.findall("collision/geometry/mesh")],
            cprims=[c.tag for c in l.findall("collision/geometry/*") if c.tag != "mesh"],
        )
    for j in root.findall("joint"):
        o, ax, lim = j.find("origin"), j.find("axis"), j.find("limit")
        d = dict(name=j.get("name"), type=j.get("type"),
                 parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                 xyz=_vec(o.get("xyz")) if o is not None else [0, 0, 0],
                 rpy=_vec(o.get("rpy")) if o is not None else [0, 0, 0],
                 axis=_vec(ax.get("xyz")) if ax is not None else None,
                 lower=lim.get("lower") if lim is not None else None,
                 upper=lim.get("upper") if lim is not None else None,
                 effort=lim.get("effort") if lim is not None else None,
                 velocity=lim.get("velocity") if lim is not None else None)
        model.joints.append(d)
        model.children.setdefault(d["parent"], []).append(d)
    return model


def zero_pose(model):
    """World (root-frame) rotation + translation of every link with all joints at zero."""
    world = {}

    def walk(name, R, t):
        world[name] = (R, t)
        for j in model.children.get(name, []):
            Rj = _mm(R, _rpy_to_m(*j["rpy"]))
            off = _mv(R, j["xyz"])
            walk(j["child"], Rj, [t[i] + off[i] for i in range(3)])

    for r in model.roots:
        walk(r, IDENTITY, [0, 0, 0])
    return world


def stl_points(path):
    with open(path, "rb") as f:
        data = f.read()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        pts = []
        for line in data.decode(errors="ignore").splitlines():
            line = line.strip()
            if line.startswith("vertex"):
                pts.append([float(x) for x in line.split()[1:4]])
        return pts
    n = struct.unpack("<I", data[80:84])[0]
    pts = []
    for i in range(n):
        off = 84 + i*50 + 12
        for k in range(3):
            pts.append(struct.unpack("<3f", data[off + k*12: off + k*12 + 12]))
    return pts


def resolve_mesh(model, filename):
    base = os.path.dirname(model.path)
    fn = filename.replace("package://", "")
    for cand in (os.path.join(base, fn), os.path.join(base, "..", fn)):
        if os.path.exists(cand):
            return os.path.normpath(cand)
    bn = os.path.basename(fn)
    for dp, _, fs in os.walk(os.path.join(base, "..")):
        if bn in fs:
            return os.path.join(dp, bn)
    return None


def mesh_extents(model, world):
    """Axis-aligned extents of all visual meshes at zero pose -> (lo, hi, missing filenames)."""
    lo, hi, missing = [1e9]*3, [-1e9]*3, []
    for n, l in model.links.items():
        if n not in world:
            continue
        R, t = world[n]
        for fn in l["meshes"]:
            p = resolve_mesh(model, fn)
            if not p:
                missing.append(fn)
                continue
            for v in stl_points(p):
                w = _mv(R, v)
                for i in range(3):
                    lo[i] = min(lo[i], t[i] + w[i])
                    hi[i] = max(hi[i], t[i] + w[i])
    return lo, hi, missing


def report(model):
    world = zero_pose(model)
    out = [f"robot name: {model.name}",
           f"links: {len(model.links)}  joints: {len(model.joints)}  roots: {model.roots}"]
    types = {}
    for j in model.joints:
        types[j["type"]] = types.get(j["type"], 0) + 1
    out += [f"joint types: {types}", f"total mass (URDF): {model.total_mass:.3f} kg", "", "MOVABLE JOINTS"]
    for j in model.joints:
        if j["type"] != "fixed":
            out.append(f"  {j['name']:<28} {j['type']:<9} axis={j['axis']} lim=[{j['lower']},{j['upper']}] "
                       f"eff={j['effort']} vel={j['velocity']}  {j['parent']} -> {j['child']}")
    out += ["", "FIXED JOINTS"]
    for j in model.joints_of_type("fixed"):
        lim = f"  lim=[{j['lower']},{j['upper']}] eff={j['effort']} vel={j['velocity']}" if j["lower"] is not None else ""
        out.append(f"  {j['name']:<28} axis={j['axis']}  {j['parent']} -> {j['child']}  "
                   f"xyz={[round(x, 4) for x in j['xyz']]}{lim}")
    out += ["", "LINKS (mass kg, zero-pose origin in root frame m, visual mesh)"]
    for n, l in model.links.items():
        t = world.get(n, (None, [float("nan")]*3))[1]
        out.append(f"  {n:<28} {l['mass']:7.3f}  [{t[0]:+.3f} {t[1]:+.3f} {t[2]:+.3f}]  "
                   f"vis={[os.path.basename(m) for m in l['meshes']]} "
                   f"col={[os.path.basename(m) for m in l['cmeshes']] or l['cprims']}")
    lo, hi, missing = mesh_extents(model, world)
    out += ["", f"zero-pose visual extents (m): x[{lo[0]:+.3f},{hi[0]:+.3f}] y[{lo[1]:+.3f},{hi[1]:+.3f}] "
                f"z[{lo[2]:+.3f},{hi[2]:+.3f}]  height={hi[2]-lo[2]:.3f}",
            f"missing meshes: {missing}"]
    return "\n".join(out)


if __name__ == "__main__":
    print(report(load(sys.argv[1])))
