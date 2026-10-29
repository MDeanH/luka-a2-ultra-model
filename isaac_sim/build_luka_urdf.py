"""Derive an Isaac Sim import candidate from Luka's checked-in vendor URDF.

This is a *model candidate*, not a calibrated description of Luka. The public
vendor robot is T2D0, while Luka reported T3D1. Arm/head position ranges and
several link inertials are unverified estimates. In particular, the 16 newly
movable arm/head joints have no known actuator effort or speed limits: their
URDF effort=0 and velocity=0 are explicit *unknown/unactuated placeholders*,
not measured physical limits. Do not use this URDF for hardware control.

Only a generated output is written. The vendor URDF, meshes, ledger, and
MuJoCo baseline are read-only inputs. No Isaac Sim package is needed to build.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path
import re
import sys
import tempfile
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[1]
VENDOR_URDF = REPO / "vendor/agibot_a2/urdf/agibot_a2_dof12.urdf"
LEDGER = REPO / "model/ledger.csv"
DEFAULT_OUTPUT = REPO / "isaac_sim/generated/luka_candidate.urdf"
GENERATED_DIR = REPO / "isaac_sim/generated"
INERTIA_KEYS = ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
JOINT_RE = re.compile(r"^idx(\d{2})_(?!\d).+$")


class BuildError(ValueError):
    """Input is incomplete or inconsistent; no candidate should be written."""


def finite(value: str, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise BuildError(f"{label}: expected a number, got {value!r}") from exc
    if not math.isfinite(number):
        raise BuildError(f"{label}: number must be finite, got {value!r}")
    return number


def vector(value: str | None, label: str) -> tuple[float, float, float]:
    parts = (value or "").split()
    if len(parts) != 3:
        raise BuildError(f"{label}: expected three values, got {value!r}")
    return tuple(finite(part, label) for part in parts)  # type: ignore[return-value]


def require_element(parent: ET.Element, tag: str, label: str) -> ET.Element:
    element = parent.find(tag)
    if element is None:
        raise BuildError(f"{label}: missing <{tag}>")
    return element


def active_ledger(path: Path) -> dict[str, str]:
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            if not {"param", "value", "unit", "status"}.issubset(reader.fieldnames or []):
                raise BuildError(f"{path}: missing ledger columns")
            active: dict[str, str] = {}
            for row in reader:
                if row["status"] != "active":
                    continue
                key = row["param"]
                value = row["value"]
                if not key or not value:
                    raise BuildError(f"{path}: blank active parameter or value")
                if key in active:
                    raise BuildError(f"{path}: duplicate active parameter {key}")
                active[key] = value
            return active
    except OSError as exc:
        raise BuildError(f"cannot read ledger {path}: {exc}") from exc


def ledger_number(active: dict[str, str], key: str) -> float:
    if key not in active:
        raise BuildError(f"missing active ledger value: {key}")
    return finite(active[key], key)


def same_number(source: float, ledger: float, label: str) -> None:
    if not math.isclose(source, ledger, rel_tol=1e-12, abs_tol=1e-12):
        raise BuildError(f"{label}: vendor {source} differs from active ledger {ledger}")


def fmt(number: float) -> str:
    return format(number, ".17g")


def validate_topology(root: ET.Element) -> tuple[list[ET.Element], list[ET.Element]]:
    if root.tag != "robot" or not root.get("name"):
        raise BuildError("vendor file must have a named <robot> root")
    links = root.findall("link")
    joints = root.findall("joint")
    link_names = [link.get("name") for link in links]
    joint_names = [joint.get("name") for joint in joints]
    if any(not name for name in link_names + joint_names):
        raise BuildError("every link and joint needs a name")
    if len(set(link_names)) != len(links) or len(set(joint_names)) != len(joints):
        raise BuildError("duplicate link or joint name")
    parents: dict[str, str] = {}
    for joint in joints:
        name = joint.get("name") or "<unnamed>"
        parent = require_element(joint, "parent", name).get("link")
        child = require_element(joint, "child", name).get("link")
        if parent not in link_names or child not in link_names or parent == child:
            raise BuildError(f"{name}: invalid parent/child link")
        if child in parents:
            raise BuildError(f"{name}: child {child} already has parent {parents[child]}")
        parents[child] = parent  # type: ignore[assignment]
    roots = set(link_names) - set(parents)
    if len(roots) != 1 or len(joints) != len(links) - 1:
        raise BuildError("vendor topology must be one connected link tree")
    for link in link_names:
        seen: set[str] = set()
        cursor = link
        while cursor in parents:
            if cursor in seen:
                raise BuildError("vendor topology contains a cycle")
            seen.add(cursor)
            cursor = parents[cursor]
    return links, joints


def validate_inertia(mass: float, values: dict[str, float], label: str) -> None:
    if mass <= 0:
        raise BuildError(f"{label}: mass must be positive")
    a, b, c = values["ixx"], values["iyy"], values["izz"]
    d, e, f = values["ixy"], values["ixz"], values["iyz"]
    # Sylvester's criterion for a symmetric 3x3 inertia tensor.
    second_minor = a * b - d * d
    determinant = a * b * c + 2 * d * e * f - a * f * f - b * e * e - c * d * d
    if a <= 0 or second_minor <= 0 or determinant <= 0:
        raise BuildError(f"{label}: inertia tensor is not positive definite")


def fill_inertials(links: list[ET.Element], active: dict[str, str]) -> None:
    for link in links:
        name = link.get("name") or "<unnamed>"
        key = f"link.{name}.mass"
        old = link.find("inertial")
        if key not in active:
            # The two arm-link bases are massless coordinate frames with no
            # visual/collision geometry in the checked-in vendor model.
            if old is not None or link.find("visual") is not None or link.find("collision") is not None:
                raise BuildError(f"missing active ledger value: {key}")
            continue
        mass = ledger_number(active, key)
        com = [ledger_number(active, f"link.{name}.com.{axis}") for axis in "xyz"]
        inertia = {part: ledger_number(active, f"link.{name}.inertia.{part}") for part in INERTIA_KEYS}
        validate_inertia(mass, inertia, name)
        if old is not None:
            # Source inertias are expressed in link axes, as is the ledger.
            origin = require_element(old, "origin", name)
            if any(abs(x) > 1e-12 for x in vector(origin.get("rpy"), f"{name}.inertial.rpy")):
                raise BuildError(f"{name}: nonzero source inertial rotation is unsupported")
            same_number(finite(require_element(old, "mass", name).get("value"), f"{name}.source.mass"), mass, key)
            source_com = vector(origin.get("xyz"), f"{name}.source.com")
            for axis, source, target in zip("xyz", source_com, com):
                same_number(source, target, f"link.{name}.com.{axis}")
            source_inertia = require_element(old, "inertia", name)
            for part, target in inertia.items():
                same_number(finite(source_inertia.get(part), f"{name}.source.{part}"), target,
                            f"link.{name}.inertia.{part}")
            link.remove(old)
        derived = ET.Element("inertial")
        ET.SubElement(derived, "origin", {"xyz": " ".join(map(fmt, com)), "rpy": "0 0 0"})
        ET.SubElement(derived, "mass", {"value": fmt(mass)})
        ET.SubElement(derived, "inertia", {part: fmt(inertia[part]) for part in INERTIA_KEYS})
        link.insert(0, derived)


def check_joint_geometry(joint: ET.Element, active: dict[str, str]) -> None:
    name = joint.get("name") or "<unnamed>"
    origin = require_element(joint, "origin", name)
    for attr, axes in (("xyz", "xyz"), ("rpy", ("roll", "pitch", "yaw"))):
        source = vector(origin.get(attr), f"{name}.origin.{attr}")
        for axis, value in zip(axes, source):
            same_number(value, ledger_number(active, f"joint.{name}.origin.{axis}"),
                        f"joint.{name}.origin.{axis}")
    axis_values = vector(require_element(joint, "axis", name).get("xyz"), f"{name}.axis")
    if math.isclose(sum(value * value for value in axis_values), 0.0, abs_tol=1e-24):
        raise BuildError(f"{name}: zero joint axis")
    for axis, value in zip("xyz", axis_values):
        same_number(value, ledger_number(active, f"joint.{name}.axis.{axis}"),
                    f"joint.{name}.axis.{axis}")


def convert_joints(joints: list[ET.Element], active: dict[str, str]) -> None:
    indexed: dict[int, ET.Element] = {}
    for joint in joints:
        match = JOINT_RE.match(joint.get("name") or "")
        if match:
            index = int(match.group(1))
            if index in indexed:
                raise BuildError(f"duplicate idx{index:02d} joint")
            indexed[index] = joint
    if set(indexed) != set(range(1, 29)):
        raise BuildError("vendor URDF must contain exactly idx01 through idx28 joints")

    for index in range(1, 29):
        joint = indexed[index]
        name = joint.get("name") or "<unnamed>"
        expected_type = "revolute" if index <= 12 else "fixed"
        if joint.get("type") != expected_type or active.get(f"joint.{name}.type") != expected_type:
            raise BuildError(f"{name}: unexpected source or ledger joint type")
        check_joint_geometry(joint, active)
        limit = require_element(joint, "limit", name)
        lower = ledger_number(active, f"joint.{name}.limit.lower")
        upper = ledger_number(active, f"joint.{name}.limit.upper")
        if not lower < upper:
            raise BuildError(f"{name}: lower limit must be below upper limit")
        for part, value in (("lower", lower), ("upper", upper)):
            same_number(finite(limit.get(part), f"{name}.source.{part}"), value,
                        f"joint.{name}.limit.{part}")
        if index <= 12:
            effort = ledger_number(active, f"joint.{name}.limit.effort")
            velocity = ledger_number(active, f"joint.{name}.limit.velocity")
            if effort <= 0 or velocity <= 0:
                raise BuildError(f"{name}: original movable joint limits must be positive")
            for part, value in (("effort", effort), ("velocity", velocity)):
                same_number(finite(limit.get(part), f"{name}.source.{part}"), value,
                            f"joint.{name}.limit.{part}")
        else:
            # Active physical effort/speed values do not exist for these 16
            # joints. Zero is an explicit unknown/unactuated URDF placeholder.
            # It is NOT a measured speed or force limit for Luka.
            effort = velocity = 0.0
            joint.set("type", "revolute")
        for part, value in (("lower", lower), ("upper", upper),
                            ("effort", effort), ("velocity", velocity)):
            limit.set(part, fmt(value))


def rewrite_meshes(root: ET.Element, source: Path, output: Path) -> int:
    count = 0
    for mesh in root.iter("mesh"):
        reference = mesh.get("filename")
        if not reference or "://" in reference:
            raise BuildError(f"unsupported or missing local mesh reference: {reference!r}")
        path = Path(reference)
        resolved = (path if path.is_absolute() else source.parent / path).resolve()
        if not resolved.is_file() or resolved.stat().st_size == 0:
            raise BuildError(f"missing or empty mesh: {reference} (looked for {resolved})")
        mesh.set("filename", Path(os.path.relpath(resolved, output.parent)).as_posix())
        count += 1
    if count == 0:
        raise BuildError("vendor URDF contains no mesh references")
    return count


def validated_output(output: Path) -> Path:
    """Keep every write inside the dedicated derived-artifact directory.

    Resolving the target also rejects ``..`` traversal and symlinks that lead
    into the vendor tree, ledger, source manifest, or other evidence files.
    """
    resolved = output.resolve()
    allowed = REPO.resolve() / "isaac_sim" / "generated"
    if resolved.suffix.lower() != ".urdf" or allowed not in resolved.parents:
        raise BuildError(f"output must be a .urdf below {allowed}")
    return resolved


def generate(vendor_urdf: Path = VENDOR_URDF, ledger: Path = LEDGER,
             output: Path = DEFAULT_OUTPUT) -> tuple[bytes, int]:
    output = validated_output(output)
    vendor_urdf, ledger = vendor_urdf.resolve(), ledger.resolve()
    if output in (vendor_urdf, ledger):
        raise BuildError("output must not overwrite an input file")
    try:
        root = ET.parse(vendor_urdf).getroot()
    except (ET.ParseError, OSError) as exc:
        raise BuildError(f"cannot parse vendor URDF {vendor_urdf}: {exc}") from exc
    links, joints = validate_topology(root)
    active = active_ledger(ledger)
    fill_inertials(links, active)
    convert_joints(joints, active)
    meshes = rewrite_meshes(root, vendor_urdf, output)
    root.set("name", "luka_candidate")
    root.insert(0, ET.Comment(
        "Derived public T2D0 candidate; not calibrated to Luka T3D1. "
        "idx13..idx28 effort=0 velocity=0 mean unknown/unactuated, not physical limits."
    ))
    ET.indent(root, space="  ")
    content = b'<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(root, encoding="utf-8") + b"\n"
    return content, meshes


def build(vendor_urdf: Path = VENDOR_URDF, ledger: Path = LEDGER,
          output: Path = DEFAULT_OUTPUT) -> int:
    output = validated_output(output)
    content, meshes = generate(vendor_urdf, ledger, output)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(mode="wb", dir=output.parent, prefix=".luka_urdf_",
                                         suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(content)
        os.replace(temporary, output)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return meshes


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vendor-urdf", type=Path, default=VENDOR_URDF)
    parser.add_argument("--ledger", type=Path, default=LEDGER)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        mesh_count = build(args.vendor_urdf, args.ledger, args.out)
    except BuildError as exc:
        print(f"Build stopped: {exc}", file=sys.stderr)
        return 1
    print(f"Wrote {args.out.resolve()} (28 movable joints, {mesh_count} mesh references)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
