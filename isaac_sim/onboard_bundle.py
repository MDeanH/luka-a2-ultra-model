"""Verify Luka's private onboard URDF bundle before importing it into Isaac Sim.

The bundle is an external input. This module reads it without modifying or
copying its URDF or meshes into the public model repository. Its MANIFEST.json
pins the derived files and records the hashes of the original private inputs.
File identity and URDF structure do not prove a physical match to Luka.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PureWindowsPath
import sys
import tempfile
import xml.etree.ElementTree as ET


URDF_NAME = "onboard_model_isaac_candidate.urdf"
MANIFEST_NAME = "MANIFEST.json"
PUBLIC_REPO = Path(__file__).resolve().parents[1]
EXPECTED_REVOLUTE_JOINTS = 52
EXPECTED_FIXED_JOINTS = 23
EXPECTED_MIMIC_JOINTS = 12
EXPECTED_LINKS = 76
EXPECTED_MESH_FILES = 30
EXPECTED_MESH_REFERENCES = 59
EXPECTED_COLLIDERS = 26
EXPECTED_BOX_COLLIDERS = 24
EXPECTED_CYLINDER_COLLIDERS = 2


class OnboardBundleError(ValueError):
    """The onboard export is missing, inconsistent, or unsafe to import."""


def _sha256_file(path: Path) -> tuple[int, str]:
    if not path.is_file():
        raise OnboardBundleError(f"missing input file: {path}")
    size = 0
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            size += len(chunk)
            digest.update(chunk)
    if size == 0:
        raise OnboardBundleError(f"empty input file: {path}")
    return size, digest.hexdigest()


def _sha256(value: object, label: str) -> str:
    if (not isinstance(value, str) or len(value) != 64
            or any(char not in "0123456789abcdef" for char in value)):
        raise OnboardBundleError(f"{label} must be a lowercase SHA-256 hex digest")
    return value


def _bundle_file(root: Path, reference: object, label: str) -> tuple[Path, str]:
    if not isinstance(reference, str) or not reference or "\\" in reference or "://" in reference:
        raise OnboardBundleError(f"{label} must be a relative bundle path: {reference!r}")
    if (Path(reference).is_absolute() or PureWindowsPath(reference).is_absolute()
            or PureWindowsPath(reference).drive):
        raise OnboardBundleError(f"{label} must be relative: {reference!r}")
    resolved = (root / reference).resolve()
    try:
        relative = resolved.relative_to(root).as_posix()
    except ValueError as exc:
        raise OnboardBundleError(f"{label} escapes the bundle: {reference!r}") from exc
    if relative != reference:
        raise OnboardBundleError(f"{label} must use a canonical bundle path: {reference!r}")
    if not resolved.is_file():
        raise OnboardBundleError(f"{label} is missing: {reference!r}")
    return resolved, relative


def _manifest(bundle: Path) -> tuple[dict, dict[str, object]]:
    manifest_path = bundle / MANIFEST_NAME
    if manifest_path.is_symlink():
        raise OnboardBundleError("MANIFEST.json must be a regular file, not a symlink")
    size, digest = _sha256_file(manifest_path)
    try:
        data = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as exc:
        raise OnboardBundleError(f"invalid {MANIFEST_NAME}: {exc}") from exc
    if not isinstance(data, dict):
        raise OnboardBundleError(f"{MANIFEST_NAME} must contain a JSON object")
    for key in ("input_app_model_index_sha256", "input_onboard_urdf_sha256", "output_urdf_sha256"):
        _sha256(data.get(key), key)
    if not isinstance(data.get("status"), str) or not data["status"]:
        raise OnboardBundleError("MANIFEST.json status is missing")
    if data.get("mesh_references") != EXPECTED_MESH_REFERENCES:
        raise OnboardBundleError(
            f"MANIFEST.json must declare {EXPECTED_MESH_REFERENCES} mesh references"
        )
    return data, {"path": MANIFEST_NAME, "bytes": size, "sha256": digest}


def _urdf_structure(urdf: Path, bundle: Path, mesh_files: set[str]) -> dict[str, object]:
    try:
        robot = ET.parse(urdf).getroot()
    except (ET.ParseError, OSError) as exc:
        raise OnboardBundleError(f"invalid URDF: {exc}") from exc
    if robot.tag != "robot":
        raise OnboardBundleError("onboard URDF must have a <robot> root")
    links = [link.get("name") for link in robot.findall("link")]
    if any(not name for name in links) or len(links) != len(set(links)):
        raise OnboardBundleError("URDF links need unique nonempty names")
    if len(links) != EXPECTED_LINKS:
        raise OnboardBundleError(f"expected {EXPECTED_LINKS} links, found {len(links)}")
    joints = robot.findall("joint")
    joint_names = [joint.get("name") for joint in joints]
    if any(not name for name in joint_names) or len(joint_names) != len(set(joint_names)):
        raise OnboardBundleError("URDF joints need unique nonempty names")
    revolute = [joint for joint in joints if joint.get("type") == "revolute"]
    fixed = [joint for joint in joints if joint.get("type") == "fixed"]
    if len(revolute) != EXPECTED_REVOLUTE_JOINTS:
        raise OnboardBundleError(
            f"expected {EXPECTED_REVOLUTE_JOINTS} revolute joints, found {len(revolute)}"
        )
    if len(fixed) != EXPECTED_FIXED_JOINTS or len(joints) != len(revolute) + len(fixed):
        raise OnboardBundleError(
            f"expected {EXPECTED_FIXED_JOINTS} fixed joints and no other joint types; "
            f"found {len(fixed)} fixed of {len(joints)} total"
        )
    mimics = [joint for joint in joints if joint.find("mimic") is not None]
    if len(mimics) != EXPECTED_MIMIC_JOINTS:
        raise OnboardBundleError(
            f"expected {EXPECTED_MIMIC_JOINTS} mimic joints, found {len(mimics)}"
        )
    for joint in mimics:
        target = joint.find("mimic").get("joint")  # type: ignore[union-attr]
        if target not in joint_names or target == joint.get("name"):
            raise OnboardBundleError(f"invalid mimic target {target!r} on {joint.get('name')}")
    for joint in joints:
        parent = joint.find("parent")
        child = joint.find("child")
        if (parent is None or child is None or parent.get("link") not in links
                or child.get("link") not in links
                or parent.get("link") == child.get("link")):
            raise OnboardBundleError(f"invalid link relationship on joint {joint.get('name')}")

    meshes = robot.findall(".//mesh")
    if len(meshes) != EXPECTED_MESH_REFERENCES:
        raise OnboardBundleError(
            f"expected {EXPECTED_MESH_REFERENCES} URDF mesh references, found {len(meshes)}"
        )
    referenced: set[str] = set()
    for mesh in meshes:
        _, relative = _bundle_file(bundle, mesh.get("filename"), "URDF mesh")
        if relative not in mesh_files:
            raise OnboardBundleError(f"URDF mesh is not pinned by MANIFEST.json: {relative}")
        referenced.add(relative)
    if referenced != mesh_files:
        raise OnboardBundleError("MANIFEST.json lists meshes the URDF does not reference")

    colliders = robot.findall(".//collision")
    boxes = cylinders = 0
    if len(colliders) != EXPECTED_COLLIDERS:
        raise OnboardBundleError(
            f"expected {EXPECTED_COLLIDERS} primitive colliders, found {len(colliders)}"
        )
    for collision in colliders:
        geometry = collision.find("geometry")
        if geometry is None or len(geometry) != 1:
            raise OnboardBundleError("each collision needs one primitive geometry")
        if geometry[0].tag == "box":
            boxes += 1
        elif geometry[0].tag == "cylinder":
            cylinders += 1
        else:
            raise OnboardBundleError(f"unsupported collision geometry: {geometry[0].tag}")
    if boxes != EXPECTED_BOX_COLLIDERS or cylinders != EXPECTED_CYLINDER_COLLIDERS:
        raise OnboardBundleError(
            f"expected {EXPECTED_BOX_COLLIDERS} boxes and {EXPECTED_CYLINDER_COLLIDERS} "
            f"cylinders, found {boxes} boxes and {cylinders} cylinders"
        )
    return {
        "robot_name": robot.get("name"),
        "expected_joint_count": len(revolute),
        "fixed_joint_count": len(fixed),
        "joint_names": [joint.get("name") for joint in revolute],
        "mimic_joint_names": [joint.get("name") for joint in mimics],
        "link_count": len(links),
        "mesh_reference_count": len(meshes),
        "collider_count": len(colliders),
        "box_collider_count": boxes,
        "cylinder_collider_count": cylinders,
    }


def verify_onboard_bundle(bundle: Path) -> dict[str, object]:
    """Return JSON-ready identity and structure after checking the private bundle."""
    bundle = bundle.resolve()
    if not bundle.is_dir():
        raise OnboardBundleError(f"private onboard bundle directory is missing: {bundle}")
    manifest, manifest_identity = _manifest(bundle)
    urdf, urdf_relative = _bundle_file(bundle, manifest.get("output_urdf"), "output_urdf")
    if urdf_relative != URDF_NAME:
        raise OnboardBundleError(f"expected {URDF_NAME}, found {urdf_relative}")
    urdf_bytes, urdf_sha = _sha256_file(urdf)
    if urdf_sha != manifest["output_urdf_sha256"]:
        raise OnboardBundleError("onboard URDF hash differs from MANIFEST.json")

    rows = manifest.get("mesh_files")
    if not isinstance(rows, list) or len(rows) != EXPECTED_MESH_FILES:
        raise OnboardBundleError(f"MANIFEST.json must list {EXPECTED_MESH_FILES} mesh files")
    mesh_identities: list[dict[str, object]] = []
    seen: set[str] = set()
    for row in rows:
        if not isinstance(row, dict):
            raise OnboardBundleError("mesh_files rows must be objects")
        path, relative = _bundle_file(bundle, row.get("derived_path"), "derived mesh")
        if relative in seen:
            raise OnboardBundleError(f"duplicate derived mesh path: {relative}")
        seen.add(relative)
        expected_bytes = row.get("source_bytes")
        if (not isinstance(expected_bytes, int) or isinstance(expected_bytes, bool)
                or expected_bytes <= 0):
            raise OnboardBundleError(f"invalid source_bytes for {relative}")
        source_name = row.get("source_name")
        if (not isinstance(source_name, str) or not source_name
                or relative != f"meshes/{source_name}"):
            raise OnboardBundleError(f"invalid source_name for {relative}")
        expected_sha = _sha256(row.get("source_sha256"), f"source_sha256 for {relative}")
        size, digest = _sha256_file(path)
        if size != expected_bytes or digest != expected_sha:
            raise OnboardBundleError(f"derived mesh hash or size differs from MANIFEST.json: {relative}")
        mesh_identities.append({
            "path": relative, "bytes": size, "sha256": digest,
            "source_name": source_name,
        })
    structure = _urdf_structure(urdf, bundle, seen)
    return {
        "schema_version": 1,
        "source_type": "onboard_urdf",
        "bundle_path": str(bundle),
        "urdf_path": str(urdf),
        "manifest": manifest_identity,
        "preparer_status": manifest["status"],
        "input_app_model_index_sha256": manifest["input_app_model_index_sha256"],
        "input_onboard_urdf_sha256": manifest["input_onboard_urdf_sha256"],
        "source_urdf": {"path": urdf_relative, "bytes": urdf_bytes, "sha256": urdf_sha},
        "mesh_files": sorted(mesh_identities, key=lambda row: row["path"]),
        **structure,
        "scope": "private source identity and URDF structure; not physical validation of Luka",
    }


def write_onboard_integrity_report(preflight: dict[str, object], asset: Path, scene: Path) -> Path:
    """Reverify inputs and save source/output hashes beside the generated USD."""
    bundle = Path(str(preflight["bundle_path"])).resolve()
    if verify_onboard_bundle(bundle) != preflight:
        raise OnboardBundleError("private onboard bundle changed while Isaac Sim was importing")
    asset, scene = asset.resolve(), scene.resolve()
    if asset.suffix.lower() not in (".usd", ".usda", ".usdc"):
        raise OnboardBundleError(f"importer output is not a USD file: {asset}")
    for path in (asset, scene):
        for forbidden_root, label in ((bundle, "private source bundle"),
                                      (PUBLIC_REPO, "public model repository")):
            try:
                path.relative_to(forbidden_root)
            except ValueError:
                continue
            raise OnboardBundleError(f"generated output must stay outside the {label}: {path}")
    report_path = asset.with_name(asset.stem + ".integrity.json")
    asset_root = asset.parent.resolve()
    generated_assets: list[dict[str, object]] = []
    for path in sorted(asset_root.rglob("*")):
        if not path.is_file() or path.name.endswith(".integrity.json") or path.name.startswith(".luka_integrity_"):
            continue
        resolved = path.resolve()
        try:
            relative = resolved.relative_to(asset_root).as_posix()
        except ValueError as exc:
            raise OnboardBundleError(f"generated asset escapes output directory: {path}") from exc
        size, digest = _sha256_file(resolved)
        generated_assets.append({"path": relative, "bytes": size, "sha256": digest})
    if asset.name not in {row["path"] for row in generated_assets}:
        raise OnboardBundleError(f"imported USD is missing from its output directory: {asset}")
    scene_bytes, scene_sha = _sha256_file(scene)
    report = {
        **preflight,
        "generated_scene": {"path": str(scene), "bytes": scene_bytes, "sha256": scene_sha},
        "generated_assets": generated_assets,
    }
    data = (json.dumps(report, indent=2, sort_keys=True) + "\n").encode("utf-8")
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(mode="wb", dir=asset_root, prefix=".luka_integrity_",
                                         suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(data)
        os.replace(temporary, report_path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return report_path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", nargs="?", type=Path, help="derived/isaac-import directory")
    parser.add_argument("--bundle", dest="bundle_option", type=Path,
                        help="derived/isaac-import directory")
    args = parser.parse_args(argv)
    if (args.bundle is None) == (args.bundle_option is None):
        parser.error("specify exactly one bundle path, positional or with --bundle")
    try:
        metadata = verify_onboard_bundle(args.bundle_option or args.bundle)
    except (OnboardBundleError, OSError) as exc:
        print(f"Onboard bundle check failed: {exc}", file=sys.stderr)
        return 1
    print(json.dumps({
        "bundle": metadata["bundle_path"],
        "manifest_sha256": metadata["manifest"]["sha256"],
        "urdf_sha256": metadata["source_urdf"]["sha256"],
        "revolute_joints": metadata["expected_joint_count"],
        "fixed_joints": metadata["fixed_joint_count"],
        "mimic_joints": len(metadata["mimic_joint_names"]),
        "mesh_files": len(metadata["mesh_files"]),
        "mesh_references": metadata["mesh_reference_count"],
        "primitive_colliders": metadata["collider_count"],
        "preparer_status": metadata["preparer_status"],
        "status": "verified",
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
