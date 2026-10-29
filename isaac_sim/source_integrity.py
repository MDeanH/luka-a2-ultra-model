"""Verify every local model input before Isaac import and record USD provenance.

The repository's SOURCE_MANIFEST.json pins the baseline MJCF, parameter
ledger, upstream vendor descriptions, and STL meshes. The contact MJCF and
28-joint URDF are reproducible derivatives; they are checked against their
generators, while the generators' hashes are recorded separately because
they were added after the original manifest was created.

This module uses only the Python standard library and can run without Isaac
Sim. A manifest match proves file identity, not physical accuracy for Luka.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[1]
MANIFEST = REPO / "SOURCE_MANIFEST.json"
GENERATED = REPO / "isaac_sim/generated"
BASELINE = REPO / "build/luka_as_is.xml"
CONTACT = REPO / "build/luka_contact_candidate.xml"
URDF = GENERATED / "luka_candidate.urdf"

# These files determined the published model's parameters and structure.
# Meshes are selected dynamically from the file references in each input.
PINNED_MODEL_INPUTS = (
    "build/luka_as_is.xml",
    "model/ledger.csv",
    "model/build_twin.py",
    "requirements-sim.txt",
    "tools/inspect_urdf.py",
    "tools/seed_ledger.py",
    "tools/estimate_inertials.py",
    "vendor/agibot_a2/SHA256SUMS",
    "vendor/agibot_a2/mjcf/agibot_a2_dof12.mjcf",
    "vendor/agibot_a2/urdf/agibot_a2_dof12.urdf",
)


class IntegrityError(ValueError):
    """An input changed, is absent, or cannot be accounted for."""


def repo_relative(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPO).as_posix()
    except ValueError as exc:
        raise IntegrityError(f"file is outside Luka repository: {resolved}") from exc


def fingerprint(path: Path) -> dict[str, object]:
    if not path.is_file():
        raise IntegrityError(f"missing file: {path}")
    digest = hashlib.sha256()
    size = 0
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
                size += len(chunk)
    except OSError as exc:
        raise IntegrityError(f"cannot read {path}: {exc}") from exc
    return {"path": repo_relative(path), "bytes": size, "sha256": digest.hexdigest()}


def load_manifest() -> tuple[dict[str, dict], dict[str, object]]:
    identity = fingerprint(MANIFEST)
    try:
        document = json.loads(MANIFEST.read_text(encoding="utf-8"))
        entries = document["files"]
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise IntegrityError(f"invalid SOURCE_MANIFEST.json: {exc}") from exc
    if not isinstance(entries, list):
        raise IntegrityError("SOURCE_MANIFEST.json: files must be a list")
    by_path: dict[str, dict] = {}
    for entry in entries:
        if not isinstance(entry, dict):
            raise IntegrityError("SOURCE_MANIFEST.json: file entry must be an object")
        name = entry.get("path")
        if not isinstance(name, str) or not name or name in by_path:
            raise IntegrityError(f"SOURCE_MANIFEST.json: duplicate or invalid path {name!r}")
        expected_size, expected_hash = entry.get("bytes"), entry.get("sha256")
        if not isinstance(expected_size, int) or expected_size < 0:
            raise IntegrityError(f"SOURCE_MANIFEST.json: invalid byte count for {name}")
        if not isinstance(expected_hash, str) or len(expected_hash) != 64 or any(
            char not in "0123456789abcdef" for char in expected_hash
        ):
            raise IntegrityError(f"SOURCE_MANIFEST.json: invalid SHA-256 for {name}")
        # Only canonical repository-relative paths can be used as identities.
        candidate = (REPO / name).resolve()
        if repo_relative(candidate) != name.replace("\\", "/"):
            raise IntegrityError(f"SOURCE_MANIFEST.json: unsafe path {name!r}")
        by_path[name] = entry
    return by_path, identity


def verify_pinned(path: Path, entries: dict[str, dict]) -> dict[str, object]:
    actual = fingerprint(path)
    name = actual["path"]
    if name not in entries:
        raise IntegrityError(f"input is not pinned by SOURCE_MANIFEST.json: {name}")
    expected = entries[name]
    if actual["bytes"] != expected["bytes"] or actual["sha256"] != expected["sha256"]:
        raise IntegrityError(f"manifest hash or size mismatch: {name}")
    return actual


def resolve_local(reference: str | None, base: Path, label: str) -> Path:
    if not reference or "://" in reference:
        raise IntegrityError(f"{label}: unsupported or missing local file reference {reference!r}")
    path = Path(reference)
    resolved = (path if path.is_absolute() else base / path).resolve()
    repo_relative(resolved)  # Reject files outside the pinned repository.
    return resolved


def mjcf_references(source: Path) -> list[Path]:
    try:
        root = ET.parse(source).getroot()
    except (ET.ParseError, OSError) as exc:
        raise IntegrityError(f"cannot parse MJCF {source}: {exc}") from exc
    if root.tag != "mujoco":
        raise IntegrityError(f"{source}: expected <mujoco> root")
    compiler = root.find("compiler")
    meshdir = compiler.get("meshdir", "") if compiler is not None else ""
    basedir = resolve_local(meshdir, source.parent, "MJCF meshdir") if meshdir else source.parent
    assets = root.find("asset")
    if assets is None:
        raise IntegrityError(f"{source}: missing <asset> section")
    references: list[Path] = []
    for element in assets.iter():
        if "file" in element.attrib:
            # MJCF texture files would resolve against texturedir, not
            # meshdir. This source has only mesh file assets; fail closed if
            # another kind is introduced without an explicit resolver.
            if element.tag != "mesh":
                raise IntegrityError(f"{source}: unsupported external asset <{element.tag} file=...>")
            references.append(resolve_local(element.get("file"), basedir, "MJCF mesh"))
    if not references:
        raise IntegrityError(f"{source}: no external mesh assets")
    return references


def urdf_references(source: Path) -> list[Path]:
    try:
        root = ET.parse(source).getroot()
    except (ET.ParseError, OSError) as exc:
        raise IntegrityError(f"cannot parse URDF {source}: {exc}") from exc
    if root.tag != "robot":
        raise IntegrityError(f"{source}: expected <robot> root")
    references: list[Path] = []
    for element in root.iter():
        if "filename" in element.attrib:
            if element.tag == "texture" and not element.get("filename"):
                # The source URDF contains empty texture placeholders; they
                # do not refer to files consumed by the importer.
                continue
            if element.tag not in ("mesh", "texture"):
                raise IntegrityError(f"{source}: unsupported external asset <{element.tag} filename=...>")
            references.append(resolve_local(element.get("filename"), source.parent, f"URDF {element.tag}"))
    if not references:
        raise IntegrityError(f"{source}: no external mesh assets")
    return references


def verify_derivative(source_type: str, source: Path, floating: bool) -> None:
    # These checks bind files not present in the original source manifest to
    # their manifest-verified parents. The caller also pins the parent files.
    sys.path.insert(0, str(REPO))
    if source_type == "mjcf" and floating:
        from sim.build_contact_scene import generate

        expected = generate()
    elif source_type == "urdf":
        from isaac_sim.build_luka_urdf import generate

        expected, _ = generate(output=source)
    else:
        return
    if source.read_bytes() != expected:
        raise IntegrityError(f"derived source differs from its generator: {repo_relative(source)}")


def verify_source_bundle(source_type: str, source: Path, floating: bool = False) -> dict[str, object]:
    """Return a JSON-ready record after verifying all import input files.

    ``source_type`` is ``mjcf`` or ``urdf``. The fixed baseline is directly
    manifest-pinned; the floating MJCF and generated URDF must also reproduce
    byte-for-byte from their generators. Referenced meshes must be pinned.
    """
    if source_type not in ("mjcf", "urdf"):
        raise IntegrityError(f"unknown source type: {source_type}")
    source = source.resolve()
    expected_source = CONTACT if floating else BASELINE
    if source_type == "urdf":
        expected_source = URDF
    if source != expected_source.resolve():
        raise IntegrityError(f"unexpected {source_type} source: {source}")
    entries, manifest_identity = load_manifest()

    pinned_paths = {REPO / name for name in PINNED_MODEL_INPUTS}
    mesh_refs = mjcf_references(source) if source_type == "mjcf" else urdf_references(source)
    pinned_paths.update(mesh_refs)
    pinned = [verify_pinned(path, entries) for path in sorted(pinned_paths)]
    verify_derivative(source_type, source, floating)
    source_identity = verify_pinned(source, entries) if source == BASELINE else fingerprint(source)

    generated_tools = [REPO / "isaac_sim/import_scene.py", Path(__file__)]
    if source_type == "urdf":
        generated_tools.append(REPO / "isaac_sim/build_luka_urdf.py")
    if source_type == "mjcf" and floating:
        generated_tools.append(REPO / "sim/build_contact_scene.py")
    return {
        "schema_version": 1,
        "source_type": source_type,
        "floating": floating,
        "source_manifest": manifest_identity,
        "source": {**source_identity, "manifest_pinned": source == BASELINE},
        "pinned_inputs": pinned,
        "referenced_mesh_count": len(mesh_refs),
        "local_tools": [fingerprint(path) for path in generated_tools],
        "scope": "file identity and reproducible derivation; not physical validation of Luka",
    }


def write_integrity_report(preflight: dict[str, object], asset: Path, scene: Path) -> Path:
    """Recheck sources after import, then save hashes beside the imported USD."""
    source_type = preflight["source_type"]
    floating = preflight["floating"]
    source_path = REPO / preflight["source"]["path"]  # type: ignore[index]
    current = verify_source_bundle(source_type, source_path, floating)  # type: ignore[arg-type]
    if current != preflight:
        raise IntegrityError("model inputs changed while Isaac Sim was importing")
    asset, scene = asset.resolve(), scene.resolve()
    repo_relative(asset)
    repo_relative(scene)
    if asset.suffix.lower() not in (".usd", ".usda", ".usdc"):
        raise IntegrityError(f"importer output is not a USD file: {asset}")
    report_path = asset.with_name(asset.stem + ".integrity.json")
    assets = []
    for path in sorted(asset.parent.rglob("*")):
        if path.is_file() and not path.name.endswith(".integrity.json") and not path.name.startswith(".luka_integrity_"):
            assets.append(fingerprint(path))
    if not any(item["path"] == repo_relative(asset) for item in assets):
        raise IntegrityError(f"imported USD is missing: {asset}")
    report = {**current, "generated_scene": fingerprint(scene), "generated_assets": assets}
    data = (json.dumps(report, indent=2, sort_keys=True) + "\n").encode("utf-8")
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(mode="wb", dir=asset.parent, prefix=".luka_integrity_",
                                         suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(data)
        os.replace(temporary, report_path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return report_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", choices=("mjcf", "urdf"), default="mjcf")
    parser.add_argument("--floating", action="store_true")
    args = parser.parse_args()
    source = URDF if args.source == "urdf" else (CONTACT if args.floating else BASELINE)
    try:
        result = verify_source_bundle(args.source, source, args.floating)
    except (IntegrityError, OSError, ValueError) as exc:
        print(f"Integrity check failed: {exc}", file=sys.stderr)
        return 1
    print(json.dumps({
        "source": result["source"]["path"],  # type: ignore[index]
        "manifest_pinned_files": len(result["pinned_inputs"]),  # type: ignore[arg-type]
        "mesh_references": result["referenced_mesh_count"],
        "status": "verified",
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
