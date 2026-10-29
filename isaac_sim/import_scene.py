"""Import Luka's private onboard URDF or public candidate into Isaac Sim 6.2.

Run this with Isaac Sim's bundled ``python.bat`` or ``python.sh``. Private
bundle and checked-in public sources are read-only inputs. Neither source
is a verified physical match to Luka or includes his balance controller.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import traceback
import uuid
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[1]
GENERATED = REPO / "isaac_sim" / "generated"
MANIFEST = REPO / "SOURCE_MANIFEST.json"
EXPECTED_JOINTS = 28  # Public candidate only; onboard bundle has its own count.
URDF_SPAWN_HEIGHT_M = 1.0228  # Candidate height from build/luka_as_is.xml.


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source", choices=("onboard", "mjcf", "urdf"), default="onboard",
        help="model to import (default: private onboard URDF bundle)",
    )
    parser.add_argument(
        "--bundle", type=Path,
        help="path to the private derived/isaac-import bundle (required for --source onboard)",
    )
    parser.add_argument(
        "--output-dir", type=Path,
        help="private USD output directory outside this public repository (required for onboard)",
    )
    parser.add_argument(
        "--floating", action="store_true",
        help="use a floating base for public MJCF/URDF (onboard import is fixed-base only)",
    )
    parser.add_argument("--headless", action="store_true", help="run Isaac Sim without a viewport")
    args = parser.parse_args()
    if args.source == "onboard" and args.bundle is None:
        parser.error("--bundle is required for the private onboard source")
    if args.source == "onboard" and args.output_dir is None:
        parser.error("--output-dir is required for the private onboard source")
    if args.source != "onboard" and args.bundle is not None:
        parser.error("--bundle is only used with --source onboard")
    if args.source != "onboard" and args.output_dir is not None:
        parser.error("--output-dir is only used with --source onboard")
    if args.source == "onboard" and args.floating:
        parser.error("the onboard bundle currently supports fixed-base import only")
    return args


def manifest_verified_baseline() -> bytes:
    source = REPO / "build" / "luka_as_is.xml"
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    entries = [entry for entry in manifest["files"] if entry["path"] == "build/luka_as_is.xml"]
    if len(entries) != 1:
        raise ValueError("SOURCE_MANIFEST.json must contain one build/luka_as_is.xml entry")
    data = source.read_bytes()
    entry = entries[0]
    if len(data) != entry["bytes"] or hashlib.sha256(data).hexdigest() != entry["sha256"]:
        raise ValueError(
            "build/luka_as_is.xml differs from SOURCE_MANIFEST.json; "
            "review provenance before importing"
        )
    return data


def preflight_mjcf(source: Path, floating: bool) -> None:
    baseline = manifest_verified_baseline()
    if floating:
        # The contact candidate is not listed separately in SOURCE_MANIFEST.
        # Its generator verifies the manifest-pinned baseline before deriving it.
        sys.path.insert(0, str(REPO))
        from sim.build_contact_scene import generate

        if source.read_bytes() != generate():
            raise ValueError(
                "build/luka_contact_candidate.xml differs from its manifest-verified "
                "generator; run python sim/build_contact_scene.py --check"
            )
    elif source.read_bytes() != baseline:
        raise ValueError("MJCF source changed after manifest verification")

    root = ET.parse(source).getroot()
    compiler = root.find("compiler")
    meshdir = compiler.get("meshdir") if compiler is not None else None
    base = (source.parent / meshdir).resolve() if meshdir else source.parent.resolve()
    meshes = root.findall("./asset/mesh")
    if not meshes:
        raise ValueError(f"{source}: no MJCF <asset><mesh> entries")
    for mesh in meshes:
        reference = mesh.get("file")
        if not reference or "://" in reference:
            raise ValueError(f"{source}: unsupported mesh reference {reference!r}")
        mesh_path = Path(reference)
        resolved = (mesh_path if mesh_path.is_absolute() else base / mesh_path).resolve()
        if not resolved.is_file() or resolved.stat().st_size == 0:
            raise FileNotFoundError(f"MJCF mesh {reference!r} is missing or empty: {resolved}")
    hinges = [joint for joint in root.findall("./worldbody//joint") if joint.get("type", "hinge") == "hinge"]
    if len(hinges) != EXPECTED_JOINTS:
        raise ValueError(f"{source}: expected {EXPECTED_JOINTS} hinge joints, found {len(hinges)}")
    freejoints = root.findall("./worldbody//freejoint")
    if len(freejoints) != int(floating):
        raise ValueError(f"{source}: unexpected floating-base joint count {len(freejoints)}")
    print(f"Preflight: {len(hinges)} MJCF hinges and {len(meshes)} local meshes verified")


def preflight_urdf(source: Path) -> None:
    if not source.is_file():
        raise FileNotFoundError(
            f"Derived URDF is missing: {source}\n"
            "Run python isaac_sim/build_luka_urdf.py from the repository root first."
        )
    sys.path.insert(0, str(REPO))
    from isaac_sim.build_luka_urdf import generate

    expected, _ = generate(output=source)
    if source.read_bytes() != expected:
        raise ValueError(
            f"Derived URDF is stale or modified: {source}\n"
            "Run python isaac_sim/build_luka_urdf.py again."
        )
    root = ET.parse(source).getroot()
    joints = [joint for joint in root.findall("joint") if joint.get("type") == "revolute"]
    if len(joints) != EXPECTED_JOINTS:
        raise ValueError(f"{source}: expected {EXPECTED_JOINTS} revolute joints, found {len(joints)}")
    meshes = root.findall(".//mesh")
    if not meshes:
        raise ValueError(f"{source}: no URDF mesh entries")
    for mesh in meshes:
        reference = mesh.get("filename")
        if not reference or "://" in reference:
            raise ValueError(f"{source}: unsupported mesh reference {reference!r}")
        mesh_path = Path(reference)
        resolved = (mesh_path if mesh_path.is_absolute() else source.parent / mesh_path).resolve()
        if not resolved.is_file() or resolved.stat().st_size == 0:
            raise FileNotFoundError(f"URDF mesh {reference!r} is missing or empty: {resolved}")
    print(f"Preflight: {len(joints)} URDF revolute joints and {len(meshes)} local meshes verified")


def import_asset(source: Path, output_dir: Path, source_type: str, floating: bool) -> Path:
    from isaacsim.core.experimental.utils.app import enable_extension

    output_dir.mkdir(parents=True, exist_ok=True)
    if source_type == "mjcf":
        if not enable_extension("isaacsim.asset.importer.mjcf"):
            raise RuntimeError("Could not enable isaacsim.asset.importer.mjcf")
        from isaacsim.asset.importer.mjcf import MJCFImporter, MJCFImporterConfig

        config = MJCFImporterConfig(
            mjcf_path=str(source),
            usd_path=str(output_dir),  # The API expects a directory, not a USD filename.
            import_scene=False,
            merge_mesh=False,
            collision_from_visuals=False,
            allow_self_collision=False,
            robot_type="Humanoid",
            fix_base=None if floating else True,
            run_asset_transformer=True,
            run_multi_physics_conversion=True,
        )
        output = MJCFImporter(config).import_mjcf()
    else:
        if not enable_extension("isaacsim.asset.importer.urdf"):
            raise RuntimeError("Could not enable isaacsim.asset.importer.urdf")
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

        config = URDFImporterConfig(
            urdf_path=str(source),
            usd_path=str(output_dir),
            merge_fixed_joints=False,
            merge_mesh=False,
            collision_from_visuals=False,
            allow_self_collision=False,
            robot_type="Humanoid",
            fix_base=not floating,
            run_asset_transformer=True,
            run_multi_physics_conversion=True,
        )
        output = URDFImporter(config).import_urdf()

    asset = Path(output).resolve()
    if not asset.is_file() or output_dir.resolve() not in asset.parents:
        raise RuntimeError(f"Importer did not create a USD asset inside {output_dir}: {output!r}")
    return asset


def select_physics_variant(prim) -> str:
    """Select a PhysX-compatible Physics payload on the referenced robot prim."""
    variant_sets = prim.GetVariantSets()
    if "Physics" not in variant_sets.GetNames():
        raise RuntimeError(
            "Imported robot has no Physics variant set. The Isaac Sim 6.2 "
            "asset transformer may not have produced a layered physics asset."
        )
    variant_set = variant_sets.GetVariantSet("Physics")
    choices = variant_set.GetVariantNames()
    # A dedicated PhysX overlay is preferable. Isaac Sim 6.2 importer output
    # can expose only 'physics' and 'none'; 'physics' carries the shared USD
    # Physics payload that PhysX consumes. Never select 'none' here.
    selected = next((name for name in ("physx", "physics") if name in choices), None)
    if selected is None:
        raise RuntimeError(
            "Imported robot has no PhysX-compatible Physics variant "
            f"('physx' or 'physics'); available variants: {choices}"
        )
    if not variant_set.SetVariantSelection(selected) or variant_set.GetVariantSelection() != selected:
        raise RuntimeError(f"Could not select Physics={selected!r} on {prim.GetPath()}")
    return selected


def derive_ground_placement(robot, clearance_m: float) -> dict[str, object]:
    """Place a model-check floor below enabled collision geometry, not markers."""
    from pxr import Usd, UsdGeom, UsdPhysics

    if not math.isfinite(clearance_m) or clearance_m < 0:
        raise ValueError("Ground clearance must be a finite, nonnegative distance")
    # Imported colliders have guide purpose and can be hidden in the viewport.
    bounds = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide],
        useExtentsHint=False,
        ignoreVisibility=True,
    )
    minimum_z = math.inf
    collision_count = 0
    for prim in Usd.PrimRange(robot, Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
            continue
        if not prim.IsA(UsdGeom.Boundable):
            raise RuntimeError(f"Collision geometry is not boundable: {prim.GetPath()}")
        world_bounds = bounds.ComputeWorldBound(prim).ComputeAlignedRange()
        if world_bounds.IsEmpty() or not all(
            math.isfinite(value)
            for point in (world_bounds.GetMin(), world_bounds.GetMax())
            for value in point
        ):
            raise RuntimeError(f"Collision geometry has invalid bounds: {prim.GetPath()}")
        minimum_z = min(minimum_z, float(world_bounds.GetMin()[2]))
        collision_count += 1
    if not collision_count:
        raise RuntimeError("Cannot place model-check ground: robot has no enabled collision geometry")
    return {
        "method": "minimum world-space collision bound minus clearance",
        "collision_count": collision_count,
        "collision_min_z_m": minimum_z,
        "clearance_m": clearance_m,
        "ground_top_z_m": minimum_z - clearance_m,
        "calibrated": False,
    }


def build_scene(asset: Path, scene_path: Path, expected_joints: int,
                spawn_height_m: float | None = None, *,
                ground_clearance_m: float | None = None) -> tuple[str, list[str], str]:
    from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

    imported = Usd.Stage.Open(str(asset))
    if imported is None or not imported.GetDefaultPrim().IsValid():
        raise RuntimeError(f"Imported asset has no valid default prim: {asset}")

    scene_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = scene_path.with_name(f".{scene_path.stem}.{uuid.uuid4().hex}.usda")
    try:
        stage = Usd.Stage.CreateNew(str(temporary))
        if stage is None:
            raise RuntimeError(f"Could not create temporary scene: {temporary}")
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        stage.SetStartTimeCode(0.0)
        stage.SetEndTimeCode(1e9)
        stage.SetTimeCodesPerSecond(60.0)
        world = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(world.GetPrim())

        # Keep the transform on a parent prim: the referenced asset's default
        # prim carries its Physics variant set and must retain its selection.
        spawn = UsdGeom.Xform.Define(stage, "/World/Luka")
        robot = UsdGeom.Xform.Define(stage, "/World/Luka/Robot")
        reference = Path(os.path.relpath(asset, scene_path.parent)).as_posix()
        robot.GetPrim().GetReferences().AddReference(reference)
        if spawn_height_m is not None:
            # The URDF root starts at the origin. Lift this uncalibrated pose
            # so its feet are approximately at ground level when floating.
            spawn.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, spawn_height_m))

        physics_variant = select_physics_variant(robot.GetPrim())

        placement = None
        ground_top_z = 0.0
        if ground_clearance_m is not None:
            placement = derive_ground_placement(robot.GetPrim(), ground_clearance_m)
            ground_top_z = float(placement["ground_top_z_m"])
            stage.GetRootLayer().customLayerData = {"luka_ground_placement": placement}

        ground = UsdGeom.Cube.Define(stage, "/World/Ground")
        ground.CreateSizeAttr(1.0)
        ground.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, ground_top_z - 0.05))
        ground.AddScaleOp().Set(Gf.Vec3f(20.0, 20.0, 0.1))
        ground.CreateDisplayColorAttr([Gf.Vec3f(0.18, 0.20, 0.24)])
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

        physics = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        physics.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
        physics.CreateGravityMagnitudeAttr(9.81)
        sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
        sun.CreateIntensityAttr(2000.0)
        sun.CreateAngleAttr(0.53)

        stage.GetRootLayer().comment = (
            "Luka/AgiBot A2 estimated model candidate. Geometry, inertials, "
            "and control are not validated against Luka hardware."
        )
        if placement is not None:
            stage.GetRootLayer().comment += (
                f" Model-check ground is {ground_clearance_m:g} m below the minimum "
                "collision bound; this placement is not a calibrated standing pose."
            )
        if not stage.GetPrimAtPath("/World/Luka/Robot").IsValid():
            raise RuntimeError("Robot USD reference did not compose")
        robot_path = robot.GetPath()
        roots = [
            str(prim.GetPath())
            for prim in stage.Traverse()
            if prim.GetPath().HasPrefix(robot_path) and prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        ]
        revolute = [
            str(prim.GetPath())
            for prim in stage.Traverse()
            if prim.GetPath().HasPrefix(robot_path) and prim.IsA(UsdPhysics.RevoluteJoint)
        ]
        other_movable = [
            str(prim.GetPath())
            for prim in stage.Traverse()
            if prim.GetPath().HasPrefix(robot_path)
            and prim.IsA(UsdPhysics.Joint)
            and not prim.IsA(UsdPhysics.RevoluteJoint)
            and not prim.IsA(UsdPhysics.FixedJoint)
        ]
        if len(roots) != 1 or len(revolute) != expected_joints or other_movable:
            raise RuntimeError(
                "Imported scene failed closed: expected one articulation root and "
                f"{expected_joints} revolute joints; found roots={roots}, "
                f"revolute={len(revolute)}, other_movable={other_movable}"
            )
        if not stage.GetRootLayer().Save():
            raise RuntimeError(f"Could not save temporary scene: {temporary}")
        # A scene reference stays relative to this directory after the rename.
        del stage
        os.replace(temporary, scene_path)
        if placement is not None:
            print(f"Uncalibrated model-check ground placement: {json.dumps(placement, sort_keys=True)}")
        return roots[0], revolute, physics_variant
    finally:
        temporary.unlink(missing_ok=True)


def main() -> None:
    args = parse_args()
    variant = "floating" if args.floating else "fixed"
    if str(REPO) not in sys.path:
        sys.path.insert(0, str(REPO))
    if args.source == "onboard":
        from isaac_sim.onboard_bundle import (
            OnboardBundleError, verify_onboard_bundle, write_onboard_integrity_report,
        )

        try:
            integrity = verify_onboard_bundle(args.bundle)
        except (OnboardBundleError, OSError) as exc:
            raise SystemExit(f"Onboard bundle check failed: {exc}") from exc
        source = Path(str(integrity["urdf_path"]))
        if source.is_relative_to(REPO):
            raise SystemExit("private onboard input must remain outside the public model repository")
        output_root = args.output_dir.resolve()
        bundle_root = args.bundle.resolve()
        if output_root.is_relative_to(REPO):
            raise SystemExit("private USD output must remain outside the public model repository")
        for ancestor in (output_root, *output_root.parents):
            if (ancestor / ".git").exists():
                raise SystemExit(
                    f"private USD output must stay outside Git checkouts: {ancestor}"
                )
        if output_root.is_relative_to(bundle_root) or bundle_root.is_relative_to(output_root):
            raise SystemExit("private USD output must not overlap the source bundle")
        expected_joints = int(integrity["expected_joint_count"])
        source_type = "urdf"
        spawn_height_m = None  # No measured private-model standing height.
        report_writer = write_onboard_integrity_report
    else:
        from isaac_sim.source_integrity import verify_source_bundle, write_integrity_report

        if args.source == "mjcf":
            source = REPO / "build" / (
                "luka_contact_candidate.xml" if args.floating else "luka_as_is.xml"
            )
            preflight_mjcf(source, args.floating)
        else:
            source = GENERATED / "luka_candidate.urdf"
            preflight_urdf(source)
        integrity = verify_source_bundle(args.source, source, args.floating)
        expected_joints = EXPECTED_JOINTS
        source_type = args.source
        spawn_height_m = URDF_SPAWN_HEIGHT_M if args.source == "urdf" else None
        report_writer = write_integrity_report
        output_root = GENERATED

    # Isaac/Omniverse imports above this point would be too early: SimulationApp
    # must be created first in a standalone Python process.
    try:
        from isaacsim import SimulationApp
    except ModuleNotFoundError as exc:
        if exc.name != "isaacsim":
            raise
        raise SystemExit(
            "Model preflight passed, but Isaac Sim is unavailable. "
            "Run this script with Isaac Sim 6.2's bundled python.bat or python.sh "
            "on a supported RTX host."
        ) from exc

    app = SimulationApp({"headless": args.headless})
    try:
        asset_dir = output_root / "usd" / f"luka_{args.source}_{variant}"
        scene = output_root / f"luka_{args.source}_{variant}_scene.usda"
        asset = import_asset(source, asset_dir, source_type, args.floating)
        root, joints, physics_variant = build_scene(
            asset, scene, expected_joints, spawn_height_m,
            ground_clearance_m=0.005 if args.source == "onboard" else None,
        )
        report = report_writer(integrity, asset, scene)
        print(f"Imported asset: {asset}")
        print(f"Selected Physics variant: {physics_variant}")
        print(f"Validated articulation root: {root}")
        print(f"Validated revolute joints: {len(joints)}")
        print(f"Saved scene: {scene}")
        print(f"Integrity report: {report}")
    except BaseException:
        # Kit's fast shutdown exits the process, so print and preserve failures
        # before closing instead of allowing close() to replace them with exit 0.
        traceback.print_exc()
        app.close(exit_code=1)
        raise
    else:
        app.close()


if __name__ == "__main__":
    main()
