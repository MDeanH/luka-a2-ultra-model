"""Contract checks for the external, private onboard URDF import bundle."""

from __future__ import annotations

import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from contextlib import redirect_stdout
import xml.etree.ElementTree as ET

from isaac_sim.onboard_bundle import (
    OnboardBundleError, main, verify_onboard_bundle, write_onboard_integrity_report,
)


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def make_fixture(root: Path) -> tuple[Path, dict]:
    bundle = root / "bundle"
    meshes = bundle / "meshes"
    meshes.mkdir(parents=True)
    rows = []
    for index in range(30):
        content = (
            f"solid mesh_{index}\nfacet normal 0 0 1\nouter loop\n"
            "vertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\n"
            f"endloop\nendfacet\nendsolid mesh_{index}\n"
        ).encode()
        name = f"mesh_{index:02d}.stl"
        (meshes / name).write_bytes(content)
        rows.append({
            "derived_path": f"meshes/{name}",
            "source_name": name,
            "source_bytes": len(content),
            "source_sha256": sha(content),
            "original_urdf_paths": [name],
        })

    robot = ET.Element("robot", name="onboard_candidate")
    links = [ET.SubElement(robot, "link", name=f"link_{i}") for i in range(76)]
    for index in range(59):
        visual = ET.SubElement(links[index], "visual")
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "mesh", filename=rows[index % 30]["derived_path"])
    for index in range(26):
        collision = ET.SubElement(links[index], "collision")
        geometry = ET.SubElement(collision, "geometry")
        if index < 24:
            ET.SubElement(geometry, "box", size="1 1 1")
        else:
            ET.SubElement(geometry, "cylinder", radius="0.1", length="0.2")
    for index in range(75):
        joint = ET.SubElement(
            robot, "joint", name=f"joint_{index}",
            type="revolute" if index < 52 else "fixed",
        )
        ET.SubElement(joint, "parent", link=f"link_{index}")
        ET.SubElement(joint, "child", link=f"link_{index + 1}")
        if 12 <= index < 24:
            ET.SubElement(joint, "mimic", joint=f"joint_{index - 12}")
    urdf_bytes = ET.tostring(robot, encoding="utf-8")
    (bundle / "onboard_model_isaac_candidate.urdf").write_bytes(urdf_bytes)
    manifest = {
        "status": "candidate",
        "input_app_model_index_sha256": "a" * 64,
        "input_onboard_urdf_sha256": "b" * 64,
        "output_urdf": "onboard_model_isaac_candidate.urdf",
        "output_urdf_sha256": sha(urdf_bytes),
        "mesh_references": 59,
        "mesh_files": rows,
        "notes": [],
    }
    (bundle / "MANIFEST.json").write_text(json.dumps(manifest), encoding="utf-8")
    return bundle, manifest


class OnboardBundleTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.bundle, self.manifest = make_fixture(self.root)

    def write_manifest(self) -> None:
        (self.bundle / "MANIFEST.json").write_text(json.dumps(self.manifest), encoding="utf-8")

    def test_bundle_identity_and_output_report(self) -> None:
        identity = verify_onboard_bundle(self.bundle)
        self.assertEqual(identity["expected_joint_count"], 52)
        self.assertEqual(len(identity["mimic_joint_names"]), 12)
        self.assertEqual(len(identity["mesh_files"]), 30)
        self.assertEqual(identity["mesh_reference_count"], 59)
        self.assertEqual(identity["box_collider_count"], 24)
        self.assertEqual(identity["cylinder_collider_count"], 2)
        output = self.root / "output"
        output.mkdir()
        asset = output / "robot.usda"
        scene = output / "scene.usda"
        asset.write_text("#usda 1.0\n", encoding="utf-8")
        scene.write_text("#usda 1.0\n", encoding="utf-8")
        report = write_onboard_integrity_report(identity, asset, scene)
        data = json.loads(report.read_text(encoding="utf-8"))
        self.assertEqual(data["source_urdf"]["sha256"], self.manifest["output_urdf_sha256"])
        self.assertEqual(data["generated_scene"]["sha256"], sha(scene.read_bytes()))

    def test_standalone_preflight_summary(self) -> None:
        output = io.StringIO()
        with redirect_stdout(output):
            self.assertEqual(main(["--bundle", str(self.bundle)]), 0)
        summary = json.loads(output.getvalue())
        self.assertEqual(summary["revolute_joints"], 52)
        self.assertEqual(summary["mesh_references"], 59)

    def test_tampered_mesh_fails_before_import(self) -> None:
        (self.bundle / "meshes" / "mesh_00.stl").write_bytes(b"modified")
        with self.assertRaisesRegex(OnboardBundleError, "hash or size"):
            verify_onboard_bundle(self.bundle)

    def test_manifest_path_traversal_is_rejected(self) -> None:
        outside = self.root / "outside.stl"
        outside.write_bytes(b"outside")
        self.manifest["mesh_files"][0]["derived_path"] = "../outside.stl"
        self.write_manifest()
        with self.assertRaisesRegex(OnboardBundleError, "escapes the bundle"):
            verify_onboard_bundle(self.bundle)

    def test_symlink_outside_bundle_is_rejected(self) -> None:
        outside = self.root / "outside.stl"
        outside.write_bytes(b"outside")
        link = self.bundle / "meshes" / "external.stl"
        try:
            link.symlink_to(outside)
        except (OSError, NotImplementedError):
            self.skipTest("this host cannot create a symlink")
        self.manifest["mesh_files"][0]["derived_path"] = "meshes/external.stl"
        self.write_manifest()
        with self.assertRaisesRegex(OnboardBundleError, "escapes the bundle"):
            verify_onboard_bundle(self.bundle)

    def test_urdf_path_traversal_is_rejected_even_with_updated_hash(self) -> None:
        outside = self.root / "outside.stl"
        outside.write_bytes(b"outside")
        urdf = self.bundle / "onboard_model_isaac_candidate.urdf"
        robot = ET.parse(urdf).getroot()
        robot.find(".//mesh").set("filename", "../outside.stl")
        content = ET.tostring(robot, encoding="utf-8")
        urdf.write_bytes(content)
        self.manifest["output_urdf_sha256"] = sha(content)
        self.write_manifest()
        with self.assertRaisesRegex(OnboardBundleError, "escapes the bundle"):
            verify_onboard_bundle(self.bundle)


if __name__ == "__main__":
    unittest.main()
