"""Checks for the derived Luka URDF, using the real public model inputs."""

from __future__ import annotations

import csv
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from build_luka_urdf import BuildError, GENERATED_DIR, LEDGER, REPO, VENDOR_URDF, build  # noqa: E402


class BuildLukaUrdfTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        GENERATED_DIR.mkdir(parents=True, exist_ok=True)

    def test_real_candidate_preserves_tree_and_meshes_and_adds_16_joints(self) -> None:
        source_hash = hashlib.sha256(VENDOR_URDF.read_bytes()).digest()
        source = ET.parse(VENDOR_URDF).getroot()
        with tempfile.TemporaryDirectory(dir=GENERATED_DIR) as directory:
            output = Path(directory) / "candidate.urdf"
            self.assertEqual(build(output=output), 94)
            candidate = ET.parse(output).getroot()
            self.assertEqual(candidate.get("name"), "luka_candidate")

            source_links = source.findall("link")
            links = candidate.findall("link")
            self.assertEqual([link.get("name") for link in links],
                             [link.get("name") for link in source_links])
            original = {joint.get("name"): joint for joint in source.findall("joint")}
            derived = {joint.get("name"): joint for joint in candidate.findall("joint")}
            self.assertEqual(set(original), set(derived))
            self.assertEqual(sum(joint.get("type") == "revolute" for joint in derived.values()), 28)
            for name, joint in derived.items():
                self.assertEqual(joint.find("parent").get("link"), original[name].find("parent").get("link"))
                self.assertEqual(joint.find("child").get("link"), original[name].find("child").get("link"))
                if name.startswith("idx") and name[3:5].isdigit() and not name[6].isdigit():
                    index = int(name[3:5])
                    if index <= 12:
                        self.assertEqual(joint.get("type"), "revolute")
                        for key in ("lower", "upper", "effort", "velocity"):
                            self.assertEqual(float(joint.find("limit").get(key)),
                                             float(original[name].find("limit").get(key)))
                    elif index <= 28:
                        self.assertEqual(original[name].get("type"), "fixed")
                        self.assertEqual(joint.get("type"), "revolute")
                        self.assertEqual(float(joint.find("limit").get("effort")), 0.0)
                        self.assertEqual(float(joint.find("limit").get("velocity")), 0.0)

            original_meshes = [mesh.get("filename") for mesh in source.iter("mesh")]
            derived_meshes = [mesh.get("filename") for mesh in candidate.iter("mesh")]
            self.assertEqual(len(original_meshes), len(derived_meshes))
            for before, after in zip(original_meshes, derived_meshes):
                self.assertEqual((VENDOR_URDF.parent / before).resolve(),
                                 (output.parent / after).resolve())
                self.assertTrue((output.parent / after).is_file())
            hands = {link.get("name"): link for link in links}
            self.assertAlmostEqual(float(hands["left_hand"].find("inertial/mass").get("value")), 1.6031401)
            self.assertEqual(hashlib.sha256(VENDOR_URDF.read_bytes()).digest(), source_hash)

    def test_missing_active_value_stops_before_writing(self) -> None:
        with tempfile.TemporaryDirectory(dir=GENERATED_DIR) as directory:
            root = Path(directory)
            ledger = self.modified_ledger(root, "link.left_hand.mass", status="missing")
            output = root / "candidate.urdf"
            with self.assertRaisesRegex(BuildError, "missing active ledger value: link.left_hand.mass"):
                build(ledger=ledger, output=output)
            self.assertFalse(output.exists())

    def test_nonfinite_joint_limit_and_invalid_inertia_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory(dir=GENERATED_DIR) as directory:
            root = Path(directory)
            output = root / "candidate.urdf"
            nonfinite = self.modified_ledger(root, "joint.idx01_left_hip_roll.limit.velocity", value="nan")
            with self.assertRaisesRegex(BuildError, "must be finite"):
                build(ledger=nonfinite, output=output)
            self.assertFalse(output.exists())
            invalid = self.modified_ledger(root, "link.left_hand.inertia.ixx", value="-1")
            with self.assertRaisesRegex(BuildError, "not positive definite"):
                build(ledger=invalid, output=output)
            self.assertFalse(output.exists())

    def test_bad_joint_axis_shape_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory(dir=GENERATED_DIR) as directory:
            root = Path(directory)
            vendor = root / "bad.urdf"
            xml = ET.parse(VENDOR_URDF)
            joint = xml.find("./joint[@name='idx13_left_arm_joint1']")
            joint.find("axis").set("xyz", "0 0")
            xml.write(vendor, encoding="utf-8")
            output = root / "candidate.urdf"
            with self.assertRaisesRegex(BuildError, "expected three values"):
                build(vendor_urdf=vendor, output=output)
            self.assertFalse(output.exists())

    def test_evidence_files_cannot_be_output_targets(self) -> None:
        for evidence in (REPO / "SOURCE_MANIFEST.json", VENDOR_URDF):
            with self.subTest(evidence=evidence):
                original = evidence.read_bytes()
                with self.assertRaisesRegex(BuildError, "output must be a .urdf below"):
                    build(output=evidence)
                self.assertEqual(evidence.read_bytes(), original)
        with tempfile.TemporaryDirectory(dir=GENERATED_DIR) as directory:
            source_copy = Path(directory) / "source.urdf"
            source_copy.write_bytes(VENDOR_URDF.read_bytes())
            original = source_copy.read_bytes()
            with self.assertRaisesRegex(BuildError, "output must not overwrite an input file"):
                build(vendor_urdf=source_copy, output=source_copy)
            self.assertEqual(source_copy.read_bytes(), original)

    @staticmethod
    def modified_ledger(directory: Path, parameter: str, **changes: str) -> Path:
        with LEDGER.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            fieldnames = reader.fieldnames
            rows = list(reader)
        matched = [row for row in rows if row["param"] == parameter and row["status"] == "active"]
        if len(matched) != 1:
            raise AssertionError(f"expected one active source row for {parameter}")
        matched[0].update(changes)
        path = directory / f"ledger_{len(list(directory.glob('ledger_*.csv')))}.csv"
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        return path


if __name__ == "__main__":
    unittest.main()
