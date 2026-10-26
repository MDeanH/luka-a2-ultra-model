# Research log

## 2026-10-26 — iPad joint-only public subset

- The paired iPad Air 11-inch (M3) had Luka app `com.mdeanproductions.luka` v0.3. Its app Documents were copied to a private local evidence folder with original filenames and device modification times. All 12 copied file sizes matched the live device listing; SHA-256 was computed after copying because the device API does not expose hashes.
- Three app playback files total 17,353,625 bytes and contain only `a2remote/joints` records. All decoded payloads used keys `t`, `j`, and `src`; no camera, depth, LiDAR, or room-map topic appeared. The original bytes and their hashes are included under `data/ipad-playback/`.
- The app also held a room-map PLY and two nonempty live recordings containing media/sensor streams; these stay in a private backup. The app figure's 30 onboard-derived meshes are distinct from the public vendor meshes and have no stated redistribution license, so they stay private as well.
- This is software/file evidence. The recordings' coordinate meanings and the public twin's fit to Luka are not physically validated.

## 2026-10-26 — first GitHub model export

- Exported the generated Luka candidate XML, report, parameter ledger, build/inspection scripts, and pristine public AgiBot A2 vendor files from the independent Luka working copy. The source project's selected model files were unchanged in its Git working tree at export; its HEAD was `08820939f675e8db4173cb969c9add814e7593a2`.
- Source model provenance: `vendor/agibot_a2/PROVENANCE.md`, public `Link-U-OS/rl_training` commit `8bf8fd8bbcc38d09731d1b15f894ce7388b08c3d`, Mulan PSL v2 license copy. The local candidate is A2 T2D0; physical compatibility with Luka's observed T3D1 software designation remains unresolved.
- Verification: all 111 staged source/preview files matched `SOURCE_MANIFEST.json`; all 98 entries in the vendor `SHA256SUMS` passed. The independent project's `bash tools/check.sh` completed with 152 tests passing.
- In the standalone export, `model/build_twin.py --out build/rebuilt.xml` produced XML byte-identical to `build/luka_as_is.xml`. The regenerated report differed only in the output filename. MuJoCo 3.17.0 loaded the exported XML with 50 bodies, 28 joints, and 12 actuators; 100 software simulation steps produced finite joint positions.
- Status: software checks only. No physical scan, geometry comparison, telemetry replay, balance-controller validation, or Luka hardware test was performed. The preview is an illustrative render of the model's zero pose.
- The 69 MB generated `.mjb` is omitted from Git history. It can be rebuilt locally from the included files.

## 2026-10-26 — simulator placement and training-pipeline check

- Sources checked: this repo's baseline XML, ledger, model report, and iPad recording manifest; the status-gated Luka iPad wiki node and project plan; the [MuJoCo XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html) and [Python API](https://mujoco.readthedocs.io/en/stable/python.html). These are software/model claims only.
- Baseline probe in MuJoCo 3.17.0: 50 bodies, 28 joints, 12 actuators, fixed base, no world floor or enabled contact. The first real training task remains unselected in the project plan.
- Built a separate derived scene with one free root and floor-only collision masks. A naive all-self-contact variant produced 59 initial robot self-contacts (deepest overlap about 37.9 mm), so that configuration was rejected. The checked-in floor-only candidate starts with zero contacts; at 1 second under zero control it has 20 floor contacts and its base z has fallen from 1.0228 to 0.2804 m. This demonstrates physics stepping and the missing stabilizer, not walking readiness.
- All three public recordings passed hash, topic, joint-name, timestamp, and finite-value checks. Each frame has 26 exact matching joint names; the two model head joints are absent. A selected frame rendered after an explicit assumed-radians mapping. These files contain positions, not actions, base pose, IMU, or contact observations.
- The fixed-base synthetic leg-posture task's deterministic two-seed, 50-step gain search returned mean score -0.928 for zero control and -0.091 for the best sampled gains (2.0 N m/rad and 0.2 N m s/rad). A higher sampled gain (5.0, 0.5) scored -130.197, so gain choice is not interchangeable. This is a software integration example with no physical transfer claim.
- Functional review found that an initial PNG-render option could overwrite an immutable recording if given that output path. It now rejects source/vendor directories, non-PNG outputs, and existing files, and creates a new output exclusively. The integration check reproduces the refusal while preserving the recording hash.
- Final local check rehashed all 98 vendor inputs and all 111 original-export manifest files, regenerated the derived XML byte-for-byte, compiled both scenes, checked all 16,653 public recording frames, and exercised deterministic reset/step. Python compilation and Git whitespace checks passed. Functional reviewers found no remaining blocking issue; all checks are software-only.

Open: choose a real training task; establish Luka-specific geometry, contact pose, dynamics, joint coordinate mapping, and an appropriate representation of the retained vendor x86 controller before calling any trained result Luka-ready.
