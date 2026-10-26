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
