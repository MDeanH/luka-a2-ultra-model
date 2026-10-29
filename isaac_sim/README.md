# Luka in NVIDIA Isaac Sim

The first import target is Luka's **private onboard URDF export**. Keep the private URDF, meshes, and generated USD outside all public Git checkouts. The public 28-joint A2 candidate remains available for comparison.

## Prepare the private onboard bundle

The private [iPad data repository guide](https://github.com/MDeanH/luka-a2-ultra-ipad-data/blob/main/sim/README.md) describes the preserved `v2026-10-26` release and its [bundle preparer](https://github.com/MDeanH/luka-a2-ultra-ipad-data/blob/main/sim/prepare_isaac_urdf.py). From a checkout of that private repository, extract the release archive into a separate directory, then run:

```text
python sim/prepare_isaac_urdf.py <extracted-release-root>
python sim/prepare_isaac_urdf.py <extracted-release-root> --check
```

The result is `<extracted-release-root>/derived/isaac-import/` with `onboard_model_isaac_candidate.urdf`, 30 STL meshes, and `MANIFEST.json`. The preparer preserves the source URDF and rewrites its mesh paths for a portable bundle. The importer verifies the bundle's hashes, 52 revolute joints, 12 mimic relations, 59 mesh references, and 26 primitive colliders before starting Isaac Sim. This verifies source identity and structure; it does not confirm the model fits Luka's hardware.

## Import and inspect

Use **Isaac Sim 6.2** on a machine that meets [NVIDIA's requirements](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/requirements.html). Set the paths below from this repository's root on Windows PowerShell:

```powershell
$isaacSimRoot = 'C:\path\to\isaac-sim'
$bundle = 'C:\path\to\extracted-release-root\derived\isaac-import'
$privateOutput = 'C:\path\to\private-luka-isaac-output'
& (Join-Path $isaacSimRoot 'python.bat') .\isaac_sim\onboard_bundle.py --bundle $bundle
& (Join-Path $isaacSimRoot 'python.bat') .\isaac_sim\import_scene.py --bundle $bundle --output-dir $privateOutput
& (Join-Path $isaacSimRoot 'python.bat') .\isaac_sim\run_scene.py --scene (Join-Path $privateOutput 'luka_onboard_fixed_scene.usda') --headless --steps 240
```

On Linux, use Isaac Sim's `python.sh`. Choose a private output directory outside every public checkout and outside the source bundle. The import writes `luka_onboard_fixed_scene.usda`, a referenced USD robot asset, and an adjacent `.integrity.json` file with source and output hashes there. The script selects the imported asset's PhysX-compatible `Physics` variant before checking for one articulation root and 52 USD revolute joints. The fixed base is for initial inspection; no standing-height or contact behavior is assumed.

The smoke runner checks that physics time advances and that runtime joint positions are finite. It prints the runtime DOF count, which must be inspected alongside the 12 mimic relations. Use `--expected-dofs N` after verifying that count in Isaac Sim. The smoke run does not establish correct contact, balance, drive behavior, or safe hardware transfer. Inspect masses, joint axes and limits, mimic behavior, colliders, drives, and the Physics variant in Isaac Sim before training.

## Public comparison model

The public [`build/luka_as_is.xml`](../build/luka_as_is.xml) is a 28-joint A2 candidate assembled from vendor geometry and an estimated parameter ledger. Import it separately:

```powershell
& (Join-Path $isaacSimRoot 'python.bat') .\isaac_sim\source_integrity.py
& (Join-Path $isaacSimRoot 'python.bat') .\isaac_sim\import_scene.py --source mjcf
& (Join-Path $isaacSimRoot 'python.bat') .\isaac_sim\run_scene.py --scene .\isaac_sim\generated\luka_mjcf_fixed_scene.usda --headless --steps 240 --expected-dofs 28
```

For a floating floor-contact candidate, add `--floating` to the MJCF import and pass `--scene .\isaac_sim\generated\luka_mjcf_floating_scene.usda` to the runner. It has no balance controller or stable standing reset. A derived 28-joint URDF fallback is available via `build_luka_urdf.py` followed by `import_scene.py --source urdf`; its 16 arm/head effort and velocity values are unknown and represented by zero importer placeholders.

The public model has 100 estimated parameter values, 84 unknown dynamics values represented as zero, and 16 arm/head joints without actuators. The private onboard URDF declares `raise_a2_t2d0_flagship`, while Luka's observed software reports `A2_T3D1_FLAGSHIP`; physical fit remains unverified for both sources. Neither import includes Luka's retained x86 balance controller.

The import uses NVIDIA's [URDF importer](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/importer_exporter/ext_isaacsim_asset_importer_urdf.html) and [MJCF importer](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/importer_exporter/import_mjcf.html). This Windows host has an RTX 2080 Ti with 11 GB VRAM, below Isaac Sim 6.2's published GPU minimum, so runtime validation needs a supported RTX host.
