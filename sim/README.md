# Luka in MuJoCo: simulation starting point

Use the public model and iPad playback files here as **simulation inputs**. This is a software setup, not a calibrated representation of Luka or a policy ready for his retained x86 balance controller.

## Run it

From the repository root with Python 3 and the pinned dependencies installed:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-sim.txt
.venv/bin/python -I sim/check_integration.py
```

Open either XML file in MuJoCo's viewer, or load it with `mujoco.MjModel.from_xml_path`:

| File | Purpose | Verified software behavior | Limit |
| --- | --- | --- | --- |
| [`build/luka_as_is.xml`](../build/luka_as_is.xml) | Preserved fixed-base model | Loads with 28 joints and 12 leg actuators | No free root, floor contact, or balance task |
| [`build/luka_contact_candidate.xml`](../build/luka_contact_candidate.xml) | Derived floating-base placement scene | Loads with a free root and floor-only contacts; under zero control it falls and makes floor contact | No stable standing reset, self-contact, or x86 balance-controller model |

The contact scene is generated from the manifest-checked baseline with [`build_contact_scene.py`](build_contact_scene.py). The collision masks allow floor-to-robot contact and suppress robot self-contact because the public collision meshes overlap in the zero pose. The baseline XML and vendor files are unchanged. Regenerate or check the derived file with:

```sh
.venv/bin/python -I sim/build_contact_scene.py --check
```

The fixed-base [`LegPostureEnv`](leg_posture_env.py) exposes `reset(seed)`, `step(action)`, 36 observation values (12 leg positions, 12 velocities, 12 synthetic targets), and 12 normalized torque actions. It is a small, deterministic training-pipeline task. [`tune_posture_demo.py`](tune_posture_demo.py) searches a few PD gains using only MuJoCo and NumPy:

```sh
.venv/bin/python -I sim/tune_posture_demo.py
```

This demonstration does not train walking, balance, arm use, or a deployable policy. Its targets are synthetic, the base is fixed, and the 16 arm/head joints have no actuators in this public model.

## Use the iPad recordings

The three original `.a2rec` files contain downsampled joint **positions** from earlier recorded bags. Each frame's 26 joint names exactly match `idx01` through `idx26` in the fixed-base XML; the two head joints are absent. No base pose, IMU, contact, velocity, effort, or action stream is present. The files are useful for kinematic pose inspection, not yet for dynamic imitation learning.

Check every frame and the manifest hash without applying any pose:

```sh
.venv/bin/python -I sim/preview_ipad_pose.py data/ipad-playback/recordings/luka-20261019-115249-legs.a2rec --frame 100
```

To make an illustrative PNG, explicitly assume the recorded values use the model's radians, signs, and zero offsets:

```sh
.venv/bin/python -I sim/preview_ipad_pose.py data/ipad-playback/recordings/luka-20261022-125322-arm.a2rec --frame 200 --assume-radians --render preview-local.png
```

The pose tool sets named joint coordinates and calls `mj_forward`; the two absent head joints stay at the model's zero pose. It does **not** step physics or infer a demonstrated action. The recorded source converter treated joint positions as radians, but the direction, offset, and frame relationship to this public T2D0 candidate remain **unverified for Luka**. A separate onboard-frame model's sign study does not validate this XML.

Rendering creates a new PNG only. It refuses to overwrite an existing file or write into the original recording or vendor directories.

## Before task training or hardware transfer

The first real training task is still a project choice. For locomotion or balance, establish Luka-specific geometry and masses, a contact-valid standing/reset pose, actuator and dynamics parameters, and a representation of the retained vendor x86 controller. For recorded-pose use, check joint units, signs, offsets, and timing against this exact model. Then define observations, actions, rewards, failure criteria, and repeatable evaluations for the chosen task. Simulation success alone would not verify behavior on Luka.

The public model report lists 100 estimated values, 84 zeroed dynamics gaps, and 16 unactuated arm/head joints. The public candidate is based on an A2 T2D0 model while Luka's software reported `A2_T3D1_FLAGSHIP`; physical fit is unresolved. Keep those provenance limits attached to any dataset or policy trained here.
