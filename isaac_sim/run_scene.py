"""Open a generated Luka USD scene and step it in Isaac Sim 6.2.

This is a physics/asset smoke run, not a balance or hardware controller.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import traceback


HERE = Path(__file__).resolve().parent
DEFAULT_SCENE = HERE / "generated" / "luka_mjcf_fixed_scene.usda"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--headless", action="store_true", help="run without a viewport")
    parser.add_argument(
        "--steps",
        type=int,
        default=None,
        help="stop after N app updates (default: 240 headless, until window closes in GUI)",
    )
    parser.add_argument("--paused", action="store_true", help="open the scene without starting physics")
    parser.add_argument(
        "--expected-dofs", type=int,
        help="assert an exact runtime DOF count after import (mimic joints may change it)",
    )
    args = parser.parse_args()
    if args.steps is not None and args.steps < 1:
        parser.error("--steps must be positive")
    if args.expected_dofs is not None and args.expected_dofs < 1:
        parser.error("--expected-dofs must be positive")
    return args


def main() -> None:
    args = parse_args()
    scene = args.scene.resolve()
    if not scene.is_file():
        raise SystemExit(
            f"Scene not found: {scene}\n"
            "Run isaac_sim/import_scene.py with Isaac Sim's python.bat first."
        )

    # Omniverse/Isaac modules must be imported only after SimulationApp starts.
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless})
    try:
        import numpy as np
        import omni.timeline
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager
        from pxr import UsdPhysics

        opened, stage = stage_utils.open_stage(str(scene))
        if not opened or stage is None:
            raise RuntimeError(f"Isaac Sim could not open {scene}")
        roots = [prim.GetPath().pathString for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
        if len(roots) != 1:
            raise RuntimeError(f"Expected one Luka articulation root, found {roots}")
        revolute = [
            prim.GetPath().pathString for prim in stage.Traverse()
            if prim.IsA(UsdPhysics.RevoluteJoint)
        ]
        if not revolute:
            raise RuntimeError("Imported Luka scene has no USD revolute joints")
        print(f"Scene: {scene}")
        print(f"Articulation root: {roots[0]}")
        print(f"USD revolute joints: {len(revolute)}")
        print("Simulation is a model check; Luka's balance controller is not represented.")

        timeline = omni.timeline.get_timeline_interface()
        physics = {"steps": 0, "seconds": 0.0}

        def observe_physics_step(dt, _context):
            physics["steps"] += 1
            physics["seconds"] += float(dt)

        callback_id = SimulationManager.register_callback(
            observe_physics_step, IsaacEvents.POST_PHYSICS_STEP
        )
        try:
            if not args.paused:
                timeline.play()
            steps = args.steps if args.steps is not None else (240 if args.headless else None)
            count = 0
            while app.is_running() and (steps is None or count < steps):
                app.update()
                count += 1
            if not args.paused:
                articulation = Articulation(roots[0])
                positions = articulation.get_dof_positions()
                values = np.asarray(
                    positions.numpy() if hasattr(positions, "numpy") else positions
                ).reshape(-1)
                if values.size < 1 or not np.all(np.isfinite(values)):
                    raise RuntimeError(
                        f"Invalid Luka joint state: expected finite DOF positions, "
                        f"found {values.size}"
                    )
                if args.expected_dofs is not None and values.size != args.expected_dofs:
                    raise RuntimeError(
                        f"Expected {args.expected_dofs} runtime DOFs, found {values.size}"
                    )
                if (physics["steps"] < 1 or not math.isfinite(physics["seconds"])
                        or physics["seconds"] <= 0):
                    raise RuntimeError(
                        "No positive physics time elapsed; app updates alone do not "
                        "confirm a running simulation"
                    )
                print(f"Validated {values.size} finite DOF positions after "
                      f"{physics['steps']} physics steps ({physics['seconds']:.3f} s).")
            print(f"Completed {count} Isaac Sim updates.")
        finally:
            timeline.stop()
            SimulationManager.deregister_callback(callback_id)
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
