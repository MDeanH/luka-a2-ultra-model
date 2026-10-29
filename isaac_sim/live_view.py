"""Show a live RTX camera feed from a Luka scene over a local HTTP connection.

Run on the GPU host, then forward port 8080 through SSH and open
http://127.0.0.1:8080/ on your computer. The HTTP server deliberately listens
only on loopback. This is a camera view of the simulation, not the Isaac editor.

Camera API reference (Isaac Sim 6.2):
https://docs.isaacsim.omniverse.nvidia.com/6.2.0/sensors/isaacsim_sensors_camera.html
"""

from __future__ import annotations

import argparse
import io
import json
import signal
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit


PAGE = b"""<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Luka live simulation</title><style>
body{margin:0;background:#10151d;color:#edf2fa;font:16px system-ui,sans-serif}
main{max-width:1080px;margin:auto;padding:24px}h1{font-size:24px;margin:0 0 8px}
p{color:#adbbcd;line-height:1.5}img{display:block;width:100%;background:#080b10;
border-radius:12px;min-height:240px;object-fit:contain;max-height:75vh}
#status{font-variant-numeric:tabular-nums}small{color:#adbbcd}
</style><main><h1>Luka live simulation</h1>
<p>Live camera from Isaac Sim. This fixed-base model check has no walking policy.</p>
<img id="view" src="/stream.mjpg" alt="Waiting for the first rendered camera frame">
<p id="status" role="status">Starting camera...</p>
<small>The connection is private through your SSH tunnel. Close the simulation process to stop rendering.</small>
</main><script>
const status=document.getElementById('status'),view=document.getElementById('view');
view.onerror=()=>setTimeout(()=>{view.src='/stream.mjpg?t='+Date.now()},1500);
async function poll(){try{const r=await fetch('/status',{cache:'no-store'}),s=await r.json();
status.textContent=s.error?'Error: '+s.error:s.frames===0?s.phase:
`${s.frames} frames | ${s.physics_steps} physics steps | ${s.sim_seconds.toFixed(2)} s simulated | frame age ${s.frame_age_seconds.toFixed(1)} s`;
}catch(e){status.textContent='Connection lost. Check the simulation process and SSH tunnel.'}}
poll();setInterval(poll,1000);
</script></html>"""


class FrameState:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.jpeg: bytes | None = None
        self.frames = 0
        self.updated_at: float | None = None
        self.physics_steps = 0
        self.sim_seconds = 0.0
        self.phase = "Starting Isaac Sim and warming the RTX renderer..."
        self.error: str | None = None
        self.closed = False

    def publish(self, jpeg: bytes, physics_steps: int, sim_seconds: float) -> None:
        with self.condition:
            self.jpeg = jpeg
            self.frames += 1
            self.updated_at = time.monotonic()
            self.physics_steps = physics_steps
            self.sim_seconds = sim_seconds
            self.phase = "Streaming"
            self.condition.notify_all()

    def status(self) -> dict:
        with self.condition:
            return {
                "phase": self.phase, "frames": self.frames,
                "physics_steps": self.physics_steps, "sim_seconds": self.sim_seconds,
                "frame_age_seconds": (time.monotonic() - self.updated_at
                                      if self.updated_at is not None else None),
                "error": self.error,
            }


def make_server(port: int, state: FrameState) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format, *args):
            pass

        def respond(self, code: int, content_type: str, body: bytes) -> None:
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            path = urlsplit(self.path).path
            try:
                if path == "/":
                    self.respond(200, "text/html; charset=utf-8", PAGE)
                elif path == "/status":
                    self.respond(200, "application/json", json.dumps(state.status()).encode())
                elif path == "/snapshot.jpg":
                    with state.condition:
                        jpeg = state.jpeg
                    self.respond(200 if jpeg else 503, "image/jpeg" if jpeg else "text/plain",
                                 jpeg or b"Waiting for a rendered frame")
                elif path == "/stream.mjpg":
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Connection", "close")
                    self.end_headers()
                    previous = 0
                    while True:
                        with state.condition:
                            state.condition.wait_for(
                                lambda: state.closed or state.frames > previous, timeout=5.0
                            )
                            if state.closed:
                                break
                            if state.frames == previous:
                                continue
                            jpeg, previous = state.jpeg, state.frames
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                            + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n"
                        )
                        self.wfile.flush()
                    self.close_connection = True
                else:
                    self.respond(404, "text/plain", b"Not found")
            except (BrokenPipeError, ConnectionResetError):
                self.close_connection = True

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.daemon_threads = True
    return server


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=10.0, help="maximum JPEG feed frames/second")
    parser.add_argument("--max-seconds", type=float, default=None, help="optional wall-clock run limit")
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error("--port must be between 1024 and 65535")
    if not (64 <= args.width <= 1920 and 64 <= args.height <= 1080):
        parser.error("resolution must be between 64x64 and 1920x1080")
    if not 0 < args.fps <= 30:
        parser.error("--fps must be between 0 and 30")
    if args.max_seconds is not None and args.max_seconds <= 0:
        parser.error("--max-seconds must be positive")
    args.scene = args.scene.resolve()
    if not args.scene.is_file():
        parser.error(f"scene not found: {args.scene}")
    return args


def main() -> int:
    args = parse_args()
    # Check the JPEG dependency before starting a GPU application.
    from PIL import Image

    state = FrameState()
    server = make_server(args.port, state)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    print(f"Viewer HTTP listening at http://127.0.0.1:{args.port}/; RTX frames pending.", flush=True)
    app = timeline = callback_id = manager = None
    exit_code = 0
    try:
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True, "width": args.width, "height": args.height})
        import numpy as np
        import omni.timeline
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
        from pxr import Gf, Usd, UsdGeom, UsdLux

        manager = SimulationManager
        opened, stage = stage_utils.open_stage(str(args.scene))
        if not opened or stage is None:
            raise RuntimeError(f"Could not load scene: {args.scene}")
        while stage_utils.is_stage_loading() and not stop.is_set():
            app.update()
        robot = stage.GetPrimAtPath("/World/Luka")
        if not robot.IsValid():
            raise RuntimeError("Scene has no /World/Luka prim")
        bounds = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
        ).ComputeWorldBound(robot).ComputeAlignedRange()
        low, high = np.array(bounds.GetMin()), np.array(bounds.GetMax())
        if (bounds.IsEmpty() or not np.isfinite([low, high]).all()
                or np.any(high < low) or np.linalg.norm(high - low) < 0.01):
            raise RuntimeError("Cannot frame Luka: empty or invalid geometry bounds")
        target = (low + high) / 2.0
        radius = np.linalg.norm(high - low) / 2.0
        direction = np.array([3.0, -4.0, 1.5])
        eye = target + direction / np.linalg.norm(direction) * radius * 3.5
        transform = Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)
        ).GetInverse()
        rotation = transform.ExtractRotationQuat()
        orientation = np.array([rotation.GetReal(), *rotation.GetImaginary()])
        camera = RtxCamera("/World/LiveCamera", tick_rate=args.fps,
                           translations=eye, orientations=orientation)
        camera.camera.set_clipping_ranges(0.01, 1000.0)
        # Author lens units directly alongside USD's default 20.955 aperture.
        # Isaac Sim 6.2's set_focal_lengths multiplies its input by 10, so
        # passing 24 there would author a 240-unit telephoto lens and crop Luka.
        usd_camera = UsdGeom.Camera.Get(stage, camera.paths[0])
        usd_camera.GetFocalLengthAttr().Set(24.0)
        UsdLux.DomeLight.Define(stage, "/World/LiveViewFill").CreateIntensityAttr(500.0)
        sensor = CameraSensor(camera, resolution=(args.height, args.width), annotators=["rgb"])
        apertures = np.array([usd_camera.GetHorizontalApertureAttr().Get(),
                              usd_camera.GetVerticalApertureAttr().Get()])
        field_of_view = np.degrees(2.0 * np.arctan(apertures / 48.0))
        print(f"CAMERA_FRAMING bounds={low.tolist()}..{high.tolist()} "
              f"distance={np.linalg.norm(eye - target):.3f} "
              f"fov_degrees={field_of_view.tolist()}", flush=True)
        timeline = omni.timeline.get_timeline_interface()
        physics = {"steps": 0, "seconds": 0.0}

        def observe_physics_step(dt, _context):
            physics["steps"] += 1
            physics["seconds"] += float(dt)

        callback_id = manager.register_callback(observe_physics_step, IsaacEvents.POST_PHYSICS_STEP)
        timeline.play()
        started = last_encoded = time.monotonic()
        first_frame = True
        while app.is_running() and not stop.is_set():
            tick_started = time.monotonic()
            if args.max_seconds is not None and tick_started - started >= args.max_seconds:
                break
            app.update()
            now = time.monotonic()
            if now - last_encoded >= 1.0 / args.fps:
                rgb, _info = sensor.get_data("rgb")
                if rgb is not None:
                    pixels = rgb.numpy() if hasattr(rgb, "numpy") else np.asarray(rgb)
                    if pixels.size:
                        if pixels.ndim != 3 or pixels.shape[2] not in (3, 4):
                            raise RuntimeError(f"Unexpected camera pixel shape: {pixels.shape}")
                        if pixels.dtype != np.uint8:
                            raise RuntimeError(f"Unexpected camera pixel type: {pixels.dtype}")
                        output = io.BytesIO()
                        Image.fromarray(pixels[..., :3]).save(output, format="JPEG", quality=80)
                        state.publish(output.getvalue(), physics["steps"], physics["seconds"])
                        last_encoded = now
                        if first_frame:
                            print(f"FIRST_RENDERED_FRAME shape={pixels.shape} range="
                                  f"{pixels.min()}..{pixels.max()} physics_steps={physics['steps']}", flush=True)
                            first_frame = False
            stop.wait(max(0.0, 1.0 / 60.0 - (time.monotonic() - tick_started)))
        if state.frames == 0:
            raise RuntimeError("No RTX camera frames were produced")
        print(f"Viewer stopped after {state.frames} frames and {physics['steps']} physics steps.", flush=True)
    except BaseException as error:
        exit_code = 1
        state.error = str(error)
        traceback.print_exc()
    finally:
        try:
            if timeline is not None:
                timeline.stop()
            if manager is not None and callback_id is not None:
                manager.deregister_callback(callback_id)
        except BaseException:
            exit_code = 1
            traceback.print_exc()
        with state.condition:
            state.closed = True
            state.condition.notify_all()
        server.shutdown()
        server.server_close()
        server_thread.join(timeout=2.0)
        if app is not None:
            app.close(exit_code=exit_code)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
