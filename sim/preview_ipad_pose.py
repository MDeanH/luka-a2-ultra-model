"""Check a public iPad recording and optionally render one assumed pose.

Exact joint names are established; coordinate units, signs, zero offsets and
physical fit are not. --assume-radians is required before applying values to
the model. This tool only calls mj_forward, never a dynamics step.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct

import mujoco


ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data/ipad-playback"
MODEL = ROOT / "build/luka_as_is.xml"
MAGIC = b"A2REC1\n"


def manifest_entry(path: Path) -> dict:
    manifest = json.loads((DATA / "MANIFEST.json").read_text())
    relative = path.resolve().relative_to(DATA.resolve()).as_posix()
    return next(item for item in manifest["files"] if item["path"] == relative)


def verified_recording(path: Path) -> dict:
    entry = manifest_entry(path)
    data = path.read_bytes()
    if len(data) != entry["bytes"] or hashlib.sha256(data).hexdigest() != entry["sha256"]:
        raise ValueError(f"recording differs from manifest: {path}")
    return entry


def inspect_frames(path: Path, expected: set[str], wanted: int):
    count = 0
    selected = None
    last_t = -math.inf
    with path.open("rb") as stream:
        if stream.read(7) != MAGIC:
            raise ValueError("invalid A2REC1 header")
        while marker := stream.read(4):
            if len(marker) != 4:
                raise ValueError("truncated frame length")
            (size,) = struct.unpack("<I", marker)
            frame = stream.read(size)
            if len(frame) != size or size < 22:
                raise ValueError("truncated frame")
            _, _, topic_size = struct.unpack_from("<ddH", frame)
            offset = 18 + topic_size
            if offset + 4 > size:
                raise ValueError("invalid topic size")
            (payload_size,) = struct.unpack_from("<I", frame, offset)
            if offset + 4 + payload_size != size:
                raise ValueError("invalid payload size")
            if frame[18:offset] != b"a2remote/joints":
                raise ValueError("unexpected topic")
            payload = json.loads(frame[offset + 4:])
            if set(payload) != {"t", "j", "src"} or set(payload["j"]) != expected:
                raise ValueError("unexpected payload fields or joint names")
            stamp = payload["t"]
            if not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or stamp <= last_t:
                raise ValueError("invalid or non-increasing timestamp")
            last_t = stamp
            if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in payload["j"].values()):
                raise ValueError("non-finite joint position")
            if count == wanted:
                selected = payload
            count += 1
    if selected is None:
        raise IndexError(f"frame {wanted} outside recording ({count} frames)")
    return count, selected


def validate_render_output(output: Path) -> None:
    resolved = output.resolve()
    if output.suffix.lower() != ".png":
        raise ValueError("render output must be a .png file")
    if resolved.is_relative_to(DATA.resolve()) or resolved.is_relative_to((ROOT / "vendor").resolve()):
        raise ValueError("render output cannot be placed among original data or vendor files")
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing file: {output}")


def render(model, data, output: Path) -> None:
    validate_render_output(output)
    with mujoco.Renderer(model, height=720, width=480) as renderer:
        camera = mujoco.MjvCamera()
        camera.lookat[:] = model.stat.center
        camera.distance = 1.9 * model.stat.extent
        camera.azimuth = 150
        camera.elevation = -10
        renderer.update_scene(data, camera=camera)
        image = renderer.render()
    # Pillow is not a simulation dependency, so write PNG directly.
    import zlib
    def chunk(tag, body):
        return struct.pack(">I", len(body)) + tag + body + struct.pack(">I", zlib.crc32(tag + body) & 0xffffffff)
    raw = b"".join(b"\x00" + image[row].tobytes() for row in range(image.shape[0]))
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("xb") as stream:
        stream.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", image.shape[1], image.shape[0], 8, 2, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path, help="one file from data/ipad-playback/recordings")
    parser.add_argument("--frame", type=int, default=0)
    parser.add_argument("--assume-radians", action="store_true", help="apply recorded values as model radians for illustration")
    parser.add_argument("--render", type=Path, help="save an illustrative PNG (requires --assume-radians)")
    args = parser.parse_args()
    if args.frame < 0 or (args.render and not args.assume_radians):
        parser.error("--frame must be nonnegative; --render requires --assume-radians")
    if args.render:
        validate_render_output(args.render)

    entry = verified_recording(args.recording)
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
    if model.njnt != 28 or not all(name.startswith(f"idx{i:02d}_") for i, name in enumerate(names, 1)):
        raise ValueError("unexpected model joint layout")
    count, selected = inspect_frames(args.recording, set(names[:26]), args.frame)
    if count != entry["messages"]:
        raise ValueError("message count differs from manifest")
    result = {
        "status": "file/software evidence; coordinate mapping and physical fit unverified",
        "recording": args.recording.name,
        "frames_checked": count,
        "selected_frame": args.frame,
        "matching_joint_names": 26,
        "missing_model_joints": names[26:],
        "source_labels": selected["src"],
        "applied_as_model_radians": args.assume_radians,
    }
    if args.assume_radians:
        data = mujoco.MjData(model)
        for name, value in selected["j"].items():
            data.joint(name).qpos[0] = value
        mujoco.mj_forward(model, data)
        if args.render:
            render(model, data, args.render)
            result["render"] = str(args.render)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
