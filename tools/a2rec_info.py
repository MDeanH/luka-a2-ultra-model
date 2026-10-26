"""Summarize A2REC1 recordings without changing them. Stdlib only.

Usage: python3 -I tools/a2rec_info.py data/ipad-playback/recordings/*.a2rec
This reports file structure and topic counts; it does not interpret joint units.
"""

import argparse
from collections import Counter
from pathlib import Path
import struct


MAGIC = b"A2REC1\n"


def summarize(path: Path):
    topics = Counter()
    first = last = None
    with path.open("rb") as stream:
        if stream.read(len(MAGIC)) != MAGIC:
            raise ValueError(f"{path}: invalid A2REC1 header")
        while marker := stream.read(4):
            if len(marker) != 4:
                raise ValueError(f"{path}: truncated record length")
            (length,) = struct.unpack("<I", marker)
            frame = stream.read(length)
            if len(frame) != length or length < 22:
                raise ValueError(f"{path}: truncated record")
            _, mono, topic_length = struct.unpack_from("<ddH", frame)
            payload_offset = 18 + topic_length
            if payload_offset + 4 > length:
                raise ValueError(f"{path}: invalid topic length")
            (payload_length,) = struct.unpack_from("<I", frame, payload_offset)
            if payload_offset + 4 + payload_length != length:
                raise ValueError(f"{path}: invalid payload length")
            topic = frame[18:payload_offset].decode("utf-8")
            topics[topic] += 1
            first = mono if first is None else first
            last = mono
    return topics, 0.0 if first is None else last - first


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="+", type=Path)
    args = parser.parse_args()
    for path in args.files:
        topics, duration = summarize(path)
        print(f"{path}: {sum(topics.values())} messages, {duration:.1f} s")
        for topic, count in topics.most_common():
            print(f"  {count:>7}  {topic}")


if __name__ == "__main__":
    main()
