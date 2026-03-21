#!/usr/bin/env python3
"""
Headless benchmark for OAK-D streams matching scripts/startup_oakd.py (DepthAI v2 API).

Polls rgb / left / right / depth queues independently, estimates FPS and inter-frame
intervals, writes JSON (+ CSV) under logs/bench_oakd/ by default.
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import depthai as dai

# Same pipeline topology as startup_oakd.py (DepthAI v2: XLinkOut + Device).


def build_pipeline() -> dai.Pipeline:
    pipeline = dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)

    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_left = pipeline.create(dai.node.XLinkOut)
    xout_right = pipeline.create(dai.node.XLinkOut)
    xout_depth = pipeline.create(dai.node.XLinkOut)

    xout_rgb.setStreamName("rgb")
    xout_left.setStreamName("left")
    xout_right.setStreamName("right")
    xout_depth.setStreamName("depth")

    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setPreviewSize(640, 480)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(False)

    cam_rgb.preview.link(xout_rgb.input)
    mono_left.out.link(xout_left.input)
    mono_right.out.link(xout_right.input)

    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)
    stereo.depth.link(xout_depth.input)

    return pipeline


def _queue_try_get(q):
    """DepthAI v2 queues expose tryGet(); fall back if missing."""
    fn = getattr(q, "tryGet", None)
    if callable(fn):
        return fn()
    if q.has():
        return q.get()
    return None


def run_benchmark(duration_s: float) -> dict:
    import platform

    pipeline = build_pipeline()
    streams = ("rgb", "left", "right", "depth")
    counts = {s: 0 for s in streams}
    last_t = {s: None for s in streams}
    deltas = {s: [] for s in streams}

    t0 = time.perf_counter()
    deadline = t0 + duration_s

    with dai.Device(pipeline) as device:
        device_name = device.getDeviceName()
        mx_id = device.getMxId()

        queues = {
            "rgb": device.getOutputQueue(name="rgb", maxSize=4, blocking=False),
            "left": device.getOutputQueue(name="left", maxSize=4, blocking=False),
            "right": device.getOutputQueue(name="right", maxSize=4, blocking=False),
            "depth": device.getOutputQueue(name="depth", maxSize=4, blocking=False),
        }

        while time.perf_counter() < deadline:
            for name, q in queues.items():
                pkt = _queue_try_get(q)
                if pkt is None:
                    continue
                now = time.perf_counter()
                counts[name] += 1
                prev = last_t[name]
                if prev is not None:
                    deltas[name].append(now - prev)
                last_t[name] = now

    elapsed = time.perf_counter() - t0

    stream_stats = {}
    for s in streams:
        n = counts[s]
        fps = n / elapsed if elapsed > 0 else 0.0
        dlist = deltas[s]
        if len(dlist) >= 1:
            dt_med = float(statistics.median(dlist))
            dt_min = float(min(dlist))
            dt_max = float(max(dlist))
        else:
            dt_med = dt_min = dt_max = None
        stream_stats[s] = {
            "frames": n,
            "fps": round(fps, 4),
            "dt_s_median": round(dt_med, 6) if dt_med is not None else None,
            "dt_s_min": round(dt_min, 6) if dt_min is not None else None,
            "dt_s_max": round(dt_max, 6) if dt_max is not None else None,
        }

    return {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "duration_wall_s": round(elapsed, 4),
        "duration_target_s": duration_s,
        "depthai_version": getattr(dai, "__version__", "unknown"),
        "python": sys.version.split()[0],
        "platform": platform.platform(),
        "device_name": device_name,
        "mx_id": mx_id,
        "streams": stream_stats,
    }


def write_outputs(result: dict, out_dir: Path, stem: str) -> tuple[Path, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / f"{stem}.json"
    csv_path = out_dir / f"{stem}.csv"

    with json_path.open("w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "stream",
                "frames",
                "fps",
                "dt_s_median",
                "dt_s_min",
                "dt_s_max",
            ]
        )
        for name, s in result["streams"].items():
            w.writerow(
                [
                    name,
                    s["frames"],
                    s["fps"],
                    s["dt_s_median"],
                    s["dt_s_min"],
                    s["dt_s_max"],
                ]
            )

    return json_path, csv_path


def main() -> int:
    p = argparse.ArgumentParser(description="Benchmark OAK-D stream rates (DepthAI v2 pipeline).")
    p.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Wall-clock sampling window in seconds (default: 10)",
    )
    p.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "logs" / "bench_oakd",
        help="Directory for JSON/CSV output",
    )
    args = p.parse_args()

    if args.duration <= 0:
        print("duration must be positive", file=sys.stderr)
        return 2

    print(f"Benchmarking for ~{args.duration}s (independent tryGet polling per stream)...")
    result = run_benchmark(args.duration)

    stem = f"oakd_rates_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
    json_path, csv_path = write_outputs(result, args.output_dir, stem)

    print(json.dumps(result["streams"], indent=2))
    print(f"Wrote {json_path}")
    print(f"Wrote {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
