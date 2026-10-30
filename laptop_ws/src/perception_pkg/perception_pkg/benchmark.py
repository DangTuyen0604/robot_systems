#!/usr/bin/env python3
"""
Time the perception pipeline per frame on a folder of camera images.

    ros2 run perception_pkg benchmark --images DIR --output docs/results/perception_benchmark

Stages: JPEG decode, lane detection + control (640x480 internally), and
sign detection with each model given by --model FILE:IMGSZ (default the
simulation model, viet_320.onnx at 320 px). Frames are processed one by one
in a single thread, like the ROS nodes do.
"""

import argparse
from datetime import date
import glob
import os
import platform
import statistics
import time

import cv2
import numpy as np

from perception_pkg.lane_detector import LaneDetector


def cpu_name():
    try:
        with open('/proc/cpuinfo') as cpuinfo:
            for line in cpuinfo:
                if line.startswith('model name'):
                    return line.split(':', 1)[1].strip()
    except OSError:
        pass
    return platform.processor() or 'unknown'


def timings_ms(fn, items, warmup=5):
    for item in items[:warmup]:
        fn(item)
    out = []
    for item in items:
        start = time.perf_counter()
        fn(item)
        out.append(1000.0 * (time.perf_counter() - start))
    return out


def describe(name, ms):
    ms_sorted = sorted(ms)
    p95 = ms_sorted[min(len(ms_sorted) - 1, int(0.95 * len(ms_sorted)))]
    mean = statistics.mean(ms)
    return (f'| {name} | {mean:.2f} | {statistics.median(ms):.2f} | {p95:.2f} | '
            f'{1000.0 / mean:.0f} |')


def model_path(name):
    if os.path.isabs(name) or os.path.exists(name):
        return name
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('perception_pkg'), name)


def main(args=None):
    parser = argparse.ArgumentParser(description='Perception timing benchmark')
    parser.add_argument('--images', required=True, help='folder of .jpg frames')
    parser.add_argument('--model', action='append', default=None,
                        help='sign model as FILE:IMGSZ, repeatable '
                             '(default viet_320.onnx:320)')
    parser.add_argument('--output', help='write <output>.md')
    args = parser.parse_args(args)
    models = args.model or ['viet_320.onnx:320']

    paths = sorted(glob.glob(os.path.join(args.images, '*.jpg')))
    if not paths:
        parser.error(f'no .jpg files in {args.images}')
    jpegs = [open(p, 'rb').read() for p in paths]
    frames = [cv2.imdecode(np.frombuffer(j, np.uint8), cv2.IMREAD_COLOR) for j in jpegs]
    height, width = frames[0].shape[:2]

    rows = [describe('JPEG decode', timings_ms(
        lambda j: cv2.imdecode(np.frombuffer(j, np.uint8), cv2.IMREAD_COLOR), jpegs))]

    detector = LaneDetector()
    p = detector.p

    def lane(frame):
        detector.control(detector.detect(cv2.resize(frame, (p.width, p.height))).lane_center)
    rows.append(describe('Lane detection + control', timings_ms(lane, frames)))

    from ultralytics import YOLO
    import onnxruntime
    for spec in models:
        name, imgsz = spec.rsplit(':', 1)
        model = YOLO(model_path(name), task='detect')
        kwargs = {'imgsz': int(imgsz), 'verbose': False}
        if not name.endswith('.onnx'):
            kwargs['device'] = 'cpu'
        rows.append(describe(f'Sign detection {name} @ {imgsz}px', timings_ms(
            lambda frame: model.predict(frame, **kwargs), frames)))

    source = os.path.basename(os.path.normpath(args.images))
    report = '\n'.join([
        '# Perception timing benchmark',
        '',
        f'- Date: {date.today().isoformat()}',
        f'- CPU: {cpu_name()} ({os.cpu_count()} threads), no GPU used',
        f'- OpenCV {cv2.__version__}, ONNX Runtime {onnxruntime.__version__} '
        f'({", ".join(onnxruntime.get_available_providers())})',
        f'- Input: {len(frames)} frames {width}x{height} from `{source}`, '
        'one frame at a time, after 5 warm-up frames',
        '',
        '| Stage | Mean (ms) | Median (ms) | p95 (ms) | Max rate (FPS) |',
        '|---|---|---|---|---|',
        *rows,
        '',
        'The sign node runs every 2nd camera frame (`process_every_n_frames: 2`), so',
        'at 20 FPS camera input it needs under 100 ms per inference to keep up.',
        '',
    ])
    print(report)
    if args.output:
        with open(args.output + '.md', 'w') as md:
            md.write(report)


if __name__ == '__main__':
    main()
