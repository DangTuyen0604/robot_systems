# Perception timing benchmark

- Date: 2026-10-30
- CPU: Intel(R) Core(TM) i7-9850H CPU @ 2.60GHz (12 threads), no GPU used
- OpenCV 4.8.1, ONNX Runtime 1.17.3 (AzureExecutionProvider, CPUExecutionProvider)
- Input: 200 frames 320x240 from `mujoco_camera_frames`, one frame at a time, after 5 warm-up frames

| Stage | Mean (ms) | Median (ms) | p95 (ms) | Max rate (FPS) |
|---|---|---|---|---|
| JPEG decode | 0.74 | 0.73 | 0.84 | 1354 |
| Lane detection + control | 4.89 | 4.84 | 5.40 | 205 |
| Sign detection viet_320.onnx @ 320px | 24.84 | 24.43 | 31.79 | 40 |
| Sign detection viet.pt @ 640px | 53.11 | 52.75 | 56.26 | 19 |

The sign node runs every 2nd camera frame (`process_every_n_frames: 2`), so
at 20 FPS camera input it needs under 100 ms per inference to keep up.
