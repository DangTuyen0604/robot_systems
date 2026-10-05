# Perception timing benchmark

- Date: 2026-10-04
- CPU: Intel(R) Core(TM) i7-9850H CPU @ 2.60GHz (12 threads), no GPU used
- OpenCV 4.8.1, ONNX Runtime 1.17.3 (AzureExecutionProvider, CPUExecutionProvider)
- Input: 200 frames 320x240 from `mujoco_camera_frames`, one frame at a time, after 5 warm-up frames

| Stage | Mean (ms) | Median (ms) | p95 (ms) | Max rate (FPS) |
|---|---|---|---|---|
| JPEG decode | 0.61 | 0.61 | 0.65 | 1649 |
| Lane detection + control | 4.77 | 4.77 | 5.13 | 210 |
| Sign detection yolov8n.onnx @ 320px | 23.53 | 22.32 | 30.04 | 42 |
| Sign detection viet.pt @ 640px | 50.21 | 50.01 | 51.91 | 20 |

The sign node runs every 2nd camera frame (`process_every_n_frames: 2`), so
at 20 FPS camera input it needs under 100 ms per inference to keep up.
