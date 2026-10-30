# MuJoCo scenario evaluation

- Date: 2026-10-30
- Runs: 5, each until the robot stops at the end of the route, sign detection on (YOLOv8 ONNX, 320 px)
- Machine: Intel(R) Core(TM) i7-9850H CPU @ 2.60GHz, MuJoCo 3.14.0, real-time factor 1.0
- Finish time: from launch until the robot is 2.9 m up the left branch.
- Lane error: distance of base_link from the lane centre on the straight stem (x 0-1.0 m) and left branch (y 0.7-2.95 m), from ground-truth /odom.
- Kept lane: error always below 100 mm. Stop counted from 4 s.

| Run | Finish time (s) | Stem mean err (mm) | Stem max err (mm) | Branch mean err (mm) | Branch max err (mm) | Kept lane | Left branch | Completed | Slow at 20 sign (s) | Stopped at STOP (s) |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 50.8 | 1.0 | 11.0 | 5.4 | 35.3 | True | True | True | 4.0 | 4.9 |
| 2 | 50.9 | 1.0 | 10.9 | 5.5 | 35.6 | True | True | True | 4.1 | 4.9 |
| 3 | 51.0 | 1.0 | 11.3 | 5.4 | 35.1 | True | True | True | 3.9 | 4.9 |
| 4 | 50.9 | 1.0 | 10.5 | 5.4 | 37.2 | True | True | True | 3.4 | 4.9 |
| 5 | 50.9 | 0.9 | 11.0 | 5.4 | 37.2 | True | True | True | 3.4 | 4.9 |
| **All** | 50.9 ± 0.1 | 1.0 ± 0.0 | 11.0 ± 0.2 | 5.4 ± 0.0 | 36.1 ± 0.9 | 5/5 | 5/5 | 5/5 | 3.7 ± 0.3 | 4.9 ± 0.0 |
