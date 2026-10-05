# MuJoCo scenario evaluation

- Date: 2026-10-04
- Runs: 5, each until the robot stops at the end of the route, sign detection on (YOLOv8 ONNX, 320 px)
- Machine: Intel(R) Core(TM) i7-9850H CPU @ 2.60GHz, MuJoCo 3.14.0, real-time factor 1.0
- Finish time: from launch until the robot is 2.9 m up the left branch.
- Lane error: distance of base_link from the lane centre on the straight stem (x 0-1.0 m) and left branch (y 0.7-2.95 m), from ground-truth /odom.
- Kept lane: error always below 100 mm. Stop counted from 4 s.

| Run | Finish time (s) | Stem mean err (mm) | Stem max err (mm) | Branch mean err (mm) | Branch max err (mm) | Kept lane | Left branch | Completed | Slow at 20 sign (s) | Stopped at STOP (s) |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 55.5 | 0.9 | 10.8 | 5.2 | 34.2 | True | True | True | 5.0 | 4.9 |
| 2 | 50.5 | 0.9 | 10.9 | 5.7 | 35.6 | True | True | True | 5.0 | 4.9 |
| 3 | 50.6 | 1.0 | 10.9 | 5.8 | 35.3 | True | True | True | 5.0 | 4.9 |
| 4 | 55.5 | 1.0 | 11.2 | 4.9 | 34.6 | True | True | True | 5.0 | 4.9 |
| 5 | 50.5 | 1.0 | 11.0 | 5.6 | 34.3 | True | True | True | 5.0 | 4.9 |
| **All** | 52.5 ± 2.4 | 1.0 ± 0.1 | 11.0 ± 0.2 | 5.4 ± 0.3 | 34.8 ± 0.6 | 5/5 | 5/5 | 5/5 | 5.0 ± 0.0 | 4.9 ± 0.0 |
