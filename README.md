# Dual Sensor Smart Line-Following Car 🚗

This project is a smart transport vehicle using dual sensors:
- Camera (8-neighborhood image processing)
- Grayscale sensor (8-channel)

## Modules
- `main.cpp`: Entry
- `image.c/h`: Image line detection & centerline completion
- `makefile` — compilation config

## Features
- Line completion using Kalman, slope extension, least squares
- Sensor fusion to improve turn angle estimation


## How to Run
1. Compile with: make
2. Run: ./main

## Tools
1.fps_check: Check all resolution and fps option of present camera

## Tests
Synthetic regression suite for the centerline reconstruction. Real footage only
covers a few continuous curves and gives no ground truth, so the lane is drawn
row by row instead — the centerline it was drawn from *is* the answer, and the
reconstruction error can be measured exactly.

```
cd tests && make && python3 bench.py
```

Two parts:

1. **Reconstruction accuracy** — a band is erased from a continuous lane and the
   patched centerline is compared against the truth. Rows the algorithm marks as
   `-1` (no measurement) are reported as coverage rather than counted as error.
2. **Sequence stability** — six scenarios (straight, curve, crossing, short gap,
   long gap, S-bend) run 48 frames each, counting frames where the centerline
   jumps sideways. Static images do not catch this class of bug; two real ones
   (whole output collapsing to a straight line when a gap sits at the bottom of
   the frame, and the default value being drawn as a centerline above the lane)
   only showed up when scanning frame by frame.

Baseline after the 2026-08 fixes: **2.1 px** mean reconstruction error, 4
anomalous frames (all at the start of the crossing case, where the crossing bar
fills the full width of the frame and there is no meaningful lane centre).

Note: `center_line_final[y]` may be `-1`, meaning that row has no measurement.
Check before using it as an x coordinate.
