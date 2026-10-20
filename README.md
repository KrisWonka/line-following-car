# Compressed-Air Line-Following Car — Lane Reconstruction & Sensor Fusion

**English** | [中文](README.zh-CN.md)

A line-following car whose only energy source is a tank of compressed air. This repository is its
perception code: a camera pipeline that **rebuilds the lane centerline where the lane disappears**
(crossings, worn paint), an 8-channel grayscale bar read over I²C, and a rule that decides which of
the two steering estimates to trust. Plain C/C++ on the CPU of an embedded Linux board (RDK X5),
no GPU, **45 fps at 188 × 120**.

<table>
  <tr>
    <td width="36%"><img src="docs/media/car.jpg" alt="The car on the test track: a low carbon-fiber chassis with a single tall column carrying a scooter tyre that serves as the air tank, and a black lane curving across the white floor."></td>
    <td width="64%"><img src="docs/media/track.webp" alt="The car following a figure-eight lane on the test floor."></td>
  </tr>
</table>
<p align="center"><sub>Left: The car — the tyre on the column is the air tank · Right: Following the lane on the test track (1.5× speed)</sub></p>

> **Project context** — MAE 106 Mechanical Systems Laboratory, DUT–UCI joint program, 7-person team,
> Feb – May 2025. My parts were the **pneumatic powertrain** and the **perception stack in this repo**.
> The algorithm is published as a sole-authored paper:
> Tian Xia, *A Bidirectional 8-Neighborhood Border Tracking and Kalman-Based Centerline Reconstruction
> Method for Line-Following Robots*, Applied and Computational Engineering 167(1), 122–129, 2025.
> [doi:10.54254/2755-2721/2025.GL25448](https://doi.org/10.54254/2755-2721/2025.GL25448) ·
> Full write-up with videos: [krisxia.com/linefollower](https://krisxia.com/linefollower.html)

## Results

Same 100 real-world frames at 188 × 120, as reported in the paper:

| Metric | Single-direction tracer | This method |
|---|---|---|
| Breakpoint recovery (within 5 px) | 48% | **94%** |
| Mean positional error | 16.3 px | **2.1 px** |
| Centerline smoothness (σ) | 6.7 | **1.6** |
| Throughput, CPU only | — | **45 fps** |

The baseline is a single-direction tracer I implemented for the comparison — this is not a benchmark
against published systems.

## The problem

The steering angle is the mean centerline offset over the **bottom 15 rows** of the frame, at about
0.4° per pixel — fifteen pixels are all the car steers on. A conventional border tracer walks up from
the bottom of the frame and **stops at the first break**. When the break sits low, the pass never gets
going and the centerline collapses to a vertical line down the middle, which the angle calculation
reads as a perfectly straight road — at exactly the moment the car is on a broken one.

<p align="center">
  <img src="docs/media/crossings.jpg" width="300" alt="Left: original grayscale frames — a clean lane as the control case on top, three crossings below. Right: the same frames binarized, with a thin centerline running continuously across each break.">
  &nbsp;
  <img src="docs/media/worn_lane.jpg" width="520" alt="Left: a frame where the lane has worn away into two separate patches. Right: the binarized view with the reconstructed centerline running down the upper patch, across the gap, and on down the lower patch.">
</p>
<p align="center"><sub>Left: crossings (top row is the unbroken control case). Right: a lane worn through into two patches — the centerline is carried across the gap.</sub></p>

## How it works

```
camera frame ─► grayscale → 5×5 Gaussian → histogram equalization → resize 188×120
             ─► Otsu threshold (+ tunable bias)                         ┐
             ─► border tracing, bottom-up   ──► dies at row y₁          │  src/image.c
             ─► border tracing, top-down    ──► dies at row y₂          │
             ─► least-squares fit both sides of the gap, blend across   │
             ─► 2-state Kalman over the rebuilt span → merged centerline┘
             ─► steering angle θ_cam = mean offset of bottom 15 rows × 0.4°/px
8-ch grayscale bar (I²C) ─► θ_gray = arctan(offset / 100 mm)            ┐  src/main.cpp
fusion: |θ_cam − θ_gray| < 10° → average; otherwise trust the steadier one┘
```

1. **Binarize.** Otsu's method plus a hand-tunable bias on its answer. Raising the bias closes gaps,
   lowering it sharpens the lane but lets noise in — that one knob does most of the work of adapting
   to a new floor.
2. **Trace from both ends.** Eight-neighborhood border following and Otsu are standard techniques;
   what this repo adds is running the tracer twice — bottom-up, then top-down with mirrored rotations.
   Each pass survives until it reaches the defect from its own side, so the band between `y₁` and `y₂`
   is exactly what has to be rebuilt. A break is a row-to-row jump of more than 5 px going up, 3 px
   coming down. If the start row has no colour flip, the seed search steps toward the middle for up
   to 30 rows instead of giving up.
3. **Bridge the gap from both ends.** Each border is fitted as `x = k·y + b` (x as a function of row,
   so a near-vertical lane stays well-conditioned) on the 10 rows above `y₂` and the 10 below `y₁`.
   Both fits are extrapolated across the gap and crossfaded with a `t²(3 − 2t)` weight, so the
   handover happens in the middle. If neither end has two usable rows, the patch is skipped rather
   than invented.
4. **Smooth and merge.** A 2-state Kalman filter (position and slope, with the row index playing the
   part of time) runs down the rebuilt span; the three bands are then merged into one centerline.
   Rows with no measurement are marked `-1`.

<p align="center">
  <img src="docs/media/merge_bands.png" width="460" alt="A binarized X-shaped junction with the final centerline drawn in three colours: blue at the bottom, orange across the reconstructed span, reddish purple at the top.">
</p>
<p align="center"><sub><b>Blue</b> (bottom): bottom-up trace · <b>orange</b> (middle): reconstructed span across the crossing · <b>purple</b> (top): top-down trace</sub></p>

**What actually earns the accuracy** — two ablations on the synthetic suite (six gap cases, 130 patched rows):

| Variant | Mean error | Worst row |
|---|---|---|
| Extrapolate from the upper fit only | 3.61 px | 13 px |
| Both ends, blended (shipped) | **2.09 px** | **5 px** |
| Both ends, Kalman switched off | 2.05 px | — |

The second anchor is what matters. The Kalman stage earns almost nothing here: the span it smooths is
already an analytic blend of two straight lines, and the noise it exists to reject was removed one
stage earlier.

<table>
  <tr>
    <td><img src="docs/media/sim_gap.webp" alt="Synthetic frames of a lane with a short gap, run through the real C implementation; left is the input, right the binarization with the reconstructed centerline."></td>
  </tr>
  <tr>
    <td><img src="docs/media/sim_crossing.webp" alt="Synthetic frames of a lane running into a crossing bar, run through the real C implementation."></td>
  </tr>
</table>
<p align="center"><sub>Synthetic frames driven through the real C code — left the input, right what it decides. Top: short gap. Bottom: crossing.</sub></p>

### Two sensors, and which one to believe

<img align="right" width="300" src="docs/media/gray_geometry.png" alt="Plan view of the car on a curving lane: eight probes across the nose, the four over the lane filled black, their midpoint marked, and the angle theta between the car's axis and the line from a point 100 mm behind the bar to that midpoint, with tan theta = x / y.">

An 8-channel grayscale bar at the nose is read over I²C. Probes darker than the threshold (170) are
over the line; their mean position `x` and the fixed 100 mm back to the turning centre give
`θ_gray = arctan(x / 100 mm)` — an arctangent, because what is measured is the angle of a real line
between two points on the car.

So there are two independent estimates of the same angle, and they do not always agree. A fixed
`0.7 × camera + 0.3 × grayscale` blend is still in the source, commented out: when one sensor is
wrong it drags the other with it. What shipped keys off disagreement instead:

- within **10°** of each other → both are probably fine, take the mean;
- further apart → one of them has been fooled (crossing, glare, worn paint), so take the one whose
  reading **changed least since the last frame**.

<br clear="right">

## The car

<img align="right" width="380" src="docs/media/engine.webp" alt="The single-cylinder air engine on the bench: valve, cylinder, connecting rod, crank and flywheel.">

- **Engine** — a 20 mm bore × 45 mm stroke cylinder driving a crank; a 2-position 5-port valve is
  tied to the piston rod so the piston's own travel reverses the air path at each end of stroke.
  The crank drives the wheel 1:1 through a 2GT belt — one cycle, one wheel turn.
- **Tank** — a 5 L tyre charged to 2 bar. Expanding it down to 0.3 bar against a 10 cm wheel works out
  to 72.67 m; the car ran **62.3 m on one fill (86%)**.
- **Chassis** — one central column carrying the tank instead of four, parallelogram front steering
  driven by a centre servo, and a carbon-fiber deck cut with large openings: on a fixed energy budget,
  every gram is distance.

<br clear="right">

## Repository layout

```
src/
  main.cpp       camera capture + preprocessing, grayscale bar over I²C, sensor fusion, live display
  image.c/.h     binarization, bidirectional tracing, break detection, gap bridging, Kalman, merge
tests/           synthetic regression suite (synth.py → runner.c → bench.py), no OpenCV needed
fps_check/       list every resolution / frame rate the camera supports (V4L2)
reso_filter/     same, filtered to one aspect ratio
pics/            sample lane photos used during development
docs/media/      images and clips for this README
```

## Build and run

On the car (Linux, OpenCV 4, the grayscale bar on `/dev/i2c-0` at address `0x4C` — both set at the top
of `src/main.cpp`):

```bash
cd src
make          # g++ main.cpp image.c, flags from pkg-config opencv4
./main        # prints CAM / GRAY / FUSED angle per frame and the FPS; ESC to quit
```

`center_line_final[y]` may be `-1`, meaning that row has no measurement — check before using it as
an x coordinate.

## Tests

Real footage only covers a few continuous curves and has no ground truth, so the regression suite
draws the lane row by row instead: the centerline it was drawn from *is* the answer, and the
reconstruction error can be measured exactly.

```bash
cd tests && make && python3 bench.py
```

1. **Reconstruction accuracy** — a band is erased from a continuous lane and the patched centerline is
   compared against the truth. Rows the algorithm marks `-1` are reported as coverage rather than
   counted as error.
2. **Sequence stability** — six scenarios (straight, curve, crossing, short gap, long gap, S-bend),
   48 frames each, counting frames where the centerline jumps sideways. Static images do not catch
   this class of bug; two real ones (the whole output collapsing to a straight line when a gap sits at
   the bottom of the frame, and the default value being drawn as a centerline above the lane) only
   showed up frame by frame.

Current baseline: **2.1 px** mean reconstruction error and 4 anomalous frames, all at the start of the
crossing case, where the crossing bar fills the full frame width and there is no meaningful lane
centre.
