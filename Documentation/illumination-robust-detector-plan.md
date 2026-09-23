# Illumination-robust occupancy detector plan

## Status and purpose

This record reviews the proposal to make the camera occupancy detector less
sensitive to illumination changes. It separates changes whose benefit follows
directly from the current implementation from ideas that need controlled
comparison before they become part of the production detector.

The current firmware uses Scharr gradients, twelve hard 15-degree direction
buckets, three hard position bands per direction, and a baseline-derived
gradient cutoff. Each qualifying gradient contributes one vote. The mismatch
is the larger of the 36-bucket distribution distance and proportional change
in the number of qualifying gradients. This is more robust than raw-pixel
comparison, but it is not illumination invariant: a contrast change can move
gradients across the cutoff and can therefore change both terms of the score.

## Do now

These changes correct known discontinuities or biases without changing the
detector's basic purpose.

### Use an approximately Euclidean gradient magnitude

Replace `abs(gx) + abs(gy)` everywhere with a cheap approximation such as:

```text
hi = max(abs(gx), abs(gy))
lo = min(abs(gx), abs(gy))
magnitude = hi + 3 * lo / 8
```

The current L1 magnitude makes a diagonal gradient appear stronger than a
horizontal or vertical gradient of the same Euclidean strength. The
approximation substantially reduces that directional bias without a square
root. It must be applied identically in camera firmware and both setup-app
previews. It changes the meaning of contrast floors and saved reference
features, so the baseline format must be versioned and existing installations
must capture a new baseline.

### Soft-vote between adjacent direction buckets

A gradient close to a 15-degree boundary should distribute its contribution
linearly between the two neighbouring circular buckets. A small change in the
estimated angle will then produce a small descriptor change instead of moving
an entire vote at once. The direction wraps at 180 degrees.

This is a direct correction to a discontinuity already identified in the
fixed-angle detector notes. Keep the three position bands unchanged for the
first implementation so the effect of angular interpolation can be measured
separately.

### Use magnitude-weighted direction votes and normalize the descriptor

Each accepted gradient should contribute its magnitude rather than a binary
count. Normalize the resulting 36-component vector by its total energy before
comparison. Uniform contrast scaling should then leave the structural
distribution approximately unchanged.

A low noise floor is still required. Normalization must not promote sensor
noise in an almost featureless patch into a confident structural descriptor.
The existing blank/textured/unknown distinction should be retained, with its
thresholds re-qualified against captured data.

### Correct the detector documentation

The active descriptor contains twelve directions times three position bands:
36 populated buckets. The runtime and analysis packet allocate a 37th element
for compatibility, but it is not populated by the fixed-direction extractor.
References to a 27-bucket current detector are stale and should be corrected.

### Keep algorithm components separately observable

Diagnostics should report, at minimum:

- normalized structural-distribution distance;
- total or robust gradient energy and its baseline-relative change;
- blank/textured/unreliable classification;
- the final combined score and decision; and
- analysis time.

Do not hide all evidence behind one maximum score. Separate measurements are
needed to tune the detector and to understand false triggers.

## Worthy of an experiment

These proposals are credible but introduce policy choices, new failure modes,
or greater state and protocol costs. They should be evaluated on recorded
clear and occupied scenes before adoption.

### Replace edge-count change with calibrated gradient energy

The binary edge count is highly sensitive to gradients crossing a hard cutoff,
but it detects a useful case: rolling stock can preserve rail-like directions
while removing sleeper or ballast texture. Test a robust energy feature as its
replacement, for example clipped or trimmed gradient magnitude per valid
sensor pixel.

Energy per area is not inherently exposure invariant. Compare it with a
learned clear-scene distribution and give it its own allowance. Test several
ways of combining it with structural distance; taking the maximum may remain
too sensitive to a single unstable feature.

### Learn per-component median and variability

During empty-scene calibration, collect every normalized descriptor component
and the energy statistic for every sample. Store a median and robust spread
such as MAD. A comparison can then ignore ordinary clear-scene movement:

```text
residual = max(0, abs(live - median) - allowance * spread)
```

This should weight stable structure more strongly than naturally unstable
reflections or shadows. The experiment must establish:

- the number and spacing of calibration samples;
- a non-zero minimum spread so a perfectly still short calibration is not
  treated as perfect future certainty;
- how component residuals become one score;
- RAM, flash, and analysis-packet representation; and
- behaviour when calibration does not include the later lighting range.

Ten seconds under one fixed light condition is not enough to characterize a
day/night installation. Do not update the baseline merely because this same
detector currently says clear; that could learn a stationary train into the
empty state. Any continuing adaptation needs independent proof that the track
is empty or an explicit operator-controlled learning period.

### Soften position-band boundaries

Linearly voting between adjacent position bands could remove another hard
boundary. Test this only after angular interpolation because the existing
eight-neighbour one-pixel search already mitigates small image translations.
Measure whether position soft-voting improves clear-scene stability without
making a vehicle that shifts structure within a sensor harder to detect.

### Add Census/LBP structural anchors as a second-stage check

Census patterns compare local ordering rather than absolute brightness and are
cheap to compare with XOR and popcount. They may distinguish preserved geometry
under changed illumination from a physical obstruction.

Test anchors only for borderline structural scores. Candidate selection,
minimum local contrast, search radius, permitted Hamming distance, and the
number of agreeing anchors all need qualification. Include camera movement,
blur, reflections, soft and hard shadows, pale rolling stock, and repeating
rail/sleeper geometry. Anchors should not be adopted merely because they reject
one lighting example; exact local features have previously proved sensitive to
small image movement.

### Add optional exposure and gain control

Test three camera modes where supported by the sensor driver:

1. automatic exposure and gain;
2. automatic settling followed by a lock; and
3. explicit manual exposure and gain.

Locking is attractive for layouts with controlled lighting but should not be a
universal default for scenes expected to span daylight to darkness. Record
exposure/gain values with every dataset where the sensor exposes them, and
treat a mode change as requiring a new baseline.

### Reconsider score combination

The current maximum of distribution change and edge-count change lets one
unstable feature dominate the result. Compare maximum, weighted sum, and
calibrated standardized residual aggregation. Select the simplest rule that
separates clear and occupied observations across all tested conditions, not
the rule that produces the best result on one sensor.

## Live-data experiment method

### What can be collected now

With a bridge connected over USB, the existing protocol can provide:

- continuous per-sensor scores and state changes;
- an on-demand current camera frame transferred through the bridge;
- an on-demand `CELL_ANALYSIS` record containing baseline/live bucket counts,
  edge totals, threshold, and current score; and
- camera and bridge diagnostics.

This is enough to investigate current false triggers and relate them to live
images. It is not enough for a fair offline comparison of arbitrary algorithms:
the bucket record has already discarded gradient magnitudes and pixel-local
structure, and on-demand frames do not guarantee that each frame corresponds
to each reported score.

### Capture facility needed for comparative experiments

Add a diagnostic capture mode that records timestamped raw grayscale frames
and a synchronized metadata row without changing the production decision.
Each observation should include:

- dataset and observation identifiers;
- camera MAC, firmware, configuration revision, frame number, and timestamp;
- resolution and camera settings, including exposure/gain when available;
- sensor geometry and baseline identifier;
- raw grayscale frame or lossless sensor crops including the one-pixel Scharr
  border;
- current production component scores, decision, and processing time; and
- an operator label: `clear`, `occupied`, or `unknown`, plus a condition label
  such as steady light, dimming, brightening, soft shadow, hard shadow, or
  camera disturbance.

Prefer full frames when transfer and storage allow them: future algorithms may
need evidence outside today's crop or a changed sensor geometry. Lossless crops
are acceptable for high-rate collection if their coordinates and borders are
preserved.

### Replay harness

Extract the detector's feature and scoring logic into a host-buildable module,
or implement a bit-exact host reference with shared test vectors. Replay every
recorded frame through several named algorithm variants. Do not flash a new
firmware build for each comparison; all candidates should see identical input.

Initially compare:

- A: current binary hard-bucket detector;
- B: approximately Euclidean magnitude only;
- C: B plus magnitude-weighted normalized votes;
- D: C plus soft angular voting;
- E: D plus calibrated robust energy; and
- F: E plus per-component median/MAD allowances.

Census anchors, soft position bands, and alternative camera exposure modes
should be subsequent isolated variants.

### Evaluation criteria

Evaluate at the sensor-event level, not just individual frames. Report:

- false occupied events per clear operating hour;
- missed occupied placements and the rolling-stock types/positions missed;
- occupied detection latency and clear latency;
- score distributions and margin between clear and occupied cases;
- unknown/unreliable time;
- sensitivity to each lighting condition;
- processing time on the target ESP32; and
- additional RAM, PSRAM, flash, and protocol cost.

Use datasets from several sensors and reserve at least one complete capture
session for validation rather than tuning. Closely adjacent frames are not
independent samples and must not be randomly split between tuning and
validation.

## Recommended sequence

1. Add synchronized lossless frame/crop and metadata capture through the
   bridge.
2. Record the current detector under unchanged clear track, deliberate lighting
   transitions, shadows, representative rolling stock, and combined
   train/lighting cases.
3. Build the replay harness and confirm that variant A reproduces firmware
   features and scores within defined integer tolerances.
4. Implement and measure the three do-now extractor changes as isolated
   variants B through D.
5. Experiment with calibrated energy and per-feature variability.
6. Select thresholds using training sessions and report results on held-out
   sessions.
7. Port the winning integer implementation to the camera, version the stored
   baseline and wire diagnostics, and repeat live target-hardware tests.
8. Investigate Census anchors, soft position bands, and exposure locking only
   for failure cases that remain in the recorded evidence.

## Decision summary

Proceed with approximately Euclidean magnitude, soft angular voting,
magnitude-weighted normalized structural descriptors, documentation correction,
and separate component diagnostics. Treat robust energy, learned component
variability, position interpolation, Census anchors, exposure/gain policy, and
score aggregation as experiments whose adoption depends on synchronized live
data and held-out replay results.

## Initial synthetic lighting pilot

The dependency-free script
[`experiments/synthetic-lighting-detector.js`](../experiments/synthetic-lighting-detector.js)
provides an initial radius-10 stress test. It renders raised rails and sleepers
under directional Lambertian illumination, applies blur and deterministic
low-level image noise, and sweeps 360 clear plus 360 occupied observations. Run
it from the repository root with:

```sh
node experiments/synthetic-lighting-detector.js
```

It writes the complete observation table and summary under
`experiments/results/`. The track is rotated 8 degrees relative to the image so
that a learned canonical orientation cannot merely permute the global bins. At
the existing 0.400 threshold, the current and global soft-voting variants found:

| Clear-scene change | Current maximum | Proposed maximum | Current false triggers | Proposed false triggers |
| --- | ---: | ---: | ---: | ---: |
| Strength only | 0.203 | 0.057 | 0/5 | 0/5 |
| Elevation only | 0.219 | 0.218 | 0/6 | 0/6 |
| Azimuth only | 0.557 | 0.610 | 4/12 | 4/12 |
| Full combined sweep | 0.804 | 0.776 | 95/360 | 132/360 |

The proposed descriptor in this pilot comprises approximately Euclidean
magnitude, magnitude-weighted votes, soft angular voting, and descriptor
normalization. It deliberately omits the proposed learned variability and
energy feature so those effects remain separable.

This result supports the narrower claim that normalization improves tolerance
of uniform strength changes. It does **not** show general invariance to moving
light: changing illumination azimuth changes the visible gradients of raised
geometry, and soft weighted voting alone did not solve that case. In the
synthetic occupied control, the proposed score also missed more observations at
the unchanged 0.400 threshold (225/360 versus 220/360). Thresholds were not
retuned, and the synthetic vehicle is only a sensitivity control, so these are
not production accuracy estimates. They are evidence that the proposed
extractor must be evaluated and tuned as a complete classifier rather than
assumed to be an unconditional improvement.

The same experiment also tests a per-sensor canonical orientation. Calibration
searches half-degree candidates for the strongest magnitude-weighted 15-degree
window, stores its centre as zero, and keeps that transform fixed at runtime.
Hard bucket zero therefore covers exactly -7.5 through +7.5 degrees. The
otherwise-identical global and canonical results were:

| 15-degree variant | Clear triggers at 0.400 | Occupied misses at 0.400 | AUC | Best balanced accuracy |
| --- | ---: | ---: | ---: | ---: |
| Global hard | 151/360 | 134/360 | 0.610 | 0.729 |
| Canonical hard | 142/360 | 75/360 | 0.641 | 0.778 |
| Global soft | 132/360 | 225/360 | 0.623 | 0.761 |
| Canonical soft | 131/360 | 217/360 | 0.635 | 0.772 |

Canonical alignment modestly improved class separation in this synthetic case,
especially for hard buckets, but did not solve illumination-induced false
structure. Calibration chose 68 degrees as the strongest window. That was not
the known rail axis: directional shading made another family stronger. A
canonical angle can still be a valid coordinate frame without being a rail,
but this demonstrates why it must be selected for stability across calibration
frames rather than labelled semantically from one strongest observation.

The renderer does not model camera auto-exposure, cast shadows, specular metal,
colour-dependent grayscale response, compression, real lens behaviour, or a
particular layout. Those omissions make synchronized bridge/camera capture the
next meaningful experiment.
