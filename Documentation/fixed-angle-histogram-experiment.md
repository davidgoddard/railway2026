# Fixed-angle histogram experiment

This document applies to the `experiment/fixed-angle-histogram` branch. It is an experimental detector, not the implementation on `main`.

## What changes

Every qualifying gradient is assigned by its physical line direction to the nearest of nine fixed centres at 0°, 20°, …, 160°. Each direction therefore covers a 20-degree range; the 0° bucket wraps across 180°. Each direction retains three coarse position bands, giving 27 normalized buckets per sensor. There is no dominant-direction threshold, perpendicular search, or catch-all `Other` direction.

The mismatch score is the larger of:

- total variation between the 27 baseline and live direction/position proportions; and
- proportional change in the total number of strong gradients.

The edge-count component is intended to detect cases in which rolling stock preserves the dominant rail direction but removes a substantial part of the empty-scene texture. Fixed buckets also preserve a new direction explicitly even when it was nearly absent from the baseline.

The fetched-frame preview displays all nine bucket centres and percentages. Lines are drawn only for buckets with at least 3% support so the crop remains readable. **Compare now** displays the camera's 27 baseline and live direction/position proportions and its combined score.

## Compatibility

- Camera firmware identifies itself as `0.2.0-fixed-histogram`.
- The bridge protocol and analysis packet sizes are unchanged.
- The baseline magic changes, so the camera safely rejects baselines made by `main` and requires a new baseline.
- The saved `angleTolerance` setting is ignored by this experiment.

## Suggested comparison procedure

1. Record the current firmware version, sensor geometry, threshold, and several clear/obstructed **Compare now** scores before flashing.
2. Flash the camera from this branch. The existing bridge firmware remains compatible.
3. Reload the setup page from this branch, reconnect, and capture a fresh empty baseline.
4. Leave the track clear and collect repeated scores under the expected lighting range.
5. Add representative rolling stock and collect scores at several positions.
6. Set each sensor threshold above its highest credible clear score and below its lowest obstruction score. For the earlier 208-to-145-edge example, begin testing near 250 rather than assuming the production default of 400.
7. Increase **Frames to occupy** to 2 if isolated frames still cross the threshold.

Hard 20-degree boundaries are deliberately retained in this first experiment. If otherwise stable lines wander between neighbouring buckets, the next refinement should use overlapping or softly weighted adjacent buckets.
