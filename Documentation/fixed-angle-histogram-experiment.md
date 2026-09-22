# Fixed-angle histogram detector

This document records the fixed-angle detector adopted on `main`. The project remains under active development and the detector still requires broader layout testing.

## Algorithm

Every qualifying gradient is assigned by its physical line direction to the nearest of twelve fixed centres at 0°, 15°, …, 165°. Each direction therefore covers a 15-degree range; the 0° bucket wraps across 180°. This includes perpendicular pairs such as 0°/90°, 15°/105°, and 30°/120° as exact bucket centres. Each direction retains three coarse position bands, giving 36 normalized buckets per sensor. There is no dominant-direction threshold, perpendicular search, or catch-all `Other` direction.

The mismatch score is the larger of:

- total variation between the 36 baseline and live direction/position proportions; and
- proportional change in the total number of strong gradients.

The edge-count component is intended to detect cases in which rolling stock preserves the dominant rail direction but removes a substantial part of the empty-scene texture. Fixed buckets also preserve a new direction explicitly even when it was nearly absent from the baseline.

The fetched-frame preview displays all twelve bucket centres and percentages. Lines are drawn only for buckets with at least 3% support so the crop remains readable. **Compare now** displays the camera's 36 baseline and live direction/position proportions and its combined score.

## Compatibility

- Camera firmware identifies itself as `0.2.7`.
- The analysis packet now carries twelve angles and 37 values per bucket array. Flash the matching bridge and camera firmware together.
- The baseline magic changes, so the camera safely rejects baselines made by `main` or an earlier experiment build and requires a new baseline.
- The saved `angleTolerance` setting is ignored by this experiment.

## Suggested comparison procedure

1. Record the current firmware version, sensor geometry, threshold, and several clear/obstructed **Compare now** scores before flashing.
2. Flash the matching current camera and bridge firmware.
3. Reload the current setup page, reconnect, and capture a fresh empty baseline.
4. Leave the track clear and collect repeated scores under the expected lighting range.
5. Add representative rolling stock and collect scores at several positions.
6. Set each sensor threshold above its highest credible clear score and below its lowest obstruction score. For the earlier 208-to-145-edge example, begin testing near 250 rather than assuming the production default of 400.
7. Increase **Frames to occupy** to 2 if isolated frames still cross the threshold.

After the first run, the setup applications offer `new` or `all`. `new` auto-sizes only sensors whose immutable creation revision is newer than the last successful auto-size, so later manual radius or threshold tuning is preserved. `all` intentionally recalculates every sensor. Both choices rebuild the empty baseline for the complete configuration.

Hard 15-degree boundaries are deliberately retained in this version. If otherwise stable lines wander between neighbouring buckets, a future refinement may use overlapping or softly weighted adjacent buckets.
