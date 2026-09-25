# Fixed-angle detector — historical experiment and current outcome

This document originally described the `0.2.15` fixed-angle experiment. Its direct 36-bin plus edge-count score is historical and is not the production algorithm. The experiment established the twelve fixed physical-line directions that remain in camera firmware `0.2.26`.

## Retained design

Qualifying Scharr gradients are assigned to directions centred at 0°, 15°, …, 165°. Each direction has three projected position bands and three equal-area concentric rings. The fetched-frame preview displays all twelve directions. **Compare now** returns the camera's saved and current diagnostic buckets, score, threshold, edge totals, and frame number.

## Production differences

The current detector does not take the larger of a 36-bin distance and proportional total-edge-count change. Instead it:

- selects the five baseline directions with the most support;
- normalizes and compares projected and radial distributions only within those directions;
- combines the complementary spatial distances and reduces confidence for sparse evidence;
- searches neighbouring one-pixel offsets only when the first result could affect state;
- classifies a baseline as structured when one direction has at least six supporting gradients; and
- requires widespread single-frame structure for a low-texture baseline: at least 16 gradients, six in one direction, two projected regions, and two rings.

Absolute brightness is not compared. Other angles remain useful diagnostics but cannot dilute the selected structured-baseline comparison.

## Testing guidance

Use **Compare now** repeatedly with clear track and representative rolling stock. A useful threshold lies above credible structured-baseline clear scores and below occupied scores. For a low-texture baseline, occupancy is a gated widespread-structure decision and therefore reports a full score when satisfied. Geometry is usually preferable to threshold changes when a structured baseline repeatedly disappears.

Automatic calibration tests five nested radii and derives individual thresholds. Normal calibration affects new sensors; **Recalibrate all sensors** intentionally replaces all automatic radius and threshold choices. Both rebuild a compatible empty baseline.
