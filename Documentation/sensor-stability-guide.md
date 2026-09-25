# Sensor stability and false-trigger guide

Use this guide when a sensor repeatedly reports occupied even though the scene appears unchanged.

## Start with live evidence

Select the sensor and use **Compare now**. This compares the saved empty-track baseline with the camera's current live frame. The circular **Fetched-frame texture** preview and its angle percentages describe the last manually fetched frame, so do not use that preview alone to diagnose a live trigger.

Record these values in both a normal clear moment and immediately after an unwanted trigger:

- mismatch score and configured threshold;
- baseline and live gradient counts;
- whether the panel describes the baseline and live image as low texture or coherent structure;
- the largest changed direction/position rows;
- whether several sensors changed together.

## Understand the detector's evidence

The detector has two valid baseline classes:

- **Low-texture baseline:** no physical direction has at least six qualifying gradient samples. Changes between isolated sparse samples are treated as noise.
- **Structured baseline:** at least six gradients support one physical direction. The five strongest baseline directions are compared using side/centre bands and concentric rings; live structure may fall to fewer samples before it is considered lost.

For a structured baseline, losing coherent structure is a full structural change. A low-texture baseline is deliberately asymmetric: it occupies in one frame only when the live patch has at least 16 gradients, at least six supporting one direction, and evidence in at least two projected regions and two concentric rings. Merely crossing a total number of edges is not sufficient.

For structured comparisons, absolute edge count is not part of the primary score. Selected spatial distributions are normalised, so broad contrast changes that reveal more of the same structure should have limited effect. Absolute dark or bright coverage is not treated as a reliability signal because a valid view may naturally contain extensive black or white scenery. Any future exposure-quality check must compare against the saved camera scene.

## Diagnose the pattern

### Score stays well below the threshold

The sensor is not currently close to firing. Wait for the unwanted transition and compare again. A previously reported state can outlive the visual event because clearing requires several frames below 70% of the threshold.

### Score jumps directly between 0 and 1000

Look for a low-texture/coherent-structure transition. Confirm that rolling stock produces coherent structure and that the empty scene does not repeatedly do so. If the empty scene alternates classes, reposition or resize the sensor so it contains either a reliably blank patch or a reliably visible line.

### Score varies gradually

Inspect the heat-coloured rows. A repeatable change concentrated in a particular direction and position is more likely to be physical motion, vibration or a moving shadow. Small changes spread across many rows are more likely exposure or noise.

### Several sensors change together

Treat this as a camera or lighting event before tuning individual sensors. Check sunlight, room lights, automatic exposure, camera vibration, focus, power stability and clipping. Re-tuning individual sensors can hide the symptom without correcting the common cause.

### Only one sensor changes

Inspect its crop for reflections, screen flicker, moving shadows, high-contrast objects near its boundary, or a radius that includes unrelated scenery. Move it one pixel at a time with the arrow keys and use **Compare now** after each candidate position.

## Corrective actions, in preferred order

1. **Observe at the moment of failure.** A comparison taken later may describe a recovered clear frame.
2. **Improve placement.** Keep the sensor on the track/vehicle region and exclude reflective or independently moving scenery. Small sensors are faster but must still capture useful evidence.
3. **Adjust radius.** Enlarge a genuinely blank sensor if rolling stock does not introduce coherent structure. Reduce a sensor that includes unstable background detail.
4. **Keep one-frame operation when response speed is essential.** Fix unstable geometry or structural classification first. Only increase Frames to occupy when added latency is acceptable; it filters brief disturbances but cannot cure a sustained false structural change.
5. **Re-tune lighting only when the empty score is persistently elevated.** Re-tune raises thresholds; it does not repair poor geometry or an unstable structural class.
6. **Raise the mismatch threshold cautiously.** Keep measured train scores comfortably above it. A threshold of 800 is already high; raising it cannot prevent a structural-class transition scoring 1000.
7. **Capture a new baseline** only after a real empty-scene, camera, lighting or sensor-geometry change. Threshold and persistence changes do not require a new baseline.

After every adjustment, run rolling stock repeatedly in both directions and use the traversal test. A useful sensor must demonstrate both reliable occupation and reliable clearing under representative lighting.

## Historical Area 6 diagnosis

The live diagnosis on 25 September 2026 showed:

- radius 11 px at `(731, 470)`;
- threshold 800, one frame to occupy and five to clear;
- baseline 8 gradients, including 6 at 0°;
- live 15 gradients with a mismatch score of 129;
- the same dominant horizontal structure remained present.

The scene was not close to the configured mismatch threshold. Its instability came from an older texture classifier treating eight total gradients as structured and seven as low texture. Current firmware classifies structure from coherent directional support instead of that total-count boundary. This example explains an obsolete fault and is not a description of the current decision rule.

## Four-minute stable-scene observation

A live observation on 25 September 2026 identified three separate behaviours:

- **Sensor 7:** an earlier detector let a low-texture baseline of seven scattered gradients occupy when only four aligned 0° gradients appeared. Firmware 0.2.26 replaces that marginal transition with the widespread single-frame test described above.
- **Sensor 4:** its structured 24-gradient baseline temporarily fell to four gradients and scored 1000. The loss lasted about 13 seconds, so a higher threshold or one extra occupancy frame would not cure it. Inspect whether its radius-11 crop loses a local edge under exposure changes; compare while the fault is present and consider moving it onto structure that remains visible under the full lighting range.
- **All sensors unknown:** every sensor became unknown together while camera health remained normal. The original observation exposed an invalid absolute dark-pixel reliability rule. A later run on the corrected build reproduced the symptom through the remaining 20% highlight-clipping rule. Both absolute rules are now removed: valid scenery can contain extensive black or white regions, and any future exposure-quality decision must be baseline-relative.

**Compare now is not a historical capture.** It analyses the newest frame available when the request reaches the camera. At roughly 1.6 frames/s, a one-frame trigger may already have disappeared, so a result of zero immediately after a transition does not disprove the transition. Preserving exact trigger-frame diagnostics is a useful future application improvement.
