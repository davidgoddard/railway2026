# Railway camera occupancy toolkit — current functional specification

## Purpose and architecture

The system provides virtual point sensors and multi-sensor blocks for a model railway. Each fixed camera processes its own grayscale frames and sends compact state, score, health, configuration, and diagnostic messages over ESP-NOW. An ESP32-C3 bridge persists configuration and publishes outputs to MQTT. Continuous monitoring does not require the setup application.

The supported setup clients are the Electron application in `Bridge_Controller/app` and the browser application in `Web_App`. They intentionally provide the same camera workspace and workflow. The browser uses Web Serial and requires a secure context; its direct MQTT viewer additionally requires MQTT over WebSockets.

## State and output model

- A **sensor** is a circular image area with a stable ID, centre, radius, contrast floor, mismatch threshold, and enter/clear frame counts.
- A **block** is a shared output ID assigned to several independently analysed sensors. It is occupied if any member is occupied, clear if all members are clear, and unknown otherwise.
- Every sensor and block is `clear`, `occupied`, or `unknown`. Unknown must never be interpreted as clear.
- New sensors default to radius 5 px, mismatch threshold 400/1000, one frame to occupy, and five frames to clear.
- A score at or above the sensor threshold qualifies for occupancy. A previously occupied sensor begins clearing below 70% of its threshold. The intermediate range holds its state.
- Single-frame occupancy is supported and is the production default. Additional enter frames are optional filtering, not a detector requirement.

## Detector

Camera firmware `0.2.26` uses normalized 3×3 Scharr gradients. Every qualifying gradient is assigned to one of twelve fixed physical-line directions at 15-degree intervals. For each direction it records three projected side/centre bands and three equal-area concentric rings.

For a structured empty baseline, the detector selects the five strongest baseline directions. It compares normalized projected and radial distributions within those directions, combines the two spatial distances, discounts sparse evidence, and checks eight neighbouring one-pixel offsets when the initial result could affect state. Absolute brightness and absolute dark/bright pixel coverage are not comparison inputs.

A baseline is structured when at least six gradients support one direction. Losing coherent support from a structured baseline is a full structural change. A low-texture baseline is deliberately asymmetric: sparse rearranged gradients remain clear, while one live frame occupies only when it contains all of the following:

- at least 16 qualifying gradients;
- at least six gradients supporting one direction;
- evidence in at least two projected regions; and
- evidence in at least two concentric rings.

This makes a normally blank sensor respond to substantial structure appearing across its area without returning to the retired total-edge-count class boundary. Camera capture failures stop new analysis; absolute scene brightness does not make all sensors unknown. Any future exposure-quality classifier must be relative to that camera's saved scene.

## Baselines and calibration

**New baseline** averages three fresh empty-scene frames and replaces the saved detector features for the current geometry. It does not resize sensors or change thresholds.

**Calibrate empty track** performs a shared ten-second run. For each selected sensor, its current radius is the maximum search size and five nested radii are tested. The smallest candidate with coherent baseline structure, at least three samples, and a worst clear score no greater than 150 is selected; otherwise the maximum radius is retained. Its threshold is twice the worst measured clear score plus 100, rounded upward to ten and limited to 300–800. Calibration may lower an inherited threshold.

Normal calibration selects sensors created since the last successful automatic calibration. **More → Recalibrate all sensors** deliberately replaces every automatic radius and threshold. Sensors outside a new-sensors-only run retain manual values, while the compatible baseline is rebuilt for the complete configuration. Calibration results are transferred reliably one sensor at a time and saved as one new revision. The clients allow 120 seconds for completion because radio retries and large configurations can finish well after the ten-second sampling period.

**Re-tune lighting** preserves geometry and baseline features. With the target definitely empty, it samples current scores for ten seconds and only raises thresholds where a safe margin exists. It is a recovery tool for current false firing, not a replacement for calibration or a new baseline.

## Application workflow

1. Connect the USB bridge and select a camera.
2. Fetch a frame, place or paint sensors, name outputs, and save/deploy the configuration.
3. Keep the complete view empty and run **Calibrate empty track**. Use **Recalibrate all sensors** when existing automatic values must be replaced. Use baseline-only capture when geometry and thresholds are already correct.
4. Use the live USB overlay to inspect individual sensor states and scores. MQTT overlay displays published output state; every circle in a block therefore shares the combined block state.
5. Run **Test detector** and move rolling stock through one block or all blocks. For each internal block sensor, the two nearest sensors in that block are its neighbours. A neighbour-to-neighbour traversal records a pass, miss, or intermittent result; endpoints are not tested.
6. Use **Compare now** for the camera's current live comparison against its saved baseline. The fetched-frame texture preview describes the last fetched still image and is not live. A brief trigger may be gone before a manual comparison arrives.
7. Use Monitor for output state/history and System for bridge, Wi-Fi, MQTT, radio, firmware, and diagnostic history.

Selecting a sensor in the image, sensor tree, or traversal results keeps the selection synchronized, scrolls it into view, and highlights it on the image. Arrow keys move a selected sensor one pixel. Radius dragging updates continuously and refreshes the expensive texture/layout views after release.

## Connectivity and recovery

Bridge firmware `0.1.21` waits up to twelve seconds for saved Wi-Fi to associate before initializing ESP-NOW, so it normally begins on the router's channel. The bridge sends fast beacons after startup or a channel change and normal beacons thereafter.

After fifteen seconds without valid bridge contact, a camera searches channels 1–13, dwelling for two seconds on each and repeating forever. On every candidate channel it sends both a broadcast HELLO and an immediate unicast HELLO to the known bridge. The bridge answers promptly and the camera republishes its complete state.

## Persistence and transport

The bridge stores camera configuration, MQTT aliases, sensor creation revisions, and the last automatic-calibration revision in LittleFS. Cameras store their applied configuration and compact derived baseline features. The applications cache the last successfully fetched frame per camera for editing and offline/MQTT viewing; that cached image is not the detector baseline.

Configuration, snapshots, baselines, calibration results, diagnostics, and state bitmaps use protocol v3 with application acknowledgements and retries where required. A full state bitmap is repeated periodically so loss of a transition packet self-heals. Snapshot transfer pauses monitoring and is intended for setup, not continuous video.

## Current limitations

The system remains under active development. Pairing is not authenticated, there is no OTA firmware path, and camera/image quality, radio coexistence, sensor capacity, and performance require validation on the installed layout. Lighting, shadows, reflections, focus, vibration, and occlusion can still alter gradient evidence. Test representative rolling stock at operating speed before relying on an output.
