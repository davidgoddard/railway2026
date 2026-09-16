# Railway camera occupancy toolkit — functional specification (draft)

## Goal

Provide configurable virtual sensors and blocks for a model railway without modifying rails or rolling stock. A camera module detects changes in assigned areas of its own image that indicate rolling stock may be present. The controller publishes each area's resulting state to MQTT for layout automation.

The system consists of an Electron setup and monitoring application, one ESP32-S3 controller, and ESP32-S3 camera modules. The application is needed for setup and diagnosis; normal monitoring continues without the computer. **Each camera captures and processes its frames locally. ESP-NOW carries compact state, configuration, health, and acknowledgement messages, not continuous images.** The controller bridges camera states to MQTT over Wi-Fi.

The initial scope is presence detection for configured areas. Train identity, vehicle counts, direction of travel, and complete layout tracking are future possibilities, not requirements for the first release.

## Terms and state model

- **Camera view:** a fixed image from one mounted camera at a configured resolution and orientation.
- **Region of interest (ROI):** a polygon drawn in a camera view, with a stable ID, display name, and type (sensor or block).
- **Sensor:** a small ROI for detecting a train at a point. **Block:** a larger ROI covering a track section. Both use the same state model.
- **Baseline:** an empty-track description captured during calibration and tied to ROI geometry and camera settings.
- **Change:** enough usable parts of an ROI differ from its baseline. Change is evidence of occupation but can also result from hands, shadows, lighting, or camera movement.
- **State:** `clear`, `occupied`, or `unknown`. Unknown means the system cannot make a trustworthy assertion and must never be interpreted as clear.

A sustained qualifying change makes an ROI occupied. A sustained return to the calibrated empty appearance makes it clear. Invalid or stale evidence makes it unknown. A single frame must not directly toggle a published state.

## Component responsibilities

| Component | Responsibilities |
| --- | --- |
| Electron application | Pair devices; store and back up each camera's full-frame empty-layout snapshot; show setup and diagnostic views; draw and edit ROIs; calibrate; tune thresholds; show live states and faults; deploy configuration. |
| Camera ESP32-S3 | Capture frames; derive and persist compact baseline features for assigned cells; process its ROIs locally; filter state over time; persist assigned configuration; send state changes and periodic health. |
| Controller ESP32-S3 | Persist the deployed configuration; distribute camera settings; validate ESP-NOW reports; track freshness; publish MQTT state and health without the application. |
| MQTT broker | Distribute current state and health to external automation clients. |

The preview and initial provisioning transport is a separate setup path. It must permit images without making continuous images part of the ESP-NOW state path. The first camera firmware tests on-demand full-frame grayscale snapshots over ESP-NOW, with chunk acknowledgements and monitoring paused during transfer. The controller must mark observations stale during that interval. A faster setup transport remains a future option. Closing previews must have no effect on normal monitoring after transfer ends.

## Setup and normal network modes (proposed)

| Mode | Controller | Camera modules | Application |
| --- | --- | --- | --- |
| Setup | Offers a temporary Wi-Fi hotspot and a provisioning service. Can configure the home Wi-Fi and MQTT connection. | A camera being configured joins the temporary hotspot and serves images and diagnostics over IP. | The laptop joins the temporary hotspot to view and configure cameras. Image transfer is allowed at a modest rate. |
| Normal | Joins the user's home Wi-Fi as a station for MQTT; receives compact camera reports over ESP-NOW. | Captures and processes frames locally. Sends only state and health messages over ESP-NOW; does not need home Wi-Fi credentials or an IP connection. | Can close or return to the home Wi-Fi; the controller and cameras continue without it. |
| Recovery | Reports missing cameras and marks their ROIs unknown. Provides an explicit way to re-enter setup. | Keeps its saved identity, keys, and configuration; searches for the paired controller if the radio channel has changed. | Shows which cameras have rejoined and their configuration versions. |

The application may fetch a camera preview directly over the controller's hotspot rather than relaying image bytes through the controller application protocol. The first setup implementation may configure one camera at a time to keep hotspot capacity and traffic bounded.

ESP-NOW is connectionless and uses a Wi-Fi channel rather than a home-network SSID. Once the controller is connected to a home access point, its ESP-NOW channel must match that access point's channel. Provision cameras with controller identity, pairing keys, and the last known channel. If contact is lost after a router or network change, cameras should search permitted channels for an authenticated controller announcement, then resume normal reporting. The controller must mark their ROIs unknown during the gap. Channel discovery and reconnection time need hardware tests before an availability target is set.

A wired state link remains an option. I²C has sufficient bandwidth for compact state messages, but bus length, pull-ups, capacitance, addressing, and fault isolation must be tested across the physical layout. If long cable runs are required, evaluate a bus designed for distributed wiring before choosing I²C. The camera algorithm and state-message format should not depend on the physical link.

## Configuration workflow

1. Pair each camera explicitly and assign a stable device ID. Reports from unpaired devices cannot alter states.
2. Display a representative camera image with its resolution and orientation. Draw polygonal ROIs around visible track and assign unique IDs and names. Warn about out-of-frame regions; label low-texture cells as blank backgrounds rather than rejecting them. Permit intentional overlap.
3. With the whole visible layout empty, capture a full-frame baseline snapshot for each camera and save it in the setup application's project data. Record camera identity, image dimensions, camera settings, image checksum, and revision. Derive each ROI's cell features from that snapshot and persist the compact results on the camera with its ROI geometry and cell layout. Require a new full-frame snapshot after camera movement, resolution or relevant exposure changes.
4. Show the live change score, state, and recent transitions for threshold tuning. Apply versioned configuration atomically. The controller shows which version each camera has acknowledged.
5. Keep the controller's configuration and each camera's assignment across power loss. Reject unsupported or oversized configurations with a reason, leaving the last valid version active.
6. At startup, each ROI is unknown until the camera has valid configuration and enough fresh frames for a decision.

The setup application's full-frame snapshot must survive power loss and remain available for months, including through an export/import or backup workflow so it can be moved to another computer. Adding or changing a sensor or block later derives its baseline from the original snapshot without requiring the layout to be empty again; removing an ROI does not delete the snapshot. The application must show the snapshot and revision so the user can confirm that a newly selected area was empty when it was taken. It verifies the camera identity, image geometry, and relevant settings before use. If the camera was moved or its image geometry changed, the user must capture a new empty-layout snapshot.

During setup, the application sends the saved snapshot, or the necessary lossless pixel regions with their gradient borders, to the camera over a setup transport. The camera uses the same feature extractor as normal calibration, returns the new cell features for review, and atomically saves its compact configuration and feature data. The camera and controller continue to operate with the application closed. Replacing the snapshot is an explicit operation that recalculates affected ROIs and keeps the prior working configuration if deployment fails. The first firmware can send a current grayscale frame to the bridge over chunked ESP-NOW on request; receiving a historical application snapshot for later cell additions is not yet implemented.

Full-frame persistent storage belongs to the application's project data; cameras only need enough flash for paired identity, configuration, and derived cell features. The ESP32 controller need not store copies of every camera image. The application must report a missing or corrupt snapshot and must not silently substitute a newer occupied view as the empty baseline.

## Local detection

The proposed first algorithm converts captured data to luminance, divides each ROI into small cells, and distinguishes blank from textured backgrounds. A blank cell is a valid reference: the appearance of significant texture is a change. Where texture is present, local edge or gradient orientations describe rails, sleepers, ballast, or scenery. The ROI score combines the proportion and spread of changed cells. A cell must become unknown only when the image evidence itself is unavailable or unreliable, not merely because the empty background lacks edges.

Production calibration should observe several empty frames per cell and record normal variation in gradient activity. This is especially important for blank cells, where sensor noise or automatic exposure changes could otherwise look like newly arrived texture. The current experiment captures one reference frame and uses a configurable contrast floor plus temporal persistence.

The feature extractor, cell size, aggregation, and thresholds are prototype choices to benchmark on the selected ESP32-S3 board and image sensor. Orientation features may tolerate moderate brightness changes, but cannot make detection lighting independent. Broad scene shifts, severe blur, saturation, and capture failure must lead to unknown when they prevent a reliable comparison. The system must not learn a stationary train into the empty baseline automatically.

Rapid sunlight changes can briefly saturate the camera while automatic exposure settles, making a textured empty cell appear blank. Track near-white pixel fraction, brightness change, and loss of reference detail. If evidence is washed out, report the affected cell or camera view as unknown and suspend enter/clear persistence until fresh usable frames arrive; do not turn missing gradients directly into occupied or clear. Distinguish whole-view lighting disruption from local objects where possible. A pale vehicle can resemble overexposure, so uncertain evidence must remain unknown rather than be silently accepted as empty track. Qualify the thresholds using both sunlight transitions and light-coloured rolling stock.

The first hardware experiment is [the Arduino camera angle sketch](../experiments/camera_module/README.md). It accepts blank or textured background references. At calibration it measures 18 gradient-angle bins and refines up to three dominant directions. For an oriented reference, live frames count strong gradients near each stored direction and in an "other" bucket without calculating an angle for each pixel; the change score compares those bucket shares. If the textured reference has no stable peak, it compares normalized angle distributions. It reports processing time and frame rate. The blank/texture cutoff, direction support, tolerance, and persistence periods are provisional and need empty-track and rolling-stock tests.

Angle proportions alone can miss a narrow object against a weakly patterned background: a carpet test produced a 0.240 angle mismatch while the previous event threshold was 0.30. The experiment now also compares the concentration of strong gradients with the empty baseline for weakly oriented cells, and uses a 0.20 default mismatch threshold with three-frame persistence. Qualify this combination against sudden lighting changes and rolling stock before adopting it in production.

At resolutions where sleepers cannot be resolved, a rail and a carriage can both produce long edges with similar angles. The camera experiment therefore stores up to two separated gradient points on the strongest baseline angle and checks whether a similarly oriented gradient remains near each point in live frames. A missing point contributes to the change score. This uses edge location and direction without comparing baseline and live pixel brightness. Evaluate false changes from small camera shifts and missed changes when a carriage edge happens to replace the rail edge at the same locations and angle.

Each ROI has distinct enter and exit thresholds and configurable persistence periods. Occupied requires a qualifying change across the enter period. Clear requires a sustained return to baseline across the exit period. Brief missing or low-confidence frames cannot clear an occupied area. Defaults will be set from layout tests.

## Line-drawn blocks (proposed)

The editor lets the user drag a line or polyline along a visible track section. It places separately calibrated cells along that path at a configurable spacing, with enough overlap that the monitored path has no gaps. The user can inspect, remove, or reposition individual cells and see their current states. A point sensor may consist of one cell; a block normally consists of many.

Each cell applies its own temporal filtering. The block becomes **occupied if any cell is occupied**. It is **clear only if every cell is clear**. If no cell is occupied and at least one cell is unknown, the block is **unknown**. The block publishes one state event; per-cell states remain available for setup and diagnosis. A single noisy cell can therefore affect the whole block, so calibration and false-trigger tests must be performed at block scale.

At normal operation the camera should evaluate a cell only once per fresh frame and reuse its result for every sensor or block that references it. Shape masks or scanline spans should be computed when configuration is deployed, not rebuilt for every frame. Capacity qualification must include the maximum number of cells across all blocks on a camera, not merely the number of MQTT topics.

The camera may skip or downsample pixels outside ROIs, provided all active ROIs are evaluated using fresh frames. Reports carry a frame timestamp or age and monotonically increasing sequence number.

## Failure and recovery behavior

| Situation | Required behavior |
| --- | --- |
| Sustained changed ROI | Transition to occupied and report promptly. |
| Sustained baseline-like ROI | Transition to clear and report promptly. |
| Startup, missing calibration, or config mismatch | Report unknown; never publish clear based only on saved state. |
| Capture or analysis failure | Report unknown after a short configured fault window and include a health reason. |
| Camera heartbeat expires | Controller publishes unknown for all that camera's ROIs. |
| Controller loses MQTT | Continue monitoring. Keep the latest state per ROI and republish a fresh snapshot on reconnect. |
| Controller restarts | Publish unknown until fresh, version-matched camera observations arrive. |
| ROI or camera removed | Retire its retained MQTT topics explicitly; define the exact removal policy before integration. |

Cameras send periodic health even without state changes. The controller validates identity, configuration version, sequence, and freshness. Duplicate or delayed reports cannot roll state backward. ESP-NOW send success alone does not prove application-level receipt; use acknowledgement or periodic full-state refresh so a missed event cannot leave a permanently wrong state.

## MQTT contract (proposed)

Use a configurable root such as `railway/<layout-id>`. Machine IDs remain stable if display names change. Example topics and payloads:

| Topic | Example payload |
| --- | --- |
| `railway/home/areas/block-1/state` | `occupied` |
| `railway/home/areas/block-1/health` | `{"camera":"cam-3","reason":"ok","age_ms":82}` |
| `railway/home/cameras/cam-3/health` | `{"status":"online","fps":10.4,"config_version":7}` |
| `railway/home/controller/health` | `online` |

State payloads are exactly clear, occupied, or unknown. Publish current states retained, with QoS 1 proposed; consumers must tolerate duplicate messages. Controller health uses an MQTT last will to become offline on an ungraceful disconnect. Retained state alone cannot prove current observation, so consumers also check controller health and state age. Republish all states after reconnect or configuration change. Store broker address, credentials, root topic, and layout ID on the controller.

## Performance and capacity guardrails

- **Frame rate:** each camera acquires and evaluates at least 10 fresh frames per second with ESP-NOW active, at a resolution where rails and sleepers in its ROIs are visibly distinguishable. The selected resolution and maximum ROI load must be demonstrated on the chosen board, not inferred from the sensor's maximum resolution.
- **State latency:** after the configured persistence period, 95% of qualifying transitions reach the broker within 500 ms of the decisive frame on a healthy local network. This is a provisional acceptance target pending measurements.
- **Overload:** report measured fps and overload status. Do not repeatedly assert clear from old frames.
- **Traffic:** normal operation sends state changes and low-rate health, not images. Preview traffic must be limited so it does not break the qualified monitoring performance.
- **Capacity:** publish tested limits for cameras per controller, ROIs and ROI area per camera, memory, radio range, and power. Do not promise 40 paired ESP-NOW cameras; Espressif documents a 20-peer unicast pairing limit and a lower configurable encrypted-peer limit.
- **Block geometry:** measured cell count and spacing must cover the drawn line. The editor must report the estimated per-frame cell work before deploying a line that exceeds the camera's frame-rate budget.

Prototype measurements must compare sensor output modes, grayscale acquisition, JPEG decode cost if used, ROI sampling, memory use, and frame rate with Wi-Fi/ESP-NOW active. High-resolution uncompressed capture can consume substantial memory bandwidth. Only qualify a processing path after it meets the frame rate and latency targets with the intended ROI load.

## Security and diagnostic guardrails

- Pairing requires a setup action. Ignore unsolicited reports and unauthorized configuration changes.
- Authenticate to the MQTT broker. Determine whether encrypted MQTT meets the board's measured performance budget.
- Keep Wi-Fi and MQTT secrets out of routine application views and diagnostic exports.
- Include protocol and configuration versions so incompatible firmware is visible.
- Show camera identity, last contact, applied configuration version, measured fps, and per-ROI state in the application.
- Provide a diagnostic mode showing change scores and transitions for calibration and fault finding.

## First-prototype acceptance scenarios

1. A calibrated empty ROI remains clear during representative normal lighting changes and passing shadows agreed for the test layout.
2. A locomotive and the smallest expected wagon or carriage each cause occupied in every ROI they cover, including while stopped.
3. An ROI clears only after the tested vehicle has fully left and the exit persistence period has elapsed.
4. Camera failure makes its ROIs unknown within the configured freshness timeout. Restart never briefly publishes a stale clear.
5. MQTT reconnection restores the latest valid snapshot while the Electron application is closed.
6. At the selected maximum ROI load, sustained testing records at least 10 evaluated frames per second and the qualified latency target, along with missed frames, false transitions, and free memory.
7. Lost, duplicate, and delayed ESP-NOW reports converge to the camera's current state through refresh or acknowledgement recovery.

## Open decisions and measurements

| Item | Needed |
| --- | --- |
| Camera hardware | Board, PSRAM, image sensor, lens, mounting distance, field of view, and power arrangement. |
| Image path | Resolution and pixel format that show rail/sleeper texture and sustain 10 fps with radio active. |
| ROI capacity | Maximum count, dimensions, and cell density per camera. |
| Detection tuning | Thresholds, persistence, baseline update policy, and behavior at turnouts, tunnels, reflections, and low light. |
| Preview/provisioning | How a factory-new camera securely joins the temporary hotspot (for example, a pairing button and one-time token), and how the application reaches its preview. |
| Radio topology | ESP-NOW and router Wi-Fi channel coordination, encrypted-peer capacity, channel search, and interference recovery. |
| Wired alternative | Whether the physical layout can support I²C cable length and fault isolation, or needs a more robust wired bus. |
| Reliability | Acceptable false clear/occupied rates, heartbeat interval, freshness timeout, and outage recovery time. |
| MQTT integration | Final topic schema, retained topic retirement, payload versioning, and security settings. |

## Hardware references

- [Espressif camera driver](https://github.com/espressif/esp32-camera/blob/master/README.md): ESP32-S3 support, PSRAM and frame buffer considerations, and RGB/YUV load with Wi-Fi.
- [Espressif ESP-NOW guide](https://docs.espressif.com/projects/esp-idf/en/release-v5.3/esp32s3/api-reference/network/esp_now.html): pairing limit and channel constraints.
- [Espressif ESP-NOW FAQ](https://docs.espressif.com/projects/esp-faq/en/latest/application-solution/esp-now.html): shared Wi-Fi channel and encrypted-peer limits.
- [Espressif Wi-Fi overview](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/wifi-driver/overview.html): simultaneous AP/station mode and precedence of the external access point's channel.
- [Espressif I²C guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/i2c.html): bus timing, pull-ups, and wire-capacitance considerations.
- [Espressif ESP-MQTT guide](https://docs.espressif.com/projects/esp-idf/en/v5.5.4/esp32s3/api-reference/protocols/mqtt.html): retained messages, QoS, and last will.
