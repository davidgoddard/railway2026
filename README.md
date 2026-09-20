# Camera-based Model Railway Occupancy Detection

Monitor model railway occupancy using cameras, without modifying the track or rolling stock. Each camera compares selected areas of the layout with an empty-track baseline and reports **clear**, **occupied**, or **unknown** states for virtual sensors and blocks. A bridge publishes these states to an MQTT broker for use by railway control software such as JMRI.

## System overview

The system has three components:

- **Camera modules:** ESP32 cameras that process images locally and report sensor and block states wirelessly over ESP-NOW.
- **Bridge controller:** an ESP32-C3 module that connects the cameras to your Wi-Fi network and MQTT broker, and provides a USB connection for setup.
- **Setup application:** the [web app](https://davidgoddard.github.io/railway2026/) or the [desktop app](Bridge_Controller/README.md), used to configure cameras, define sensors and blocks, and view their states.

```text
Camera modules                  Bridge controller             Railway control software
Local image processing   →      ESP32-C3 SuperMini     →      MQTT broker and clients
                         ESP-NOW                       Wi-Fi
                                      ↑
                                     USB
                                      ↑
                               Setup application
```

Camera modules need power and communicate with the bridge without data cables. After configuration, the bridge also needs only power; the setup computer can be disconnected.

## Getting started

1. Install the [camera firmware](Camera_Module/README.md) and [bridge firmware](Bridge_Controller/README.md), following their hardware and upload instructions. Supported camera pin maps cover the AI Thinker ESP32-CAM and ESP32-S3-EYE; other ESP32-S3 camera boards require a matching pin map. ESP32-P4 is not currently supported.
2. Mount and power each camera so that the monitored track is clearly visible. Keep the camera fixed and provide consistent lighting.
3. Connect the bridge to your computer by USB. Open the [web setup app](https://davidgoddard.github.io/railway2026/) in desktop Chrome or Edge, choose its USB device, and connect to the bridge. The desktop app is also available; see the [bridge guide](Bridge_Controller/README.md).
4. Select each discovered camera, fetch a frame, and draw sensor areas over the track. Combine sensors into blocks where needed, then save the configuration.
5. With all monitored track clear, capture an empty-track baseline for each camera.
6. Configure the bridge's Wi-Fi and MQTT connection. Check sensor and block transitions with your rolling stock in the setup app and in your railway control software.
7. Leave the cameras and bridge powered for normal operation. Reconnect the setup app whenever you need to change settings or inspect the system.

Detection response time depends on the camera board, image resolution, sensor size and count, and configured persistence settings. Check performance with your layout and train speeds when positioning sensors. Fetching a setup snapshot pauses camera monitoring during the transfer.

## Why consider cameras?

Traditional model railway detection works well, but its cost and installation effort grow with the number of places monitored. The comparison below uses **indicative UK component prices checked in September 2026**, before postage, power supplies, wiring, interfaces, installation, and any rolling-stock modifications. Prices vary by supplier and board specification.

| Method | Indicative component cost | Installation and detection trade-off |
| --- | --- | --- |
| DCC track-current sensing | [About £30 for a four-block detector board](https://www.coastaldcc.co.uk/products/rr-cirkits/feedback/), plus sensing coils and feedback hardware | Detects current-drawing stock within electrically isolated track sections. Requires track wiring and suitable loads in stock to be detected. |
| Reed switch and magnet | [About £0.85 for a bare switch and £0.24–£0.36 per small magnet](https://www.railwayscenics.com/reed-switches-magnets-c-20_27_69.html) | Cheap point detection, but needs a switch and wiring at each location and a magnet on each vehicle that should trigger it. A passing pulse does not itself prove a whole block remains occupied. |
| Infrared sensor or break beam | [About £3.50 for a reflective sensor](https://thepihut.com/products/infrared-proximity-sensor) or [£5.20 for a break-beam pair](https://thepihut.com/products/ir-break-beam-sensor-5mm-leds) | Detects at a particular position without altering rolling stock; needs mounting, power, and wiring at each sensing point. Reflective readings depend on the target and environment. |
| ESP32 camera module | Modules are advertised at **around £5 each**; one [ESP32-S3 camera listing is £5.95](https://www.fruugo.co.uk/search?brand=Unbranded&merchantId=24938&pageSize=128&sorting=nameasc&whcat=3416) | One mounted camera can cover multiple virtual sensors and blocks without cutting rails or attaching magnets. It still needs power, a mount, a controller, and reliable sightlines and lighting. |

One camera can reduce hardware and wiring **per monitored area** when its view covers several locations. Plan camera coverage around sightlines, image detail, lighting, and processing capacity. Include power supplies, mounts, and the bridge when comparing installed costs. Current sensing, reed switches, and infrared are also useful where their detection properties suit the layout.

## How the camera firmware detects changes

The detector does **not** ask whether every live pixel is identical to the reference image. That would be too sensitive to normal camera noise, small exposure changes, and gradual lighting drift. Instead, each configured sensor area is reduced to a compact description of its **edge directions**: the proportions of strong brightness transitions at each angle. The baseline records up to three well-supported directions; a cell without stable peaks uses the full angle distribution.

A useful way to think about the process is:

```text
camera frame
   ↓
configured sensor area
   ↓
edge strength + edge direction
   ↓
compact baseline features  ← captured while the track is known to be clear
   ↓
compare each live frame with that baseline
   ↓
change score from 0 to 1000
   ↓
enter/clear persistence rules
   ↓
CLEAR or OCCUPIED
```

The algorithm is intentionally local. A train, hand, shadow, scenery change, or other disturbance only matters to a sensor if it changes the edge directions **inside that sensor's configured area**. It does not fit a continuous line or compare saved edge-pixel locations.

### 1. Analyse only the configured sensor area

Each sensor is defined by a centre point, a radius and a shape. The firmware supports circular and square cells. Although the camera captures a complete grayscale frame, the detector walks only the pixels that belong to the sensor being analysed.

![Configured circular and square sensor areas inside a camera frame](Documentation/assets/01_sensor_area_overview.svg)

This has two benefits. First, one camera can monitor several independent locations without treating the whole image as a single detector. Second, processing time is spent only on the parts of the frame that matter.

The edge calculation uses a 3×3 neighbourhood around each pixel, so the outermost image border is deliberately excluded. That guarantees that the algorithm can safely inspect the neighbouring pixels above, below and to either side of every analysed point.

> A sensor area is similar to a virtual electronic detector drawn on the camera image. Moving something elsewhere in the picture should not affect that sensor.

### 2. Convert brightness changes into edges

For every pixel inside a sensor, the firmware applies a small 3×3 **Sobel-style gradient** calculation. This estimates how quickly brightness changes horizontally and vertically around that point.

![Sobel-style gradient extraction and conversion to edge direction bins](Documentation/assets/02_gradient_and_orientation.svg)

The two gradient components are called `gx` and `gy`. The implementation uses their absolute values to form a simple edge-strength measure:

```text
edge strength = |gx| + |gy|
```

A flat piece of image, where neighbouring pixels have almost the same brightness, produces a small value. A rail edge, sleeper edge, wagon outline or other sharp transition produces a larger value.

Not every gradient is retained. The firmware calculates an edge cutoff using the larger of:

- the configured **contrast floor**; and
- one quarter of the reference area's maximum gradient.

For live frames, that second term comes from the **saved baseline**, rather than being recalculated from the live frame. This is important: a newly introduced bright object should not be allowed to raise the threshold that is being used to detect that same object.

For retained edges, the algorithm also measures direction from 0° to 180°. Opposite gradient signs represent the same physical edge direction, so an edge has an orientation rather than a one-way heading. When a full orientation histogram is needed, directions are accumulated into 18 bins of 10° each.

> The detector is interested less in the exact shade of a rail or sleeper and more in the pattern of strong lines and boundaries visible in the area.

### 3. Capture an empty-track baseline

A baseline is captured when the monitored area is in its known **clear** condition. The firmware analyses every configured sensor and stores compact measurements rather than retaining the complete reference image for normal comparison.

![Baseline calibration stores edge directions and normalized angle counts](Documentation/assets/03_baseline_calibration.svg)

For each sensor, the baseline includes measurements such as:

- number of sampled pixels and detected edges;
- maximum gradient strength, used to set the edge cutoff;
- mean brightness and the number of almost-white pixels;
- up to three dominant edge directions;
- the proportion of edges assigned to those directions, including an “other directions” bucket.

A sensor is treated as having useful texture once at least eight qualifying edges are found. If a clear reference contains strong repeated geometry—rails are a good example—the orientation histogram normally contains obvious peaks. The firmware searches for up to three sufficiently strong, sufficiently separated peaks and refines their angles using neighbouring histogram bins.

Once dominant directions have been found, the reference edges are projected into a small set of buckets: one bucket for each dominant direction plus a catch-all bucket for edges that do not fit them. This gives the live detector a compact description such as "most strong edges still run approximately along these rail directions" without requiring pixel-for-pixel alignment.

> The baseline is a fingerprint of the clear scene. It records the important structure of the image, not a photographic copy that must match exactly.

### 4. Build the same features for each live frame

Once a baseline exists, each new frame is analysed using the same sensor geometry and gradient cutoff rules. The comparison method depends on what was found in the reference:

- **Reference with dominant directions:** live edges are assigned to the saved direction buckets and the proportions are compared.
- **Textured reference without reliable dominant peaks:** the smoothed 18-bin orientation histograms are compared instead.
- **Very weak reference and very weak live image:** the area is treated as unchanged rather than manufacturing a large score from sparse data.
- **One image textured and the other not:** this is treated as a strong change unless both frames have fewer than 16 edges and the baseline has no stable peak.

Both the bucket comparison and histogram comparison are normalized by the number of edges. That matters because the detector is comparing the *distribution* of edge structure, rather than simply assuming that a frame with more edge pixels must be occupied.

### 5. Allow a one-pixel image shift

A small cell can change its apparent angle mix when the camera image moves by just one pixel. If the initial score could make a cell occupied, or keep an occupied cell from clearing, the detector checks up to eight neighbouring image positions. It keeps the lowest usable mismatch found and stops early if the score falls below the relevant threshold. The extra comparisons run only when needed, but may reduce frame rate if many cells regularly score high.

This is tolerance for small image movement, not a requirement that a particular edge pixel remain in place. A changed object with the same angle proportions as the empty scene can still be missed.

### 6. Convert the comparison into a 0–1000 change score

The normalized comparison produces a value between 0 and 1, which the firmware exposes as an integer score from **0 to 1000**:

```text
0      = closely matches the clear baseline
1000   = very strong difference from the baseline
```

The configured `thresholdPermille` decides how much difference is required before a frame counts as evidence of occupancy.

![Live angle comparison, one-pixel tolerance, and state transitions](Documentation/assets/04_live_change_decision.svg)

A score is therefore not a probability that a train is present. It is a **difference measure**: how unlike the saved clear condition the current sensor looks according to the available edge evidence.

### 7. Reject obviously unreliable exposure changes

A camera can temporarily produce a bad image because of glare, overexposure or a sudden lighting change. Treating such a frame as evidence of occupancy can create false transitions.

The camera supplies one grayscale value per pixel, rather than separate R, G, and B values. The firmware counts pixels at 250–255 across the **whole frame**. If at least 20% are near white, it treats that frame as overexposed. This is an absolute threshold, so a scene with a large naturally white area can also trigger it. There is no separate exposure-recovery timer: the first frame below this limit is analysed normally.

When a frame is judged overexposed, the detector reports **every sensor as unknown** rather than retaining a potentially stale clear or occupied result. It resets the consecutive-frame counters; useful visual evidence must then satisfy the configured count to establish a new state. The serial log prints the clipped-pixel count and percentage.

> "I cannot trust this image" is different from "the track is clear". Reporting unknown avoids presenting a stale occupied or clear state as a fresh observation.

### 8. Require repeated evidence before changing state

The raw score is deliberately separated from the final state. A single changed frame does not necessarily mean that a train has arrived; it could be noise, motion blur or a brief shadow. New sensors default to a 400/1000 mismatch threshold and one qualifying frame to occupy, prioritizing prompt detection. More frames can be configured to reject brief false triggers.

The detector therefore uses two persistence counters:

1. If the score is **at or above the configured threshold**, the `enter` counter advances. The sensor becomes `OCCUPIED` only after the configured number of consecutive enter frames.
2. If the score falls below **70% of the threshold**, the `clear` counter advances. The sensor becomes `CLEAR` only after the configured number of consecutive clear frames.
3. Scores between those two levels change neither state immediately. This creates hysteresis and prevents rapid toggling around the threshold.

At the new-sensor defaults, a score of at least 400 occupies the sensor on the first qualifying frame, while clearing requires scores below 280 for five consecutive frames. Existing saved sensors retain their configured threshold and frame counts until changed in the bridge app and saved.

This makes three settings conceptually different:

- **contrast floor** — how strong a local brightness transition must be before it can count as an edge;
- **change threshold** — how different the live edge structure must be from the clear baseline; and
- **enter/clear frame counts** — how long that evidence must persist before the reported state changes.

### 9. Combine sensors into blocks

The multi-cell firmware can associate several sensors with the same group or block. Individual sensors are analysed independently first. The group state is then derived from its members:

- if any member sensor is `OCCUPIED`, the group is `OCCUPIED`;
- otherwise, if any member is `UNKNOWN`, the group is `UNKNOWN`;
- otherwise the group is `CLEAR`.

The reported group score is the highest score among its member sensors. This makes a block conservative: one occupied member is enough to make the complete block occupied.

### Placement, lighting, and diagnostics

The detector uses railway geometry such as rails, sleepers, and vehicle outlines to identify changes while tolerating small grayscale variations. Each camera monitors several independent sensor areas.

Reliable detection depends on clear sightlines, stable camera mounting, suitable sensor placement, and consistent lighting. Shadows, reflections, camera or scenery movement, occlusion, and low-texture areas can affect results. Check both clear and occupied states with the rolling stock and lighting used on your layout, and adjust thresholds and persistence settings as needed. Recapture the baseline after changing the camera view or sensor configuration.

With debug output enabled, the camera reports baseline peak angles over USB serial. On a state transition, it reports the cell ID, change score, edge counts, and strongest gradients. The setup app displays live sensor states over the most recently fetched still frame; the image itself is not a live video feed. Use these diagnostics to tune sensor placement and thresholds.

The [camera guide](Camera_Module/README.md) describes configuration, baseline storage, and detector behaviour. The [bridge guide](Bridge_Controller/README.md) covers MQTT, setup controls, connection history, and troubleshooting.

## Reference documentation

| Path | Contents |
| --- | --- |
| [`Camera_Module/`](Camera_Module/) | Camera firmware, supported hardware, installation, and detector configuration. |
| [`Bridge_Controller/`](Bridge_Controller/) | Bridge firmware, desktop setup application, MQTT configuration, and troubleshooting. |
| [`Web_App/`](Web_App/) | Browser setup application and browser-specific capabilities and limitations. |
| [`Documentation/functional-specification.md`](Documentation/functional-specification.md) | Design reference, including proposed behaviour and open design decisions. |
| [`experiments/camera_module/`](experiments/camera_module/) | Separate development tool for inspecting angle histograms and timing; its scoring rules may differ from the camera firmware. |
