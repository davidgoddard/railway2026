# Camera capture and detection optimisation handover

Updated: 2026-09-29. Current experimental firmware: **0.2.35**.

## Objective and current position

Increase **sustained updates per sensor per second**, preserving image detail, sensor placement and reliable occupancy decisions. The user has an **AI Thinker ESP32-CAM with an OV2640**, using **640×480 grayscale**. Do not assume that a faster CPU-side detector, a higher advertised video frame rate, or fewer output pixels automatically improves delivered detection FPS.

The working clock setting is **divide by 4**, giving approximately **2.7–3.1 FPS** in the supplied logs. Divide by 2 produced repeated incomplete frames and was reverted. Direct driver-buffer ownership and pipeline timings are implemented.

**0.2.32 contains an automatically sized crop benchmark, not automatic cropped occupancy detection.** It computes an enclosing window from the deployed sensors, tests actual camera cropping, and restores full-frame operation. Hardware crop timing, pixel alignment, scaling and exposure effects are not yet verified. This is the next required board experiment before cropped pixels can safely be associated with existing sensor geometry and baselines. Version 0.2.33 additionally makes the OV2640 clock override capability-based: it is available at every configured resolution and on either supported board map when the detected sensor is an OV2640. Version 0.2.34 added a resolution-independent 30 MHz OV5640 PCLK trial on ESP32-S3, measured at approximately 6 FPS on the supplied SVGA setup. Version 0.2.35 raises that trial to 40 MHz. Only the original AI Thinker/VGA OV2640 divide-by-4 and S3/OV5640 30 MHz results have hardware measurements so far.

No firmware has been flashed by the agent. All hardware observations below came from the user. Changes in this working tree span the earlier capture experiments; do not discard them as unrelated edits.

## Evidence from the physical camera

| Firmware / experiment | Observations | Conclusion |
| --- | --- | --- |
| Before 0.2.27 | User reported about 1.6 FPS. | Initial baseline. |
| 0.2.27, direct driver buffers | Still about 1.6 FPS. | Eliminating the application copy did not produce a measurable speed gain in this configuration. |
| 0.2.28, timing diagnostics | VGA, two sensors, 1.5–1.6 FPS; typical foreground capture wait 627,541–628,346 µs; analysis 7,328–8,121 µs; shared gradients about 1,844 µs; no fallback frames or acquisition failures. | Frame delivery dominates. Detector optimisation cannot fix this particular FPS limit. |
| 0.2.29, divide by 4 | Normal detection 2.9–3.1 FPS with two sensors. Typical wait about 301–323 ms; analysis about 15–17 ms in that scene. Following recalibration to one sensor, clear-frame analysis about 1.1 ms, typical wait about 317 ms. Some windows were 2.7 FPS. | Approximately doubled throughput at unchanged resolution. Scene/configuration changed between samples: do not attribute changes in analysis cost solely to the clock. |
| 0.2.29 long waits | Occasional maximum waits about 622–639 ms, roughly two ordinary intervals; application acquisition failures remained zero. | Reason for occasional missed/delayed deliveries is unproven. Investigate below the application counter. |
| 0.2.30, divide by 2 | Repeated `cam_hal: FB-SIZE: 305280 != 307200`. | Frames were 1,920 bytes short. This is equivalent in size to three VGA grayscale rows, but does not locate missing data or establish the DMA/timing cause. Reject this operating point on the tested setup. |
| 0.2.31 | Restored divide by 4 as the default. | Build and host tests passed; no separate post-revert hardware log was supplied before crop work. |
| 0.2.32 | Crop planner, isolated benchmark and paired image export. | Host-tested and firmware-compiled; crop hardware results pending. |

The original supplied log contained snapshots, configuration changes and calibration. Exclude those reporting intervals when comparing steady detection FPS. Repeated clear/occupied events in a log are not by themselves proof of either correct detection or false triggers without knowing what was on the track.

## Existing pipeline and completed changes

Normal operation is:

`OV2640 → camera driver grayscale framebuffer → ImageSource lease → OccupancyDetector → compact radio states`

The image is normally processed locally. There is no normal image upload, JPEG decode, PNG conversion or image write to flash to remove. Snapshot transfer and baseline/calibration are separate operations.

### Direct frame ownership (0.2.27)

Previously, one driver framebuffer was copied into one of two application PSRAM buffers for every frame. `ImageSource.h` now uses **two driver framebuffers** with `CAMERA_GRAB_WHEN_EMPTY`, one leased to the consumer and one pending successor. The application-level full-frame `memcpy` and the two application image allocations were removed. Driver-internal DMA copies/conversion may still occur.

- The current lease remains valid through processing, capture timeout and pause.
- A successful next capture returns the previous lease to the driver.
- Stop the capture task and return both leases **before** driver deinitialisation.
- Capture and analysis still overlap; a single held driver buffer would risk serialising them.
- Baseline averaging reads its temporary mean image directly rather than copying that mean into driver-owned memory.
- Grayscale multi-buffer capture remains a hardware-dependent experiment. [Espressif's driver guidance](https://github.com/espressif/esp32-camera#important-to-remember) recommends multiple buffers for JPEG; compilation is not proof of reliable grayscale operation on every board/resolution.

### Timing diagnostics (0.2.28)

`pipeline` reports acquisition/analysis FPS, application frame age, failures and PSRAM. `pipeline_timing` adds:

| Field | Meaning |
| --- | --- |
| `capture_wait_us` | Mean foreground wait for a published frame per main-loop capture attempt, including failed attempts. Not sensor exposure time. |
| `analysis_us` | Mean total analysis time per analysed main-loop frame, including frame binding. |
| `prepare_us` | Work-map preparation time, a subset of analysis. |
| `shared_us` | Shared gradient feature extraction time, a subset of analysis. |
| `shift_us` | Extra one-pixel tolerance comparisons, a subset of analysis. |
| `shifts` | Total shifted comparisons in the window, not per frame. |
| `fallback_frames` | Frames using per-cell fallback because work-map preparation failed. |
| `max_wait_us`, `max_analysis_us` | Worst foreground wait/analysis in that reporting window. |
| `attempts`, `frames`, `window_ms` | Denominators/context for interpreting averages. |

The detector receives an optional clock callback, keeping the shared detector independent of platform timer APIs. Timing calls are outside pixel loops. The first health report seeds FPS counters correctly instead of dividing startup counts by the next window.

The application UI estimates FPS from camera frame-number changes between health reports arriving at the computer. It is not a direct sensor-FPS measurement. Calibration captures can advance frame numbers without normal analysis. Capture acquisition overlaps processing, so do not sum background acquisition duration and analysis as though they were serial.

`frame_age_ms` starts at publication after driver acquisition, not at exposure start. `acquisition_failures` counts errors visible to `ImageSource`; it does **not** count all driver-discarded partial frames. The legacy replaced-frame counter stays zero for the bounded mailbox.

### Sensor clock experiments (0.2.29–0.2.31)

The original VGA trial used:

```cpp
#define CAMERA_OV2640_VGA_CLOCK_TRIAL 1
#define CAMERA_OV2640_VGA_CLOCK_DIVISOR 4
```

The override applies only to AI Thinker builds, detected OV2640, VGA, and an existing full CLKRC register value of 7. The sensor-bank register address for the driver's `get_reg`/`set_reg` API is **0x111**. Its six divider bits encode **divisor minus one**. The trial changes 7 to 3 (÷8 to ÷4), verifies readback and preserves other controls. External XCLK remains 20 MHz. Unexpected existing values are logged and left unchanged; write/readback failures abort initialization through the existing retry path.

Set `CAMERA_OV2640_VGA_CLOCK_TRIAL=0` to retain driver defaults. Explicit build flags override header defaults. Divisor 2 is retained only as an explicit experiment and is known to fail in the user's tested setup. Only divisor values 2 and 4 currently compile; **divide by 3 has been discussed but is not implemented or tested**.

Version 0.2.33 replaces that board/resolution gate with detected-sensor capability gating. `CAMERA_OV2640_CLOCK_OVERRIDE=1` and `CAMERA_OV2640_CLOCK_DIVISOR=4` now apply to an OV2640 at every supported resolution and on either board map. The implementation preserves non-divider CLKRC bits and accepts explicit divisors 1–64 for testing. Non-OV2640 sensors are logged and unchanged. The former VGA macro names remain build-flag aliases. This broader availability is not broader validation: each board, resolution and sensor-clock combination still requires complete-frame, image-quality, exposure and detection testing.

Version 0.2.34 detects OV5640 separately and, on ESP32-S3, calls the driver's `set_pll` API for a 30 MHz PCLK at every supported resolution. `CAMERA_OV5640_PLL_OVERRIDE=0` restores driver defaults; `CAMERA_OV5640_PCLK_MHZ` accepts 4–40 for controlled experiments. External XCLK remains 20 MHz. The affected registers are snapshotted and multiplier/divider readback is verified, with restoration and failed initialization if the trial cannot be applied. This preserves resolution as user configuration rather than inferring it from deployed sensor locations.

Version 0.2.35 changes the default OV5640 trial to 40 MHz after the user's SVGA module measured around 6 FPS at 30 MHz. Forty MHz is the documented ESP32-S3 ceiling and is not yet hardware-validated here. Revert to 30 MHz on incomplete frames, tearing, capture failures, unstable exposure or detection regressions.

`Arduino.h` now loads target macros before automatic board-map selection. Previously that selection occurred before the target definitions were loaded. Preserve this ordering so S3 selects its own map and stays outside the AI Thinker trial.

Relevant primary sources: [OV2640 clock/window implementation](https://github.com/espressif/esp32-camera/blob/master/sensors/ov2640.c), [clock bitfields](https://github.com/espressif/esp32-camera/blob/master/sensors/private_include/ov2640_settings.h), and [driver partial-frame rejection](https://github.com/espressif/esp32-camera/blob/master/driver/cam_hal.c).

## Automatic enclosing crop experiment (0.2.32)

### What is implemented

- `Camera_Module/CaptureWindow.h` computes the union of **all configured sensor radii**, including block sensors, plus a two-pixel margin: one for movement tolerance and one for the gradient stencil.
- It selects the smallest-area **eligible preset** containing that union at an aligned origin. It is not an arbitrary sparse selection of sensor pixels.
- Eligible output dimensions are 96×96, 128×128, 176×144, 240×176, 240×240, 320×240, 320×320, 480×320, and 640×480.
- Positions align to 16 VGA pixels. Presets/origins preserve the intended 5:4 mapping to the OV2640 VGA path's 800×600 native mode and its four-pixel DSP size-register units. Some standard driver presets, such as 160×120, are deliberately excluded because they do not preserve that ratio exactly through these size registers.
- No sensors, invalid geometry, or a union spanning the view falls back to the full frame. Four corner sensors will normally do so.
- Initial hardware scope is **AI Thinker/OV2640, VGA, no horizontal mirror or vertical flip**. Other settings retain normal full-frame operation.
- The driver is rebuilt using the matching output preset so buffer size and expected frame length agree. Programming a small sensor output while the driver still expects 307,200 bytes would simply create more `FB-SIZE` errors.
- `set_res_raw` is used with its OV2640-specific SVGA mode selector and native crop coordinates. Controls and the existing successful ÷4 clock are reapplied. Three initial frames are drained before the normal producer starts.
- Each benchmark phase has three further warm-up frames and 16 measured captures. The full phase runs before the crop phase at the same current settings. Reported timings exclude camera rebuild and binary export time, but include foreground heartbeat servicing during the measured phase.
- Full-frame and crop images can be exported as a pair. The crop is placed on a black 640×480 canvas at its **expected** original position to make alignment errors visible. This placement does not prove the hardware mapping is correct.
- The benchmark marks outputs unknown, signals a suspended image operation through the existing health flag, and restores full-frame capture on completion or failure. It does not feed crop images into the detector, persist geometry changes, or replace calibration/baselines.

### Why this is not enabled for normal detection yet

OV2640 window registers are in the sensor's DSP path. Smaller output is not proof of a shorter sensor exposure/readout cycle. Driver `set_res_raw` also has sensor-specific semantics, not a universal crop API. The intended native/output mapping needs verification against photographs of the real layout. Cropping can affect exposure and scaling/filtering even if output dimensions are correct.

Using unverified crop coordinates with saved full-frame baselines could make a sensor observe the wrong track. The experiment establishes timing and alignment first; it deliberately returns to the existing detection pipeline afterward. Completing automatic **monitoring** integration requires the board observations listed below, not another permission prompt.

### Running it on the board

Upload 0.2.32. Keep the camera at VGA with existing deployed sensors and the default ÷4 clock. Keep the scene stationary for the comparison. Do not start calibration or snapshot transfer simultaneously.

For timing only, open the **camera's** USB serial monitor at 921600 baud, newline ending, and send:

```text
O
```

Look for:

```text
crop experiment begin x=... y=... width=... height=...
crop_benchmark mode=full ... fps=...
crop_benchmark mode=crop ... fps=...
crop_alignment ... mean_absolute_difference=...
crop experiment complete: full-frame monitoring restored; crop NOT used for detection
```

If no smaller eligible window fits, the command says so and leaves full-frame operation untouched. Unsupported flips/resolutions/boards are likewise rejected before reconfiguration. On recovery failure it leaves monitoring unavailable and asks for a camera restart.

For timing plus comparison images, close the serial monitor first. From `Bridge_Controller`, with its existing Node dependencies installed:

```sh
node tools/capture-camera-frame.js /dev/cu.usbserial-REPLACE crop-check.png --crop
```

Replace the device path. This sends `O FRAME` and saves:

- `crop-check-full.png`: last full-frame sample;
- `crop-check-crop.png`: camera-cropped pixels placed at their expected position on a black full-size canvas;
- `crop-check.log`: text diagnostics, including the chosen coordinates and timing.

The ordinary command without `--crop` still sends `F` and saves one full frame. Do not send `O FRAME` into a text terminal: it emits binary image data. At 921600 baud, exporting both uncompressed VGA canvases still takes several seconds; occupancy is suspended and the bridge may temporarily time out while binary output prevents interleaved heartbeat logging. The capture utility waits for confirmation that full-frame monitoring was restored, not merely for the second image. The simple `O` timing test is much shorter.

### Required next observations and integration work

1. Compare crop and full FPS at the same ÷4 clock. Record valid frame delivery and driver errors, not only attempted captures or compressed transfer speed.
2. Compare rails and other features at the **same coordinates and pixel scale** in both images. Mean absolute difference is diagnostic only: it mixes alignment, exposure, noise, and scene changes. It is not an automatic acceptance criterion.
3. Test a single sensor off centre, clustered sensors, sensors near each edge, and four corner sensors. Check that the two-pixel margin is present.
4. Verify repeated crop experiments, failures and return to full-frame snapshots/detection on the real board. Record any exposure settling or image corruption.
5. If timing and mapping pass, add a frame view with actual stride and crop origin to the detector. Keep UI/protocol sensor coordinates in the original full-frame coordinate system. Current detector indexing and calibration buffers still assume full frames.
6. Reconcile baseline capture, auto-sizing's maximum trial radii, lighting re-tune, saved baseline compatibility, and work-map cache invalidation with the crop. Never calibrate one view and silently analyse a different pixel mapping.
7. Recompute the enclosing window only when geometry/settings change, not for every frame. Retain full-frame fallback and a diagnostic disable switch.
8. Restore full view for app/USB snapshots and return to the chosen crop after transfer completion, abort or timeout. Handle stale queued frames and exposure settling at every switch. Avoid resetting the app's frame-number/FPS estimate on every view switch; current cold rebuilds reset the mailbox sequence.
9. Add mirror/flip transformations and other resolutions/boards only with verified mappings. Do not extrapolate this VGA experiment to arbitrary cameras.

## Other optimisation candidates

| Candidate | Expected value / dependency | Status |
| --- | --- | --- |
| Intermediate ÷3 sensor divider | Potential compromise between reliable ÷4 and failed ÷2 at the same resolution. Verify clocks, image integrity and exposure. | Discussed, not implemented; current macro validation accepts only 2/4. |
| Driver scheduling, DMA and pixel clock | Investigate intermittent doubled frame intervals and the exact ÷2 failure mechanism. Sensor clock and DVP output clock are distinct. Avoid arbitrary register changes without measurement. | Open. |
| Lower capture resolution | Could materially improve capture rate, but changes image detail and sensor coordinates/radii. Requires deliberate rescaling/recalibration. | Not applied. |
| Automatic enclosing rectangle | Good when sensors cluster; four corners may require the entire frame. Smaller rectangles need not yield proportional FPS gains. | Capture experiment implemented; monitoring integration pending hardware evidence. |
| Separate crop for each sensor or groups | Each window needs a fresh capture and reconfiguration/settling. Measure complete revisit time per sensor, not cropped frames/second. Nearby sensors should share a window. | Not implemented. |
| JPEG capture and decode | Driver JPEG path has different throughput, but this detector needs grayscale pixels. Include decode cost, memory, latency and compression artefacts in any comparison. | Not implemented. |
| Explicit work-map invalidation | `bind()` and `prepareWorkMap()` hash configuration repeatedly. Replace with reliable invalidation on every relevant change; geometry-only changes should control geometry maps. | Not implemented. Tiny at two sensors. |
| Cache baseline comparison data | Dominant directions, reference totals and cutoffs need rebuilding only when their inputs change. Preserve exact tie/order semantics. | Not implemented. |
| Combine live gradient passes | Fallback and shifted analyses currently compute gradients twice even though the live cutoff comes from the baseline. Retain max-gradient and early-return semantics; baseline calculation still needs its own cutoff discovery. | Not implemented. |
| Reuse shifted/overlapping gradient calculations | Could reduce expensive tolerance work with many or triggering sensors. More cache/storage complexity and PSRAM traffic. | Not implemented. |
| Remove movement tolerance | Would save processing but changes false-trigger resistance. At the observed capture limit, little throughput benefit. | Do not remove as a performance shortcut. |
| Diagnostics/score traffic | Reduce serial logging or optional score reporting after diagnosis. The image path still dominates. Keep compact state reporting. | Existing optional scores; no further change. |
| Snapshot decoupling | Keep a stable snapshot buffer while detection resumes. Trades memory/PSRAM/radio contention for fewer monitoring interruptions. | Not implemented. Current transfers pause monitoring. |
| Setup workflow simplification | Auto-calibration already combines radius/threshold tuning and baseline creation. Avoid redundant baseline runs afterward unless needed. This affects setup duration, not normal FPS. | Separate from current capture work. |
| Remove unused counters | `analysisNumber_` and legacy replaced-frame diagnostics are cleanup candidates. Preserve consumers of log formats. | Not implemented; negligible speed benefit. |

## Existing optimisations worth preserving

- Native grayscale input to the detector; no application colour conversion.
- Shared sparse work map: overlapping sensors reuse one gradient calculation per unique pixel.
- Integer Scharr calculation using eight contributing pixels; no general matrix convolution.
- Fixed-point direction projections with trigonometric coefficients prepared once.
- Early gradient-cutoff rejection and movement-tolerance comparisons only when needed.
- Reused detector work buffers, compact state bitmap and optional batched scores.
- Capture/analysis overlap and explicit buffer ownership.
- No universal dark/bright pixel rule masquerading as camera health; detector behaviour is structure-based.

Do not remove frame validity checks, unknown-state handling, baseline compatibility, averaging, tolerance or occupancy persistence to make a benchmark look faster. Faster FPS shortens the real-time duration of frame-count persistence settings even when those counts are unchanged.

## Code and validation map

| File | Responsibility |
| --- | --- |
| `Camera_Module/ImageSource.h` | Driver buffer ownership, board capture settings, clock override, experimental crop setup and lease lifecycle. |
| `Camera_Module/CaptureWindow.h` | Hardware-independent VGA crop planner, aligned preset selection and validation. |
| `Camera_Module/Camera_Module.ino` | Firmware version, pipeline timing logs, calibration/snapshots, USB `O` / `O FRAME` benchmark, suspension/recovery. |
| `Camera_Module/OccupancyDetector.h` | Detector and optional timing instrumentation; normal full-frame indexing remains. |
| `Bridge_Controller/tools/capture-camera-frame.js` | Single or paired USB image capture and PNG/log output. |
| `Bridge_Controller/tools/camera-frame-stream.js` | Fragment-tolerant binary frame parser with CRC and size validation. |
| `tests/camera/` | Host model of driver/RTOS ownership and crop planner tests. |
| `Camera_Module/README.md` | Firmware usage and versioned experiment notes. |

Local toolchain: Arduino CLI bundled at `/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli`; installed Arduino-ESP32 **3.3.8**. Builds use temporary directories, not tracked binaries.

Build from the repository root:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32cam --build-path /tmp/railway-direct-esp32 Camera_Module
arduino-cli compile --fqbn esp32:esp32:esp32s3:PSRAM=opi --build-path /tmp/railway-direct-s3 Camera_Module
```

Use the bundled CLI's absolute path if `arduino-cli` is not on PATH. No source changes or clock experiments were applied to an external Arduino library installation.

Host checks from the repository root:

```sh
c++ -std=c++17 -Wall -Wextra -Werror tests/camera/capture-window.test.cpp -o /tmp/railway-capture-window-test
/tmp/railway-capture-window-test
c++ -std=c++17 -pthread -Wall -Wextra -Werror -I tests/camera/stubs tests/camera/image-source.test.cpp -o /tmp/railway-image-source-test
/tmp/railway-image-source-test
c++ -std=c++17 -pthread -Wall -Wextra -Werror -I tests/camera/stubs tests/camera/crop-source.test.cpp -o /tmp/railway-crop-source-test
/tmp/railway-crop-source-test
c++ -std=c++17 -pthread -Wall -Wextra -Werror -DCAMERA_BOARD_ESP32S3_EYE=1 -I tests/camera/stubs tests/camera/crop-source.test.cpp -o /tmp/railway-crop-s3-test
/tmp/railway-crop-s3-test
node --test Bridge_Controller/tools/camera-frame-stream.test.js
node --check Bridge_Controller/tools/capture-camera-frame.js
git diff --check
```

Ownership tests also support `-DCAMERA_OV2640_VGA_CLOCK_DIVISOR=2` or `-DCAMERA_OV2640_VGA_CLOCK_TRIAL=0`. Host checks cover returned-buffer poisoning, pause/timeouts, invalid frames, initialization failures, sensor/board gating, clock readback failures, crop setup failures and full-frame recovery. The crop planner checks 10,000 deterministic layouts against an independent exhaustive placement test, including margins and four-corner fallback. Stream tests cover fragmented paired images, existing single-frame compatibility, CRC failures and invalid headers.

These tests do **not** simulate camera DMA bandwidth, the optical/DSP crop mapping, exposure, radio contention, or real board FPS. The next agent should start with the user's 0.2.32 crop benchmark log and paired images, not assume that cropped detection is already deployed.
