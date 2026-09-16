# Railway camera occupancy toolkit

An experimental toolkit for detecting occupation on a **model railway** using cameras instead of track or rolling-stock modifications. A camera watches selected parts of the layout and compares each new frame with an empty-track reference. The intended output is a clear, occupied, or unknown state for each virtual sensor or block.

This repository is at the **specification and camera-algorithm experiment** stage. The working code is a single-camera Arduino sketch with a local web page. The controller, ESP-NOW messaging, MQTT publishing, multi-cell blocks, and setup application described below are planned work.

## Intended system

```text
Camera modules                         Controller                 Layout automation
ESP32-S3, local frame processing  →   ESP32-S3              →   MQTT broker and clients
compact state over ESP-NOW             state and health bridge

Setup application and temporary Wi-Fi hotspot: configuration, preview, diagnostics
```

Each camera will process images locally and report compact state and health messages over ESP-NOW. The controller will publish those states to MQTT. Camera images are intended for setup and diagnosis over Wi-Fi, rather than continuous transfer during normal monitoring. The [draft functional specification](Documentation/functional-specification.md) defines the proposed state model, network modes, failure handling, MQTT contract, and performance guardrails.

The target is **at least 10 fresh, analysed frames per second** at a resolution that makes rails and sleepers distinguishable, under the intended cell load and with the radio active. This is a target to test on hardware, not a measured capability of the current sketch.

## Why consider cameras?

Traditional model railway detection works well, but its cost and installation effort grow with the number of places monitored. The comparison below uses **indicative UK component prices checked in September 2026**, before postage, power supplies, wiring, interfaces, installation, and any rolling-stock modifications. Prices vary by supplier and board specification.

| Method | Indicative component cost | Installation and detection trade-off |
| --- | --- | --- |
| DCC track-current sensing | [About £30 for a four-block detector board](https://www.coastaldcc.co.uk/products/rr-cirkits/feedback/), plus sensing coils and feedback hardware | Detects current-drawing stock within electrically isolated track sections. Requires track wiring and suitable loads in stock to be detected. |
| Reed switch and magnet | [About £0.85 for a bare switch and £0.24–£0.36 per small magnet](https://www.railwayscenics.com/reed-switches-magnets-c-20_27_69.html) | Cheap point detection, but needs a switch and wiring at each location and a magnet on each vehicle that should trigger it. A passing pulse does not itself prove a whole block remains occupied. |
| Infrared sensor or break beam | [About £3.50 for a reflective sensor](https://thepihut.com/products/infrared-proximity-sensor) or [£5.20 for a break-beam pair](https://thepihut.com/products/ir-break-beam-sensor-5mm-leds) | Detects at a particular position without altering rolling stock; needs mounting, power, and wiring at each sensing point. Reflective readings depend on the target and environment. |
| ESP32 camera module | Modules are advertised at **around £5 each**; one [ESP32-S3 camera listing is £5.95](https://www.fruugo.co.uk/search?brand=Unbranded&merchantId=24938&pageSize=128&sorting=nameasc&whcat=3416) | One mounted camera could cover multiple virtual sensors and blocks without cutting rails or attaching magnets. It still needs power, a mount, a controller, and reliable sightlines and lighting. |

The camera approach may reduce hardware and wiring **per monitored area** when one view covers several locations. Its full installed cost and reliability are unproven: camera count, usable field of view, resolution, processing capacity, shadows, and occlusion must be measured on a real layout. Current sensing, reed switches, and infrared remain useful where their detection properties fit better.

## Current experiment

The [camera module experiment](experiments/camera_module/README.md) runs on an AI Thinker ESP32-CAM with PSRAM. It also includes an ESP32-S3-EYE pin map for later testing; other S3 camera boards need their own pin maps. The [Arduino sketch](experiments/camera_module/camera_module.ino) creates a Wi-Fi hotspot and serves a browser page where you can:

- Choose camera resolution and place one circular or square sensor cell on the preview.
- Capture an empty-background reference, including a cell with little or no texture.
- Compare live gradient-angle patterns with the reference and tune the contrast floor, angular tolerance, and mismatch threshold.
- Inspect frame rate, capture and processing times, and a circle-versus-square timing comparison.
- Review recent frame comparisons and download a CSV log containing settings, brightness, gradient counts, selected angles, histograms, scores, and timing. This helps investigate false detections caused by lighting changes.

The experiment uses one cell and does **not** yet send ESP-NOW or MQTT messages, persist settings, or implement drawn blocks. Its baseline is captured from one frame, and its classification thresholds still need testing on actual track and rolling stock.

### Try it

1. Install the Espressif ESP32 Arduino core, select the appropriate camera board, and enable PSRAM.
2. Open `experiments/camera_module/camera_module.ino` in Arduino IDE and upload it. The default pin map is for the AI Thinker ESP32-CAM.
3. Join the `Railway-Angle-Lab` Wi-Fi hotspot and open `http://192.168.4.1/`. The experiment's default password is documented in the [camera guide](experiments/camera_module/README.md); change it in the sketch before use near other people.
4. Select a resolution, position a cell, and capture a reference while the area is empty. Change what the cell sees, then inspect the comparison and log.

The sketch version appears at the top of the `.ino` file, on the web page, and in Serial Monitor, so an uploaded build can be identified. For hardware and upload details, calibration steps, algorithm behaviour, and known limitations, see the [camera experiment guide](experiments/camera_module/README.md).

## Repository layout

| Path | Contents |
| --- | --- |
| [`Documentation/functional-specification.md`](Documentation/functional-specification.md) | Draft goals, architecture, behaviour, guardrails, and open decisions. |
| [`experiments/camera_module/`](experiments/camera_module/) | Arduino camera sketch and experiment instructions. |

## Next steps

1. Collect logs from stationary empty track under changing daylight, artificial light, and camera exposure; tune false-change behaviour before treating the algorithm as reliable.
2. Test resolution, frame rate, PSRAM use, and cell count on the chosen ESP32-S3 camera hardware.
3. Add multiple cells along a drawn block, combine their states, and measure the resulting frame rate.
4. Implement configuration persistence, ESP-NOW state and health reporting, controller MQTT publishing, and the setup application according to the functional specification.

The specification deliberately marks unresolved design choices and acceptance scenarios. Results from the camera experiment should update it as the hardware and algorithm are validated.
