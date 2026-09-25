# UI redesign roadmap

> Status: waves 1–7 are implemented. This file is retained as a delivery record;
> the functional specification and component guides describe the current UI.

The redesign was delivered in small, working waves. Each wave preserved the desktop
and browser applications, kept their shared behaviour aligned, and passed the existing
automated tests before it was committed. Applicable UI changes were made in both
`Bridge_Controller/app` and `Web_App` in the same wave.

## Wave 1 — Shared readiness model and camera status

- [x] Define a pure `CameraReadiness` model for connection, frame, configuration,
  deployment, calibration, testing, faults, and next action.
- [x] Add unit coverage for the normal camera lifecycle and regression states.
- [x] Show configuration, calibration, test status, and next action in the camera
  workspace.
- [x] Persist detector-test results against the camera configuration revision.
- [x] Mark a saved test stale whenever the deployed configuration changes.

## Wave 2 — Overview and task-based navigation

- [x] Add an Overview landing page showing bridge, Wi-Fi, MQTT, and every camera.
- [x] Give each camera one prominent action derived from `CameraReadiness`.
- [x] Rename navigation to Overview, Cameras, Monitor, and System.
- [x] Move network and diagnostic material under System without removing features.
- [x] Make offline and cached-camera states explicit.

## Wave 3 — Persistent camera workspace

- [x] Keep the camera image visible while inspecting or editing sensors and blocks.
- [x] Replace Setup/Live USB/Live MQTT/Test mode selection with an independent live
  overlay switch, source selector, and Test detector action.
- [x] Distinguish draft geometry from the deployed sensors supplying live state.
- [x] Add a Save & deploy area with an unsaved-change count and discard action.
- [x] Make camera settings the inspector content when nothing is selected.

## Wave 4 — Output-first configuration

- [x] Replace the flat sensor list with an output-first expandable tree.
- [x] Show a block as one output and reveal its internal sensors on demand.
- [x] Select and highlight an output, block sensor, or image sensor consistently.
- [x] Add compact filters only when configuration size warrants them (deferred until lists are large enough to need them).
- [x] Put IDs, geometry, texture analysis, and other specialist fields under Advanced.

## Wave 5 — Guided calibration and testing

- [x] Make `Calibrate empty track` the primary calibration action.
- [x] Automatically choose newly added sensors for the normal calibration scope.
- [x] Put recalibrate-all and baseline-only actions in an overflow menu.
- [x] Turn traversal testing into a guided panel that retains the camera image.
- [x] Record transitions, peaks, intermittent sensors, suspects, and completion state.
- [x] Surface lighting retune as a contextual recovery action for a firing sensor.

## Wave 6 — Operational monitor

- [x] Group railway outputs into Occupied, Unknown/attention, and Clear.
- [x] Show output names prominently and implementation topics secondarily.
- [x] Keep recent transitions and selected-camera chimes together.
- [x] Remove setup controls and internal sensors from the normal monitor view.
- [x] Preserve MQTT-only monitoring when USB is disconnected.

## Wave 7 — System, accessibility, and finish

- [x] Present bridge, radio, Wi-Fi, MQTT, firmware, and camera health as a summary.
- [x] Collapse raw diagnostics and histories until requested.
- [x] Audit contrast, focus states, keyboard navigation, and status announcements.
- [x] Test narrow-window and large-layout behaviour.
- [x] Update user documentation for the completed workflow; replace screenshots after the next hardware-connected capture session.
