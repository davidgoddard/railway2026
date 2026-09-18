# Railway ESP32 bridge controller

`Bridge_Controller.ino` is an Arduino-ESP32 bridge for `Camera_Module.ino`. It uses ESP-NOW for camera traffic, native USB serial for a setup application, LittleFS for each camera's assigned configuration, NVS for Wi-Fi/MQTT settings, and MQTT for occupancy. The bridge runs without the setup application. New cameras announce their MAC in `HELLO`; the bridge reports them as `EVENT DISCOVER` and waits for a user-supplied configuration.

## Build and hardware

The bridge target is an **ESP32-C3 SuperMini**. Use Arduino-ESP32 3.x and install the **PubSubClient** Arduino library. Select an ESP32-C3 board profile and a partition scheme with LittleFS space. For a SuperMini wired to the C3's native USB Serial/JTAG pins, enable **USB CDC On Boot** and choose **Hardware CDC and JTAG** if that menu is shown. Some SuperMini variants instead expose USB through a UART chip; use that board's serial port settings. Use 115200 baud. The camera sketch also needs a LittleFS partition large enough for its maximum grayscale frame plus configured runtime cells; XGA alone takes 786,432 bytes. Upload the camera sketch after selecting its correct camera board and PSRAM settings.

The bridge sends a beacon every two seconds on its current radio channel. A camera scans channels 1–13 when it has not heard that beacon for 12 seconds, then resumes the bridge's channel. An associated Wi-Fi network sets the bridge's channel. Channel search can take time and should be tested on the chosen hardware. The C3 build has **eight camera slots** as an initial RAM guardrail; each camera's cells are allocated to its configured count, and staging rejects an allocation when less than 24 KB of free heap remains. This is a prototype limit, not a measured capacity. The C3 has a single core and much less RAM than an S3, so snapshot throughput, Wi-Fi/MQTT coexistence, and eight loaded cameras need hardware measurement.

## USB protocol

Commands and responses are UTF-8 lines, separated by `\n`. Fields are separated by spaces. Send one command, wait for its `OK` or `ERR` response, then send the next. MAC addresses use `AA:BB:CC:DD:EE:FF`. Text values that may contain spaces are uppercase hex of their UTF-8 bytes; `-` represents an empty value. The bridge never prints stored Wi-Fi or MQTT secrets in `LIST` or `GET`.

| Command | Meaning |
| --- | --- |
| `LIST` | Return one `CAMERA mac online/offline saved_revision remote_revision count baseline` row per known camera, then `OK LIST`. |
| `GET mac` | Return a `CONFIG` row, ordered `CELL` rows, and `TOPIC` rows, then `OK GET`. |
| `STATUS` | Return current `WIFI_STATUS` and `MQTT_STATUS` rows, then `OK STATUS`. |
| `STATES` | Return `SENSOR mac id state` rows for every cell and `OUTPUT mac id state topic-name` rows for MQTT outputs, then `OK STATES`. |
| `TOPIC mac id name_hex` | Set a unique MQTT area name for a configured output ID. |
| `BEGIN mac revision count resolution brightness contrast saturation vflip hmirror` | Stage a complete replacement configuration. Revision must increase. Resolution 0–3 is QVGA, VGA, SVGA, XGA. |
| `CELL mac index id group x y radius shape contrast_floor angle_tolerance threshold_permille enter_frames clear_frames` | Add an ordered cell. Shape 0 is circle, 1 square; group 0 is independent. Bounds match the camera sketch. |
| `COMMIT mac` | Save the complete staged config to flash, then send it to the camera if online. Wait for `EVENT CONFIG_APPLIED` to confirm remote application. |
| `BASELINE mac` | Tell the camera to capture a fresh **empty** scene and save its image and derived features in camera flash. `EVENT REQUEST_ACK mac 5 0` confirms success. |
| `FRAME mac` | Ask for a current grayscale frame. The transfer pauses camera monitoring. |
| `WIFI ssid_hex password_hex` | Try a Wi-Fi network. Save credentials only after connecting; restore prior saved credentials on failure. |
| `MQTT host_hex port topic_root_hex user_hex password_hex` | Try an MQTT broker. Save settings only after a successful MQTT connection; restore prior saved settings on failure. The root might be `railway/home`. |
| `FORGET_WIFI` | Remove saved Wi-Fi credentials, disconnect Wi-Fi and MQTT, and stop Wi-Fi retries. Saved MQTT settings remain available for later use. The bridge returns to ESP-NOW channel 1; cameras may take time to find it again. |
| `FORGET_MQTT` | Remove saved broker settings, disconnect MQTT, and stop MQTT retries. Wi-Fi remains connected. |

Example for one independent sensor:

```text
BEGIN AA:BB:CC:DD:EE:FF 1 1 0 0 0 0 0 0
CELL AA:BB:CC:DD:EE:FF 0 101 0 160 120 8 0 80 10 200 3 5
COMMIT AA:BB:CC:DD:EE:FF
BASELINE AA:BB:CC:DD:EE:FF
```

The bridge emits asynchronous `EVENT` lines: `DISCOVER`, `CONFIG_APPLIED`, `CONFIG_ERROR`, `STATE`, `CELL_STATE`, `HEALTH`, `CAMERA`, `TIMEOUT`, and snapshot events. `EVENT CELL_STATE mac cell_id state score_permille frame` reports each sensor for the app's Live camera overlay. `EVENT STATE mac output_id state score_permille frame` reports an independent sensor or combined block MQTT output. Individual block members do not publish MQTT topics. Each attempted publish is also printed as `EVENT MQTT topic payload`, whether or not the broker is connected. Wi-Fi and broker attempts emit `EVENT WIFI` and `EVENT MQTT_STATUS` with `connecting`, `connected`, `failed`, or `disconnected`. The app should treat these as asynchronous and correlate `CONFIG_APPLIED` and `REQUEST_ACK` with the target MAC. There is no request ID on USB in this first protocol version.

For `FRAME`, the bridge emits `EVENT SNAP_BEGIN mac frame width height bytes crc32`, then ordered `EVENT SNAP_DATA mac offset HEXPIXELS` lines, then `EVENT SNAP_END mac ok|crc_error`. Decode the hex bytes at each offset and verify byte count and CRC32 before displaying or saving the image. The frame is row-major, one grayscale byte per pixel. Radio retries can duplicate packets; the bridge outputs each chunk once. Capture can take many seconds at larger resolutions.

## Runtime behavior

After `COMMIT`, the bridge writes the new assignment to LittleFS, uploads `CONFIG_BEGIN`, each `CONFIG_CELL`, then `CONFIG_COMMIT`, waiting for the camera's application ACK on every packet. It retries the same packet and sequence up to eight times. A camera that restarts or joins with an older revision gets the bridge's saved version. An unconfigured camera's states are ignored. The bridge validates the sender MAC, configuration revision, sensor/group ID, state value, and increasing radio sequence before publishing a state.

State topics are `<root>/areas/<topic-name>/state` (numeric ID by default) with retained payload `clear`, `occupied`, or `unknown`. Camera health is `<root>/cameras/<MAC>/health`, and controller health is `<root>/controller/health` with an MQTT offline will. If a camera is silent for 15 seconds, its configured IDs are published as `unknown`. The bridge also prints equivalent MQTT events on USB when the broker is unavailable. Use globally unique sensor/group IDs and topic names across cameras.

This is a first integration sketch. ESP-NOW pairing is unauthenticated and MQTT uses unencrypted TCP. Configure it on a trusted local network. The Electron application below uses this USB contract.

## Desktop setup app

The Electron app lives in this folder. From `Bridge_Controller`, run `npm install` and `npm start`. It supports macOS and Windows, and `npm run pack:mac` or `npm run pack:win` builds installers on the corresponding operating system. The computer needs permission to open the bridge's 115200-baud serial port. Plug in the bridge, select its port, and connect. Cameras appear after they announce themselves over ESP-NOW.

The app fetches a grayscale frame for positioning, loads the bridge's saved sensor configuration, lets you add, move and remove circular sensors, and draws block sensors at one-radius intervals during a mouse drag. The radius starts at the last used value or 5 px and can be adjusted from 3 to 50 px. Save sends a complete revision to the bridge. Wait for the camera application event, clear the scene, then capture a new baseline. Frame transfer pauses monitoring and may take many seconds at high resolution. The Live monitor shows output states and MQTT publications received over USB; the app does not need a separate broker connection. Network settings can be entered in the app.

After fetching a frame, turn on **Live** in the camera view to color each sensor circle green when clear, red when occupied, and gray when unknown. The image behind the circles is the last fetched still frame; only the sensor colors update live. Edit controls are disabled in Live mode, and unsaved camera changes must be saved before entering it. Flash both the updated camera and bridge sketches to receive individual block-member states; older firmware can still report the combined block output but cannot color its separate sensors.

Each output can have a local display name and a bridge-stored MQTT topic name. Display names are saved on that computer in the app's local storage; topic names are stored by the bridge and travel with its configuration. Topic names allow lowercase letters, digits, hyphen and underscore, up to 32 characters. The bridge publishes `<root>/areas/<topic-name>/state`. Its default topic name remains the numeric output ID. Named topics require this updated bridge firmware. The `TOPIC mac id name_hex` USB command sets one output's name; `GET` returns `TOPIC id name` rows. Renaming or deleting an output publishes `unknown` to the old retained topic. Every bridge publish also appears as `EVENT MQTT topic payload` on USB so the monitor works while MQTT is connected.

The app checks frame length, ordering and CRC32 before displaying a snapshot. It also validates cell bounds, IDs, count, and topic names before uploading. Hardware integration, installer signing/notarization, and serial permissions still need testing on the target Mac/Windows machines.

The app keeps the last successfully fetched grayscale frame for each camera in its application data directory. Selecting a camera displays that saved frame immediately, including after an app restart. The label gives the capture time so an older still image is clear; **Fetch frame** replaces it. Saved frames are compressed on the computer and checked with CRC32 before display. The cached image does not change sensor states or the camera baseline.

Updated camera firmware tries fast lossless DEFLATE before an ESP-NOW snapshot. It uses compression only if the wire image is at least 10% smaller; otherwise it sends the original pixels. If a bridge does not acknowledge the compressed header, the camera falls back to the older raw snapshot format. The bridge checks the transferred bytes, and the app checks both the compressed transfer and decompressed image CRCs. Compression can reduce transfer time when the scene has repeated detail, but may add processing time to a noisy image; the camera serial log reports both byte counts and compression time. The bridge acknowledges each validated chunk before writing its USB line to avoid unnecessary radio retries when USB output stalls.

Wi-Fi and broker setup are separate checks. A `WIFI` command starts a 20-second connection attempt; `EVENT WIFI connected ip` means the credentials were saved in NVS. `EVENT WIFI failed reason` means the attempt failed and the previous saved Wi-Fi settings were restored. An `MQTT` command starts a 30-second broker attempt; `EVENT MQTT_STATUS connected` means host, port, topic root, and credentials were saved in NVS. On `EVENT MQTT_STATUS failed reason`, the previous broker settings are restored. `STATUS` can be used at any time, including after a reboot, to inspect current connectivity without exposing secrets. The app shows these events on its Network page. Re-enter all password fields when changing network settings; the bridge intentionally does not return stored secrets.

The Network page also provides separate **Forget Wi-Fi** and **Forget MQTT** buttons. They erase the matching saved settings on the bridge and stop reconnection attempts until new settings are entered; forgetting Wi-Fi does not erase saved MQTT settings. Forgetting Wi-Fi returns the bridge radio to channel 1, so cameras can briefly appear offline while scanning for it. When a broker is configured but unavailable, automatic MQTT retries are spaced 30 seconds apart and use short connection timeouts to reduce pauses in camera processing. Upload the matching bridge firmware before using the new buttons.

## USB and first-boot troubleshooting

For an ESP32-C3 SuperMini on native USB, select **ESP32C3 Dev Module**, **USB CDC On Boot: Enabled**, and **Default 4MB with spiffs** (or another partition scheme with a `spiffs` data partition). Upload the bridge sketch again after changing the CDC setting. The bridge should then print `READY bridge ...` at 115200 baud and answer `LIST` with `OK LIST`. If CDC is disabled, firmware `Serial` output may be routed away from the USB port even though low-level ESP logging appears there.

The bridge initializes LittleFS automatically only when the entire `spiffs` partition is blank. It leaves an existing nonblank partition untouched if mount fails, and prints `ERR storage mount` in that case. `ERR storage partition_missing` means the selected partition scheme has no suitable partition. Do not erase a nonblank partition without first considering saved camera assignments and topic names.

Close Arduino Serial Monitor before connecting the Electron app; only one application can own a serial port at a time. The app scans for an Espressif USB device and highlights its port. If opening it fails because another program holds it, the app shows a port-in-use message.

The setup app keeps baseline errors visible beside the camera controls. Baseline capture is available after a configuration revision has been saved and applied to the camera, and after a frame transfer has finished. Camera ACK status `2` means the configuration is not applied or a snapshot is active; status `4` means capture or baseline storage failed. The app includes the raw bridge event in the error so it can be reported during diagnosis.

A new camera configuration revision invalidates the previous baseline, even when the app has no unsaved edits. The app now explains why baseline capture is disabled and clears a pending baseline request when the camera goes offline or the request times out. The bridge reports interrupted requests and ends a stalled frame transfer after ten seconds without snapshot data, so a later Fetch frame is not blocked indefinitely.
The bridge allows up to 90 seconds for a baseline request, retrying the same sequence every ten seconds. The camera sends heartbeats during a long baseline write; the app keeps the request pending until the camera confirms success or the bridge reports failure.

## Connection history

The Network page shows a connection history. The bridge keeps its latest 64 diagnostic entries in RAM and returns them with the USB `LOG` command (`LOG_NOW`, `DIAG` rows, `OK LOG`). The app also saves the latest 300 entries on that computer, so bridge history remains visible after the bridge restarts. **Refresh log** loads the bridge's current entries; **Clear local history** removes the computer's copy only. A bridge restart clears its RAM log.

The bridge records camera online/offline transitions, Wi-Fi radio/channel changes, frame and baseline requests, request ACKs, retries, timeouts, snapshot start/end, and transfer stalls. The camera reports its boot/reset reason, the previous operation from RTC memory, camera initialization, bridge join/rejoin, and frame/baseline starts, results, and failures. Camera reports have a boot ID and event number and are retried until the bridge acknowledges them. A camera reset discards queued reports; its next boot report can still identify an interrupted snapshot or baseline when RTC memory survived the reset. Full power loss may erase that RTC summary. Events delayed by a disconnect are timestamped when the bridge receives them; `ms=` within the camera report is the camera's uptime when the event occurred.

For a frame failure, compare `FRAME_REQUEST`, `REQUEST_ACK` or `REQUEST_RETRY`, `FRAME_BEGIN`, `FRAME_STALLED` or `CAMERA_OFFLINE` with the camera's `CAMERA_FRAME_START`, `CAMERA_SEND_FAILED`, `CAMERA_FRAME_ABORTED`, or next `CAMERA_BOOT`. A camera appearing online means its ESP-NOW hello reached the bridge after setup completed; the `CAMERA_READY` report confirms camera initialization. A frame or baseline operation can still fail after that point. Both updated sketches are needed for camera-side reports.

For configuration uploads, `UPLOAD_SEND` and `REQUEST_RETRY` show whether the bridge queued each ESP-NOW packet. `CAMERA_CONFIG_BEGIN` confirms the camera received the first packet; `CAMERA_CONFIG_ACK_FAILED` means it could not queue the acknowledgement. An `EVENT TIMEOUT ... 2` means the bridge saved the configuration but never received a `CONFIG_BEGIN` acknowledgement. The bridge now allows 1.5 seconds per upload attempt and continues to try again when the camera announces a mismatched revision. The app reports this as pending camera sync rather than a failed bridge save.

The bridge replies to each camera health report over ESP-NOW. The camera treats those replies and other valid bridge packets as contact, so a missed broadcast beacon no longer starts channel scanning while unicast traffic is working. This specifically addresses repeated offline/rejoin cycles without a camera reboot.

During a new baseline request the bridge now marks configured outputs `unknown` until fresh camera states arrive, so the monitor does not display a previous occupied result as if it belonged to the new baseline. The monitor shows the latest mismatch score reported for each output when available. A block is occupied if any member sensor becomes occupied; the camera sets all members clear at baseline capture, then evaluates subsequent frames against it. Small 5 px cells and a 0.200 default mismatch threshold can be sensitive to image noise or slight motion across a long block, so investigate repeated occupied scores under an unchanged empty scene before relying on the result.
