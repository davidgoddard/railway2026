# Railway camera module firmware

`Camera_Module.ino` is the first multi-cell camera firmware. It captures grayscale frames locally, compares configured cells with a captured empty-layout baseline, and sends compact occupancy events to a bridge over ESP-NOW. It has no web server, hotspot, station association, IP address, or MQTT client. ESP-NOW still needs the Wi-Fi radio in station mode. The compatible ESP-NOW bridge and Electron setup application are in `Bridge_Controller`.

The sketch version is at the first line. `CAMERA_DEBUG_SERIAL` is `1` for development. Set it to `0` to suppress USB text and raw frame output; command input remains available. The serial monitor uses **115200 baud** and newline-terminated commands.

## Hardware and build

The sketch now feeds the same `OccupancyDetector` class from an `ImageSource` adapter. `OccupancyDetector.h` owns grayscale feature extraction, baseline calibration, per-cell persistence and group transitions; it emits state changes through a callback. `ImageSource.h` owns camera pins, sensor setup, capture and the capture interval. The sketch still owns configuration, storage, ESP-NOW, snapshots and scheduling. To add another camera, supply grayscale, row-major, one-byte-per-pixel frames at the configured width and height through the image-source interface; detector code does not need board conditionals.

The default board map follows the Arduino target: classic ESP32 selects AI Thinker and ESP32-S3 selects S3-EYE. A different S3 module needs its own pin map and sensor validation. ESP32-P4 is a future target, but the current sketch cannot run on it yet: its camera path typically uses ESP Video, and its lack of an integrated radio also requires a companion transport for the existing ESP-NOW protocol. The build fails explicitly for P4 until those board-specific adapters are supplied. See [Espressif's Arduino library support table](https://docs.espressif.com/projects/arduino-esp32/en/latest/libraries.html) and [Arduino ESP Video DVP example](https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP_Video/examples/dvp_camera/dvp_camera.ino).

- The default pin map is the **AI Thinker ESP32-CAM**, with its flash LED on GPIO 4. The **ESP32-S3-EYE** map is selected automatically when compiling for ESP32-S3. Its LED uses GPIO 3 in [Espressif's S3-EYE board definition](https://github.com/espressif/esp-bsp/blob/master/bsp/esp32_s3_eye/esp32_s3_eye.json). Verify the status LED behavior on the particular board revision before relying on its flash pattern. Other S3 camera modules need a new pin map.
- PSRAM is required for the frame buffer and the two cell arrays. The sketch supports 320×240, 640×480, 800×600, and 1024×768 grayscale modes, subject to the actual camera and PSRAM. Camera XCLK is 20 MHz, one PSRAM frame buffer is used, and a separate full-frame copy is retained for processing and snapshots.
- Compile with Arduino-ESP32 core 3.3.8 or a compatible 3.x core. For AI Thinker, select **AI Thinker ESP32-CAM** and enable PSRAM; its serial commands need a USB-to-UART adapter. For S3-EYE, select the appropriate ESP32-S3 board and PSRAM configuration and enable USB CDC if using native USB serial. The default sketch compiles with `arduino-cli compile --fqbn esp32:esp32:esp32cam Camera_Module`.
- Power the camera from a stable supply. The flash LED may draw much more current than the status logic alone.

At boot the module broadcasts a `HELLO` packet containing its station MAC address every two seconds. The bridge emits a beacon on its current ESP-NOW channel. If the camera has no bridge contact for five seconds, it scans channels 1–13 with a 600 ms dwell on each. When it hears a returning bridge, it schedules an immediate HELLO and refreshes its sensor states. The default channel is **1**; use USB `C` to change it for the current boot. After a complete configuration is committed and a fresh baseline has been captured, the status LED flashes twice and monitoring starts. The active configuration, derived features, and raw grayscale baseline image are saved to LittleFS and restored on restart. Fresh frames must still establish each state after restart. Do not issue baseline capture while a train occupies a configured cell.

## Configuration and states

The bridge uploads a whole revision with `CONFIG_BEGIN`, zero to 300 ordered `CONFIG_CELL` records, and `CONFIG_COMMIT`. The camera stages records in a separate PSRAM array. A failed or incomplete upload leaves the active configuration intact. The commit clears the old baseline and sets all cells to `UNKNOWN` until `CAPTURE_BASELINE` succeeds. Changing camera resolution or image settings also requires a new empty-layout baseline. The configuration includes camera resolution, brightness, contrast, saturation, vertical flip, and horizontal mirror settings; it does not currently override the sensor's automatic exposure and gain behavior.

Each cell has a unique nonzero sensor ID, centre `(x,y)`, radius 3–50 pixels, circle or square shape, contrast floor 10–500, angle tolerance 0–20°, mismatch threshold 0.050–1.000, and enter/clear persistence counts. A `groupId` of zero makes it an independent sensor. Cells with the same nonzero `groupId` form one block: **any occupied member makes the block occupied**; otherwise an unknown member makes it unknown; otherwise it is clear. The camera sends every cell's state for the setup app's Live overlay and also sends one combined state for each block. Only independent sensors and combined blocks become MQTT outputs. Up to 64 distinct groups are supported.

An occupancy change is queued for radio transmission. The queue coalesces newer states for the same ID and drains at most one event every 20 ms. The camera also queues a full state refresh every 30 seconds, so a lost event can eventually be corrected. The bridge must still validate the camera MAC, configuration revision, sequence and freshness, and treat a silent or stale camera as unknown. ESP-NOW send acceptance alone does not prove that the bridge application processed an event.

## Detection algorithm

The detector retains **up to three dominant gradient angles**. These are gradient-normal angles: the visible rail or carriage edge runs 90° from the angle reported in the debug data. A cell with no stable angle is still usable; a blank reference can detect new texture. Saved edge locations are no longer used to decide occupancy.

Old baseline files remain readable because the persisted record layout is unchanged. Legacy edge-point fields are ignored and cleared when a new baseline is captured. A fresh empty-track baseline is still recommended after installing this algorithm change.

1. Baseline capture takes one fresh grayscale frame and calibrates every configured cell from it. The first pixel pass calculates Sobel gradient magnitude and brightness statistics. The second pass counts qualifying gradient angles in 18 bins of 10°. A qualifying gradient exceeds both the cell's contrast floor and one quarter of that cell's strongest gradient. Peaks must have enough support to be retained. For an oriented reference, the code precomputes unit direction vectors and baseline counts near each peak.
   The camera sends periodic bridge heartbeats while writing the baseline image to flash, and its serial log reports the capture duration. Large baseline writes can pause normal monitoring without implying a reboot.
2. Every live frame makes the same two pixel passes for each cell. Where reference peaks exist, the second pass uses integer dot and cross products to count gradients near each stored direction; it does not calculate `atan2` per live edge. Other gradients go into an additional bucket. Without a stable reference peak, it compares smoothed 18-bin angle distributions.
3. A mismatch compares the proportions of gradients near the saved directions, or the smoothed angle histogram when no stable peak exists. A blank/textured classification change scores 1.0. If the first comparison would trigger, the detector checks the same cell mask at the eight neighbouring one-pixel image offsets and keeps the lowest usable mismatch. This tolerates a one-pixel image shift without storing specific edge points. No per-pixel baseline brightness patch is stored or compared.
4. A sudden bright washout can make existing edges disappear. The provisional exposure guard suspends state transitions if brightness rose and near-white clipping or strong detail loss indicates an unreliable frame. It preserves the prior state and logs the condition. This guard and the contrast floor are still affected by illumination; the detector is **not** lighting independent.
5. A score at or above the configured threshold for `enterFrames` consecutive usable frames marks the cell occupied. A score below 70% of that threshold for `clearFrames` consecutive usable frames marks it clear. The intermediate range holds the previous state.

Very small patches with fewer than 16 edges and no stable angle have noisy angle histograms. When both the baseline and live patch are this weak, the camera treats their difference as zero; a live patch with 16 or more edges is still evaluated. Live edge detection uses the cutoff calibrated from the baseline, so a changing strongest pixel cannot change the cutoff for the rest of the patch. A block can still be held occupied by any other member, so use serial `I` to inspect individual cell states if it stays occupied. Trigger logs include the final angle mismatch, live/reference edge counts, and strongest gradients. Periodic block state messages report the highest current member score instead of a placeholder zero.

Two pixel passes per cell, plus up to eight extra comparisons for a cell whose first score would trigger, mean hundreds of large or overlapping cells may take longer than the camera capture interval. The loop aims to start a new capture after 100 ms; this is **not a guaranteed 10 fps rate**. Measure the actual board, resolution, and cell set before using it for traffic decisions. A carriage whose edges have the same angle proportions as the empty track can still look like the baseline.

## ESP-NOW wire protocol

All fields are packed and little-endian, as sent by ESP32. The packet starts with `uint16 magic=0x5243`, `uint8 version=1`, `uint8 type`, `uint32 seq`, `uint16 payload_length`, followed by exactly that many payload bytes. Maximum packet size is **210 bytes**, within the legacy 250-byte ESP-NOW payload limit. A bridge should implement this exact format and reject wrong lengths, versions, source MACs, and stale revisions. The receive callback only queues valid packets; the main loop handles configuration and camera work. The queue has 16 slots, so configuration senders must wait for each application-level ACK before sending the next item. [Espressif documents the 250-byte v1 limit and the need for application acknowledgements.](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32/api-reference/network/esp_now.html)

| Type | Number | Direction | Payload |
| --- | ---: | --- | --- |
| `HELLO` | 1 | broadcast from camera | MAC `[6]`, revision `u32`, width/height/count `u16`, channel and baseline-ready `u8` |
| `CONFIG_BEGIN` | 2 | bridge → camera | revision `u32`, count `u16`, camera settings: resolution `u8`, brightness/contrast/saturation `i8`, vertical flip/horizontal mirror `u8` |
| `CONFIG_CELL` | 3 | bridge → camera | index `u16`, then `CellConfig`: id/group ID `u32`, centre x/y `u16`, radius/shape `u8`, contrast floor/threshold in thousandths `u16`, tolerance/enter/clear `u8` |
| `CONFIG_COMMIT` | 4 | bridge → camera | revision `u32` |
| `CAPTURE_BASELINE` | 5 | bridge → camera | empty payload; triggers a **fresh** frame and recalibrates every active cell |
| `SNAPSHOT_REQUEST` | 6 | bridge → camera | empty payload; starts transfer of the latest retained grayscale frame |
| `ACK` | 7 | camera → bridge | original type `u8`, status `u8`, detail `u16`; header sequence echoes the request |
| `STATE` | 8 | camera → bridge | id/revision/frame `u32`, score in thousandths `u16`, state and grouped flag `u8` |
| `HEALTH` | 9 | camera → bridge | revision/frame/capture failures/free heap `u32`, cell count/width/height/capture age ms `u16`, baseline-ready/snapshot-active `u8` |
| `SNAPSHOT_BEGIN` | 10 | camera → bridge | frame/byte count/CRC32 `u32`, width/height `u16` |
| `SNAPSHOT_CHUNK` | 11 | camera → bridge | byte offset `u32`, followed by up to 180 grayscale bytes |
| `SNAPSHOT_END` | 12 | camera → bridge | frame and CRC32 `u32` |
| `SNAPSHOT_ACK` | 13 | bridge → camera | empty payload; header sequence echoes each received snapshot packet |

ACK status values are `0 OK`, `1 bad payload`, `2 wrong order`, `3 no memory` (reserved), `4 camera error`, and `5 busy`. The bridge should retry a configuration packet with the **same sequence and contents** if its ACK is lost. Repeated `CONFIG_CELL` records at an already staged index are accepted only when their contents match. Repeated `CONFIG_COMMIT` for the active revision succeeds. A repeated baseline command with the same sequence is acknowledged without recapturing, preventing a lost ACK from replacing the empty reference with a later occupied frame. A new sequence explicitly requests a new baseline.

Snapshot transfer is stop-and-wait. The camera sends `SNAPSHOT_BEGIN`, waits for `SNAPSHOT_ACK`, then sends one indexed chunk at a time and finally `SNAPSHOT_END`; each packet requires an ACK with the same sequence. It retries after 250 ms, up to eight times. The bridge must verify the byte count and CRC32. The frame is **raw grayscale, row-major, one byte per pixel**, and is intended for the bridge to forward to the Electron app or save as a baseline snapshot. At 800×600 it is 480,000 bytes and needs about 2,667 chunks plus acknowledgements. Monitoring pauses while the transfer holds the frame; `HEALTH.snapshotActive` and `captureAgeMs` tell the bridge to treat camera observations as stale. This diagnostic path needs throughput testing. The normal state path sends no images.

A repeated `SNAPSHOT_REQUEST` with the same sequence is acknowledged as the active request. A different request while a snapshot is active receives `ACK_BUSY` (status 5). This lets the bridge retry a request whose initial response was delayed without reporting a false busy error.
Snapshot sends that cannot be queued on ESP-NOW now back off and count toward the retry limit. The camera logs `snapshot send failed` and then `snapshot aborted` if the link does not recover, so a failed transfer cannot spin indefinitely and starve normal monitoring.

The first valid bridge beacon or `CONFIG_BEGIN` source becomes the bridge for that boot, or use USB `P` to set its MAC explicitly. This is **commissioning only**: peer traffic is currently unencrypted and unauthenticated. Do not treat the first-source pairing as secure deployment. The camera scans channels after losing the bridge beacon; channel changes need hardware testing.

## USB commands

Commands are single letters followed by space-separated arguments, with a newline at the end. Run `H` for the built-in summary. Serial output includes boot identity, configuration entries, baseline angles for each sensor, occupancy triggers, sent states, snapshot progress, and errors while `CAMERA_DEBUG_SERIAL=1`.

| Command | Action |
| --- | --- |
| `H` | Help |
| `I` | Identity, frame and health summary, then every cell's metadata, first angle, score and state |
| `P AA:BB:CC:DD:EE:FF` | Set bridge MAC for this boot |
| `C 6` | Set ESP-NOW channel 6 for this boot; valid range 1–13 |
| `B 1 2 2` | Start revision 1, two cells, SVGA (resolution index 2). Optional five camera settings follow: brightness, contrast, saturation, vflip, hmirror |
| `S 0 101 0 400 300 5 C 80 10 200 3 5` | Stage cell 0, sensor ID 101, independent, centre (400,300), circle radius 5, contrast floor 80, tolerance 10°, threshold 0.200, enter after 3 frames, clear after 5 |
| `S 1 102 900 420 300 5 S 80 10 200 3 5` | Stage cell 1 as part of block/group ID 900; `S` shape means square |
| `A` | Commit the staged configuration; baseline becomes invalid |
| `R` | Capture a **fresh** empty-layout frame and recalibrate all cells; LED flashes twice |
| `F` | Write a binary snapshot of the latest frame to USB |
| `X` | Clear the active configuration from RAM |

For `F`, the binary output starts with eight ASCII bytes `RAILFRM1`, then little-endian frame number `u32`, byte count `u32`, CRC32 `u32`, width `u16`, height `u16`, followed immediately by the raw grayscale bytes. Text debug output may precede or follow it; a receiver should scan for the magic and read the exact declared byte count. A terminal is unsuitable for this binary stream. Use a serial capture program instead.

To save a PNG directly from the camera when the bridge frame transfer is unavailable, close Arduino Serial Monitor and run `node tools/capture-camera-frame.js /dev/cu.usbserial-110 camera-frame.png` from `Bridge_Controller` (replace the port with the camera's serial port). The utility sends `F`, validates the frame CRC, and writes a grayscale PNG. It uses the existing `serialport` dependency installed by `npm install` in `Bridge_Controller`.

## Current limits

This is a hardware-testable first camera sketch, not a completed controller system. ESP-NOW configuration/snapshot transfer, S3-EYE LED mapping, image quality, radio throughput, channel scanning, flash capacity, and hundreds-of-cells frame rate need tests on target hardware. LittleFS must have enough free space for the full grayscale baseline plus runtime cell records, or baseline capture fails. The camera stores a local baseline; the application should also back up snapshots for editing and recovery. Pairing is not authenticated, and there is no OTA path yet. See the [bridge controller](../Bridge_Controller/README.md) for USB and MQTT integration.

A fresh camera with a completely blank `spiffs` data partition initializes LittleFS on first boot and prints `LittleFS initialized` when debug serial is enabled. A nonblank partition that fails to mount is left untouched and prints `LittleFS unavailable; persistent baseline disabled`; baseline capture will fail until storage is repaired. A successful `FRAME` request does not prove that baseline storage is available, since frame transfer uses RAM.

If a previously used or corrupted LittleFS partition cannot mount, the camera preserves it instead of formatting it automatically. With the updated firmware loaded, send `Z FORMAT` at 115200 baud on the camera USB serial port to explicitly erase and recreate only the camera's LittleFS partition. This removes any saved baseline and camera-side configuration; the bridge retains its own configuration and uploads it again. The camera prints `LittleFS formatted; capture a new baseline` on success. Do not use this command when you need to recover existing camera flash data.
# Baseline storage

New baselines save the calibrated sensor features and camera settings, without a duplicate full grayscale image. The camera still reads older baseline files that contain an image. The ESP32-CAM board's default Huge APP partition gives SPIFFS/LittleFS 917,504 bytes; two SVGA images alone require 960,000 bytes, so replacing a previous SVGA baseline could fail. The compact format avoids that. If storage still fails, the serial log prints the failure stage and remaining LittleFS bytes; camera diagnostic events use `value=201` through `205` for the corresponding storage stage.
