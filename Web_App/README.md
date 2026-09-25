# Railway Bridge Controller — web application

This is the browser-hosted counterpart of the Electron application in `Bridge_Controller/app`. It deliberately shares the same HTML structure, styling, setup workflow and renderer logic so changes can be evaluated without maintaining two different user experiences.

> [!WARNING]
> This project is under active development and is not ready for download or operational use.

## Run

Serve this directory over HTTPS and open it in desktop Chrome or Edge. GitHub Pages is suitable. For local development, `localhost` is treated as a secure context by supported browsers.

1. Click **Scan USB ports** or **Choose USB device** and approve the ESP32 bridge.
2. Select the approved device and click **Connect bridge**.
3. The browser opens Web Serial at 115200 baud and uses the same bridge line protocol as Electron.

Opening `index.html` directly with `file://` is not supported because Web Serial requires a secure context.

## Electron-parity features

- Camera discovery, configuration revision handling and health display.
- Lossless frame transfer with CRC validation and compressed-frame decompression.
- Cached camera frames and local output display names.
- Sensor creation, movement, deletion, block painting and block extension.
- Live overlays selectable between individual USB sensor events and MQTT output states. In MQTT mode, all circles belonging to a block show its combined published state.
- Previously known cameras, configurations, and cached images remain selectable without USB, allowing the MQTT overlay to operate as a standalone layout view.
- Camera settings, per-sensor persistence settings and mismatch thresholds.
- Fetched-frame texture preview using the camera's Scharr gradients and twelve fixed 15° direction buckets.
- On-demand live camera comparison against the saved baseline. Structured baselines use normalised projected bands and concentric rings within their five strongest directions; low-texture baselines require newly appearing, coherent, spatially widespread structure.
- Ten-second fixed-centre radius and threshold calibration for newly added sensors, with an explicit recalibrate-all option. New baseline refreshes only the visual reference.
- The same persistent Cameras workspace, live USB/MQTT overlay, traversal test, sensor focus, keyboard movement, and continuous radius control as the Electron app.
- Bridge Wi-Fi and MQTT configuration.
- Diagnostics, USB MQTT events and local state history.
- Direct MQTT state monitoring when the broker exposes MQTT over WebSockets.

## Browser-specific differences

Web Serial is available only in browsers that implement it, currently desktop Chromium-family browsers. Device permission is controlled by the browser and may need to be granted again.

Browsers cannot connect to ordinary MQTT TCP port 1883. The direct monitor therefore requires an MQTT-over-WebSocket listener, commonly on port 9001, using `ws://` locally or `wss://` from an HTTPS page. The GitHub Pages app cannot connect to an insecure `ws://` listener and the certificate on `wss://` must be trusted by the browser. Enter a full URL such as `wss://broker.example/mqtt` when the endpoint needs a particular path. A local broker exposing only `ws://` can instead be used from a copy of the app served at `http://localhost`. The bridge itself continues using ordinary MQTT TCP and is unaffected.

The web application loads the pinned MQTT.js browser bundle from jsDelivr. Without internet access, USB setup and monitoring still work, but the direct MQTT viewer cannot start unless that dependency is hosted locally.

Viewer settings are stored in browser `localStorage`, including any entered password. Use non-sensitive development credentials until secure credential handling is implemented.

Fetched grayscale frames are rendered as soon as their transfer and CRC checks complete, then cached as binary data in browser IndexedDB. Older number-array frame caches in `localStorage` are migrated when read. This avoids blocking the page while serializing hundreds of thousands of pixels at SVGA or XGA resolution.

## GitHub Pages

The directory has no build step. Publish `Web_App` as the Pages source or copy its static contents to the configured Pages directory. The application must remain on HTTPS for Web Serial and secure MQTT WebSockets.
