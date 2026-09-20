# Railway Bridge Controller — static web comparison build

This is a browser-hosted comparison version of the Electron UI in `Bridge_Controller/app`.

## Run

Serve this directory over HTTPS (GitHub Pages is suitable), then open it in desktop Chrome or Edge.

1. Click **Choose USB device** and approve the ESP32 serial device.
2. Click **Connect bridge**.
3. Web Serial opens it at 115200 baud and uses the same line protocol as the Electron app.

Opening `index.html` directly with `file://` is not recommended; Web Serial requires a secure context (HTTPS, or localhost during development).

## Implemented for comparison

- Same dark UI structure and three workspaces: Camera setup, Live monitor, Network.
- Web Serial permission chooser and 115200-baud bridge connection.
- `LIST`, `GET`, `STATES`, `STATUS`, `LOG`, `FRAME`, `BASELINE`, `BEGIN/CELL/COMMIT/TOPIC`, Wi-Fi and MQTT bridge commands.
- Camera discovery/state events and basic sensor editing on the image.
- Snapshot CRC checking and browser decompression for compressed frames when `DecompressionStream('deflate-raw')` is supported.
- Cached frames and local display names stored in browser localStorage.

## Intentional difference / unfinished item

The direct MQTT viewer is left as a UI-compatible placeholder in this comparison build. A browser cannot use MQTT TCP on port 1883; the broker needs an MQTT-over-WebSocket listener (`ws://` or preferably `wss://`). The bridge itself continues using its existing MQTT TCP connection unchanged.

For production, viewer passwords should normally be session-only rather than persisted in localStorage. This comparison build stores the entered viewer settings only to demonstrate the flow; do not use sensitive credentials there.

## GitHub Pages

The directory has no build step and can be copied to a `docs/` directory or a Pages branch. All application code is static HTML/CSS/JavaScript.
