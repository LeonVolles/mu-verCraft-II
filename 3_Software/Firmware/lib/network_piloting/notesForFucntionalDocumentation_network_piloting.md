<!-- Note, it is intentionnal that the first heading starts as a "level2"/## and not as level1/# and should be kept like this!! -->
## Network piloting (web UI + WebSocket control/telemetry)

### Main idea
This module hosts a small, embedded controller webpage and a WebSocket endpoint to send pilot commands (lift/thrust/steering + arm) to the firmware and to stream basic telemetry back to the browser.

### Where it is used / how it is called
- Class/API: [lib/network_piloting/include/network_piloting.h](lib/network_piloting/include/network_piloting.h) (implementation in [lib/network_piloting/src/network_piloting.cpp](lib/network_piloting/src/network_piloting.cpp)).
- Instantiated as a global object in [src/main.cpp](src/main.cpp).
- Started in `setup()` via `networkPiloting.begin()` after the AP is created by `wifiManager.startAccessPoint(...)`.
- `setLiftCallback()`, `setThrustCallback()`, `setSteeringCallback()`, `setArmCallback()` are wired in [src/main.cpp](src/main.cpp) to push the received commands into the control queue.
- Runtime maintenance (`networkPiloting.loop()` → `ws_.cleanupClients()`) is called from `task_wifiManager()` in [src/main.cpp](src/main.cpp), which is pinned to ESP32 **core 0**.
- Telemetry broadcast is fed from the battery task via `networkPiloting.sendTelemetry(v, a, mah)` in [src/main.cpp](src/main.cpp).

### Functionalities implemented

#### Website (UI + transport)
- **Embedded controller page:** the HTML/CSS/JS UI is stored in flash (`PROGMEM`) and served for `/`, `/controller.html`, and as a fallback for unknown routes.
- **WebSocket link (`/ws`):** the browser opens a WebSocket and sends JSON messages like `{ "lift": 60, "thrust": -10, "steering": 25, "motorsEnabled": true }`.
- **Telemetry to UI:** the firmware can broadcast compact JSON frames to all clients (currently battery voltage/current/used mAh).
- **Client-side safety UX:** the page springs controls back to zero on pointer release and also resets on tab blur / page hidden / WS disconnect.

#### “Intelligence / math” (control semantics + safety)
- **Input shaping on the webpage:** the drag pads map pointer position to a normalized command in $[-100,100]$ with a deadzone and an expo curve (smooth low-sensitivity near center).
- **Lift presets:** the UI toggles lift through a preset list; the preset values and start index are injected by the firmware at page-serve time.
- **Fail-safe on disconnect:** on WebSocket disconnect, the firmware forces `motorsEnabled=false` and calls the user callbacks with `lift=0`, `thrust=0`, `steering=0`.
- **Clamping + lightweight parsing (firmware-side):** incoming values are clamped to valid ranges and parsed without a full JSON dependency (simple string searches + `toFloat()`).

### Methods (overview)
- `begin()` – configures routes for the embedded page, attaches the WebSocket handler, and starts the async webserver.
- `loop()` – performs periodic housekeeping (`cleanupClients()`).
- `set*Callback(...)` – registers callbacks for lift/thrust/steering/arm updates.
- `sendTelemetry(voltage, current, usedMah)` – broadcasts telemetry JSON to all connected clients (no-op if no clients).
- `getLift()`, `getThrust()`, `getSteering()`, `motorsEnabled()` – returns latest received command state.

### Parameters used

#### Internal (class state)
- `server_` – `AsyncWebServer` instance (constructed with the configured port).
- `ws_` – `AsyncWebSocket` at path `/ws`.
- `lift_`, `thrust_`, `steering_`, `motorsEnabled_` – last received command values.
- `onLift_`, `onThrust_`, `onSteering_`, `onArm_` – callback hooks into the rest of the firmware.

#### External (project config)
Used from [src/hovercraft_variables.h](src/hovercraft_variables.h) / [src/hovercraft_variables.cpp](src/hovercraft_variables.cpp):
- `global_WebServerPort` – port used by the HTTP server.
- `global_WebLiftPresetPercent_Array`, `global_WebLiftPresetPercent_Array_len` – lift preset list injected into the webpage.
- `global_WebLiftPresetPercent_Array_startIndex` – which preset becomes active first when toggling lift ON.

