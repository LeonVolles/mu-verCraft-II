<!-- Note, it is intentionnal that the first heading starts as a "level2"/## and not as level1/# and should be kept like this!! -->
## WiFi manager (ESP32 SoftAP helper)

### Main idea
This module wraps the ESP32 Arduino `WiFi` API to start and manage a WiFi Access Point (SoftAP) used for the hovercraft’s web UI / telemetry links.

### Where it is used / how it is called
- Class/API: [lib/wifi_manager/include/wifi_manager.h](lib/wifi_manager/include/wifi_manager.h) (implementation in [lib/wifi_manager/src/wifi_manager.cpp](lib/wifi_manager/src/wifi_manager.cpp)).
- Instantiated as a global object in [src/main.cpp](src/main.cpp).
- The AP is started once during `setup()` via `wifiManager.startAccessPoint(global_WifiApSsid, global_WifiApPassword)`.
- Runtime web handling is performed by the async webserver (see `network_piloting`).
- The project’s `task_wifiManager()` in [src/main.cpp](src/main.cpp) is mainly used for periodic diagnostics (heap/stack logging), not for driving the webserver.

### Functionalities implemented
- **Start SoftAP:** configures `WiFi.mode(WIFI_AP)` and starts `WiFi.softAP(...)` with SSID/password/channel/max connections.
- **Low-latency radio setting:** calls `WiFi.setSleep(false)` to keep the WiFi link responsive.
- **Connection / IP helpers:** small wrappers to check “connected” state and return the current IP address (AP IP vs station IP).

### Methods (overview)
- `startAccessPoint(ssid, password, channel=1, maxConnections=4)` – starts the ESP32 SoftAP and logs IP to Serial.
- `isConnected()` – returns true if AP is active or station is connected.
- `getIp()` / `getIpString()` – returns IP depending on current WiFi mode.

### Parameters used

#### Internal (class state)
- `apActive_` – remembers whether the SoftAP was successfully started.

#### External (project config)
Provided by [src/hovercraft_variables.h](src/hovercraft_variables.h) / [src/hovercraft_variables.cpp](src/hovercraft_variables.cpp):
- `global_WifiApSsid`, `global_WifiApPassword` (passed to `startAccessPoint()` in [src/main.cpp](src/main.cpp))

Note: `global_WebServerPort` exists in the project config, but it is used by the webserver/network layer (see `network_piloting`), not by `WifiManager` itself.

