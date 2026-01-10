#include "network_piloting.h"

// NetworkPiloting::NetworkPiloting() {
//     // TODO: Initialize member variables
// }

// NetworkPiloting::~NetworkPiloting() {
//     // TODO: Cleanup resources
// }

// void NetworkPiloting::init() {
//     // TODO: Initialize network piloting
// }

// void NetworkPiloting::startServer() {
//     // TODO: Start web server
// }

// void NetworkPiloting::processClient() {
//     // TODO: Process client requests
// }

// void NetworkPiloting::setTargetThrust(float v) {
//     // TODO: Set target thrust value
// }

// void NetworkPiloting::setTargetSteering(float v) {
//     // TODO: Set target steering value
// }

// float NetworkPiloting::getTargetThrust() {
//     // TODO: Return target thrust
//     return 0.0;
// }

// float NetworkPiloting::getTargetSteering() {
//     // TODO: Return target steering
//     return 0.0;
// }

// void NetworkPiloting::setArmed(bool v) {
//     // TODO: Set armed state
// }

// bool NetworkPiloting::isArmed() {
//     // TODO: Return armed state
//     return false;
// }


#include "network_piloting.h"

namespace
{
// Controller HTML served from flash; also written to lib/website/controller.html for easier editing.
// WebSocket endpoint: /ws
const char CONTROLLER_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
	<meta charset="UTF-8" />
	<meta name="viewport" content="width=device-width, initial-scale=1.0" />
	<title>Hovercraft Control</title>
	<style>
		:root {
			--bg: #0b1724;
			--panel: #12263a;
			--accent: #2ec4b6;
			--accent-2: #ff9f1c;
			--text: #e8eef4;
			--muted: #7c8ca5;
			--shadow: rgba(0, 0, 0, 0.35);
		}
		* { box-sizing: border-box; }
		body {
			margin: 0;
			font-family: "Segoe UI", system-ui, -apple-system, sans-serif;
			background: radial-gradient(circle at 20% 20%, rgba(46, 196, 182, 0.08), transparent 30%),
									radial-gradient(circle at 80% 10%, rgba(255, 159, 28, 0.08), transparent 25%),
									radial-gradient(circle at 50% 80%, rgba(46, 196, 182, 0.06), transparent 30%),
									var(--bg);
			color: var(--text);
			min-height: 100vh;
			display: flex;
			align-items: center;
			justify-content: center;
			padding: 24px;
		}
		.card {
			width: min(520px, 100%);
			background: var(--panel);
			border-radius: 16px;
			box-shadow: 0 18px 35px var(--shadow);
			padding: 24px;
			border: 1px solid rgba(255, 255, 255, 0.04);
		}
		h1 {
			margin: 0 0 6px;
			font-weight: 700;
			letter-spacing: 0.2px;
		}
		.subtitle {
			margin: 0 0 18px;
			color: var(--muted);
			font-size: 14px;
		}
		.slider-row {
			margin-top: 18px;
			padding: 16px;
			background: rgba(255, 255, 255, 0.02);
			border-radius: 12px;
			border: 1px solid rgba(255, 255, 255, 0.04);
		}
		label { display: flex; align-items: center; justify-content: space-between; font-weight: 600; }
		.value-pill {
			padding: 6px 10px;
			border-radius: 999px;
			background: rgba(46, 196, 182, 0.12);
			color: var(--accent);
			font-variant-numeric: tabular-nums;
			font-size: 13px;
			border: 1px solid rgba(46, 196, 182, 0.22);
		}
		input[type=range] {
			width: 100%;
			margin: 14px 0 6px;
			-webkit-appearance: none;
			height: 6px;
			background: linear-gradient(90deg, var(--accent), var(--accent-2));
			border-radius: 10px;
			outline: none;
			box-shadow: inset 0 0 0 1px rgba(0,0,0,0.25);
		}
		input[type=range]::-webkit-slider-thumb {
			-webkit-appearance: none;
			appearance: none;
			width: 22px;
			height: 22px;
			background: #fff;
			border-radius: 50%;
			border: 3px solid var(--accent);
			box-shadow: 0 4px 10px rgba(0,0,0,0.25);
			cursor: pointer;
			transition: transform 80ms ease;
		}
		input[type=range]:active::-webkit-slider-thumb { transform: scale(1.05); }
		.status {
			display: flex;
			gap: 8px;
			align-items: center;
			margin-top: 12px;
			font-size: 14px;
			color: var(--muted);
		}
		.dot { width: 10px; height: 10px; border-radius: 50%; background: #f25f4c; box-shadow: 0 0 0 6px rgba(242,95,76,0.15); }
		.dot.ok { background: #2ec4b6; box-shadow: 0 0 0 6px rgba(46,196,182,0.18); }
		.footer { margin-top: 16px; font-size: 13px; color: var(--muted); }
	</style>
</head>
<body>
	<div class="card">
		<h1>Hovercraft Control</h1>
		<p class="subtitle">Live lift control (0-100%). Values stream over WebSocket.</p>

		<div class="slider-row">
			<label for="lift">Lift <span class="value-pill" id="liftVal">0%</span></label>
			<input id="lift" type="range" min="0" max="100" step="1" value="0" />
		</div>

		<div class="status">
			<div id="wsDot" class="dot"></div>
			<span id="wsStatus">Connecting...</span>
		</div>
		<div class="footer">Keep this tab open while piloting.</div>
	</div>

	<script>
		const lift = document.getElementById('lift');
		const liftVal = document.getElementById('liftVal');
		const wsStatus = document.getElementById('wsStatus');
		const wsDot = document.getElementById('wsDot');

		let ws;
		let sendPending = false;
		let latestValue = lift.value;

		function connectWs() {
			const proto = location.protocol === 'https:' ? 'wss' : 'ws';
			ws = new WebSocket(`${proto}://${location.host}/ws`);

			ws.onopen = () => {
				wsStatus.textContent = 'Connected';
				wsDot.classList.add('ok');
				sendValue(latestValue, true);
			};

			ws.onclose = () => {
				wsStatus.textContent = 'Reconnecting...';
				wsDot.classList.remove('ok');
				setTimeout(connectWs, 750);
			};

			ws.onerror = () => {
				wsStatus.textContent = 'Error (retrying)';
				wsDot.classList.remove('ok');
			};

			ws.onmessage = (evt) => {
				// Server may echo {"lift": number}
				try {
					const data = JSON.parse(evt.data);
					if (typeof data.lift === 'number') {
						lift.value = data.lift;
						updateLabel(data.lift);
					}
				} catch (_) { /* ignore parse errors */ }
			};
		}

		function updateLabel(val) {
			liftVal.textContent = `${Math.round(val)}%`;
		}

		function sendValue(val, force = false) {
			latestValue = val;
			if (!ws || ws.readyState !== WebSocket.OPEN) return;
			if (!force && sendPending) return;
			sendPending = true;
			requestAnimationFrame(() => {
				ws.send(JSON.stringify({ lift: Number(val) }));
				sendPending = false;
			});
		}

		lift.addEventListener('input', (e) => {
			const val = e.target.value;
			updateLabel(val);
			sendValue(val);
		});

		updateLabel(lift.value);
		connectWs();
	</script>
</body>
</html>
)rawliteral";
} // namespace

NetworkPiloting::NetworkPiloting() : server_(80), ws_("/ws"), lift_(0.0f) {}

void NetworkPiloting::begin()
{
	ws_.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
		handleWebSocketEvent(server, client, type, arg, data, len);
	});

	server_.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
		request->send_P(200, "text/html", CONTROLLER_HTML);
	});

	server_.onNotFound([](AsyncWebServerRequest *request) {
		request->send(404, "text/plain", "Not found");
	});

	server_.addHandler(&ws_);
	server_.begin();
	Serial.println("NetworkPiloting server started (WebSocket /ws)");
}

void NetworkPiloting::loop()
{
	ws_.cleanupClients();
}

void NetworkPiloting::setLiftCallback(const std::function<void(float)> &callback)
{
	onLift_ = callback;
}

float NetworkPiloting::getLift() const
{
	return lift_;
}

void NetworkPiloting::handleWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
	if (type == WS_EVT_CONNECT)
	{
		client->text("{\"status\":\"connected\"}");
		return;
	}

	if (type == WS_EVT_DATA)
	{
		// Expecting JSON like {"lift": 42}
		String payload;
		payload.reserve(len + 1);
		for (size_t i = 0; i < len; i++)
		{
			payload += static_cast<char>(data[i]);
		}

		int keyPos = payload.indexOf("\"lift\"");
		if (keyPos == -1)
		{
			return;
		}
		int colonPos = payload.indexOf(':', keyPos);
		if (colonPos == -1)
		{
			return;
		}

		float newLift = payload.substring(colonPos + 1).toFloat();
		if (isnan(newLift))
		{
			return;
		}

		// Clamp to [0,100]
		if (newLift < 0.0f)
		{
			newLift = 0.0f;
		}
		else if (newLift > 100.0f)
		{
			newLift = 100.0f;
		}

		lift_ = newLift;
		if (onLift_)
		{
			onLift_(lift_);
		}

		// Echo back latest value to keep UI in sync if multiple clients
		server->textAll(String("{\"lift\":") + String(lift_, 1) + "}");
	}
}