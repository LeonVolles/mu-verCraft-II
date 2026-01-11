// #include "network_piloting.h"

// NetworkPiloting::NetworkPiloting() {
//     // TODO: Initialize member variables
// }

// NetworkPiloting::~NetworkPiloting() {
//     // TODO: Cleanup resources
// }

// void NetworkPiloting::startServer() {
//     // TODO: Start web server
// }

// void NetworkPiloting::processClient() {
//     // TODO: Process client requests
// }

// void NetworkPiloting::setTargetThrust(float v) {
#include "network_piloting.h"
#include <pgmspace.h>

// Embedded controller page so no external filesystem is required
static const char CONTROLLER_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
	<meta charset="UTF-8" />
	<meta name="viewport" content="width=device-width, initial-scale=1.0" />
	<title>Hovercraft Control</title>
	<style>
		:root {
			--bg: #040b14;
			--panel: #0e1b2a;
			--accent: #39c0ff;
			--accent-2: #7cf29c;
			--warn: #ff5c5c;
			--text: #f1f6ff;
			--muted: #8ea2c2;
			--shadow: rgba(0, 0, 0, 0.45);
		}
		* { box-sizing: border-box; }
		body {
			margin: 0;
			font-family: "Segoe UI", system-ui, -apple-system, sans-serif;
			background: #02060c;
			color: var(--text);
			min-height: 100vh;
			display: flex;
			align-items: center;
			justify-content: center;
			padding: 0;
		}
		.card {
			position: relative;
			width: 100%;
			max-width: 1200px;
			background: transparent;
			border-radius: 0;
			box-shadow: none;
			padding: 12px;
			border: none;
		}
		.topbar {
			display: grid;
			grid-template-columns: auto auto auto auto auto;
			align-items: center;
			gap: 12px;
			margin-bottom: 12px;
		}
		.status {
			display: inline-flex;
			align-items: center;
			gap: 8px;
			color: var(--muted);
			font-size: 14px;
			justify-content: center;
		}
		.metric {
			display: inline-flex;
			align-items: center;
			gap: 6px;
			padding: 8px 12px;
			border-radius: 12px;
			background: rgba(255,255,255,0.04);
			border: 1px solid rgba(255,255,255,0.06);
			color: var(--text);
			font-size: 13px;
			min-width: 120px;
			justify-content: center;
		}
		.dot { width: 11px; height: 11px; border-radius: 50%; background: var(--warn); box-shadow: 0 0 0 6px rgba(255,92,92,0.12); }
		.dot.ok { background: #3ed598; box-shadow: 0 0 0 6px rgba(62,213,152,0.18); }
		.lift-btn,
		.arm-btn {
			border: 1px solid rgba(255,255,255,0.08);
			background: linear-gradient(135deg, #15314b, #0f2336);
			color: var(--text);
			padding: 10px 16px;
			border-radius: 12px;
			font-weight: 700;
			letter-spacing: 0.2px;
			cursor: pointer;
			transition: transform 120ms ease, box-shadow 120ms ease, background 120ms ease;
			box-shadow: 0 10px 18px rgba(0,0,0,0.25);
		}
		.lift-btn.on,
		.arm-btn.on { background: linear-gradient(135deg, #18c27a, #0fa35f); box-shadow: 0 12px 24px rgba(24,194,122,0.25); }
		.lift-btn.off,
		.arm-btn.off { background: linear-gradient(135deg, #b32424, #7d1818); border-color: rgba(255,92,92,0.4); box-shadow: 0 12px 24px rgba(255,92,92,0.2); }
		.lift-btn:active,
		.arm-btn:active { transform: translateY(1px); }
		.grid {
			display: grid;
			grid-template-columns: 1fr;
			grid-template-rows: auto;
			gap: 14px;
			align-items: stretch;
		}
		.panel {
			background: rgba(255,255,255,0.03);
			border: 1px solid rgba(255,255,255,0.05);
			border-radius: 16px;
			padding: 12px;
			height: 100%;
		}
		.panel.view { position: relative; }
		.label-row { display: flex; justify-content: space-between; align-items: center; gap: 8px; color: var(--muted); font-size: 13px; }
		.value-pill { padding: 5px 10px; border-radius: 999px; background: rgba(57,192,255,0.14); color: #7cf29c; font-variant-numeric: tabular-nums; border: 1px solid rgba(124,242,156,0.25); }
		.range {
			width: 100%;
			margin: 12px 0 4px;
			-webkit-appearance: none;
			height: 7px;
			background: linear-gradient(90deg, var(--accent), var(--accent-2));
			border-radius: 12px;
			outline: none;
			box-shadow: inset 0 0 0 1px rgba(0,0,0,0.25);
		}
		.range::-webkit-slider-thumb {
			-webkit-appearance: none;
			appearance: none;
			width: 24px;
			height: 24px;
			background: #0b1321;
			border-radius: 50%;
			border: 3px solid var(--accent);
			box-shadow: 0 5px 12px rgba(0,0,0,0.28);
			cursor: pointer;
			transition: transform 80ms ease;
		}
		.range:active::-webkit-slider-thumb { transform: scale(1.05); }
		.vertical-wrap { display: flex; justify-content: center; align-items: center; height: 100%; }
		.vertical-range { writing-mode: bt-lr; transform: rotate(-90deg); width: 200px; height: 42px; }
		.controls-row {
			display: flex;
			align-items: center;
			justify-content: space-between;
			gap: 18px;
			margin-top: 8px;
		}
		.joystick-slot {
			width: 200px;
			height: 200px;
			display: flex;
			align-items: center;
			justify-content: center;
		}
		.video-frame {
			flex: 1;
			max-width: 820px;
			min-width: 240px;
			aspect-ratio: 4 / 3;
			border-radius: 16px;
			background: radial-gradient(circle at 30% 30%, rgba(124,242,156,0.14), rgba(57,192,255,0.1)),
			            linear-gradient(135deg, rgba(57,192,255,0.08), rgba(14,27,42,0.9));
			border: 1px dashed rgba(124,242,156,0.25);
			overflow: hidden;
			position: relative;
		}
		.video-inner {
			position: absolute;
			top: 0;
			left: 0;
			width: 100%;
			height: 100%;
			background: repeating-linear-gradient(45deg, rgba(255,255,255,0.05), rgba(255,255,255,0.05) 12px, transparent 12px, transparent 24px);
			mix-blend-mode: screen;
		}
		.range.steer-range {
			background: linear-gradient(90deg, rgba(255,255,255,0.15), rgba(255,255,255,0.15));
			box-shadow: inset 0 0 0 1px rgba(255,255,255,0.1);
			writing-mode: horizontal-tb;
			width: 180px;
			height: 42px;
			transform: none;
		}
		.range.steer-range::-webkit-slider-thumb {
			background: rgba(255,255,255,0.6);
			border: 3px solid rgba(255,255,255,0.65);
		}
		.range.thrust-range {
			background: linear-gradient(90deg, rgba(255,255,255,0.15), rgba(255,255,255,0.15));
			box-shadow: inset 0 0 0 1px rgba(255,255,255,0.1);
		}
		.range.thrust-range::-webkit-slider-thumb {
			background: rgba(255,255,255,0.6);
			border: 3px solid rgba(255,255,255,0.65);
		}
		.footer { margin-top: 12px; color: var(--muted); font-size: 12px; text-align: center; }
	</style>
</head>
<body>
		<div class="card">
			<div class="footer" style="margin-bottom:8px; text-align:center;">Keep this page open while piloting. Motors default OFF and will auto-off on disconnect.</div>
			<div class="topbar">
				<button id="liftBtn" class="lift-btn off" style="justify-self:flex-start;">Lift OFF</button>
				<div class="metric" id="metricBatt">Batt: --.- V</div>
				<div class="status" style="justify-self:center;"><div id="wsDot" class="dot"></div><span id="wsStatus">Connecting...</span></div>
				<div class="metric" id="metricCurrent">Used: --- mAh</div>
				<button id="armBtn" class="arm-btn off" style="justify-self:flex-end;">Motors OFF</button>
			</div>
			<div class="controls-row">
				<div class="joystick-slot">
					<div class="vertical-wrap">
						<input id="thrust" class="range vertical-range thrust-range" type="range" min="-100" max="100" step="1" value="0" />
					</div>
				</div>
				<div class="video-frame">
					<div class="video-inner"></div>
				</div>
				<div class="joystick-slot">
					<div class="vertical-wrap">
						<input id="steer" class="range steer-range" type="range" min="-100" max="100" step="1" value="0" />
					</div>
				</div>
			</div>
		</div>

	<script>
		const thrust = document.getElementById('thrust');
		const steer = document.getElementById('steer');
		const wsStatus = document.getElementById('wsStatus');
		const wsDot = document.getElementById('wsDot');
		const armBtn = document.getElementById('armBtn');
		const liftBtn = document.getElementById('liftBtn');
		const metricBatt = document.getElementById('metricBatt');
		const metricCurrent = document.getElementById('metricCurrent');

		let ws;
		let sendPending = false;
		const state = { lift: 0, thrust: 0, steering: 0, motorsEnabled: false };
		let thrustActive = false;
		let steerActive = false;
		const LIFT_PRESET = 60; // percent lift when toggle is on
		let liftToggle = false;

		function updateLabels() {
			armBtn.textContent = state.motorsEnabled ? 'Motors ON' : 'Motors OFF';
			armBtn.classList.toggle('on', state.motorsEnabled);
			armBtn.classList.toggle('off', !state.motorsEnabled);
			const liftIsOn = Math.abs(state.lift - LIFT_PRESET) < 0.5;
			liftToggle = liftIsOn;
			liftBtn.textContent = liftToggle ? 'Lift ON' : 'Lift OFF';
			liftBtn.classList.toggle('on', liftToggle);
			liftBtn.classList.toggle('off', !liftToggle);
		}

		function sendState(force = false) {
			if (!ws || ws.readyState !== WebSocket.OPEN) return;
			if (!force && sendPending) return;
			sendPending = true;
			const payload = JSON.stringify(state);
			requestAnimationFrame(() => {
				ws.send(payload);
				sendPending = false;
			});
		}

		function setStatus(text, ok) {
			wsStatus.textContent = text;
			wsDot.classList.toggle('ok', ok);
		}

		function connectWs() {
			const proto = location.protocol === 'https:' ? 'wss' : 'ws';
			ws = new WebSocket(`${proto}://${location.host}/ws`);

			ws.onopen = () => {
				setStatus('Connected', true);
				sendState(true);
			};

			ws.onclose = () => {
				setStatus('Reconnecting...', false);
				state.motorsEnabled = false;
				state.thrust = 0;
				state.steering = 0;
				state.lift = 0;
				thrust.value = 0;
				steer.value = 0;
				updateLabels();
				setTimeout(connectWs, 800);
			};

			ws.onerror = () => {
				setStatus('Error (retrying)', false);
			};

			ws.onmessage = (evt) => {
				try {
					const data = JSON.parse(evt.data);
					if (typeof data.thrust === 'number') {
						state.thrust = data.thrust;
						thrust.value = data.thrust;
					}
					if (typeof data.steering === 'number') {
						state.steering = data.steering;
						steer.value = data.steering;
					}
					if (typeof data.motorsEnabled === 'boolean') {
						state.motorsEnabled = data.motorsEnabled;
						if (!state.motorsEnabled) {
							state.lift = 0;
							liftToggle = false;
						}
					}
					if (typeof data.batt === 'number') {
						metricBatt.textContent = `Batt: ${data.batt.toFixed(2)} V`;
					}
					if (typeof data.mah === 'number') {
						metricCurrent.textContent = `Used: ${data.mah.toFixed(0)} mAh`;
					}
					updateLabels();
				} catch (_) { /* ignore parse errors */ }
			};
		}

		function clamp(val, min, max) { return Math.min(max, Math.max(min, val)); }

		function addSpringControl(el, prop) {
			el.addEventListener('input', (e) => {
				state[prop] = clamp(Number(e.target.value), -100, 100);
				updateLabels();
				sendState();
			});
			el.addEventListener('pointerdown', () => { if (prop === 'thrust') thrustActive = true; else steerActive = true; });
			window.addEventListener('pointerup', () => {
				if ((prop === 'thrust' && thrustActive) || (prop === 'steering' && steerActive)) {
					state[prop] = 0;
					el.value = 0;
					if (prop === 'thrust') thrustActive = false; else steerActive = false;
					updateLabels();
					sendState();
				}
			});
		}

		addSpringControl(thrust, 'thrust');
		addSpringControl(steer, 'steering');

		armBtn.addEventListener('click', () => {
			state.motorsEnabled = !state.motorsEnabled;
			if (!state.motorsEnabled) {
				state.lift = 0;
				liftToggle = false;
			}
			updateLabels();
			sendState(true);
		});

		liftBtn.addEventListener('click', () => {
			liftToggle = !liftToggle;
			state.lift = liftToggle ? LIFT_PRESET : 0;
			updateLabels();
			sendState(true);
		});

		updateLabels();
		connectWs();
	</script>
</body>
</html>
)rawliteral";

NetworkPiloting::NetworkPiloting() : server_(80), ws_("/ws"), lift_(0.0f), thrust_(0.0f), steering_(0.0f), motorsEnabled_(false) {}

void NetworkPiloting::begin()
{
	ws_.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
		handleWebSocketEvent(server, client, type, arg, data, len);
	});

	// Serve embedded page from flash
	server_.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
		request->send_P(200, "text/html", CONTROLLER_HTML);
	});
	server_.on("/controller.html", HTTP_GET, [](AsyncWebServerRequest *request) {
		request->send_P(200, "text/html", CONTROLLER_HTML);
	});
	server_.onNotFound([](AsyncWebServerRequest *request) {
		request->send_P(200, "text/html", CONTROLLER_HTML);
	});

	server_.addHandler(&ws_);
	server_.begin();
	Serial.println("NetworkPiloting server started (WebSocket /ws, embedded controller page)");
}

void NetworkPiloting::loop()
{
	ws_.cleanupClients();
}

void NetworkPiloting::setLiftCallback(const std::function<void(float)> &callback)
{
	onLift_ = callback;
}

void NetworkPiloting::setThrustCallback(const std::function<void(float)> &callback)
{
	onThrust_ = callback;
}

void NetworkPiloting::setSteeringCallback(const std::function<void(float)> &callback)
{
	onSteering_ = callback;
}

void NetworkPiloting::setArmCallback(const std::function<void(bool)> &callback)
{
	onArm_ = callback;
}

float NetworkPiloting::getLift() const
{
	return lift_;
}

float NetworkPiloting::getThrust() const
{
	return thrust_;
}

float NetworkPiloting::getSteering() const
{
	return steering_;
}

bool NetworkPiloting::motorsEnabled() const
{
	return motorsEnabled_;
}

void NetworkPiloting::handleWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
	if (type == WS_EVT_CONNECT)
	{
		client->text("{\"status\":\"connected\"}");
		return;
	}

	if (type == WS_EVT_DISCONNECT)
	{
		// Fail-safe: drop motors and zero controls on disconnect
		applyArm(false);
		if (onThrust_)
		{
			onThrust_(0.0f);
		}
		if (onSteering_)
		{
			onSteering_(0.0f);
		}
		if (onLift_)
		{
			onLift_(0.0f);
		}
		thrust_ = 0.0f;
		steering_ = 0.0f;
		lift_ = 0.0f;
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

		// Parse fields independently (lightweight, no full JSON dep)
		auto parseNumber = [&payload](const char *key, float &outVal, float minVal, float maxVal) {
			int pos = payload.indexOf(key);
			if (pos == -1) return false;
			int cPos = payload.indexOf(':', pos);
			if (cPos == -1) return false;
			float val = payload.substring(cPos + 1).toFloat();
			if (isnan(val)) return false;
			if (val < minVal) val = minVal;
			if (val > maxVal) val = maxVal;
			outVal = val;
			return true;
		};

		bool updated = false;
		float newLift = lift_;
		float newThrust = thrust_;
		float newSteer = steering_;

		if (parseNumber("\"lift\"", newLift, 0.0f, 100.0f))
		{
			lift_ = newLift;
			if (onLift_)
			{
				onLift_(lift_);
			}
			updated = true;
		}

		if (parseNumber("\"thrust\"", newThrust, -100.0f, 100.0f))
		{
			thrust_ = newThrust;
			if (onThrust_)
			{
				onThrust_(thrust_);
			}
			updated = true;
		}

		if (parseNumber("\"steering\"", newSteer, -100.0f, 100.0f))
		{
			steering_ = newSteer;
			if (onSteering_)
			{
				onSteering_(steering_);
			}
			updated = true;
		}

		int armPos = payload.indexOf("\"motorsEnabled\"");
		if (armPos != -1)
		{
			int colon = payload.indexOf(':', armPos);
			if (colon != -1)
			{
				bool enable = payload.substring(colon + 1).startsWith("true");
				applyArm(enable);
				updated = true;
			}
		}

		if (updated)
		{
			server->textAll(String("{\"lift\":") + String(lift_, 1) +
			               ",\"thrust\":" + String(thrust_, 1) +
			               ",\"steering\":" + String(steering_, 1) +
			               ",\"motorsEnabled\":" + (motorsEnabled_ ? "true" : "false") + "}");
		}
	}
}

void NetworkPiloting::applyArm(bool enabled)
{
	motorsEnabled_ = enabled;
	if (onArm_)
	{
		onArm_(motorsEnabled_);
	}
}

void NetworkPiloting::sendTelemetry(float voltage, float current, float usedMah)
{
	if (ws_.count() == 0) {
		return; // no clients to receive this frame
	}
	// Compact JSON broadcast to all clients; values are in volts/amps/mAh already.
	ws_.textAll(String("{\"batt\":") + String(voltage, 2) +
	            ",\"curr\":" + String(current, 2) +
	            ",\"mah\":" + String(usedMah, 0) + "}");
}