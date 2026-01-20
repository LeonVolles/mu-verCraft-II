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

// Defined in src/hovercraft_variables.cpp (project-level config)
extern const float global_WebLiftPresetPercent_Array[];
extern const size_t global_WebLiftPresetPercent_Array_len;
extern const int global_WebLiftPresetPercent_Array_startIndex;
extern const uint16_t global_WebServerPort;
extern const float global_BatteryVoltageLow_WarningLow;
extern const float global_BatteryVoltageLow_MotorCutoffLow;

// Embedded controller page so no external filesystem is required
static const char CONTROLLER_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
	<meta charset="UTF-8" />
	<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no, viewport-fit=cover" />
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
		html, body {
			height: 100%;
			overflow: hidden;
			overscroll-behavior: none;
			touch-action: none;
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
			-webkit-text-size-adjust: 100%;
			-webkit-user-select: none;
			user-select: none;
			-webkit-touch-callout: none;
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
		.mode-btn {
			width: 26px;
			height: 26px;
			padding: 0;
			border-radius: 8px;
			border: 1px solid rgba(255,255,255,0.10);
			background: linear-gradient(135deg, rgba(255,255,255,0.10), rgba(255,255,255,0.04));
			color: var(--text);
			font-weight: 800;
			font-size: 13px;
			line-height: 26px;
			cursor: pointer;
			transition: transform 120ms ease, background 120ms ease, box-shadow 120ms ease;
		}
		.mode-btn.on {
			background: linear-gradient(135deg, #18c27a, #0fa35f);
			border-color: rgba(62,213,152,0.45);
			box-shadow: 0 10px 18px rgba(24,194,122,0.18);
		}
		.mode-btn:disabled {
			opacity: 0.55;
			cursor: not-allowed;
			background: rgba(255,255,255,0.04);
			box-shadow: none;
		}
		.mode-btn:active { transform: translateY(1px); }
		.heading-box {
			width: 26px;
			height: 26px;
			border-radius: 8px;
			border: 1px solid rgba(255,255,255,0.10);
			background: rgba(255,255,255,0.04);
			color: rgba(255,255,255,0.78);
			font-weight: 800;
			font-size: 12px;
			line-height: 26px;
			text-align: center;
			font-variant-numeric: tabular-nums;
			user-select: none;
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
		.metric.batt-warn {
			background: linear-gradient(135deg, rgba(163,91,0,0.55), rgba(88,44,0,0.60));
			border-color: rgba(255,162,74,0.55);
			color: #ffe2c2;
		}
		.metric.batt-cutoff {
			background: linear-gradient(135deg, rgba(179,36,36,0.60), rgba(125,24,24,0.65));
			border-color: rgba(255,92,92,0.55);
			color: #ffe1e1;
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
			height: 14px;
			background: linear-gradient(90deg, var(--accent), var(--accent-2));
			border-radius: 12px;
			outline: none;
			box-shadow: inset 0 0 0 1px rgba(0,0,0,0.25);
		}
		.range::-webkit-slider-thumb {
			-webkit-appearance: none;
			appearance: none;
			width: 44px;
			height: 44px;
			background: #0b1321;
			border-radius: 50%;
			border: 3px solid var(--accent);
			box-shadow: 0 5px 12px rgba(0,0,0,0.28);
			cursor: pointer;
			transition: transform 80ms ease;
		}
		.range:active::-webkit-slider-thumb { transform: scale(1.05); }
		.vertical-wrap { display: flex; justify-content: center; align-items: center; height: 100%; }
		.vertical-range { writing-mode: bt-lr; transform: rotate(-90deg); width: 260px; height: 54px; }
		.controls-row {
			display: flex;
			align-items: center;
			justify-content: space-between;
			gap: 18px;
			margin-top: 8px;
		}
		.joystick-slot {
			width: 260px;
			height: 260px;
			display: flex;
			align-items: center;
			justify-content: center;
			touch-action: none;
			user-select: none;
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
			width: 240px;
			height: 54px;
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
				<div class="status" style="justify-self:center;"><span id="headingBox" class="heading-box">---</span><div id="wsDot" class="dot"></div><span id="wsStatus">Connecting...</span><button id="modeBtn" class="mode-btn" type="button" title="Autonomous mode (coming soon)" disabled>M</button></div>
				<div class="metric" id="metricCurrent">Used: --- mAh</div>
				<button id="armBtn" class="arm-btn off" style="justify-self:flex-end;">Motors OFF</button>
			</div>
			<div class="controls-row">
				<div class="joystick-slot" id="thrustPad">
					<div class="vertical-wrap">
						<input id="thrust" class="range vertical-range thrust-range" type="range" min="-100" max="100" step="1" value="0" />
					</div>
				</div>
				<div class="video-frame">
					<div class="video-inner"></div>
				</div>
				<div class="joystick-slot" id="steerPad">
					<div class="vertical-wrap">
						<input id="steer" class="range steer-range" type="range" min="-100" max="100" step="1" value="0" />
					</div>
				</div>
			</div>
		</div>

	<script>
		// Prevent accidental page scroll/zoom on mobile while piloting.
		// (iOS Safari can still try pinch-zoom unless we block these.)
		document.addEventListener('gesturestart', (e) => e.preventDefault());
		document.addEventListener('gesturechange', (e) => e.preventDefault());
		document.addEventListener('gestureend', (e) => e.preventDefault());
		document.addEventListener('dblclick', (e) => e.preventDefault(), { passive: false });
		document.addEventListener('touchmove', (e) => {
			// Allow native range dragging if the user directly grabs the slider.
			const t = e.target;
			if (t && (t.id === 'thrust' || t.id === 'steer')) return;
			e.preventDefault();
		}, { passive: false });

		const thrust = document.getElementById('thrust');
		const steer = document.getElementById('steer');
		const thrustPad = document.getElementById('thrustPad');
		const steerPad = document.getElementById('steerPad');
		const wsStatus = document.getElementById('wsStatus');
		const wsDot = document.getElementById('wsDot');
		const headingBox = document.getElementById('headingBox');
		const modeBtn = document.getElementById('modeBtn');
		const armBtn = document.getElementById('armBtn');
		const liftBtn = document.getElementById('liftBtn');
		const metricBatt = document.getElementById('metricBatt');
		const metricCurrent = document.getElementById('metricCurrent');

		let ws;
		let sendPending = false;
		let pollTimer = null;
		const state = { lift: 0, thrust: 0, steering: 0, motorsEnabled: false, autoMode: false };
		let autoModeRequested = false;
		// Track active pointer IDs to make "spring back to 0" reliable on mobile.
		// (Otherwise, a late range 'input' event can overwrite the reset.)
		const active = {
			thrust: { down: false, pointerId: null },
			steering: { down: false, pointerId: null }
		};
		const LIFT_PRESETS = __LIFT_PRESETS__; // injected JSON array (percent values)
		const LIFT_START_INDEX = __LIFT_START_INDEX__;
		const BATT_WARN_V = __BATT_WARN_V__;
		const BATT_CUTOFF_V = __BATT_CUTOFF_V__;
		let nextLiftPresetIndex = clamp(Number(LIFT_START_INDEX) || 0, 0, Math.max(0, LIFT_PRESETS.length - 1));

		function updateBatteryUi(voltage) {
			metricBatt.classList.remove('batt-warn', 'batt-cutoff');
			if (typeof voltage !== 'number' || !isFinite(voltage)) return;
			if (voltage <= Number(BATT_CUTOFF_V)) {
				metricBatt.classList.add('batt-cutoff');
			} else if (voltage <= Number(BATT_WARN_V)) {
				metricBatt.classList.add('batt-warn');
			}
		}

		function updateLabels() {
			armBtn.textContent = state.motorsEnabled ? 'Motors ON' : 'Motors OFF';
			armBtn.classList.toggle('on', state.motorsEnabled);
			armBtn.classList.toggle('off', !state.motorsEnabled);
			const liftPct = clamp(Number(state.lift) || 0, 0, 100);
			liftBtn.textContent = `Lift ${Math.round(liftPct)}%`;
			liftBtn.classList.toggle('on', liftPct > 0.5);
			liftBtn.classList.toggle('off', liftPct <= 0.5);
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

		function setAutoModeUi(on) {
			autoModeRequested = !!on;
			modeBtn.classList.toggle('on', autoModeRequested);
		}

		async function pollDebugOnce() {
			try {
				const res = await fetch('/debug', { cache: 'no-store' });
				if (!res.ok) return;
				const j = await res.json();

				const yaw = j && j.yaw && typeof j.yaw.deg === 'number' ? j.yaw.deg : NaN;
				if (isFinite(yaw)) {
					const deg = Math.round(((yaw % 360) + 360) % 360);
					headingBox.textContent = String(deg);
				}

				const autoMode = j && j.mode && typeof j.mode.autoMode === 'boolean' ? j.mode.autoMode : null;
				if (autoMode !== null) {
					state.autoMode = autoMode;
					setAutoModeUi(state.autoMode);
				}

				const battV = j && j.battery && typeof j.battery.V === 'number' ? j.battery.V : NaN;
				if (isFinite(battV)) {
					metricBatt.textContent = `Batt: ${battV.toFixed(2)} V`;
					updateBatteryUi(battV);
				}
				const usedMah = j && j.battery && typeof j.battery.mAh === 'number' ? j.battery.mAh : NaN;
				if (isFinite(usedMah)) {
					metricCurrent.textContent = `Used: ${usedMah.toFixed(0)} mAh`;
				}
			} catch (_) { /* ignore */ }
		}

		function startDebugPolling() {
			if (pollTimer != null) return;
			pollDebugOnce();
			pollTimer = setInterval(pollDebugOnce, 250);
		}
		function stopDebugPolling() {
			if (pollTimer == null) return;
			clearInterval(pollTimer);
			pollTimer = null;
		}

		function connectWs() {
			const proto = location.protocol === 'https:' ? 'wss' : 'ws';
			ws = new WebSocket(`${proto}://${location.host}/ws`);

			ws.onopen = () => {
				setStatus('Connected', true);
				modeBtn.disabled = false;
				startDebugPolling();
				sendState(true);
			};

			ws.onclose = () => {
				setStatus('Reconnecting...', false);
				modeBtn.disabled = true;
				stopDebugPolling();
				state.autoMode = false;
				setAutoModeUi(false);
				updateBatteryUi(NaN);
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
					}
					updateLabels();
				} catch (_) { /* ignore parse errors */ }
			};
		}

		function clamp(val, min, max) { return Math.min(max, Math.max(min, val)); }

		function setActive(prop, pointerId) {
			active[prop].down = true;
			active[prop].pointerId = pointerId;
		}
		function clearActive(prop) {
			active[prop].down = false;
			active[prop].pointerId = null;
		}
		function isActive(prop, pointerId) {
			if (!active[prop].down) return false;
			if (pointerId == null) return true;
			return active[prop].pointerId === pointerId;
		}
		function resetControl(prop, el, forceSend = true) {
			state[prop] = 0;
			el.value = 0;
			clearActive(prop);
			updateLabels();
			sendState(forceSend);
		}

		function resetAllControls(reason) {
			// Only reset if something was actively controlled.
			const anyActive = active.thrust.down || active.steering.down;
			resetControl('thrust', thrust, anyActive);
			resetControl('steering', steer, anyActive);
		}

		function addSpringControl(el, prop) {
			el.addEventListener('input', (e) => {
				// Some mobile browsers dispatch a final 'input' after pointerup.
				// Ignore it unless this control is currently active.
				if (!active[prop].down) return;
				state[prop] = clamp(Number(e.target.value), -100, 100);
				updateLabels();
				sendState();
			});
			el.addEventListener('pointerdown', (e) => {
				setActive(prop, e.pointerId);
				try { el.setPointerCapture(e.pointerId); } catch (_) { /* ignore */ }
			});
			el.addEventListener('pointerup', (e) => {
				if (!isActive(prop, e.pointerId)) return;
				resetControl(prop, el, true);
			});
			el.addEventListener('pointercancel', (e) => {
				if (!isActive(prop, e.pointerId)) return;
				resetControl(prop, el, true);
			});
			el.addEventListener('lostpointercapture', () => {
				if (!active[prop].down) return;
				resetControl(prop, el, true);
			});
		}

		// Drag-anywhere controls: you don't need to grab the thumb.
		// This is much easier to manipulate without looking.
		function addPadControl(padEl, sliderEl, prop, axis /* 'x' or 'y' */) {
			if (!padEl) return;
			const deadzone = 0.06; // fraction of full deflection
			const expo = 0.35;     // 0..1; higher = softer near center

			function applyExpo(n) {
				// Smooth expo: mix linear with cubic.
				return (1 - expo) * n + expo * n * n * n;
			}

			function updateFromEvent(e) {
				const r = padEl.getBoundingClientRect();
				let n = 0;
				if (axis === 'y') {
					// top = +1, bottom = -1
					const y = (e.clientY - r.top);
					n = -((y - r.height / 2) / (r.height / 2));
				} else {
					// left = -1, right = +1
					const x = (e.clientX - r.left);
					n = ((x - r.width / 2) / (r.width / 2));
				}
				n = clamp(n, -1, 1);
				// deadzone
				if (Math.abs(n) < deadzone) n = 0;
				// expo
				n = applyExpo(n);
				const v = clamp(n * 100, -100, 100);
				state[prop] = v;
				sliderEl.value = v;
				updateLabels();
				sendState();
			}

			padEl.addEventListener('pointerdown', (e) => {
				e.preventDefault();
				padEl.setPointerCapture(e.pointerId);
				setActive(prop, e.pointerId);
				updateFromEvent(e);
			});
			padEl.addEventListener('pointermove', (e) => {
				if (!isActive(prop, e.pointerId)) return;
				updateFromEvent(e);
			});
			padEl.addEventListener('pointerup', (e) => {
				if (!isActive(prop, e.pointerId)) return;
				resetControl(prop, sliderEl, true);
			});
			padEl.addEventListener('pointercancel', () => {
				resetControl(prop, sliderEl, true);
			});
			padEl.addEventListener('lostpointercapture', () => {
				if (!active[prop].down) return;
				resetControl(prop, sliderEl, true);
			});
		}

		addSpringControl(thrust, 'thrust');
		addSpringControl(steer, 'steering');
		addPadControl(thrustPad, thrust, 'thrust', 'y');
		addPadControl(steerPad, steer, 'steering', 'x');

		// Safety: if the browser loses focus or the tab is backgrounded,
		// always spring everything back to 0.
		window.addEventListener('blur', () => resetAllControls('blur'));
		document.addEventListener('visibilitychange', () => {
			if (document.hidden) resetAllControls('hidden');
		});
		window.addEventListener('pointerup', (e) => {
			// Fallback in case some elements don't receive pointerup.
			if (isActive('thrust', e.pointerId)) resetControl('thrust', thrust, true);
			if (isActive('steering', e.pointerId)) resetControl('steering', steer, true);
		});

		armBtn.addEventListener('click', () => {
			state.motorsEnabled = !state.motorsEnabled;
			if (!state.motorsEnabled) {
				state.lift = 0;
			}
			updateLabels();
			sendState(true);
		});

		modeBtn.addEventListener('click', () => {
			state.autoMode = !state.autoMode;
			setAutoModeUi(state.autoMode);
			sendState(true);
		});

		liftBtn.addEventListener('click', () => {
			const liftPct = clamp(Number(state.lift) || 0, 0, 100);
			if (liftPct > 0.5) {
				// Turn lift off; advance preset index for the next ON toggle.
				state.lift = 0;
				nextLiftPresetIndex = (nextLiftPresetIndex + 1) % Math.max(1, LIFT_PRESETS.length);
			} else {
				// Turn lift on to next preset.
				const v = Number(LIFT_PRESETS[nextLiftPresetIndex]) || 0;
				state.lift = clamp(v, 0, 100);
			}
			updateLabels();
			sendState(true);
		});

		updateLabels();
		connectWs();
	</script>
</body>
</html>
)rawliteral";

// Minimal PID tuning page (no external assets).
static const char PID_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
	<meta charset="utf-8" />
	<meta name="viewport" content="width=device-width, initial-scale=1" />
	<title>PID Tuning</title>
	<style>
		:root { color-scheme: dark; }
		body { font-family: system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif; margin: 0; padding: 16px; background: #0b0d12; color: #e7e9ee; }
		h1 { margin: 0 0 10px; font-size: 20px; }
		.card { background: #121724; border: 1px solid #232b3d; border-radius: 12px; padding: 12px; margin: 12px 0; }
		.row { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px; }
		label { display: block; font-size: 12px; opacity: 0.85; margin-bottom: 6px; }
		input { width: 100%; box-sizing: border-box; padding: 10px; border-radius: 10px; border: 1px solid #2a3550; background: #0d1220; color: #e7e9ee; font-size: 14px; }
		.btnRow { display: flex; gap: 10px; align-items: center; }
		button { padding: 10px 14px; border-radius: 10px; border: 1px solid #2a3550; background: #1b2540; color: #e7e9ee; font-weight: 600; }
		button:active { transform: translateY(1px); }
		.status { font-size: 13px; opacity: 0.9; }
		a { color: #93b4ff; text-decoration: none; }
	</style>
</head>
<body>
	<h1>PID Tuning</h1>
	<div class="status" id="status">Loading…</div>

	<div class="card">
		<div style="margin-bottom:8px; font-weight:700;">Yaw-rate PID (inner loop)</div>
		<div class="row">
			<div><label for="yawKp">Kp</label><input id="yawKp" inputmode="decimal" /></div>
			<div><label for="yawKi">Ki</label><input id="yawKi" inputmode="decimal" /></div>
			<div><label for="yawKd">Kd</label><input id="yawKd" inputmode="decimal" /></div>
		</div>
	</div>

	<div class="card">
		<div style="margin-bottom:8px; font-weight:700;">Heading PID (outer loop)</div>
		<div class="row">
			<div><label for="headingKp">Kp</label><input id="headingKp" inputmode="decimal" /></div>
			<div><label for="headingKi">Ki</label><input id="headingKi" inputmode="decimal" /></div>
			<div><label for="headingKd">Kd</label><input id="headingKd" inputmode="decimal" /></div>
		</div>
	</div>

	<div class="btnRow">
		<button id="refreshBtn">Refresh</button>
		<button id="applyBtn">Apply</button>
		<div class="status" style="margin-left:auto;"><a href="/">Back</a></div>
	</div>

	<script>
		const statusEl = document.getElementById('status');
		const ids = ['yawKp','yawKi','yawKd','headingKp','headingKi','headingKd'];
		const el = Object.fromEntries(ids.map(id => [id, document.getElementById(id)]));

		function setStatus(text) { statusEl.textContent = text; }
		function fmt(v) {
			if (typeof v !== 'number' || !isFinite(v)) return '';
			return String(v);
		}
		function readNum(id) {
			const s = (el[id].value || '').trim();
			const v = Number(s);
			return isFinite(v) ? v : NaN;
		}

		async function refresh() {
			try {
				setStatus('Loading…');
				const res = await fetch('/pid.json', { cache: 'no-store' });
				const j = await res.json();
				el.yawKp.value = fmt(j?.yaw?.kp);
				el.yawKi.value = fmt(j?.yaw?.ki);
				el.yawKd.value = fmt(j?.yaw?.kd);
				el.headingKp.value = fmt(j?.heading?.kp);
				el.headingKi.value = fmt(j?.heading?.ki);
				el.headingKd.value = fmt(j?.heading?.kd);
				setStatus('Ready');
			} catch (e) {
				setStatus('Error loading /pid.json');
			}
		}

		async function apply() {
			const body = new URLSearchParams({
				yawKp: String(readNum('yawKp')),
				yawKi: String(readNum('yawKi')),
				yawKd: String(readNum('yawKd')),
				headingKp: String(readNum('headingKp')),
				headingKi: String(readNum('headingKi')),
				headingKd: String(readNum('headingKd')),
			}).toString();

			try {
				setStatus('Applying…');
				const res = await fetch('/pid', {
					method: 'POST',
					headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
					body,
				});
				const j = await res.json().catch(() => null);
				if (!res.ok || !j?.ok) {
					setStatus(j?.error || 'Update rejected');
					return;
				}
				setStatus('Applied');
				setTimeout(refresh, 200);
			} catch (e) {
				setStatus('Error applying PID');
			}
		}

		document.getElementById('refreshBtn').addEventListener('click', refresh);
		document.getElementById('applyBtn').addEventListener('click', apply);
		refresh();
	</script>
</body>
</html>
)rawliteral";

NetworkPiloting::NetworkPiloting()
	: server_(global_WebServerPort),
	  ws_("/ws"),
	  wsMutex_(xSemaphoreCreateMutex()),
	  lift_(0.0f),
	  thrust_(0.0f),
	  steering_(0.0f),
	  motorsEnabled_(false),
	  autoModeRequested_(false)
{
}

bool NetworkPiloting::lockWs(uint32_t timeoutTicks)
{
	if (wsMutex_ == nullptr)
	{
		return true;
	}
	return xSemaphoreTake(wsMutex_, timeoutTicks) == pdTRUE;
}

void NetworkPiloting::unlockWs()
{
	if (wsMutex_ == nullptr)
	{
		return;
	}
	xSemaphoreGive(wsMutex_);
}

void NetworkPiloting::begin()
{
	ws_.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
				{ handleWebSocketEvent(server, client, type, arg, data, len); });

	auto sendControllerHtml = [](AsyncWebServerRequest *request)
	{
		// We store the HTML in flash (PROGMEM) but inject a few tunables at runtime.
		String page = FPSTR(CONTROLLER_HTML);

		// Inject lift presets as a JSON array literal (e.g. [45,60,75]).
		String presets = "[";
		for (size_t i = 0; i < global_WebLiftPresetPercent_Array_len; i++)
		{
			presets += String(global_WebLiftPresetPercent_Array[i], 0);
			if (i + 1 < global_WebLiftPresetPercent_Array_len)
				presets += ",";
		}
		presets += "]";

		int startIndex = global_WebLiftPresetPercent_Array_startIndex;
		if (startIndex < 0)
			startIndex = 0;
		if (global_WebLiftPresetPercent_Array_len == 0)
			startIndex = 0;
		else if ((size_t)startIndex >= global_WebLiftPresetPercent_Array_len)
			startIndex = (int)global_WebLiftPresetPercent_Array_len - 1;

		page.replace("__LIFT_PRESETS__", presets);
		page.replace("__LIFT_START_INDEX__", String(startIndex));
		page.replace("__BATT_WARN_V__", String(global_BatteryVoltageLow_WarningLow, 2));
		page.replace("__BATT_CUTOFF_V__", String(global_BatteryVoltageLow_MotorCutoffLow, 2));
		request->send(200, "text/html", page);
	};

	// Serve embedded page (with runtime-injected presets)
	server_.on("/", HTTP_GET, sendControllerHtml);
	server_.on("/controller.html", HTTP_GET, sendControllerHtml);

	// Debug endpoint: returns JSON. Useful when no serial monitor is attached.
	server_.on("/debug", HTTP_GET, [this](AsyncWebServerRequest *request)
			   {
				char msg[768];
				size_t n = 0;
				if (debugProvider_)
				{
					n = debugProvider_(msg, sizeof(msg));
				}
				if (n == 0)
				{
					const int nn = snprintf(
						msg,
						sizeof(msg),
						"{\"ok\":true,\"lift\":%.1f,\"thrust\":%.1f,\"steering\":%.1f,\"motorsEnabled\":%s,\"autoMode\":%s}",
						(double)lift_,
						(double)thrust_,
						(double)steering_,
						motorsEnabled_ ? "true" : "false",
						autoModeRequested_ ? "true" : "false");
					n = (nn > 0) ? (size_t)nn : 0;
				}
				if (n >= sizeof(msg))
				{
					n = sizeof(msg) - 1;
				}
				msg[n] = '\0';
				request->send(200, "application/json", msg); });

	// PID tuning page + endpoints.
	server_.on("/pid", HTTP_GET, [](AsyncWebServerRequest *request)
			   { request->send_P(200, "text/html", PID_HTML); });

	server_.on("/pid.json", HTTP_GET, [this](AsyncWebServerRequest *request)
			   {
				PidTunings t{0, 0, 0, 0, 0, 0};
				if (pidGetProvider_)
				{
					pidGetProvider_(t);
				}
				char msg[256];
				int n = snprintf(msg, sizeof(msg),
								   "{\"ok\":true,\"yaw\":{\"kp\":%.6f,\"ki\":%.6f,\"kd\":%.6f},\"heading\":{\"kp\":%.6f,\"ki\":%.6f,\"kd\":%.6f}}",
								   (double)t.yawKp, (double)t.yawKi, (double)t.yawKd,
								   (double)t.headingKp, (double)t.headingKi, (double)t.headingKd);
				if (n < 0)
					n = 0;
				if ((size_t)n >= sizeof(msg))
					n = (int)sizeof(msg) - 1;
				msg[n] = '\0';
				request->send(200, "application/json", msg); });

	server_.on("/pid", HTTP_POST, [this](AsyncWebServerRequest *request)
			   {
				auto getFloatParam = [request](const char *name, float &out) -> bool
				{
					if (!request->hasParam(name, true))
						return false;
					AsyncWebParameter *p = request->getParam(name, true);
					if (p == nullptr)
						return false;
					const String &v = p->value();
					out = v.toFloat();
					return isfinite(out);
				};

				PidTunings t;
				if (!getFloatParam("yawKp", t.yawKp) || !getFloatParam("yawKi", t.yawKi) || !getFloatParam("yawKd", t.yawKd) ||
					!getFloatParam("headingKp", t.headingKp) || !getFloatParam("headingKi", t.headingKi) || !getFloatParam("headingKd", t.headingKd))
				{
					request->send(400, "application/json", "{\"ok\":false,\"error\":\"Missing/invalid fields\"}");
					return;
				}

				bool ok = false;
				if (pidSetHandler_)
				{
					ok = pidSetHandler_(t);
				}
				if (!ok)
				{
					request->send(400, "application/json", "{\"ok\":false,\"error\":\"PID update rejected\"}");
					return;
				}
				request->send(200, "application/json", "{\"ok\":true}"); });
	server_.onNotFound(sendControllerHtml);

	server_.addHandler(&ws_);
	server_.begin();
	Serial.println("NetworkPiloting server started (WebSocket /ws, embedded controller page)");
}

void NetworkPiloting::loop()
{
	if (!lockWs(0))
	{
		return;
	}
	ws_.cleanupClients();
	unlockWs();
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

void NetworkPiloting::setAutoModeCallback(const std::function<void(bool)> &callback)
{
	onAutoMode_ = callback;
}

void NetworkPiloting::setPidGetProvider(const std::function<void(PidTunings &out)> &provider)
{
	pidGetProvider_ = provider;
}

void NetworkPiloting::setPidSetHandler(const std::function<bool(const PidTunings &in)> &handler)
{
	pidSetHandler_ = handler;
}

void NetworkPiloting::setDebugProvider(const std::function<size_t(char *out, size_t outSize)> &provider)
{
	debugProvider_ = provider;
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

bool NetworkPiloting::autoModeRequested() const
{
	return autoModeRequested_;
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
		autoModeRequested_ = false;
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

		// Parse fields independently (lightweight, no full JSON dep)
		auto parseNumber = [&payload](const char *key, float &outVal, float minVal, float maxVal)
		{
			int pos = payload.indexOf(key);
			if (pos == -1)
				return false;
			int cPos = payload.indexOf(':', pos);
			if (cPos == -1)
				return false;
			float val = payload.substring(cPos + 1).toFloat();
			if (isnan(val))
				return false;
			if (val < minVal)
				val = minVal;
			if (val > maxVal)
				val = maxVal;
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

		int autoPos = payload.indexOf("\"autoMode\"");
		if (autoPos != -1)
		{
			int colon = payload.indexOf(':', autoPos);
			if (colon != -1)
			{
				bool enable = payload.substring(colon + 1).startsWith("true");
				if (enable != autoModeRequested_)
				{
					autoModeRequested_ = enable;
					if (onAutoMode_)
					{
						onAutoMode_(enable);
					}
				}
				updated = true;
			}
		}

		if (updated)
		{
			char msg[192];
			const int n = snprintf(
				msg,
				sizeof(msg),
				"{\"lift\":%.1f,\"thrust\":%.1f,\"steering\":%.1f,\"motorsEnabled\":%s,\"autoMode\":%s}",
				(double)lift_,
				(double)thrust_,
				(double)steering_,
				motorsEnabled_ ? "true" : "false",
				autoModeRequested_ ? "true" : "false");
			if (n > 0)
			{
				if (lockWs(0))
				{
					server->textAll(msg);
					unlockWs();
				}
			}
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
	if (!lockWs(0))
	{
		return;
	}
	if (ws_.count() == 0)
	{
		unlockWs();
		return; // no clients to receive this frame
	}
	// Compact JSON broadcast to all clients; values are in volts/amps/mAh already.
	char msg[96];
	const int n = snprintf(
		msg,
		sizeof(msg),
		"{\"batt\":%.2f,\"curr\":%.2f,\"mah\":%.0f}",
		(double)voltage,
		(double)current,
		(double)usedMah);
	if (n > 0)
	{
		ws_.textAll(msg);
	}
	unlockWs();
}

void NetworkPiloting::sendHeading(float heading_deg)
{
	if (!lockWs(0))
	{
		return;
	}
	if (ws_.count() == 0)
	{
		unlockWs();
		return;
	}
	if (!isfinite(heading_deg))
	{
		unlockWs();
		return;
	}
	char msg[48];
	const int n = snprintf(msg, sizeof(msg), "{\"yaw\":%.1f}", (double)heading_deg);
	if (n > 0)
	{
		ws_.textAll(msg);
	}
	unlockWs();
}

void NetworkPiloting::sendAutoMode(bool enabled)
{
	if (!lockWs(0))
	{
		return;
	}
	if (ws_.count() == 0)
	{
		unlockWs();
		return;
	}
	ws_.textAll(enabled ? "{\"autoMode\":true}" : "{\"autoMode\":false}");
	unlockWs();
}