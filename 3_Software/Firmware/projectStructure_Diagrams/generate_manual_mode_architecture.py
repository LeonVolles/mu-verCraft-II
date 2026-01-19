"""Generate the growing 'manual mode' architecture overview.

This diagram is intentionally a conceptual, high-level view. It will grow step by
step as manual mode gains features.

Current nodes (requested):
- WiFi (SoftAP + Web UI / WebSocket commands)
- Main (application entry point + RTOS tasks + control queue)
- IMU
- Filter (yaw complementary filter)
- hovercraft_variables (global config)
- Motors (mixer + controller + ESC/motors)

This script generates:
- a high-level overview diagram
- sub-diagrams when a block becomes too detailed (clearly labeled)

Requires:
- `pip install diagrams`
- Graphviz `dot` on PATH
"""

from __future__ import annotations

import os
import sys
from pathlib import Path


def _print_prereq_help() -> None:
    print(
        "Missing Graphviz executable 'dot'.\n\n"
        "Install Graphviz and ensure it is on PATH:\n"
        "  - Windows (winget): winget install Graphviz.Graphviz\n"
        "  - Windows (choco):  choco install graphviz\n\n"
        "After installing, reopen the terminal and run this script again.\n"
    )


def _check_graphviz_dot_on_path() -> bool:
    from shutil import which

    return which("dot") is not None


def _diagram_style() -> tuple[dict[str, str], dict[str, str], dict[str, str]]:
    graph_attr = {"labelloc": "t", "fontsize": "20", "fontname": "Segoe UI", "rankdir": "LR", "splines": "spline", "pad": "0.2"}

    node_attr = {
        "fontname": "Segoe UI",
        "fontsize": "12",
        # Reduce icon/text collisions for image-based nodes (diagrams uses Graphviz images).
        "imagescale": "true",
        "imagepos": "mc",
        "labelloc": "b",
        # Add a bit of internal padding so the first label line doesn't touch the icon.
        "margin": "1.0,1.0",
    }
    edge_attr = {"fontname": "Segoe UI", "fontsize": "11"}
    return graph_attr, node_attr, edge_attr


def _icons_dir() -> Path:
    return Path(__file__).resolve().parent / "icons"


def _custom_icon(label: str, filename: str):
    from diagrams.custom import Custom

    return Custom(label, str(_icons_dir() / filename))


def _fallback_icon(label: str):
    # Used when none of the custom firmware icons fit.
    return _custom_icon(label, "cpp.png")


def _cfg(label: str):
    return _custom_icon(label, "Hovercraft_variables.png")


def _task(label: str):
    return _custom_icon(label, "task.png")


def _data(label: str):
    return _custom_icon(label, "data.png")


def _fn(label: str):
    return _custom_icon(label, "function.jpg")


def _motor(label: str):
    return _custom_icon(label, "motor.png")


def _generate_overview(output_dir: Path) -> Path:
    from diagrams import Cluster, Diagram, Edge

    out_path = output_dir / "manual_mode_architecture"
    graph_attr, node_attr, edge_attr = _diagram_style()

    with Diagram(
        "Firmware: Manual mode overview",
        filename=str(out_path),
        outformat="png",
        show=False,
        graph_attr=graph_attr,
        node_attr=node_attr,
        edge_attr=edge_attr,
    ):
        with Cluster("Configuration"):
            cfg = _cfg("hovercraft_variables\n(src/hovercraft_variables.*)")

        with Cluster("WiFi + Web control"):
            wifi = _fallback_icon("WifiManager\n(SoftAP)")
            web = _fallback_icon("NetworkPiloting\n(Web UI + WebSocket)")
            wifi >> Edge(label="SoftAP") >> web

        with Cluster("RTOS + App glue (src/main.cpp)"):
            main = _fn("Main\nsetup() + globals")
            queue = _data("g_controlQueue\n(len=1, overwrite)")
            task_wifi = _task("task_wifiManager\n(core0, ~50ms)\nnetworkPiloting.loop()")
            task_imu = _task("task_imu\n(core1, 200Hz)")
            task_motor = _task("task_motorManagement\n(core1, control loop)")
            task_batt = _task("task_batteryMonitor\n(core1, ~1Hz)\ntelemetry + cutoff")

            main >> Edge(label="creates") >> queue
            main >> Edge(label="starts") >> task_wifi
            main >> Edge(label="starts") >> task_imu
            main >> Edge(label="starts") >> task_motor
            main >> Edge(label="starts") >> task_batt

        with Cluster("Sensing"):
            imu = _fallback_icon("IMU\n(lib/imu)")
            filt = _fn("Yaw complementary filter\n(inside IMU)")
            imu >> Edge(label="gyro Z + mag heading") >> filt

        with Cluster("Actuation"):
            mixer = _fallback_icon("MotorMixer\n(lib/motor_mixer)")
            ctrl = _fallback_icon("MotorCtrl\n(lib/motor_ctrl)")
            motors = _motor("ESC + Motors\n(DSHOT)")
            mixer >> Edge(label="per-motor %") >> ctrl
            ctrl >> Edge(label="DSHOT") >> motors

        # Config wiring (from docs)
        cfg >> Edge(label="pins + tuning") >> main
        cfg >> Edge(label="SSID/PW + port + UI presets") >> web
        cfg >> Edge(label="yawAlpha") >> filt
        cfg >> Edge(label="motor pins + scaling") >> ctrl

        # Manual mode command flow
        web >> Edge(label="callbacks\n(lift/thrust/steering/arm)") >> queue
        task_motor << Edge(label="xQueueReceive(latest)") << queue
        task_wifi >> Edge(label="housekeeping") >> web
        task_batt >> Edge(label="sendTelemetry(V/A/mAh)") >> web

        # Control loop dependencies
        task_imu >> Edge(label="updates yaw/yawRate") >> filt
        filt >> Edge(label="shared yaw/yawRate") >> task_motor
        task_motor >> Edge(label="lift/thrust/diff") >> mixer

        # Sub-diagrams callouts (keep the overview readable)
        main >> Edge(label="subgraph:\nweb control details", style="dashed") >> web
        main >> Edge(label="subgraph:\ncontrol + actuation details", style="dashed") >> task_motor
        main >> Edge(label="subgraph:\nmotor task details", style="dashed") >> task_motor
        main >> Edge(label="subgraph:\nIMU + filter details", style="dashed") >> task_imu
        main >> Edge(label="subgraph:\nbattery safety + telemetry", style="dashed") >> task_batt

    return out_path.with_suffix(".png")


def _generate_sub_control_actuation(output_dir: Path) -> Path:
    """Detailed manual control/actuation dataflow.

    Focus: WebSocket commands -> queue -> motor task -> mixer -> motor controller -> DSHOT.
    Auto/M-mode override logic exists in main.cpp but is intentionally *not* expanded here.
    """

    from diagrams import Cluster, Diagram, Edge

    out_path = output_dir / "manual_mode_sub_control_actuation"
    graph_attr, node_attr, edge_attr = _diagram_style()

    with Diagram(
        "Manual mode subgraph: control + actuation",
        filename=str(out_path),
        outformat="png",
        show=False,
        graph_attr=graph_attr,
        node_attr=node_attr,
        edge_attr=edge_attr,
    ):
        with Cluster("Web UI / WebSocket"):
            web = _fallback_icon("NetworkPiloting\n(lift/thrust/steering/arm)")
            callbacks = _fn("callbacks in main.cpp\n(set*Callback lambdas)")
            web >> Edge(label="on message") >> callbacks

        with Cluster("Command state"):
            latest = _data("g_latestSetpoints\nControlSetpoints:\n lift, thrust, diffThrust\n motorsEnabled, liftEnabled")
            queue = _data("g_controlQueue\nlen=1\nxQueueOverwrite")
            thrust_scale = _fn("thrust scaling\n(global_WebThrustPresetPercent)")

            callbacks >> Edge(label="update fields") >> latest
            callbacks >> Edge(label="scale thrust") >> thrust_scale
            thrust_scale >> Edge(label="write scaled thrust") >> latest
            latest >> Edge(label="xQueueOverwrite(latest)\n(always newest)") >> queue

        with Cluster("Control loop (task_motorManagement)"):
            motor_task = _task("task_motorManagement")
            yaw_map = _fn("steering% -> yawRate setpoint\n(Betaflight-style rates)")
            pid = _fn("yawRate PID\n(diffCmd %)\n(only if gyroFresh)")

            safety = _data(
                "safety gates\n- effectiveMotorsEnabled = motorsEnabled && !g_lowBatteryMotorCutoff\n"
                "- liftEnabled controls lift motors\n- disarmed => thrust/diff forced to 0"
            )

            queue >> Edge(label="xQueueReceive(latest)") >> motor_task
            motor_task >> Edge(label="apply") >> safety
            motor_task >> Edge(label="compute") >> yaw_map
            yaw_map >> Edge(label="setpoint dps") >> pid

        with Cluster("Motor outputs"):
            mixer = _fallback_icon("MotorMixer\n(lift/thrust/diff)")
            ctrl = _fallback_icon("MotorCtrl\n(safety gate + scaling)")
            motors = _motor("ESC + Motors\n(DSHOT)")

            lift_gate = _data("lift gate\nif liftEnabled=false:\n motorCtrl.applyLiftOff()\n mixer.setLift(0)")

            pid >> Edge(label="diffCmd %") >> mixer
            motor_task >> Edge(label="lift + thrust") >> mixer

            mixer >> Edge(label="per-motor %") >> ctrl
            safety >> Edge(label="setMotorsEnabled(effective)") >> ctrl
            safety >> Edge(label="if lift OFF") >> lift_gate
            lift_gate >> Edge(label="apply") >> ctrl
            ctrl >> Edge(label="DSHOT") >> motors

        with Cluster("Boundary"):
            not_shown = _data("NOT expanded here:\nAutonomousSequence + HeadingHold\n(separate Auto/M-mode diagram later)")
            motor_task >> Edge(label="overrides (dashed)", style="dashed") >> not_shown

        # Notes for future refinement:
        # - If needed, split safety logic into its own sub-diagram.

    return out_path.with_suffix(".png")


def _generate_sub_sensing_filter(output_dir: Path) -> Path:
    """Detailed sensing/filter dataflow.

    Focus: IMU task -> complementary filter -> shared globals -> motor task freshness checks.
    """

    from diagrams import Cluster, Diagram, Edge

    out_path = output_dir / "manual_mode_sub_sensing_filter"
    graph_attr, node_attr, edge_attr = _diagram_style()

    with Diagram(
        "Manual mode subgraph: IMU + filter",
        filename=str(out_path),
        outformat="png",
        show=False,
        graph_attr=graph_attr,
        node_attr=node_attr,
        edge_attr=edge_attr,
    ):
        with Cluster("IMU task (task_imu @ 200Hz)"):
            imu_task = _task("task_imu")
            imu = _fallback_icon("IMU\n(lib/imu)")
            gyro = _fn("gyro read\ngetGyro_raw()")
            yaw_rate_conv = _fn("yawRate_dps = gz * 180/pi")
            mag = _fn("mag read (slower)\n~20Hz (>=50ms)")
            filt = _fn("Yaw complementary filter\nupdateYawComplementaryFrom(gz, heading)")
            imu_task >> Edge(label="calls") >> imu
            imu >> Edge(label="read") >> gyro
            gyro >> Edge(label="convert") >> yaw_rate_conv
            imu >> Edge(label="occasionally") >> mag
            yaw_rate_conv >> Edge(label="gz + heading") >> filt
            mag >> Edge(label="heading_deg") >> filt

        with Cluster("Shared globals (main.cpp)"):
            yaw_rate = _data("g_yawRateMeasured_dps\n+ g_lastYawRateUpdate_us\n(updated @ 200Hz)")
            yaw = _data("g_yawMeasured_deg\n+ g_lastYawUpdate_us\n(updated when finite)")
            yaw_rate_conv >> Edge(label="if finite") >> yaw_rate
            filt >> Edge(label="yaw estimate") >> yaw

        with Cluster("Motor task consumption"):
            motor_task = _task("task_motorManagement")
            freshness = _fn("freshness checks\n- gyroFresh: age < 50ms\n- yawFresh:  age < 150ms")
            yaw_rate >> Edge(label="age check") >> freshness
            yaw >> Edge(label="age check") >> freshness
            freshness >> Edge(label="gate PID + heading-hold") >> motor_task

    return out_path.with_suffix(".png")


def _generate_sub_web_control(output_dir: Path) -> Path:
    """Detailed web control + fail-safe behavior.

    Focus:
    - WifiManager SoftAP bring-up
    - NetworkPiloting HTTP+WebSocket server
    - lightweight JSON parsing + clamping
    - disconnect fail-safe that forces motors OFF and zeroes controls
    """

    from diagrams import Cluster, Diagram, Edge

    out_path = output_dir / "manual_mode_sub_web_control"
    graph_attr, node_attr, edge_attr = _diagram_style()

    with Diagram(
        "Manual mode subgraph: web control (WiFi + WebSocket)",
        filename=str(out_path),
        outformat="png",
        show=False,
        graph_attr=graph_attr,
        node_attr=node_attr,
        edge_attr=edge_attr,
    ):
        with Cluster("WiFi bring-up"):
            cfg = _cfg("hovercraft_variables\nSSID + password")
            wifi = _fallback_icon("WifiManager\nstartAccessPoint()")
            wifi_impl = _fn("WiFi.mode(WIFI_AP)\nWiFi.setSleep(false)\nWiFi.softAP(...)")
            cfg >> Edge(label="pass SSID/PW") >> wifi
            wifi >> Edge(label="uses") >> wifi_impl

        with Cluster("HTTP server"):
            np_begin = _fn("NetworkPiloting.begin()")
            html = _data("Embedded controller page\n(PROGMEM HTML)")
            inject = _fn("inject presets + battery thresholds\ninto HTML at runtime")
            ws = _fallback_icon("WebSocket /ws\n(AsyncWebSocket)")
            np_begin >> Edge(label="serves") >> html
            html >> Edge(label="page.replace(...)") >> inject
            np_begin >> Edge(label="attaches") >> ws

        with Cluster("WebSocket event handling"):
            ws_connect = _fn("WS connect\nreply {status: connected}")
            ws_data = _data("WS data\n(JSON payload)")
            parse = _fn("lightweight parsing\nkey search + toFloat()")
            clamp = _fn("clamp ranges\n lift: 0..100\n thrust: -100..100\n steering: -100..100")
            callbacks = _fn("callbacks into main.cpp\n(onLift/onThrust/onSteering/onArm)")
            echo = _fn("textAll(...)\necho current state to all clients")
            ws_disconnect = _fn("WS disconnect fail-safe\napplyArm(false)\nset lift/thrust/steering=0\ncall callbacks with 0")

            ws >> Edge(label="WS_EVT_CONNECT") >> ws_connect
            ws >> Edge(label="WS_EVT_DATA") >> ws_data
            ws_data >> Edge(label="parse") >> parse
            parse >> Edge(label="apply") >> clamp
            clamp >> Edge(label="call") >> callbacks
            callbacks >> Edge(label="broadcast") >> echo
            ws >> Edge(label="WS_EVT_DISCONNECT") >> ws_disconnect

        with Cluster("RTOS housekeeping"):
            task_wifi = _task("task_wifiManager\n(~50ms)")
            cleanup = _fn("networkPiloting.loop()\nws.cleanupClients()")
            task_wifi >> Edge(label="calls") >> cleanup
            cleanup >> Edge(label="manages clients") >> ws

        with Cluster("Boundary"):
            auto = _data("NOT expanded here:\nautoMode field + callbacks\n(handled in Auto/M-mode diagram later)")
            ws_data >> Edge(label="autoMode (dashed)", style="dashed") >> auto

    return out_path.with_suffix(".png")


def _generate_sub_motor_task_details(output_dir: Path) -> Path:
    """Detailed motor task inner loop (manual path).

    This is the best place to understand the control loop and safety behavior.
    Auto/M-mode overrides exist but are treated as a boundary in this diagram.
    """

    from diagrams import Cluster, Diagram, Edge

    out_path = output_dir / "manual_mode_sub_motor_task_details"
    graph_attr, node_attr, edge_attr = _diagram_style()

    with Diagram(
        "Manual mode subgraph: task_motorManagement details",
        filename=str(out_path),
        outformat="png",
        show=False,
        graph_attr=graph_attr,
        node_attr=node_attr,
        edge_attr=edge_attr,
    ):
        with Cluster("Inputs"):
            queue = _data("g_controlQueue\nlen=1\nxQueueReceive(timeout=0)")
            imu_globals = _data("IMU shared globals\n yawRate + yaw\n + timestamps")
            batt_gate = _data("battery cutoff\ng_lowBatteryMotorCutoff")

        with Cluster("Loop timing"):
            dt = _fn("dt_s from micros()\nclamp to [0.001..0.05]")

        with Cluster("Setpoint handling"):
            read = _fn("read latest setpoints\n(mySetPoint)")
            copy = _data("appliedSetpoints = mySetPoint\n(copy before overrides)")
            effective = _data("effectiveMotorsEnabled\n= motorsEnabled && !batteryCutoff")
            queue >> Edge(label="non-blocking") >> read
            read >> Edge(label="copy") >> copy
            batt_gate >> Edge(label="gate") >> effective

        with Cluster("Yaw control"):
            map_yaw = _fn("diffThrust% (steering)\n-> yawRateSetpoint_dps\n(actual-rates curve)")
            freshness = _fn("freshness checks\n gyroFresh: <50ms\n yawFresh: <150ms")
            pid = _fn("yawRate PID\nif (effective && gyroFresh)\n diffCmd = update(..., dt_s)\nelse reset + diffCmd=0")
            dt >> Edge(label="feeds") >> pid
            copy >> Edge(label="steering%") >> map_yaw
            imu_globals >> Edge(label="age check") >> freshness
            map_yaw >> Edge(label="setpoint") >> pid
            freshness >> Edge(label="enables") >> pid

        with Cluster("Safety + actuation"):
            motor_ctrl = _fallback_icon("MotorCtrl\nsetMotorsEnabled(effective)")
            disarmed = _data("if NOT effective:\n thrust=0, diff=0\n lift latched if liftEnabled")
            lift_gate = _data("if liftEnabled=false (while armed):\n motorCtrl.applyLiftOff()\n mixer.setLift(0)")
            mixer = _fallback_icon("MotorMixer\nsetLift / setThrust / setDiffThrust")

            effective >> Edge(label="apply") >> motor_ctrl
            effective >> Edge(label="false path") >> disarmed
            disarmed >> Edge(label="commands") >> mixer
            effective >> Edge(label="true path") >> lift_gate
            pid >> Edge(label="diffCmd") >> mixer
            copy >> Edge(label="lift + thrust") >> mixer

        with Cluster("Boundary"):
            auto = _data("NOT expanded here:\nAutonomousSequence overrides\nHeading-hold outer loop")
            copy >> Edge(label="overrides (dashed)", style="dashed") >> auto

    return out_path.with_suffix(".png")


def _generate_sub_battery_safety_telemetry(output_dir: Path) -> Path:
    """Battery safety + telemetry chain.

    Focus:
    - BatteryMonitor ADC measurements + mAh integration
    - Periodic telemetry to Web UI
    - Low-battery warning LED behavior
    - Low-battery cutoff -> motor disable + setpoint reset + queue overwrite
    - Motor task uses cutoff flag as a safety gate
    """

    from diagrams import Cluster, Diagram, Edge

    out_path = output_dir / "manual_mode_sub_battery_safety_telemetry"
    graph_attr, node_attr, edge_attr = _diagram_style()

    with Diagram(
        "Manual mode subgraph: battery safety + telemetry",
        filename=str(out_path),
        outformat="png",
        show=False,
        graph_attr=graph_attr,
        node_attr=node_attr,
        edge_attr=edge_attr,
    ):
        with Cluster("Configuration (hovercraft_variables)"):
            cfg_pins = _cfg(
                "battery ADC pins + calibration\n"
                "- global_PIN_BATTERY_VOLTAGE_MONITOR\n"
                "- global_PIN_BATTERY_CURRENT_MONITOR\n"
                "- divider ratios + current scale"
            )
            cfg_thresholds = _cfg("low-battery thresholds\n" "- WarningLow (LED solid)\n" "- CutoffLow + CutoffSamples")

        with Cluster("RTOS battery task (src/main.cpp)"):
            task_batt = _task("task_batteryMonitor\n(~1Hz)")
            batt_obj = _fallback_icon("BatteryMonitor\n(lib/battery_monitor)")
            adc = _fn("ADC read\nanalogReadMilliVolts()")
            compute = _fn("compute V/A + integrate mAh\n(Betaflight scale)")
            telemetry = _fn("networkPiloting.sendTelemetry\n(voltage, current, usedMah)")

            cfg_pins >> Edge(label="init(...) params") >> batt_obj
            task_batt >> Edge(label="calls update()") >> batt_obj
            batt_obj >> Edge(label="reads") >> adc
            adc >> Edge(label="scaled") >> compute
            compute >> Edge(label="getVoltage/getCurrent/getMAH") >> telemetry

        with Cluster("Low-battery warning (LED)"):
            warn_flag = _data("g_lowBatteryLedSolidOn\n(v>0 && v < WarningLow)")
            blink_task = _task("task_blink")
            led = _data("LED_PIN\nsolid ON when low")

            cfg_thresholds >> Edge(label="WarningLow") >> warn_flag
            compute >> Edge(label="voltage") >> warn_flag
            warn_flag >> Edge(label="read") >> blink_task
            blink_task >> Edge(label="digitalWrite") >> led

        with Cluster("Motor cutoff safety"):
            cutoff_counter = _data("cutoffSampleCount\n(count consecutive low samples)")
            cutoff_flag = _data("g_lowBatteryMotorCutoff\n(count > CutoffSamples)")
            latch = _fn(
                "on cutoff entry (latched):\n"
                "- disarm + zero setpoints\n"
                "- xQueueOverwrite(g_controlQueue)\n"
                "- motorCtrl.setMotorsEnabled(false)"
            )
            motor_task = _task("task_motorManagement")
            gate = _data("effectiveMotorsEnabled\n= motorsEnabled && !cutoff")

            cfg_thresholds >> Edge(label="CutoffLow + CutoffSamples") >> cutoff_counter
            compute >> Edge(label="voltage") >> cutoff_counter
            cutoff_counter >> Edge(label="updates") >> cutoff_flag
            cutoff_flag >> Edge(label="triggers") >> latch
            cutoff_flag >> Edge(label="gates") >> gate
            gate >> Edge(label="used by") >> motor_task

        with Cluster("Telemetry consumer"):
            web = _fallback_icon("NetworkPiloting\n(WebSocket clients)")
            ui = _data("Web UI\nshows batt V + used mAh")
            telemetry >> Edge(label="broadcast") >> web
            web >> Edge(label="updates") >> ui

    return out_path.with_suffix(".png")


def generate(output_dir: Path) -> list[Path]:
    if not _check_graphviz_dot_on_path():
        _print_prereq_help()
        raise SystemExit(2)

    try:
        from diagrams import Cluster, Diagram, Edge
        from diagrams.generic.compute import Rack
        from diagrams.generic.network import Firewall
        from diagrams.generic.storage import Storage
    except Exception:
        print("Failed to import diagrams. Did you install it in this environment?")
        raise

    output_dir.mkdir(parents=True, exist_ok=True)

    pngs: list[Path] = []
    pngs.append(_generate_overview(output_dir))
    pngs.append(_generate_sub_web_control(output_dir))
    pngs.append(_generate_sub_control_actuation(output_dir))
    pngs.append(_generate_sub_motor_task_details(output_dir))
    pngs.append(_generate_sub_sensing_filter(output_dir))
    pngs.append(_generate_sub_battery_safety_telemetry(output_dir))
    return pngs


def main(argv: list[str]) -> int:
    script_dir = Path(__file__).resolve().parent
    out_dir = script_dir

    if len(argv) >= 2 and argv[1].strip():
        out_dir = Path(argv[1]).expanduser().resolve()
    elif os.environ.get("DIAGRAM_OUT_DIR"):
        out_dir = Path(os.environ["DIAGRAM_OUT_DIR"]).expanduser().resolve()

    png_paths = generate(out_dir)
    for p in png_paths:
        print(f"Wrote: {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
