"""Generate diagrams for the autonomous "M-Mode" (Auto Mode).

Goal
- Keep manual-mode diagrams focused on manual control.
- Provide separate diagrams for autonomous mode, centered around `AutonomousSequence`.

This script generates:
- an autonomous overview (how M-Mode plugs into the existing control stack)
- a detailed "sequence core" diagram (the autonomous_sequence state machine)

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
        "imagepos": "tc",
        "labelloc": "b",
        # Add a bit of internal padding so the first label line doesn't touch the icon.
        "margin": "0.03,0.02",
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


def _task(label: str):
    return _custom_icon(label, "task.png")


def _data(label: str):
    return _custom_icon(label, "data.png")


def _fn(label: str):
    return _custom_icon(label, "function.jpg")


def _motor(label: str):
    return _custom_icon(label, "motor.png")


def _generate_overview(output_dir: Path) -> Path:
    """High-level dataflow for autonomous (M-Mode).

    Focus:
    - Web UI toggles M-Mode (autoMode)
    - Motor task ticks AutonomousSequence deterministically
    - Sequence overrides thrust and heading target
    - Heading hold uses IMU complementary filter (yaw) + yaw-rate (gyro)
    - Outputs go through the existing mixer/motor chain (DSHOT)
    """

    from diagrams import Cluster, Diagram, Edge
    from diagrams.generic.compute import Rack
    from diagrams.generic.storage import Storage

    out_path = output_dir / "autonomous_mode_architecture"
    graph_attr, node_attr, edge_attr = _diagram_style()

    with Diagram(
        "Firmware: Autonomous mode (M-Mode) overview",
        filename=str(out_path),
        outformat="png",
        show=False,
        graph_attr=graph_attr,
        node_attr=node_attr,
        edge_attr=edge_attr,
    ):
        with Cluster("WiFi + Web control"):
            web = _fallback_icon("NetworkPiloting\n(Web UI + WebSocket)")
            auto_toggle = _data("M button / autoMode\n(setAutoModeCallback)")
            web >> Edge(label="toggle") >> auto_toggle

        with Cluster("RTOS + App glue (src/main.cpp)"):
            task_wifi = _task("task_wifiManager\nnetworkPiloting.loop()")
            task_imu = _task("task_imu\n(200Hz)")
            task_motor = _task("task_motorManagement\n(control loop)")

            auto_toggle >> Edge(label="enabled -> requestStart()\ndisabled -> requestStop()") >> task_motor
            task_wifi >> Edge(label="housekeeping") >> web

        with Cluster("Sensing"):
            imu = _fallback_icon("IMU\n(lib/imu)")
            filt = _fn("yaw complementary filter\n(IMU::updateYawComplementaryFrom)")
            yaw_globals = _data("g_yawMeasured_deg\n+ yawFresh (<150ms)")
            yawrate_globals = _data("g_yawRateMeasured_dps\n+ gyroFresh (<50ms)")

            task_imu >> Edge(label="reads") >> imu
            imu >> Edge(label="integrate gyro + correct w/ mag") >> filt
            filt >> Edge(label="publish") >> yaw_globals
            imu >> Edge(label="gyro z") >> yawrate_globals

        with Cluster("Autonomous core"):
            seq = Rack("AutonomousSequence\n(lib/autonomous_sequence)")
            seq_state = Storage("state machine\nCalibrate -> HoldStart -> Turn-90 -> Turn-180")
            seq_out = _data("outputs\n- thrust override\n- heading target\n- exit request")

            task_motor >> Edge(label="update(nowMs, yaw, yawFresh, motorsEnabled)") >> seq
            seq >> Edge(label="state") >> seq_state
            seq_state >> Edge(label="commands") >> seq_out

        with Cluster("Heading hold cascade"):
            hh = _fn("setHeading(target)\n-> g_headingHoldActive")
            heading_pid = _fallback_icon("HeadingController\n(lib/heading_controller)")
            yawrate_pid = _fn("YawRate PID\n(inner loop)")

            seq_out >> Edge(label="wantsHeadingHold()") >> hh
            hh >> Edge(label="heading error") >> heading_pid
            yaw_globals >> Edge(label="yawMeasured") >> heading_pid
            yawrate_globals >> Edge(label="yawRateMeasured") >> heading_pid
            heading_pid >> Edge(label="yawRateSetpoint_dps") >> yawrate_pid

        with Cluster("Actuation"):
            mixer = _fallback_icon("MotorMixer\n(lib/motor_mixer)")
            ctrl = _fallback_icon("MotorCtrl\n(lib/motor_ctrl)")
            motors = _motor("ESC + Motors\n(DSHOT)")

            seq_out >> Edge(label="override thrust%") >> task_motor
            task_motor >> Edge(label="appliedSetpoints + diffCmd") >> mixer
            yawrate_pid >> Edge(label="diffCmd %") >> mixer
            mixer >> Edge(label="per-motor %") >> ctrl
            ctrl >> Edge(label="DSHOT") >> motors

        with Cluster("Details"):
            core = _data("subgraph: sequence core\n(state machine details)")
            task_motor >> Edge(label="dashed", style="dashed") >> core

    return out_path.with_suffix(".png")


def _generate_sequence_core(output_dir: Path) -> Path:
    """Detailed autonomous_sequence state machine.

    This diagram is meant as the "core" reference:
    - inputs (yaw + freshness + motorsEnabled)
    - calibration (circular mean)
    - time-based transitions
    - outputs used by task_motorManagement

    Based directly on:
    - lib/autonomous_sequence/src/autonomous_sequence.cpp
    - src/main.cpp integration in task_motorManagement()
    """

    from diagrams import Cluster, Diagram, Edge

    out_path = output_dir / "autonomous_mode_sequence_core"
    graph_attr, node_attr, edge_attr = _diagram_style()

    with Diagram(
        "Autonomous mode core: autonomous_sequence state machine",
        filename=str(out_path),
        outformat="png",
        show=False,
        graph_attr=graph_attr,
        node_attr=node_attr,
        edge_attr=edge_attr,
    ):
        with Cluster("Inputs (from motor task)"):
            tick = _task("task_motorManagement\ncontrol tick")
            in_time = _data("nowMs = millis()")
            in_yaw = _data("yawMeasured_deg\n(from IMU comp filter)")
            in_fresh = _data("headingFresh = yawFresh\n(age < 150ms)")
            in_arm = _data("motorsEnabled\n(effectiveMotorsEnabled)")

            tick >> Edge(label="calls update()") >> in_time
            tick >> Edge(label="provides") >> in_yaw
            tick >> Edge(label="provides") >> in_fresh
            tick >> Edge(label="provides") >> in_arm

        with Cluster("Entry points"):
            req_start = _fn("requestStart()\n(startRequested_=true)")
            req_stop = _fn("requestStop()\n(stopRequested_=true)")
            idle = _data("State: Idle\n(isActive=false)")

            req_start >> Edge(label="handled only when Idle") >> idle
            req_stop >> Edge(label="handled any time") >> idle

        with Cluster("Calibration (circular mean)"):
            accum = _fn("accumulateHeading()\n(sumSin/sumCos, sampleCount)\n(only if headingFresh)")
            avg = _fn("computeAverageHeading()\natan2(sumSin,sumCos) -> wrap360")

            in_fresh >> Edge(label="true") >> accum
            in_yaw >> Edge(label="heading_deg") >> accum
            accum >> Edge(label="after 2.0s") >> avg

        with Cluster("State machine (time-based)"):
            cal = _data("Calibrating (2.0s)\n" "outputs: overrideThrust=1, thrust=0\n" "wantsHeadingHold=0")
            wait = _data("WaitingForArm (∞)\n" "outputs: overrideThrust=1, thrust=0\n" "wantsHeadingHold=0")
            hold = _data("HoldStart (1.0s)\n" "outputs: thrust=20%\n" "headingTarget = startHeading")
            turn90 = _data("TurnMinus90 (1.0s)\n" "outputs: thrust=20%\n" "headingTarget = startHeading - 90")
            turn180 = _data("TurnMinus180 (3.0s)\n" "outputs: thrust=20%\n" "headingTarget = startHeading - 180")
            exitreq = _data("ExitRequested (one tick)\n" "outputs: exitRequestLatched=1\n" "then enter Idle")

            idle >> Edge(label="startRequested") >> cal

            cal >> Edge(label="after 2.0s") >> avg
            avg >> Edge(label="startHeadingDeg_ set") >> cal

            cal >> Edge(label="t>=2s AND motorsEnabled") >> hold
            cal >> Edge(label="t>=2s AND !motorsEnabled") >> wait

            wait >> Edge(label="motorsEnabled") >> hold

            hold >> Edge(label="t>=1s") >> turn90
            turn90 >> Edge(label="t>=1s") >> turn180
            turn180 >> Edge(label="t>=3s") >> exitreq
            exitreq >> Edge(label="consumeExitRequest()") >> idle

        with Cluster("Safety / abort paths"):
            disarm = _data("if motors disarmed\nduring thrust phases -> ExitRequested")
            stop = _data("if stopRequested_ -> ExitRequested")

            in_arm >> Edge(label="false while Hold/Turn") >> disarm
            req_stop >> Edge(label="next tick") >> stop
            disarm >> Edge(label="enter") >> exitreq
            stop >> Edge(label="enter") >> exitreq

        with Cluster("Outputs (used by motor task)"):
            out_thrust = _data("overrideThrust() + thrustOverride_percent()")
            out_heading = _data("wantsHeadingHold() + headingTarget_deg()")
            out_exit = _data("consumeExitRequest()\n(one-shot)")

            hold >> Edge(label="sets") >> out_thrust
            turn90 >> Edge(label="sets") >> out_thrust
            turn180 >> Edge(label="sets") >> out_thrust
            hold >> Edge(label="sets") >> out_heading
            turn90 >> Edge(label="sets") >> out_heading
            turn180 >> Edge(label="sets") >> out_heading
            exitreq >> Edge(label="latches") >> out_exit

        with Cluster("Integration notes (src/main.cpp)"):
            integration = _data(
                "motor task behavior while active:\n"
                "- Calibrating/WaitingForArm: force lift/thrust/diff = 0\n"
                "- thrust phases: diffThrust forced to 0\n"
                "- wantsHeadingHold -> setHeading(target)\n"
                "- on exit: cancelHeadingHold + motors OFF + sync UI"
            )
            out_exit >> Edge(label="triggers") >> integration

    return out_path.with_suffix(".png")


def generate(output_dir: Path) -> list[Path]:
    if not _check_graphviz_dot_on_path():
        _print_prereq_help()
        raise SystemExit(2)

    try:
        from diagrams import Diagram  # noqa: F401
    except Exception:
        print("Failed to import diagrams. Did you install it in this environment?")
        raise

    output_dir.mkdir(parents=True, exist_ok=True)

    pngs: list[Path] = []
    pngs.append(_generate_overview(output_dir))
    pngs.append(_generate_sequence_core(output_dir))
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
