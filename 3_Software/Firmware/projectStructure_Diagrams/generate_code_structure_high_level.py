"""Generate very high-level diagrams explaining code structure.

This script intentionally stays *conceptual* (documentation-level), not a full
architecture map.

Outputs:
- high_level_setup.png: main creates RTOS tasks/threads (nothing more)
- high_level_manual_piloting.png: typical manual/user piloting signal flow

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

    node_attr = {"fontname": "Segoe UI", "fontsize": "12", "imagescale": "true", "imagepos": "tc", "labelloc": "b", "margin": "0.03,0.02"}

    edge_attr = {"fontname": "Segoe UI", "fontsize": "11"}
    return graph_attr, node_attr, edge_attr


def _icons_dir() -> Path:
    return Path(__file__).resolve().parent / "icons"


def _custom_icon(label: str, filename: str):
    from diagrams.custom import Custom

    return Custom(label, str(_icons_dir() / filename))


def _cpp(label: str):
    return _custom_icon(label, "cpp.png")


def _task(label: str):
    return _custom_icon(label, "task.png")


def _data(label: str):
    return _custom_icon(label, "data.png")


def _fn(label: str):
    return _custom_icon(label, "function.jpg")


def _motor(label: str):
    return _custom_icon(label, "motor.png")


def _generate_setup(output_dir: Path) -> Path:
    from diagrams import Diagram, Edge

    out_path = output_dir / "high_level_setup"
    graph_attr, node_attr, edge_attr = _diagram_style()

    with Diagram(
        "High level: Setup (main creates threads)",
        filename=str(out_path),
        outformat="png",
        show=False,
        graph_attr=graph_attr,
        node_attr=node_attr,
        edge_attr=edge_attr,
    ):
        main = _cpp("Main\\n(setup + create tasks)")

        t_blink = _task("task_blink")
        t_wifi = _task("task_wifiManager")
        t_imu = _task("task_imu")
        t_motor = _task("task_motorManagement")
        t_ir = _task("task_irSensors")
        t_batt = _task("task_batteryMonitor")

        main >> Edge(label="creates") >> t_blink
        main >> Edge(label="creates") >> t_wifi
        main >> Edge(label="creates") >> t_imu
        main >> Edge(label="creates") >> t_motor
        main >> Edge(label="creates") >> t_ir
        main >> Edge(label="creates") >> t_batt

    return out_path.with_suffix(".png")


def _generate_manual_piloting(output_dir: Path) -> Path:
    from diagrams import Diagram, Edge

    out_path = output_dir / "high_level_manual_piloting"
    graph_attr, node_attr, edge_attr = _diagram_style()

    with Diagram(
        "High level: Typical manual piloting",
        filename=str(out_path),
        outformat="png",
        show=False,
        graph_attr=graph_attr,
        node_attr=node_attr,
        edge_attr=edge_attr,
    ):
        web_app = _cpp("Web-App")

        yaw_cmd = _data("yaw-commands")
        yaw_pid = _cpp("yaw-PID")

        ten_dof = _cpp("10DOF")
        comp_filter = _fn("complementary filter")

        motor_mixer = _cpp("motor mixer")
        motor_ctrl = _cpp("motor control")
        motors = _motor("4 motors")

        web_app >> Edge(label="sends") >> yaw_cmd >> Edge(label="to") >> yaw_pid

        ten_dof >> Edge(label="output") >> comp_filter >> Edge(label="to PID") >> yaw_pid

        yaw_pid >> Edge(label="yaw-balance") >> motor_mixer
        web_app >> Edge(label="Thrust/Lift") >> motor_mixer

        motor_mixer >> Edge(label="mixing") >> motor_ctrl
        motor_ctrl >> Edge(label="DSHOT") >> motors

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
    pngs.append(_generate_setup(output_dir))
    pngs.append(_generate_manual_piloting(output_dir))
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
