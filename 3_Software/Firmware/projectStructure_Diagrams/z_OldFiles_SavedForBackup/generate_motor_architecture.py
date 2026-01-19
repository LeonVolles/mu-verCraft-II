"""Generate architecture diagrams for the firmware project.

This script focuses on the motor stack (motor_mixer + motor_ctrl) to provide a
first "architecture overview" diagram.

It uses the `diagrams` Python package which relies on Graphviz.
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
    # diagrams uses graphviz via the `dot` binary.
    from shutil import which

    return which("dot") is not None


def generate(output_dir: Path) -> Path:
    if not _check_graphviz_dot_on_path():
        _print_prereq_help()
        raise SystemExit(2)

    try:
        from diagrams import Cluster, Diagram, Edge
        from diagrams.generic.compute import Rack
        from diagrams.generic.storage import Storage
    except Exception as exc:  # pragma: no cover
        print("Failed to import diagrams. Did you install it in this environment?")
        raise

    output_dir.mkdir(parents=True, exist_ok=True)

    out_path = output_dir / "motor_architecture"

    # NOTE: `filename` is without extension; diagrams adds .png by default.
    graph_attr = {"labelloc": "t", "fontsize": "20", "fontname": "Segoe UI", "rankdir": "LR", "splines": "spline", "pad": "0.2"}

    node_attr = {"fontname": "Segoe UI", "fontsize": "12"}

    edge_attr = {"fontname": "Segoe UI", "fontsize": "11"}

    with Diagram(
        "Firmware: Motor stack overview",
        filename=str(out_path),
        outformat="png",
        show=False,
        graph_attr=graph_attr,
        node_attr=node_attr,
        edge_attr=edge_attr,
    ):
        # External-ish inputs
        with Cluster("Inputs"):
            pilot_cmd = Storage("Pilot/autonomy\n(mix inputs)")
            state = Storage("State/feedback\n(imu, etc.)")

        with Cluster("Motor stack"):
            motor_mixer = Rack("motor_mixer\n(lib/motor_mixer)")
            motor_ctrl = Rack("motor_ctrl\n(lib/motor_ctrl)")

        with Cluster("Outputs"):
            motors = Storage("Motors / ESC\n(DSHOT)")

        pilot_cmd >> Edge(label="desired forces\n/ setpoints") >> motor_mixer
        state >> Edge(label="constraints\n/ limits") >> motor_mixer

        motor_mixer >> Edge(label="per-motor\ncommands") >> motor_ctrl
        motor_ctrl >> Edge(label="DSHOT") >> motors

    return out_path.with_suffix(".png")


def main(argv: list[str]) -> int:
    script_dir = Path(__file__).resolve().parent
    out_dir = script_dir

    # Allow overriding output dir via env var or arg.
    if len(argv) >= 2 and argv[1].strip():
        out_dir = Path(argv[1]).expanduser().resolve()
    elif os.environ.get("DIAGRAM_OUT_DIR"):
        out_dir = Path(os.environ["DIAGRAM_OUT_DIR"]).expanduser().resolve()

    png_path = generate(out_dir)
    print(f"Wrote: {png_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
