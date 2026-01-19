# Project Structure Diagrams

This folder contains scripts that generate architecture/project diagrams.

## Icon legend (local)

The generators use custom icons from `projectStructure_Diagrams/icons/`:
- `Hovercraft_variables.png` = configuration blocks
- `task.png` = RTOS tasks
- `data.png` = data/state/queues/globals
- `function.jpg` = functions/logic blocks
- `motor.png` = motors / propulsion

When none of these fit, the generators use `cpp.png` as a generic fallback icon.

## High-level code-structure diagrams

Generator:
- `generate_code_structure_high_level.py`

Outputs:
- `high_level_setup.png` (only: `main` creates RTOS tasks)
- `high_level_manual_piloting.png` (Web-App -> yaw PID + 10DOF/comp filter -> mixer -> motor ctrl -> DSHOT -> motors)

## Motor stack overview

Generates a first architecture overview for:
- `lib/motor_mixer`
- `lib/motor_ctrl`

### Prerequisites (Windows)

1. Python package:
   - `pip install diagrams`
2. Graphviz (required by `diagrams`):
   - `winget install Graphviz.Graphviz`
   - or `choco install graphviz`

Make sure the Graphviz `dot` executable is available on your `PATH` (open a new terminal after installing).

### Generate

From the firmware root:

```powershell
python .\projectStructure_Diagrams\generate_motor_architecture.py
```

Output:
- `projectStructure_Diagrams/motor_architecture.png`

## Manual mode overview (growing)

Generates a conceptual diagram for the current manual-mode chain:

### Generate

From the firmware root:

```powershell
python .\projectStructure_Diagrams\generate_manual_mode_architecture.py
```

Output:

This script also generates clearly labeled sub-diagrams (to keep the overview readable):

## Autonomous mode (M-Mode) diagrams

Generates diagrams for autonomous mode (the website “M” button), centered around:
- `lib/autonomous_sequence`

### Generate

From the firmware root:

```powershell
python .\projectStructure_Diagrams\generate_autonomous_mode_architecture.py
```

Output:
- `projectStructure_Diagrams/autonomous_mode_architecture.png`
- `projectStructure_Diagrams/autonomous_mode_sequence_core.png`

### Optional: custom output directory

```powershell
python .\projectStructure_Diagrams\generate_motor_architecture.py .\projectStructure_Diagrams
```

Or via env var:

```powershell
$env:DIAGRAM_OUT_DIR = ".\\projectStructure_Diagrams"
python .\projectStructure_Diagrams\generate_motor_architecture.py
```
