<!-- Note, it is intentionnal that the first heading starts as a "level2"/## and not as level1/# and should be kept like this!! -->
## autonomous_sequence (M-Mode Sequencer)

### Purpose

General idea: autonomous behavior is easiest to maintain as a **small state machine**. Each state defines “what the craft should do now” (e.g. hold a heading, apply a fixed thrust), and transitions to the next state are triggered by **conditions**. In this project, conditions are currently mostly **time-based** (elapsed milliseconds), but the same structure can be extended to **event-based triggers** such as IR line detections (see the IR sensors module).

Implements the time-based autonomous sequence executed when the website “M” button (Auto Mode) is enabled.

The sequencer is designed to run deterministically inside the motor/control task and can:

- Calibrate a **start heading** by averaging the complementary-filter yaw heading over time.
- Command a **heading target** via the existing heading-hold cascade (outer heading controller → inner yaw-rate PID).
- Override user thrust while active.
- Abort at any time when the user toggles M off.

### Integration (where it runs)

- The web UI callback calls `AutonomousSequence::requestStart()` / `AutonomousSequence::requestStop()`.
- The motor/control task calls `autonomousSequence.update(millis(), yawMeasured_deg, yawFresh, effectiveMotorsEnabled)` once per control tick.
- Outputs are applied in the motor task:
  - During `Calibrating` and `WaitingForArm`, the motor task forces lift/thrust/yaw to **0** for stability.
  - During the thrust phases, thrust is overridden and yaw steering is disabled (`diffThrust = 0`), while heading-hold is enabled with the requested target.
  - When the sequencer requests exit, the motor task forces **motors OFF** and syncs the UI Auto Mode state back to OFF.

### Sequence behavior (example)

State machine (`AutonomousSequence::State`):

1. **Idle**
  - Not running.

2. **Calibrating** (2.0 s)
  - Goal: compute `startHeadingDeg_` using a circular mean of heading samples.
  - Heading samples are accumulated only when `headingFresh == true`.
  - Motors are held at zero by the motor task (safety/stability).
  - After 2.0 s:
    - If motors are already armed (`motorsEnabled==true`) → proceed to **HoldStart**.
    - Else → proceed to **WaitingForArm**.

3. **WaitingForArm** (indefinite)
  - Motors are held at zero by the motor task.
  - When `motorsEnabled==true` → proceed to **HoldStart**.

4. **HoldStart** (1.0 s)
  - Thrust override: **20%**.
  - Heading target: `startHeading`.

5. **TurnMinus90** (1.0 s)
  - Thrust override: **20%**.
  - Heading target: `startHeading - 90°` (wrapped to 0..360).

6. **TurnMinus180** (3.0 s)
  - Thrust override: **20%**.
  - Heading target: `startHeading - 180°` (wrapped to 0..360).

7. **ExitRequested** (one tick)
  - Latches a one-shot exit request and returns to **Idle**.
  - Note: on the tick where exit is generated, `isActive()` may already be false (state returns to `Idle` in the same update), so consumers must rely on `consumeExitRequest()`.

### Inputs and outputs

#### Inputs (to `update()`)

- `nowMs`: current `millis()`.
- `heading_deg`: current yaw/heading estimate (expected 0..360).
- `headingFresh`: whether the heading estimate is fresh/valid.
- `motorsEnabled`: whether motors are currently armed/enabled.

#### Outputs (polled by the motor task)

- Thrust override:
  - `overrideThrust()` / `thrustOverride_percent()`
- Heading command:
  - `wantsHeadingHold()` / `headingTarget_deg()`
- Exit / abort handshake:
  - `consumeExitRequest()` (one-shot)

### Safety and edge cases

- Abort at any time: `requestStop()` causes an immediate exit request on the next `update()` tick.
- If motors become disarmed during thrust phases (`HoldStart`, `TurnMinus90`, `TurnMinus180`), the sequencer triggers an exit request.
- Heading wrap-around is handled via circular mean (sin/cos accumulation) and `wrap360()`.

### Files

- lib/autonomous_sequence/include/autonomous_sequence.h
- lib/autonomous_sequence/src/autonomous_sequence.cpp
