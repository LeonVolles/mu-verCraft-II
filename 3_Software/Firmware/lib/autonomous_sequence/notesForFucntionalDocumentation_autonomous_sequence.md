<!-- Note, it is intentionnal that the first heading starts as a "level2"/## and not as level1/# and should be kept like this!! -->
## autonomous_sequence (M-Mode Sequencer)

### Purpose

`autonomous_sequence` implements the autonomous competition behavior executed when “M-Mode” (Auto Mode) is enabled. It is implemented as a **small, deterministic state machine** that runs inside the motor/control task.

High-level behavior:

- Calibrate an initial **start heading** by averaging the complementary-filter heading for 2 seconds.
- Wait for motors to be armed.
- Drive “blind” for a short time to pass the start line.
- Then run an **event-driven loop** where each **IR line crossing event** recomputes the next heading setpoint.
- Allow abort at any time (user toggles M off / stop request / motors disarm).

The line-driven pattern after the blind start is intended to approximate the sector pattern:

- `-90°, -90°, 0°, 0°` (repeat)

### Integration (where it runs)

- The UI / mode logic triggers:
  - `AutonomousSequence::requestStart()` when M-Mode is enabled.
  - `AutonomousSequence::requestStop()` when M-Mode is disabled.
- The motor/control task ticks it once per control loop via `AutonomousSequence::update(...)`.
- The motor/control task applies outputs:
  - If `overrideThrust()` is true, it uses `thrustOverride_percent()` instead of user thrust.
  - If `wantsHeadingHold()` is true, it enables heading-hold using `headingTarget_deg()` as setpoint.
  - If `consumeExitRequest()` returns true (one-shot), the caller should force motors off and leave M-Mode.

Important note: `headingTarget_deg()` is a **persistent setpoint**. The implementation intentionally does **not** reset it at the start of each tick; it must remain valid between line events.

### Public interface

Entry points:

- `requestStart()`
  - Latches a start request; the next tick in `Idle` enters calibration.
- `requestStop()`
  - Latches a stop request; the next tick transitions to `ExitRequested` (if active).
- `update(nowMs, heading_deg, headingFresh, motorsEnabled, lineEventFresh, lineAlpha_deg, lineVPerp_mps, headingAtLine_deg)`
  - Advances the state machine and updates output commands.

Observability helpers:

- `isActive()`, `state()`
- `startHeading_deg()`
- `lastLineAlpha_deg()`, `lastLineVelocityPerp_mps()` (latched on every `lineEventFresh`)

### Inputs (to `update()`)

- `nowMs`: current `millis()`.
- `heading_deg`: current heading estimate (deg). Used mainly for calibration.
- `headingFresh`: when true, the heading sample is accumulated during calibration.
- `motorsEnabled`: whether motors are armed/enabled.
- `lineEventFresh`: one-shot flag indicating a **new** line-crossing event this tick.
- `lineAlpha_deg`: IR-derived line angle $\alpha$ (deg). Valid only when `lineEventFresh==true`.
- `lineVPerp_mps`: IR-derived perpendicular speed (m/s). Currently only latched for debugging/telemetry.
- `headingAtLine_deg`: heading snapshot taken when the IR module generated the line event.

Why `headingAtLine_deg` matters:

- The setpoint recomputation uses the heading that corresponds to the same instant as the IR measurement.
- The caller should ensure `lineAlpha_deg` and `headingAtLine_deg` are from the same event.

### Outputs (polled by the caller)

- Thrust override:
  - `overrideThrust()`
  - `thrustOverride_percent()`
- Heading-hold command:
  - `wantsHeadingHold()`
  - `headingTarget_deg()`
- Exit handshake:
  - `consumeExitRequest()` returns true exactly once per exit request.

### State machine (current implementation)

States are defined in `AutonomousSequence::State`.

#### Idle

- Not running.
- A latched start request transitions to **Calibrating**.

#### Calibrating (2.0 s)

- Motors commanded to stop (thrust override = 0%, heading-hold disabled).
- While `headingFresh==true`, the module accumulates heading samples.
- After 2000 ms:
  - Computes `startHeading` as a circular mean (sin/cos average).
  - If motors are armed: sets `headingTarget = startHeading` and transitions to **StartBlind**.
  - Else transitions to **WaitingForArm**.

#### WaitingForArm (indefinite)

- Motors commanded to stop (thrust override = 0%, heading-hold disabled).
- When `motorsEnabled==true`: sets `headingTarget = startHeading` and transitions to **StartBlind**.

#### StartBlind (1.0 s)

Goal: pass the start line without relying on the IR line event.

- Thrust override: **30%**
- Heading-hold: enabled
  - `headingTarget = startHeading`
- After 1000 ms: transitions to **DriveStraight_SecondSector**.

#### DriveStraight_SecondSector (event-driven)

This state holds the current heading target and waits for a line event.

- Thrust override: **10%**
- Heading-hold: enabled
  - `headingTarget = headingTarget` (keep previous)

On `lineEventFresh==true`:

- Computes next heading setpoint for the first “-90° curve sector” using:

$$\text{headingTarget} = \text{wrap360}(\text{headingAtLine} - \alpha - 90^\circ - |\text{driftFirst}|)$$

where the current hard-coded drift compensation is:

- `driftFirst = 57.0°`

Then transitions to **DriveCurveMinus90_FirstSector**.

#### DriveCurveMinus90_FirstSector (event-driven)

- Thrust override: **10%**
- Heading-hold: enabled
- Waits for the next line event.

On `lineEventFresh==true`:

$$\text{headingTarget} = \text{wrap360}(\text{headingAtLine} - \alpha - 90^\circ - |\text{driftSecond}|)$$

where the current hard-coded drift compensation is:

- `driftSecond = 7.5°`

Then transitions to **DriveCurveMinus90_SecondSector**.

#### DriveCurveMinus90_SecondSector (event-driven)

- Thrust override: **10%**
- Heading-hold: enabled
- Waits for the next line event.

On `lineEventFresh==true` (end of “-90° segment”):

$$\text{headingTarget} = \text{wrap360}(\text{headingAtLine} - \alpha)$$

Then transitions to **DriveStraight_FirstSector**.

#### DriveStraight_FirstSector (event-driven)

- Thrust override: **10%**
- Heading-hold: enabled
- Waits for the next line event.

On `lineEventFresh==true`:

$$\text{headingTarget} = \text{wrap360}(\text{headingAtLine} - \alpha)$$

Then transitions back to **DriveStraight_SecondSector**.

#### Loop summary

After the blind start, the loop is:

- `DriveStraight_SecondSector`
  → (line) `DriveCurveMinus90_FirstSector`
  → (line) `DriveCurveMinus90_SecondSector`
  → (line) `DriveStraight_FirstSector`
  → (line) `DriveStraight_SecondSector` (repeat)

### Stop/abort behavior and safety

- `requestStop()` can be called at any time.
  - On the next tick, if the sequence is active, it transitions to **ExitRequested**.
- If motors are disarmed while the sequencer is in any “motion” state (`StartBlind`, the straight states, or the curve states), it transitions to **ExitRequested**.
- **ExitRequested** is a one-tick state:
  - Commands thrust override 0% and disables heading-hold.
  - Latches an exit request (`consumeExitRequest()` will return true once).
  - Immediately returns to `Idle` in the same tick.

### Implementation notes / gotchas

- Angle wrap-around: all computed headings are wrapped into $[0,360)$ via `wrap360()`.
- Calibration uses a circular mean (sin/cos accumulation) to avoid errors near 0/360.
- Debug prints: the implementation currently emits `Serial.println(...)` messages on transitions and line events.

### Files

- lib/autonomous_sequence/include/autonomous_sequence.h
- lib/autonomous_sequence/src/autonomous_sequence.cpp
