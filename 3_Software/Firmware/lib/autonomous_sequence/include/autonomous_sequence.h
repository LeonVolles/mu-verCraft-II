#pragma once

#include <Arduino.h>
#include <math.h>

// AutonomousSequence
// Event-based "SPS-like" runner for autonomous competition mode ("M-Mode").
// - Calibrates a start heading by averaging the complementary-filter heading over time.
// - Waits for motors to be armed.
// - Executes a short blind start to pass the start line.
// - Then runs a looping, line-event-driven sequence using IR line measurements:
//   pattern (after StartBlind): -90°,-90°,0°,0°,... (repeat)
// - Can be aborted at any time.
//
// This class is designed to be ticked from the motor/control task (deterministic timing).

class AutonomousSequence
{
public:
	enum class State : uint8_t
	{
		Idle = 0,
		Calibrating,
		WaitingForArm,
		StartBlind,
		DriveStraight_FirstSector,
		DriveCurveMinus90_FirstSector,
		DriveCurveMinus90_SecondSector,
		DriveStraight_SecondSector,
		ExitRequested,
	};

	AutonomousSequence() = default;

	// Request starting the sequence.
	// Behavior:
	// - Enters calibration immediately (outputs remain 0).
	// - After calibration, waits until motorsEnabled==true to run the thrust/heading steps.
	void requestStart();

	// Request aborting the sequence. Causes an exit request (motors off + M-Mode off).
	void requestStop();

	// Tick the sequence.
	// nowMs: current millis()
	// heading_deg: current yaw/heading estimate (0..360), from complementary filter
	// headingFresh: true if the heading reading is considered fresh/valid
	// motorsEnabled: true if motors are currently enabled/armed
	// lineEventFresh: true if a new IR line crossing event is available this tick
	// lineAlpha_deg: IR-derived angle to the line (deg). Valid when lineEventFresh==true, else ignored.
	// lineVPerp_mps: IR-derived perpendicular velocity to the line (m/s). Valid when lineEventFresh==true, else ignored.
	// headingAtLine_deg: heading snapshot taken at the moment the line event was recorded.
	void update(uint32_t nowMs,
				float heading_deg,
				bool headingFresh,
				bool motorsEnabled,
				bool lineEventFresh,
				float lineAlpha_deg,
				float lineVPerp_mps,
				float headingAtLine_deg);

	bool isActive() const;
	State state() const;

	// Outputs for the caller (motor task) to apply.
	bool overrideThrust() const;
	float thrustOverride_percent() const;

	bool wantsHeadingHold() const;
	float headingTarget_deg() const;

	// One-shot: returns true once when the sequence wants to force-stop motors and exit M-Mode.
	bool consumeExitRequest();

	float startHeading_deg() const;
	float lastLineAlpha_deg() const;
	float lastLineVelocityPerp_mps() const;

	static float wrap360(float deg);

private:
	static float deg2rad(float deg);

	void enterState(State s, uint32_t nowMs);
	void resetCalibration();
	void accumulateHeading(float heading_deg);
	float computeAverageHeading() const;

	volatile bool startRequested_ = false;
	volatile bool stopRequested_ = false;

	State state_ = State::Idle;
	uint32_t stateStartMs_ = 0;

	// Calibration accumulator (circular mean)
	float sumSin_ = 0.0f;
	float sumCos_ = 0.0f;
	uint32_t sampleCount_ = 0;

	float startHeadingDeg_ = 0.0f;

	float lastLineAlphaDeg_ = NAN;
	float lastLineVelPerp_mps_ = NAN;

	// Cached outputs
	bool overrideThrust_ = false;
	float thrustOverridePercent_ = 0.0f;
	bool wantsHeadingHold_ = false;
	float headingTargetDeg_ = 0.0f;

	bool exitRequestLatched_ = false;
};
