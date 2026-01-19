#pragma once

#include <Arduino.h>

// AutonomousSequence
// Simple time-based sequence runner for "M-Mode".
// - Calibrates a start heading by averaging the complementary-filter heading over time.
// - Then runs a fixed heading/thrust sequence.
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
		HoldStart,
		TurnMinus90,
		TurnMinus180,
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
	void update(uint32_t nowMs, float heading_deg, bool headingFresh, bool motorsEnabled);

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

	// Cached outputs
	bool overrideThrust_ = false;
	float thrustOverridePercent_ = 0.0f;
	bool wantsHeadingHold_ = false;
	float headingTargetDeg_ = 0.0f;

	bool exitRequestLatched_ = false;
};
