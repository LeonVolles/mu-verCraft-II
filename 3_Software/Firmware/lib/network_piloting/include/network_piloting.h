// #ifndef NETWORK_PILOTING_H
// #define NETWORK_PILOTING_H

// #include <Arduino.h>
// #include <WebServer.h>

// class NetworkPiloting {
// private:
//     WebServer* server;
//     float targetThrust;
//     float targetSteering;
//     bool armed;

// public:
//     NetworkPiloting();
//     ~NetworkPiloting();

//     // Initialization
//     void init();

//     // Start the web server
//     void startServer();

//     // Process client requests (call in main loop)
//     void processClient();

//     // Remote control values
//     void setTargetThrust(float v);
//     void setTargetSteering(float v);
//     float getTargetThrust();
//     float getTargetSteering();

//     // Safety arming system
//     void setArmed(bool v);
//     bool isArmed();
// };

// #endif // NETWORK_PILOTING_H

#pragma once

#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <freertos/semphr.h>
#include <functional>

class NetworkPiloting
{
public:
	struct PidTunings
	{
		float yawKp;
		float yawKi;
		float yawKd;
		float headingKp;
		float headingKi;
		float headingKd;
	};

	NetworkPiloting();

	void begin();
	void loop();

	void setLiftCallback(const std::function<void(float)> &callback);
	void setThrustCallback(const std::function<void(float)> &callback);
	void setSteeringCallback(const std::function<void(float)> &callback);
	void setArmCallback(const std::function<void(bool)> &callback);
	void setAutoModeCallback(const std::function<void(bool)> &callback);
	// Optional: provide current PID tunings for /pid.json.
	void setPidGetProvider(const std::function<void(PidTunings &out)> &provider);
	// Optional: handle PID tuning updates posted to /pid.
	// Return true if values were accepted.
	void setPidSetHandler(const std::function<bool(const PidTunings &in)> &handler);
	// Optional: provide JSON for /debug endpoint.
	// The callback should write a null-terminated JSON string into out (max outSize).
	// Return number of bytes written (excluding null terminator). Return 0 to fall back to default.
	void setDebugProvider(const std::function<size_t(char *out, size_t outSize)> &provider);
	void sendTelemetry(float voltage, float current, float usedMah);
	void sendHeading(float heading_deg);
	void sendAutoMode(bool enabled);
	float getLift() const;
	float getThrust() const;
	float getSteering() const;
	bool motorsEnabled() const;
	bool autoModeRequested() const;

private:
	AsyncWebServer server_;
	AsyncWebSocket ws_;
	SemaphoreHandle_t wsMutex_;
	float lift_;
	float thrust_;
	float steering_;
	bool motorsEnabled_;
	bool autoModeRequested_;
	std::function<void(float)> onLift_;
	std::function<void(float)> onThrust_;
	std::function<void(float)> onSteering_;
	std::function<void(bool)> onArm_;
	std::function<void(bool)> onAutoMode_;
	std::function<void(PidTunings &out)> pidGetProvider_;
	std::function<bool(const PidTunings &in)> pidSetHandler_;
	std::function<size_t(char *out, size_t outSize)> debugProvider_;

	void handleWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
	void applyArm(bool enabled);
	bool lockWs(uint32_t timeoutTicks = 0);
	void unlockWs();
};