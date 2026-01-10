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
#include <functional>

class NetworkPiloting
{
public:
	NetworkPiloting();

	void begin();
	void loop();

	void setLiftCallback(const std::function<void(float)> &callback);
	float getLift() const;

private:
	AsyncWebServer server_;
	AsyncWebSocket ws_;
	float lift_;
	std::function<void(float)> onLift_;

	void handleWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
};