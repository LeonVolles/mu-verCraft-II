#pragma once

#include <Arduino.h>
#include <ESPAsyncWebServer.h>

class CameraPlaceholder
{
public:
	CameraPlaceholder();

	// Initialize camera and register HTTP endpoints (/stream and /api/camera/status).
	// Returns true if the camera was detected and initialized.
	bool begin(AsyncWebServer &server);

	bool isAvailable() const;

private:
	bool cameraAvailable_;
	volatile bool streamActive_;

	// Internal helpers
	void registerHttpEndpoints(AsyncWebServer &server);
};