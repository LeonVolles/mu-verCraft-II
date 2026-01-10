#include "camera_placeholder.h"

#include <esp_camera.h>
#include <ESPAsyncWebServer.h>
#include <esp_system.h>

// Lightweight MJPEG response that yields between frames to keep AsyncTCP responsive
class AsyncJpegStreamResponse : public AsyncAbstractResponse
{
public:
	explicit AsyncJpegStreamResponse(uint32_t frameIntervalMs) : frameIntervalMs_(frameIntervalMs)
	{
		_code = 200;
		_contentLength = 0;       // unknown; streaming
		_sendContentLength = false;
		_contentType = "multipart/x-mixed-replace; boundary=frame";
	}

	bool _sourceValid() const override { return true; }

	size_t _fillBuffer(uint8_t *buf, size_t maxLen) override
	{
		const size_t trailerLen = 2; // CRLF

		if (fb_ == nullptr)
		{
			uint32_t now = millis();
			if (now - lastFrameMs_ < frameIntervalMs_)
			{
				return RESPONSE_TRY_AGAIN; // let other tasks run
			}

			fb_ = esp_camera_fb_get();
			if (!fb_)
			{
				return RESPONSE_TRY_AGAIN;
			}

			headerLen_ = static_cast<size_t>(snprintf(header_, sizeof(header_), "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb_->len));
			totalLen_ = headerLen_ + fb_->len + trailerLen;
			sent_ = 0;
			lastFrameMs_ = now;
		}

		size_t remaining = totalLen_ - sent_;
		size_t toSend = remaining > maxLen ? maxLen : remaining;
		size_t written = 0;

		while (written < toSend)
		{
			size_t globalPos = sent_ + written;
			if (globalPos < headerLen_)
			{
				size_t copyLen = min(headerLen_ - globalPos, toSend - written);
				memcpy(buf + written, header_ + globalPos, copyLen);
				written += copyLen;
			}
			else if (globalPos < headerLen_ + fb_->len)
			{
				size_t frameOffset = globalPos - headerLen_;
				size_t copyLen = min(fb_->len - frameOffset, toSend - written);
				memcpy(buf + written, fb_->buf + frameOffset, copyLen);
				written += copyLen;
			}
			else
			{
				size_t trailerOffset = globalPos - headerLen_ - fb_->len;
				buf[written++] = (trailerOffset == 0) ? '\r' : '\n';
			}
		}

		sent_ += written;

		if (sent_ >= totalLen_)
		{
			esp_camera_fb_return(fb_);
			fb_ = nullptr;
			sent_ = 0;
			totalLen_ = 0;
			headerLen_ = 0;
			// Yield between frames so the async task can feed control/websocket too
			return RESPONSE_TRY_AGAIN;
		}

		return written;
	}

private:
	camera_fb_t *fb_ = nullptr;
	size_t sent_ = 0;
	size_t totalLen_ = 0;
	size_t headerLen_ = 0;
	char header_[128];
	uint32_t lastFrameMs_ = 0;
	uint32_t frameIntervalMs_ = 50; // ~20 fps cap
};

// Pin map for Seeed XIAO ESP32S3 Sense with OV3660
static camera_config_t makeCameraConfig()
{
	camera_config_t config = {};
	config.ledc_channel = LEDC_CHANNEL_0;
	config.ledc_timer = LEDC_TIMER_0;
	// Pin map per Seeed XIAO ESP32S3 Sense camera_pins.h
	config.pin_d0 = 15; // Y2
	config.pin_d1 = 17; // Y3
	config.pin_d2 = 18; // Y4
	config.pin_d3 = 16; // Y5
	config.pin_d4 = 14; // Y6
	config.pin_d5 = 12; // Y7
	config.pin_d6 = 11; // Y8
	config.pin_d7 = 48; // Y9
	config.pin_xclk = 10;
	config.pin_pclk = 13;
	config.pin_vsync = 38;
	config.pin_href = 47;
	config.pin_sscb_sda = 40;
	config.pin_sscb_scl = 39;
	config.pin_pwdn = -1;
	config.pin_reset = -1;
	config.xclk_freq_hz = 20000000;
	config.pixel_format = PIXFORMAT_JPEG;
	config.frame_size = FRAMESIZE_QQVGA; // start very low for stability; can be raised later
	config.jpeg_quality = 15;            // start with smaller frames
	config.fb_count = psramFound() ? 2 : 1;
	config.grab_mode = psramFound() ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY;
	config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
	return config;
}

CameraPlaceholder::CameraPlaceholder() : cameraAvailable_(false), streamActive_(false) {}

bool CameraPlaceholder::begin(AsyncWebServer &server)
{
	camera_config_t config = makeCameraConfig();

	if (esp_camera_init(&config) == ESP_OK)
	{
		sensor_t *s = esp_camera_sensor_get();
		if (s != nullptr)
		{
			if (s->id.PID == OV3660_PID)
			{
				// Adjust defaults for OV3660 to avoid washed colors
				s->set_brightness(s, 1);
				s->set_saturation(s, 0);
			}
			else if (s->id.PID == OV2640_PID)
			{
				// OV2640 baseline tweaks
				s->set_brightness(s, 0);
				s->set_saturation(s, 0);
			}
			// Keep very small frame to avoid WDT; can be tuned upward when stable
			s->set_framesize(s, FRAMESIZE_QQVGA);
			s->set_quality(s, 15);
		}

		cameraAvailable_ = true;
		registerHttpEndpoints(server);
		Serial.println("Camera initialized (OV3660 detected)");
	}
	else
	{
		cameraAvailable_ = false;
		Serial.println("Camera not found or failed to init; continuing without video");
	}

	return cameraAvailable_;
}

bool CameraPlaceholder::isAvailable() const
{
	return cameraAvailable_;
}

void CameraPlaceholder::registerHttpEndpoints(AsyncWebServer &server)
{
	// Status endpoint for UI to detect availability
	server.on("/api/camera/status", HTTP_GET, [this](AsyncWebServerRequest *request) {
		if (cameraAvailable_)
		{
			request->send(200, "application/json", "{\"available\":true}");
		}
		else
		{
			request->send(503, "application/json", "{\"available\":false}");
		}
	});

	// MJPEG stream endpoint
	server.on("/stream", HTTP_GET, [this](AsyncWebServerRequest *request) {
		if (!cameraAvailable_)
		{
			request->send(503, "text/plain", "Camera not available");
			return;
		}

		if (streamActive_)
		{
			request->send(429, "text/plain", "Camera stream already in use");
			return;
		}

		streamActive_ = true;

		AsyncJpegStreamResponse *response = new AsyncJpegStreamResponse(80); // start ~12 fps for stability
		response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
		response->addHeader("Pragma", "no-cache");
		request->onDisconnect([this]() { streamActive_ = false; });
		request->send(response);
	});
}