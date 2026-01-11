#pragma once

#include <Arduino.h>
#include <WiFi.h>

class WifiManager
{
public:
	WifiManager();

	bool startAccessPoint(const String &ssid, const String &password, uint8_t channel = 1, uint8_t maxConnections = 4);
	bool isConnected() const;
	IPAddress getIp() const;
	String getIpString() const;

private:
	bool apActive_;
};