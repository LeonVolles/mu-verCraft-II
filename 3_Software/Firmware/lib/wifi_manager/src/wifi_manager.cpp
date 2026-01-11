#include "wifi_manager.h"

WifiManager::WifiManager() : apActive_(false) {}

bool WifiManager::startAccessPoint(const String &ssid, const String &password, uint8_t channel, uint8_t maxConnections)
{
	WiFi.mode(WIFI_AP);
	WiFi.setSleep(false); // keep radio responsive for websocket/video traffic
	apActive_ = WiFi.softAP(ssid.c_str(), password.c_str(), channel, 0, maxConnections);
	if (apActive_)
	{
		Serial.print("AP started. SSID: ");
		Serial.print(ssid);
		Serial.print(" IP: ");
		Serial.println(WiFi.softAPIP());
	}
	else
	{
		Serial.println("Failed to start AP");
	}
	return apActive_;
}

bool WifiManager::isConnected() const
{
	return apActive_ || WiFi.isConnected();
}

IPAddress WifiManager::getIp() const
{
	return WiFi.getMode() == WIFI_AP ? WiFi.softAPIP() : WiFi.localIP();
}

String WifiManager::getIpString() const
{
	return getIp().toString();
}