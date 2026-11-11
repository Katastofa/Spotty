#pragma once
#include "web_host.h"
enum { WL_CONNECTED = 3, WL_DISCONNECTED = 6 };
enum wifi_mode_t { WIFI_OFF = 0, WIFI_STA = 1, WIFI_AP = 2 };
struct IPAddress { String toString() const { return String("0.0.0.0"); } };
struct WiFiClass {
  void persistent(bool) {} bool mode(wifi_mode_t) { return true; } bool setSleep(bool) { return true; }
  bool setHostname(const char *) { return true; } int begin(const char *, const char *) { return 0; }
  int status() { return 0; } IPAddress localIP() { return IPAddress(); } int RSSI() { return 0; }
  bool disconnect(bool = false) { return true; }
  bool reconnect() { return true; }
  bool setAutoReconnect(bool) { return true; }
  bool softAP(const char *, const char *, int = 1, int = 0, int = 4) { return true; }
  IPAddress softAPIP() { return IPAddress(); }
  int softAPgetStationNum() { return 0; }
};
extern WiFiClass WiFi;
