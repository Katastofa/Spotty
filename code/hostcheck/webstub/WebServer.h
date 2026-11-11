#pragma once
#include "web_host.h"
#include <cstddef>
enum HTTPMethod { HTTP_ANY, HTTP_GET, HTTP_POST };
#define CONTENT_LENGTH_UNKNOWN ((size_t)-1)
class WebServer {
public:
  typedef void (*THandler)();
  WebServer(int = 80) {}
  void on(const char *, HTTPMethod, THandler) {}
  void on(const char *, THandler) {}
  void onNotFound(THandler) {}
  void begin() {} void handleClient() {}
  void send(int, const char *, const char *) {}
  void send(int, const char *, const String &) {}
  void send_P(int, const char *, const char *) {}
  void sendHeader(const char *, const char *, bool = false) {}
  void setContentLength(size_t) {}
  void sendContent(const char *) {}
  void sendContent(const char *, size_t) {}
  bool hasArg(const char *) { return false; }
  String arg(const char *) { return String(); }
  String header(const char *) { return String(); }
  bool hasHeader(const char *) { return false; }
  void collectHeaders(const char **, size_t) {}
  HTTPMethod method() { return HTTP_GET; }
};
