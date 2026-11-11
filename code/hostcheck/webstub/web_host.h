#pragma once
// Заглушки ESP32-ядра ТОЛЬКО для websyntax.sh (-fsyntax-only, без линковки).
// Проверяют, что spotty_web.h и webPublish собираются с SPOTTY_WEB 1.
#include <string>
#include <cstdint>
#include <cstring>
class String {
  std::string s;
public:
  String() {}
  String(const char *c) : s(c ? c : "") {}
  const char *c_str() const { return s.c_str(); }
  unsigned length() const { return (unsigned)s.size(); }
  bool operator==(const char *c) const { return s == c; }
  bool operator!=(const char *c) const { return s != c; }
  bool equals(const char *c) const { return s == c; }
  char operator[](unsigned i) const { return s[i]; }
  void trim() {}
  bool startsWith(const char *c) const { return s.rfind(c, 0) == 0; }
};
struct EspClass { uint32_t getFreeHeap() { return 0; } void restart() {} };
extern EspClass ESP;
#include <cstdarg>
#include <cstdio>
#ifndef PROGMEM
#define PROGMEM
#endif
class Print {
public:
  virtual ~Print() {}
  virtual size_t write(uint8_t) = 0;
  virtual size_t write(const uint8_t *b, size_t n) { for (size_t i = 0; i < n; i++) write(b[i]); return n; }
  size_t printf(const char *f, ...) { char t[256]; va_list a; va_start(a, f); int w = vsnprintf(t, sizeof t, f, a); va_end(a); return write((const uint8_t *)t, w > 0 ? (size_t)w : 0); }
  size_t print(const char *s) { return write((const uint8_t *)s, strlen(s)); }
  size_t print(const String &s) { return print(s.c_str()); }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(int v) { return printf("%d", v); }
  size_t print(unsigned v) { return printf("%u", v); }
  size_t print(long v) { return printf("%ld", v); }
  size_t print(unsigned long v) { return printf("%lu", v); }
  size_t print(double v, int d = 2) { return printf("%.*f", d, v); }
  size_t println() { return print("\n"); }
  template <class T> size_t println(const T &v) { size_t n = print(v); return n + println(); }
  size_t println(double v, int d) { size_t n = print(v, d); return n + println(); }
  void flush() {}
};
