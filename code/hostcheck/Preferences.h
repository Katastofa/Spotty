#pragma once
// Заглушка NVS для офлайн-сборки. Хранит данные в памяти процесса: скетчу
// нужно, чтобы код компилировался и логика ветвилась, а не чтобы что-то
// пережило перезапуск. Настоящий Preferences живёт только в ESP32.
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

class Preferences {
 public:
  bool begin(const char *ns, bool ro = false) { _ns = ns ? ns : ""; (void)ro; return true; }
  void end() {}
  bool isKey(const char *k) { return _store().count(_key(k)) != 0; }
  bool remove(const char *k) { return _store().erase(_key(k)) != 0; }
  bool clear() {
    for (auto it = _store().begin(); it != _store().end();) {
      if (it->first.compare(0, _ns.size() + 1, _ns + "/") == 0) it = _store().erase(it);
      else ++it;
    }
    return true;
  }
  size_t putBytes(const char *k, const void *v, size_t n) {
    auto &d = _store()[_key(k)];
    d.assign((const uint8_t *)v, (const uint8_t *)v + n);
    return n;
  }
  size_t getBytes(const char *k, void *out, size_t n) {
    auto it = _store().find(_key(k));
    if (it == _store().end()) return 0;
    size_t m = it->second.size() < n ? it->second.size() : n;
    memcpy(out, it->second.data(), m);
    return m;
  }
  size_t putDouble(const char *k, double v) { return putBytes(k, &v, sizeof v); }
  double getDouble(const char *k, double d = 0) { double v; return getBytes(k, &v, sizeof v) == sizeof v ? v : d; }
  size_t putFloat(const char *k, float v) { return putBytes(k, &v, sizeof v); }
  float  getFloat(const char *k, float d = 0) { float v; return getBytes(k, &v, sizeof v) == sizeof v ? v : d; }
  size_t getBytesLength(const char *k) {
    auto it = _store().find(_key(k));
    return it == _store().end() ? 0 : it->second.size();
  }

 private:
  std::string _ns;
  std::string _key(const char *k) { return _ns + "/" + (k ? k : ""); }
  static std::map<std::string, std::vector<uint8_t>> &_store() {
    static std::map<std::string, std::vector<uint8_t>> s;
    return s;
  }
};
