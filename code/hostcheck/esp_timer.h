#pragma once
// Заглушка: время ESP32 в мкс. На стенде не идёт — замер генератора отказывает.
#include <cstdint>
static inline int64_t esp_timer_get_time(){return 0;}
