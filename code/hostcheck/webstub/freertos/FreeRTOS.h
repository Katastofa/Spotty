#pragma once
#include <cstdint>
typedef void *QueueHandle_t; typedef void *TaskHandle_t; typedef int BaseType_t; typedef uint32_t TickType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY 0xffffffffu
#define pdMS_TO_TICKS(x) (x)
static inline BaseType_t xTaskCreatePinnedToCore(void (*)(void *), const char *, uint32_t, void *, unsigned, TaskHandle_t *, int) { return pdPASS; }
static inline void vTaskDelay(TickType_t) {}
static inline void vTaskDelete(TaskHandle_t) {}
static inline QueueHandle_t xQueueCreate(unsigned, unsigned) { return nullptr; }
static inline BaseType_t xQueueSend(QueueHandle_t, const void *, TickType_t) { return pdTRUE; }
static inline BaseType_t xQueueReceive(QueueHandle_t, void *, TickType_t) { return pdFALSE; }
static inline unsigned uxQueueMessagesWaiting(QueueHandle_t) { return 0; }
