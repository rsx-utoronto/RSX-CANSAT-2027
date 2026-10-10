#pragma once
#include <freertos/FreeRTOS.h>
#include <cstring>
// Sequential test double only; never used in firmware or as a concurrency proof.
inline QueueHandle_t xQueueCreateStatic(std::size_t count, std::size_t width, std::uint8_t* storage, StaticQueue_t* q) {
    *q = {count,width,0,0,storage}; return q;
}
inline BaseType_t xQueueSend(QueueHandle_t q, const void* item, int) {
    if (q->count == q->capacity) return 0;
    std::memcpy(q->storage + ((q->head + q->count) % q->capacity) * q->width, item, q->width);
    ++q->count; return pdTRUE;
}
inline BaseType_t xQueueReceive(QueueHandle_t q, void* item, int) {
    if (!q->count) return 0;
    std::memcpy(item, q->storage + q->head * q->width, q->width);
    q->head = (q->head + 1) % q->capacity; --q->count; return pdTRUE;
}
