// ESP-IDF Motion Queue Implementation
// Thread-safe circular buffer for motion commands

#include "MotionQueue.h"
#include "esp_log.h"
#include <cstdlib>

static const char* TAG = "MotionQueue";

MotionQueue::MotionQueue()
    : buffer(nullptr)
    , head(0)
    , tail(0)
    , count(0)
    , size(0)
    , mutex(nullptr)
{
}

MotionQueue::~MotionQueue() {
    if (buffer) {
        free(buffer);
        buffer = nullptr;
    }
    if (mutex) {
        vSemaphoreDelete(mutex);
        mutex = nullptr;
    }
    size = head = tail = count = 0;
}

bool MotionQueue::push(const MotionCommand& cmd) {
    if (!mutex || !buffer) {
        return false;
    }

    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    bool result = false;
    if (count < size) {
        buffer[tail] = cmd;
        tail = (tail + 1) % size;
        count++;
        result = true;
    }

    xSemaphoreGive(mutex);
    return result;
}

bool MotionQueue::pop(MotionCommand& cmd) {
    if (!mutex || !buffer) {
        return false;
    }

    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    bool result = false;
    if (count > 0) {
        cmd = buffer[head];
        head = (head + 1) % size;
        count--;
        result = true;
    }

    xSemaphoreGive(mutex);
    return result;
}

bool MotionQueue::peekLast(MotionCommand& cmd) const {
    if (!mutex || !buffer) {
        return false;
    }

    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    bool result = false;
    if (count > 0) {
        // Calculate index of last item (one before tail)
        size_t lastIndex = (tail == 0) ? (size - 1) : (tail - 1);
        cmd = buffer[lastIndex];
        result = true;
    }

    xSemaphoreGive(mutex);
    return result;
}

void MotionQueue::clear() {
    if (!mutex) {
        return;
    }

    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        head = tail = count = 0;
        xSemaphoreGive(mutex);
    }
}

bool MotionQueue::init(size_t queueSize) {
    if (queueSize == 0) {
        return false;
    }

    // Limit maximum queue size to prevent excessive memory usage
    if (queueSize > 1000) {
        queueSize = 1000;
    }

    buffer = (MotionCommand*)malloc(queueSize * sizeof(MotionCommand));
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate motion queue buffer (%zu bytes)", queueSize * sizeof(MotionCommand));
        return false;
    }

    mutex = xSemaphoreCreateMutex();
    if (!mutex) {
        free(buffer);
        buffer = nullptr;
        return false;
    }

    size = queueSize;
    head = tail = count = 0;

    ESP_LOGI(TAG, "Motion queue initialized with size %zu", queueSize);
    return true;
}
