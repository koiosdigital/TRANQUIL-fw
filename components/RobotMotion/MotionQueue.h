// ESP-IDF Motion Queue for Polar Robot System
// Thread-safe circular buffer for motion commands

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstddef>

// Polar coordinates
struct PolarPoint {
    float theta;  // Angle in degrees
    float rho;    // Normalized radius (0.0 = center, 1.0 = maximum)
};

// Motion command
struct MotionCommand {
    PolarPoint target;
    float feedRate;     // RPM
    bool isRelative;
    uint32_t commandId;
};

class MotionQueue {
public:
    MotionQueue();
    ~MotionQueue();

    // Initialize queue with specified size
    bool init(size_t queueSize);

    // Add command to queue (thread-safe)
    bool push(const MotionCommand& cmd);

    // Remove and return next command (thread-safe)
    bool pop(MotionCommand& cmd);

    // Peek at last added command without removing (thread-safe)
    bool peekLast(MotionCommand& cmd) const;

    // Clear all commands from queue (thread-safe)
    void clear();

    // Get available space in queue
    size_t available() const { return size - count; }

    // Get current queue size
    size_t getCount() const { return count; }

    // Get maximum queue size
    size_t getSize() const { return size; }

    // Check if queue is empty
    bool isEmpty() const { return count == 0; }

    // Check if queue is full
    bool isFull() const { return count >= size; }

private:
    MotionCommand* buffer;
    size_t head, tail, count, size;
    SemaphoreHandle_t mutex;

    // Non-copyable
    MotionQueue(const MotionQueue&) = delete;
    MotionQueue& operator=(const MotionQueue&) = delete;
};
