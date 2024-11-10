/*
 * event_queue.h
 *
 *  Created on: Nov 9, 2024
 *      Author: nick
 */

#include <stdint.h>
#include <stdbool.h>

#ifndef EVENT_QUEUE_H
#define EVENT_QUEUE_H

// Define event types
typedef enum {
    EVENT_NFC,
    EVENT_TIMER
} EventType;

// Event structure
typedef struct {
    EventType type;
    uint8_t data;  // For NFC event, data is the digit
} Event;

// Event queue structure
#define EVENT_QUEUE_SIZE 16  // IMPORTANT: Must be a power of two
#define EVENT_QUEUE_MASK (EVENT_QUEUE_SIZE - 1)

typedef struct {
    Event buffer[EVENT_QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
} EventQueue;

// Function prototypes
void event_queue_init(EventQueue *queue);
bool event_queue_put(EventQueue *queue, const Event *event);
bool event_queue_get(EventQueue *queue, Event *event);

#endif // EVENT_QUEUE
