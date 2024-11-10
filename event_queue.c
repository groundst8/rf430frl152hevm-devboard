/*
 * event_queue.c
 *
 *  Created on: Nov 9, 2024
 *      Author: nick
 */


#include "event_queue.h"

// Initialize the event queue
void event_queue_init(EventQueue *queue) {
    queue->head = 0;
    queue->tail = 0;
}

// Add an event to the queue
bool event_queue_put(EventQueue *queue, const Event *event) {
    // bitwise AND instead of modulo for wrapping to save power, buffer must be power of 2
    uint8_t next_head = (queue->head + 1) & EVENT_QUEUE_MASK;
    if (next_head == queue->tail) {
        // Queue is full
        return false;
    }
    queue->buffer[queue->head] = *event;
    queue->head = next_head;
    return true;
}

// Get an event from the queue
bool event_queue_get(EventQueue *queue, Event *event) {
    if (queue->head == queue->tail) {
        // Queue is empty
        return false;
    }
    *event = queue->buffer[queue->tail];
    // bitwise AND instead of modulo for wrapping to save power, buffer must be power of 2
    queue->tail = (queue->tail + 1) & EVENT_QUEUE_MASK;
    return true;
}
