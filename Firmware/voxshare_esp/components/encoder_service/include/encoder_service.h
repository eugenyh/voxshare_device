#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ENCODER_EVENT_ROTATE,
    ENCODER_EVENT_BUTTON_CLICK,
    ENCODER_EVENT_BUTTON_DOUBLE_CLICK,
    ENCODER_EVENT_BUTTON_LONG_PRESS
} encoder_event_type_t;

typedef struct {
    encoder_event_type_t type;
    int32_t rotate_delta;  // Только для ROTATE
} encoder_event_t;

void encoder_service_init(int gpio_a, int gpio_b, int gpio_button);
void encoder_service_start(void);
QueueHandle_t encoder_service_get_event_queue(void);

#ifdef __cplusplus
}
#endif

