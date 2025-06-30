#include "encoder_service.h"

#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#define DEBOUNCE_MS       20
#define LONG_PRESS_MS    600
#define DOUBLE_CLICK_MS  400

// === PCNT ===
static pcnt_unit_handle_t pcnt_unit = NULL;
static QueueHandle_t event_queue = NULL;

// === BUTTON FSM ===
static int btn_gpio = -1;
static bool btn_pressed = false;
static int64_t last_press_time = 0;

static esp_timer_handle_t debounce_timer;
static esp_timer_handle_t long_timer;
static esp_timer_handle_t double_timer;

typedef enum {
    BTN_STATE_IDLE,
    BTN_STATE_WAIT_RELEASE,
    BTN_STATE_WAIT_SECOND
} btn_state_t;

static btn_state_t btn_state = BTN_STATE_IDLE;

static bool double_click_waiting = false;


// === ISR ===
static void IRAM_ATTR button_isr(void *arg) {
    esp_timer_start_once(debounce_timer, DEBOUNCE_MS * 1000);
}

// === Timer Callbacks ===

static void debounce_cb(void *arg) {
    int level = gpio_get_level(btn_gpio);

    if (level == 0) { // Pressed
        btn_pressed = true;
        last_press_time = esp_timer_get_time();
        esp_timer_start_once(long_timer, LONG_PRESS_MS * 1000);
        btn_state = BTN_STATE_WAIT_RELEASE;

    } else if (level == 1 && btn_state == BTN_STATE_WAIT_RELEASE) { // Released
        esp_timer_stop(long_timer);

        // Если это второй клик (таймер уже был запущен)
        if (double_click_waiting) {
            double_click_waiting = false;
            esp_timer_stop(double_timer);  // не ждать больше
            encoder_event_t evt = { .type = ENCODER_EVENT_BUTTON_DOUBLE_CLICK };
            xQueueSend(event_queue, &evt, 0);
            btn_state = BTN_STATE_IDLE;
        } else {
            // Первый клик — ждем второй
            double_click_waiting = true;
            esp_timer_start_once(double_timer, DOUBLE_CLICK_MS * 1000);
            btn_state = BTN_STATE_WAIT_SECOND;
        }
    }
}

static void long_cb(void *arg) {
    if (btn_state == BTN_STATE_WAIT_RELEASE && gpio_get_level(btn_gpio) == 0) {
        encoder_event_t evt = { .type = ENCODER_EVENT_BUTTON_LONG_PRESS };
        xQueueSend(event_queue, &evt, 0);
        btn_state = BTN_STATE_IDLE;
    }
}

static void double_cb(void *arg) {
    if (double_click_waiting) {
        double_click_waiting = false;
        encoder_event_t evt = { .type = ENCODER_EVENT_BUTTON_CLICK };
        xQueueSend(event_queue, &evt, 0);
        btn_state = BTN_STATE_IDLE;
    }
}

// === PCNT Callback ===
static bool encoder_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {

    int value = 0;
    pcnt_unit_get_count(unit, &value);
    encoder_event_t evt = {
        .type = ENCODER_EVENT_ROTATE,
        .rotate_delta = (value > 0) ? 1 : -1
    };

    xQueueSendFromISR(event_queue, &evt, NULL);
    pcnt_unit_clear_count(unit);
    return true;
}

// === Init ===
void encoder_service_init(int gpio_a, int gpio_b, int gpio_btn) {
    btn_gpio = gpio_btn;
    event_queue = xQueueCreate(8, sizeof(encoder_event_t));

    // === PCNT Setup ===
    pcnt_unit_config_t unit_config = {
        .high_limit = 10,
        .low_limit = -10
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_chan_config_t chan_a = {
        .edge_gpio_num = gpio_a,
        .level_gpio_num = gpio_b
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a, &pcnt_chan));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    pcnt_event_callbacks_t cbs = {
        .on_reach = encoder_pcnt_on_reach
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, NULL));

    // === Watch points for +1 and -1 counts
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, 2));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, -2));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));    

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));

    // === GPIO Button Setup ===
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << btn_gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_conf);

    gpio_isr_handler_add(btn_gpio, button_isr, NULL);

    // === Timers ===
    esp_timer_create(&(esp_timer_create_args_t){
        .callback = debounce_cb,
        .name = "debounce"
    }, &debounce_timer);

    esp_timer_create(&(esp_timer_create_args_t){
        .callback = long_cb,
        .name = "longpress"
    }, &long_timer);

    esp_timer_create(&(esp_timer_create_args_t){
        .callback = double_cb,
        .name = "doubleclick"
    }, &double_timer);
}

void encoder_service_start(void) {
    ESP_LOGI("ENCODER", "Starting PCNT unit...");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

QueueHandle_t encoder_service_get_event_queue(void) {
    return event_queue;
}
