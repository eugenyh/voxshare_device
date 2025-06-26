#include <string.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include "st7735s_display.h"
#include "font8x8_basic.h"


// TFT Display SPI Configuration
#define LCD_HOST     SPI3_HOST
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   15
#define PIN_NUM_DC   2
#define PIN_NUM_RST  17
#define PIN_NUM_BCKL 21   

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 160

#define CHAR_WIDTH  8
#define CHAR_HEIGHT 8

#define X_OFFSET 2
#define Y_OFFSET 1

static spi_device_handle_t spi;

// Logging feature
static uint8_t current_line = 0;  
const uint8_t MAX_LINES = 16;      

#define CLEAR_BLOCK_PIXELS 64
static uint8_t clear_block[CLEAR_BLOCK_PIXELS * 2];

static uint8_t reverse_bits(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

static void lcd_cmd(uint8_t cmd) {
    gpio_set_level(PIN_NUM_DC, 0);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd
    };
    spi_device_transmit(spi, &t);
}

static void lcd_data(const uint8_t *data, int len) {
    gpio_set_level(PIN_NUM_DC, 1);
    while (len > 0) {
        int chunk = (len > 64) ? 64 : len;

        spi_transaction_t t = {
            .length = chunk * 8,      // в битах
            .tx_buffer = data,
            .flags = 0
        };
        esp_err_t ret = spi_device_transmit(spi, &t);
        assert(ret == ESP_OK);

        data += chunk;
        len -= chunk;
    }
}

static void lcd_reset(void) {
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void set_address_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    uint8_t data[4];

    x0 += X_OFFSET;
    x1 += X_OFFSET;
    y0 += Y_OFFSET;
    y1 += Y_OFFSET;

    lcd_cmd(0x2A); // Column addr set
    data[0] = 0x00; data[1] = x0;
    data[2] = 0x00; data[3] = x1;
    lcd_data(data, 4);

    lcd_cmd(0x2B); // Row addr set
    data[0] = 0x00; data[1] = y0;
    data[2] = 0x00; data[3] = y1;
    lcd_data(data, 4);

    lcd_cmd(0x2C); // Memory write
}

void st7735_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_DC) | (1ULL << PIN_NUM_RST),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&io_conf);

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_DISABLED);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
    };
    spi_bus_add_device(SPI3_HOST, &devcfg, &spi);

    lcd_reset();

    // Инициализация ST7735S (базовая)
    lcd_cmd(0x01); // Software reset
    vTaskDelay(pdMS_TO_TICKS(150));

    lcd_cmd(0x11); // Sleep out
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_cmd(0x3A); // Interface Pixel Format
    uint8_t colmod = 0x05; // 16bit/pixel
    lcd_data(&colmod, 1);

    lcd_cmd(0x36);  // Memory Access Control
    uint8_t madctl = 0xC0; // Нормальная ориентация
    lcd_data(&madctl, 1);

    lcd_cmd(0x29); // Display ON
    vTaskDelay(pdMS_TO_TICKS(50));

    st7735_clear(0x0000); // black
}

void st7735_clear(uint16_t color) {
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;
    for (int i = 0; i < CLEAR_BLOCK_PIXELS; i++) {
        clear_block[i * 2]     = hi;
        clear_block[i * 2 + 1] = lo;
    }

    set_address_window(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1);
    int total_pixels = SCREEN_WIDTH * SCREEN_HEIGHT;
    while (total_pixels > 0) {
        int count = (total_pixels > CLEAR_BLOCK_PIXELS) ? CLEAR_BLOCK_PIXELS : total_pixels;
        lcd_data(clear_block, count * 2);
        total_pixels -= count;
    }
}

void st7735_draw_char(uint8_t x, uint8_t y, char ch, uint16_t color, uint16_t bg) {
    uint8_t c = (uint8_t)ch;
    if (c > 127) return;

    set_address_window(x, y, x + CHAR_WIDTH - 1, y + CHAR_HEIGHT - 1);
    uint8_t line[CHAR_WIDTH * CHAR_HEIGHT * 2];
    int p = 0;
    for (uint8_t row = 0; row < CHAR_HEIGHT; row++) {
        uint8_t bits = reverse_bits(font8x8_basic[(uint8_t)ch][row]);
        for (uint8_t col = 0; col < CHAR_WIDTH; col++) {
            uint16_t pixel = (bits & (1 << (7 - col))) ? color : bg;
            line[p++] = pixel >> 8;
            line[p++] = pixel & 0xFF;
        }
    }
    lcd_data(line, sizeof(line));
}

void st7735_draw_string(uint8_t x, uint8_t y, const char *str, uint16_t color, uint16_t bg) {
    while (*str) {
        st7735_draw_char(x, y, *str++, color, bg);
        x += CHAR_WIDTH;
        if (x + CHAR_WIDTH > SCREEN_WIDTH) {
            x = 0;
            y += CHAR_HEIGHT;
        }
    }
}

// Logging feature
void display_log(const char* msg, uint16_t color, uint16_t bg) {
    st7735_draw_string(0, current_line*10, msg, color, bg);
    
    if(++current_line >= MAX_LINES) {
        current_line = 0;  // Сброс при достижении низа
        st7735_clear(bg);
    }
}

void reset_log_lines() {
    current_line = 0;  // Доступ к переменной возможен
}
