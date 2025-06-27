#pragma once

// Основные цвета (RGB565)
#define TFT_COLOR_RED        0xF800     // 🔴 Красный:    R=31, G=0,  B=0
#define TFT_COLOR_GREEN      0x07E0     // 🟢 Зелёный:    R=0,  G=63, B=0
#define TFT_COLOR_BLUE       0x001F     // 🔵 Синий:      R=0,  G=0,  B=31
#define TFT_COLOR_YELLOW     0xFFE0     // 🟡 Жёлтый:     R=31, G=63, B=0
#define TFT_COLOR_CYAN       0x07FF     // 💧 Голубой:    R=0,  G=63, B=31
#define TFT_COLOR_MAGENTA    0xF81F     // 🟣 Пурпурный:  R=31, G=0,  B=31
#define TFT_COLOR_WHITE      0xFFFF     // ⬜ Белый:      R=31, G=63, B=31
#define TFT_COLOR_BLACK      0x0000     // ⬛ Чёрный:     R=0,  G=0,  B=0

// Дополнительные цвета (RGB565)
#define TFT_COLOR_ORANGE     0xFD20     // 🟠 Оранжевый
#define TFT_COLOR_PINK       0xFC9F     // 💖 Розовый
#define TFT_COLOR_GRAY       0x8410     // ◼️ Серый

void st7735_init(void);
void st7735_clear(uint16_t color);
void st7735_draw_char(uint8_t x, uint8_t y, char ch, uint16_t color, uint16_t bg);
void st7735_draw_string(uint8_t x, uint8_t y, const char *str, uint16_t color, uint16_t bg);

// Display log feature
void display_log(const char* msg, uint16_t color, uint16_t bg); 
void reset_log_lines();

char* adjust_string_width(const char* original_str, size_t width);
