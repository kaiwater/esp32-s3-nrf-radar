/* ST7735 160x128 radar UI for the ESP32-S3 NRF24L01 link-quality sensor. */
#include "st7735.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_st7735.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define WIDTH 160
#define HEIGHT 128
#define STRIP_H 8
#define STRIP_PIXELS (WIDTH * STRIP_H)

#define PIN_SCK 12
#define PIN_MOSI 11
#define PIN_CS 10
#define PIN_DC 9
#define PIN_RST 8
#define PIN_BL 7

#define PIN_BTN_UP 4
#define PIN_BTN_DOWN 5
#define PIN_BTN_OK 6
#define PIN_BTN_BACK 14

#define LINK_Y 20
#define LINK_STEP 18
#define HIST_Y 80
#define HIST_STEP 10
#define STATUS_Y 112

#define RGB565(r, g, b) ((uint16_t)((((r) >> 3) << 11) | (((g) >> 2) << 5) | ((b) >> 3)))
#define COL_BG     RGB565(0x00, 0x00, 0x00)
#define COL_TEXT   RGB565(0xCC, 0xFF, 0xDD)
#define COL_GREEN  RGB565(0x33, 0xFF, 0x66)
#define COL_DIM    RGB565(0x1E, 0x8A, 0x46)
#define COL_DARK   RGB565(0x06, 0x2A, 0x14)

static const char *TAG = "st7735";
static esp_lcd_panel_handle_t s_panel;
static esp_lcd_panel_io_handle_t s_io;
static SemaphoreHandle_t s_flush_done;
static uint16_t *s_strip;
static int s_strip_y;
static uint8_t s_hist[3][WIDTH];

/* 8x8 font, public domain (font8x8 / VGA-derived). */
static const uint8_t s_font8[64][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /*   */
    {0x18, 0x3C, 0x3C, 0x18, 0x18, 0x00, 0x18, 0x00}, /* ! */
    {0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* quote */
    {0x36, 0x36, 0x7F, 0x36, 0x7F, 0x36, 0x36, 0x00}, /* # */
    {0x0C, 0x3E, 0x03, 0x1E, 0x30, 0x1F, 0x0C, 0x00}, /* $ */
    {0x00, 0x63, 0x33, 0x18, 0x0C, 0x66, 0x63, 0x00}, /* % */
    {0x1C, 0x36, 0x1C, 0x6E, 0x3B, 0x33, 0x6E, 0x00}, /* & */
    {0x06, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00}, /* ' */
    {0x18, 0x0C, 0x06, 0x06, 0x06, 0x0C, 0x18, 0x00}, /* ( */
    {0x06, 0x0C, 0x18, 0x18, 0x18, 0x0C, 0x06, 0x00}, /* ) */
    {0x00, 0x66, 0x3C, 0xFF, 0x3C, 0x66, 0x00, 0x00}, /* * */
    {0x00, 0x0C, 0x0C, 0x3F, 0x0C, 0x0C, 0x00, 0x00}, /* + */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x06}, /* , */
    {0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00}, /* - */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00}, /* . */
    {0x60, 0x30, 0x18, 0x0C, 0x06, 0x03, 0x01, 0x00}, /* / */
    {0x3E, 0x63, 0x73, 0x7B, 0x6F, 0x67, 0x3E, 0x00}, /* 0 */
    {0x0C, 0x0E, 0x0C, 0x0C, 0x0C, 0x0C, 0x3F, 0x00}, /* 1 */
    {0x1E, 0x33, 0x30, 0x1C, 0x06, 0x33, 0x3F, 0x00}, /* 2 */
    {0x1E, 0x33, 0x30, 0x1C, 0x30, 0x33, 0x1E, 0x00}, /* 3 */
    {0x38, 0x3C, 0x36, 0x33, 0x7F, 0x30, 0x78, 0x00}, /* 4 */
    {0x3F, 0x03, 0x1F, 0x30, 0x30, 0x33, 0x1E, 0x00}, /* 5 */
    {0x1C, 0x06, 0x03, 0x1F, 0x33, 0x33, 0x1E, 0x00}, /* 6 */
    {0x3F, 0x33, 0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x00}, /* 7 */
    {0x1E, 0x33, 0x33, 0x1E, 0x33, 0x33, 0x1E, 0x00}, /* 8 */
    {0x1E, 0x33, 0x33, 0x3E, 0x30, 0x18, 0x0E, 0x00}, /* 9 */
    {0x00, 0x0C, 0x0C, 0x00, 0x00, 0x0C, 0x0C, 0x00}, /* : */
    {0x00, 0x0C, 0x0C, 0x00, 0x00, 0x0C, 0x0C, 0x06}, /* ; */
    {0x18, 0x0C, 0x06, 0x03, 0x06, 0x0C, 0x18, 0x00}, /* < */
    {0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F, 0x00, 0x00}, /* = */
    {0x06, 0x0C, 0x18, 0x30, 0x18, 0x0C, 0x06, 0x00}, /* > */
    {0x1E, 0x33, 0x30, 0x18, 0x0C, 0x00, 0x0C, 0x00}, /* ? */
    {0x3E, 0x63, 0x7B, 0x7B, 0x7B, 0x03, 0x1E, 0x00}, /* @ */
    {0x0C, 0x1E, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x00}, /* A */
    {0x3F, 0x66, 0x66, 0x3E, 0x66, 0x66, 0x3F, 0x00}, /* B */
    {0x3C, 0x66, 0x03, 0x03, 0x03, 0x66, 0x3C, 0x00}, /* C */
    {0x1F, 0x36, 0x66, 0x66, 0x66, 0x36, 0x1F, 0x00}, /* D */
    {0x7F, 0x46, 0x16, 0x1E, 0x16, 0x46, 0x7F, 0x00}, /* E */
    {0x7F, 0x46, 0x16, 0x1E, 0x16, 0x06, 0x0F, 0x00}, /* F */
    {0x3C, 0x66, 0x03, 0x03, 0x73, 0x66, 0x7C, 0x00}, /* G */
    {0x33, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x33, 0x00}, /* H */
    {0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00}, /* I */
    {0x78, 0x30, 0x30, 0x30, 0x33, 0x33, 0x1E, 0x00}, /* J */
    {0x67, 0x66, 0x36, 0x1E, 0x36, 0x66, 0x67, 0x00}, /* K */
    {0x0F, 0x06, 0x06, 0x06, 0x46, 0x66, 0x7F, 0x00}, /* L */
    {0x63, 0x77, 0x7F, 0x7F, 0x6B, 0x63, 0x63, 0x00}, /* M */
    {0x63, 0x67, 0x6F, 0x7B, 0x73, 0x63, 0x63, 0x00}, /* N */
    {0x1C, 0x36, 0x63, 0x63, 0x63, 0x36, 0x1C, 0x00}, /* O */
    {0x3F, 0x66, 0x66, 0x3E, 0x06, 0x06, 0x0F, 0x00}, /* P */
    {0x1E, 0x33, 0x33, 0x33, 0x3B, 0x1E, 0x38, 0x00}, /* Q */
    {0x3F, 0x66, 0x66, 0x3E, 0x36, 0x66, 0x67, 0x00}, /* R */
    {0x1E, 0x33, 0x07, 0x0E, 0x38, 0x33, 0x1E, 0x00}, /* S */
    {0x3F, 0x2D, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00}, /* T */
    {0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x3F, 0x00}, /* U */
    {0x33, 0x33, 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x00}, /* V */
    {0x63, 0x63, 0x63, 0x6B, 0x7F, 0x77, 0x63, 0x00}, /* W */
    {0x63, 0x63, 0x36, 0x1C, 0x1C, 0x36, 0x63, 0x00}, /* X */
    {0x33, 0x33, 0x33, 0x1E, 0x0C, 0x0C, 0x1E, 0x00}, /* Y */
    {0x7F, 0x63, 0x31, 0x18, 0x4C, 0x66, 0x7F, 0x00}, /* Z */
    {0x1E, 0x06, 0x06, 0x06, 0x06, 0x06, 0x1E, 0x00}, /* [ */
    {0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x40, 0x00}, /* backslash */
    {0x1E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1E, 0x00}, /* ] */
    {0x08, 0x1C, 0x36, 0x63, 0x00, 0x00, 0x00, 0x00}, /* ^ */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF}, /* _ */
};

static unsigned glyph3x5(char c)
{
    switch (c) {
    case '0': return 075557; case '1': return 026227; case '2': return 071747;
    case '3': return 071717; case '4': return 055711; case '5': return 074717;
    case '6': return 074757; case '7': return 071111; case '8': return 075757;
    case '9': return 075717; case 'A': return 025755; case 'B': return 065656;
    case 'C': return 074447; case 'D': return 065556; case 'E': return 074647;
    case 'F': return 074644; case 'G': return 074557; case 'H': return 055755;
    case 'I': return 072227; case 'J': return 011157; case 'K': return 055655;
    case 'L': return 044447; case 'M': return 057555; case 'N': return 057755;
    case 'O': return 025552; case 'P': return 075744; case 'Q': return 075573;
    case 'R': return 075765; case 'S': return 074717; case 'T': return 072222;
    case 'U': return 055557; case 'V': return 055552; case 'W': return 055575;
    case 'X': return 055255; case 'Y': return 055222; case 'Z': return 071247;
    case '-': return 000700; case '.': return 000002; case '/': return 011244;
    case ':': return 002020; case '+': return 002720; case '!': return 022200;
    default: return 0;
    }
}

static void px(int x, int y, uint16_t color)
{
    int row = y - s_strip_y;
    if (x < 0 || x >= WIDTH || row < 0 || row >= STRIP_H) return;
    s_strip[row * WIDTH + x] = color;
}

static void rect(int x, int y, int w, int h, uint16_t color)
{
    if (x < 0) { w += x; x = 0; }
    if (x + w > WIDTH) w = WIDTH - x;
    if (w <= 0) return;
    int y0 = y < s_strip_y ? s_strip_y : y;
    int y1 = y + h > s_strip_y + STRIP_H ? s_strip_y + STRIP_H : y + h;
    for (int yy = y0; yy < y1; yy++)
        for (int xx = x; xx < x + w; xx++) s_strip[(yy - s_strip_y) * WIDTH + xx] = color;
}

static void text3x5(int x, int y, const char *s, int scale, uint16_t color)
{
    for (; *s; s++, x += 4 * scale) {
        unsigned bits = glyph3x5(*s);
        for (int row = 0; row < 5; row++)
            for (int col = 0; col < 3; col++)
                if (bits & (1u << (14 - row * 3 - col)))
                    rect(x + col * scale, y + row * scale, scale, scale, color);
    }
}

static void text8(int x, int y, const char *s, int scale, uint16_t color)
{
    for (; *s; s++, x += 8 * scale) {
        char c = *s;
        if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
        if (c < 32 || c > 95) c = '?';
        const uint8_t *g = s_font8[c - 32];
        for (int row = 0; row < 8; row++)
            for (int col = 0; col < 8; col++)
                if (g[row] & (1u << col))
                    rect(x + col * scale, y + row * scale, scale, scale, color);
    }
}

static void clear_strip(void)
{
    uint32_t pair = ((uint32_t)COL_BG << 16) | COL_BG;
    uint32_t *p = (uint32_t *)s_strip;
    for (int i = 0; i < STRIP_PIXELS / 2; i++) p[i] = pair;
}

static bool on_color_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *edata, void *ctx)
{
    (void)io;
    (void)edata;
    (void)ctx;
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(s_flush_done, &hp);
    return hp == pdTRUE;
}

void st7735_init(void)
{
    const spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_SCK,
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = STRIP_PIXELS * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = PIN_CS,
        .dc_gpio_num = PIN_DC,
        .spi_mode = 0,
        .pclk_hz = 27 * 1000 * 1000,
        .trans_queue_depth = 4,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &s_io));

    s_flush_done = xSemaphoreCreateBinary();
    if (!s_flush_done) abort();
    const esp_lcd_panel_io_callbacks_t cbs = {.on_color_trans_done = on_color_done};
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(s_io, &cbs, NULL));

    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7735(s_io, &panel_cfg, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    const gpio_config_t bl = {
        .pin_bit_mask = 1ULL << PIN_BL,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&bl));
    ESP_ERROR_CHECK(gpio_set_level(PIN_BL, 1));

    s_strip = heap_caps_malloc(STRIP_PIXELS * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_strip) {
        ESP_LOGE(TAG, "no memory for strip buffer");
        abort();
    }
    memset(s_strip, 0, STRIP_PIXELS * sizeof(uint16_t));
    memset(s_hist, 0, sizeof(s_hist));

    const gpio_config_t btn = {
        .pin_bit_mask = (1ULL << PIN_BTN_UP) | (1ULL << PIN_BTN_DOWN) |
                        (1ULL << PIN_BTN_OK) | (1ULL << PIN_BTN_BACK),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&btn));
    ESP_LOGI(TAG, "ST7735 ready");
}

void st7735_draw_radar(const radar_view_link_t *links, unsigned nlinks,
                       unsigned modules, unsigned channel,
                       bool motion, bool paused, bool calibrating)
{
    if (!s_panel || !s_strip) return;

    for (int i = 0; i < 3; i++) {
        memmove(&s_hist[i][0], &s_hist[i][1], WIDTH - 1);
        unsigned v = 0;
        if (i < (int)nlinks && links[i].active) {
            v = links[i].loss_pct * 2 + links[i].retries;
            if (v > 255) v = 255;
        }
        s_hist[i][WIDTH - 1] = (uint8_t)v;
    }

    char ch[8];
    snprintf(ch, sizeof(ch), "CH %u", channel);

    for (int y0 = 0; y0 < HEIGHT; y0 += STRIP_H) {
        s_strip_y = y0;
        clear_strip();

        if (y0 < LINK_Y) {
            text8(4, 1, "RF SENSE", 2, COL_TEXT);
            text3x5(136, 8, ch, 1, COL_DIM);
        }

        for (int i = 0; i < 3; i++) {
            int y = LINK_Y + i * LINK_STEP;
            if (y >= y0 + STRIP_H || y + 12 <= y0) continue;
            if (i < (int)nlinks && links[i].active) {
                text8(4, y, links[i].name, 1, COL_GREEN);
                const int bx = 28, by = y + 1, bw = 88, bh = 8;
                rect(bx, by, bw, bh, COL_DARK);
                unsigned q = links[i].quality;
                if (q > 100) q = 100;
                rect(bx, by, (int)(bw * q / 100), bh, COL_GREEN);
                rect(bx, by, bw, 1, COL_DIM);
                char t[16];
                snprintf(t, sizeof(t), "L%u", links[i].loss_pct);
                text3x5(120, y + 2, t, 1, COL_DIM);
                snprintf(t, sizeof(t), "R%u", links[i].retries);
                text3x5(140, y + 2, t, 1, COL_DIM);
            } else {
                text8(4, y, "--", 1, COL_DIM);
            }
        }

        for (int i = 0; i < 3; i++) {
            int y = HIST_Y + i * HIST_STEP;
            if (y >= y0 + STRIP_H || y + 6 <= y0) continue;
            for (int x = 0; x < WIDTH; x++) {
                uint8_t v = s_hist[i][x];
                uint16_t c = v < 24 ? COL_DARK : v < 80 ? COL_DIM : v < 160 ? COL_GREEN : COL_TEXT;
                for (int k = 0; k < 6; k++) px(x, y + k, c);
            }
        }

        if (y0 + STRIP_H > STATUS_Y) {
            if (nlinks == 0) {
                if (modules == 0) {
                    text8(4, 116, "NO NRF", 1, COL_DIM);
                } else {
                    const char *name = (modules & 1) ? "A" : (modules & 2) ? "B" : "C";
                    char m[20];
                    snprintf(m, sizeof(m), "%s OK  NEED 2", name);
                    text8(4, 116, m, 1, COL_GREEN);
                }
            } else if (paused) {
                text8(4, 116, "PAUSED", 1, COL_DIM);
            } else if (calibrating) {
                text8(4, 116, "CALIBRATING", 1, COL_GREEN);
            } else if (motion) {
                rect(0, STATUS_Y, WIDTH, HEIGHT - STATUS_Y, COL_GREEN);
                text8(4, 116, "MOTION", 1, COL_BG);
            } else {
                text8(4, 116, "CLEAR", 1, COL_GREEN);
                text8(108, 116, "K3 CAL", 1, COL_DIM);
            }
        }

        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(s_panel, 0, y0, WIDTH, y0 + STRIP_H, s_strip));
        if (s_flush_done) xSemaphoreTake(s_flush_done, pdMS_TO_TICKS(200));
    }
}

int st7735_button(void)
{
    static uint8_t stable;
    static uint32_t last;
    static uint32_t down_at[4];
    static bool was_down[4];

    uint8_t raw = 0;
    if (gpio_get_level(PIN_BTN_UP) == 0) raw |= 1;
    if (gpio_get_level(PIN_BTN_DOWN) == 0) raw |= 2;
    if (gpio_get_level(PIN_BTN_OK) == 0) raw |= 4;
    if (gpio_get_level(PIN_BTN_BACK) == 0) raw |= 8;

    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now - last < 20) return 0;
    last = now;
    stable = raw;

    for (int i = 0; i < 4; i++) {
        bool down = (stable >> i) & 1;
        if (down && !was_down[i]) down_at[i] = now;
        if (!down && was_down[i]) {
            was_down[i] = false;
            if (now - down_at[i] >= 30) return i + 1;
        }
        was_down[i] = down;
    }
    return 0;
}
