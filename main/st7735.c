/* ST7735 160x128 radar UI: 10 palettes, link stats, RPD noise index. */
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
#define FRAME_PIXELS (WIDTH * HEIGHT)

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

#define STATUS_Y 18
#define LINK_Y 24
#define STATS1_Y 44
#define STATS2_Y 54
#define STATS3_Y 64
#define HIST_Y 82
#define HIST_X0 28
#define HIST_W (WIDTH - HIST_X0)
#define HIST_STEP 14

#define BTN_LONG_MS 700

#define RGB565(r, g, b) ((uint16_t)((((r) >> 3) << 11) | (((g) >> 2) << 5) | ((b) >> 3)))

typedef struct {
    const char *abbr;
    uint16_t bg, text, accent, dim, dark;
} ui_theme_t;

/* Ten well-known editor/terminal palettes plus phosphor/amber/ice CRT. */
static const ui_theme_t s_themes[] = {
    {"AMB", RGB565(0x00,0x00,0x00), RGB565(0xFF,0xE6,0xC0), RGB565(0xFF,0xB0,0x00), RGB565(0x7A,0x4A,0x00), RGB565(0x24,0x14,0x00)},
    {"ICE", RGB565(0x00,0x00,0x00), RGB565(0xE0,0xF8,0xFF), RGB565(0x38,0xD8,0xFF), RGB565(0x1E,0x6E,0x8A), RGB565(0x04,0x22,0x2E)},
    {"PHO", RGB565(0x00,0x00,0x00), RGB565(0xD8,0xFF,0xE4), RGB565(0x38,0xFF,0x78), RGB565(0x18,0x7A,0x3C), RGB565(0x04,0x24,0x14)},
    {"MAG", RGB565(0x0A,0x00,0x10), RGB565(0xFF,0xE0,0xF4), RGB565(0xFF,0x4F,0xD8), RGB565(0x8A,0x1E,0x78), RGB565(0x2A,0x00,0x24)},
    {"SOL", RGB565(0x00,0x2B,0x36), RGB565(0xF2,0xE8,0xC9), RGB565(0xE0,0xA0,0x00), RGB565(0x6A,0x6A,0x50), RGB565(0x07,0x36,0x42)},
    {"DRA", RGB565(0x28,0x2A,0x36), RGB565(0xF8,0xF8,0xF2), RGB565(0xFF,0x79,0xC6), RGB565(0x62,0x72,0xA4), RGB565(0x44,0x47,0x5A)},
    {"NRD", RGB565(0x2E,0x34,0x40), RGB565(0xEC,0xEF,0xF4), RGB565(0x88,0xC0,0xD0), RGB565(0x4C,0x56,0x6A), RGB565(0x3B,0x42,0x52)},
    {"GRU", RGB565(0x28,0x28,0x28), RGB565(0xEB,0xDB,0xB2), RGB565(0xFE,0x80,0x19), RGB565(0x92,0x83,0x74), RGB565(0x3C,0x38,0x36)},
    {"TOK", RGB565(0x1A,0x1B,0x26), RGB565(0xC0,0xCA,0xF5), RGB565(0xBB,0x9A,0xF7), RGB565(0x56,0x5F,0x89), RGB565(0x24,0x28,0x3B)},
    {"MON", RGB565(0x00,0x00,0x00), RGB565(0xFF,0xFF,0xFF), RGB565(0xC0,0xC0,0xC0), RGB565(0x60,0x60,0x60), RGB565(0x20,0x20,0x20)},
    {"CYB", RGB565(0x00,0x08,0x14), RGB565(0xE0,0xFF,0xFF), RGB565(0x00,0xE5,0xFF), RGB565(0x8A,0x1E,0x78), RGB565(0x1A,0x00,0x30)},
    {"LIM", RGB565(0x0A,0x00,0x14), RGB565(0xF0,0xE6,0xFF), RGB565(0xB6,0xFF,0x00), RGB565(0x5A,0x1E,0x8A), RGB565(0x1E,0x00,0x33)},
    {"ORB", RGB565(0x00,0x14,0x2E), RGB565(0xFF,0xE8,0xCC), RGB565(0xFF,0x7A,0x00), RGB565(0x1E,0x4A,0x8A), RGB565(0x00,0x22,0x4A)},
    {"RDC", RGB565(0x00,0x10,0x18), RGB565(0xE0,0xFF,0xFF), RGB565(0x00,0xE5,0xFF), RGB565(0x8A,0x1E,0x1E), RGB565(0x00,0x22,0x2E)},
    {"CAN", RGB565(0x1A,0x0A,0x14), RGB565(0xFF,0xE6,0xF2), RGB565(0xFF,0x6F,0xB5), RGB565(0x3A,0x8A,0x6A), RGB565(0x2E,0x14,0x24)},
    {"MIN", RGB565(0x0A,0x14,0x10), RGB565(0xE6,0xFF,0xF4), RGB565(0x5F,0xFF,0xB0), RGB565(0x8A,0x5A,0x3A), RGB565(0x14,0x2A,0x22)},
    {"LEM", RGB565(0x14,0x12,0x0A), RGB565(0xFF,0xFB,0xE0), RGB565(0xFF,0xE4,0x4D), RGB565(0x5A,0x3A,0x8A), RGB565(0x26,0x22,0x14)},
    {"SKY", RGB565(0x0A,0x12,0x20), RGB565(0xE6,0xF4,0xFF), RGB565(0x5F,0xB8,0xFF), RGB565(0x8A,0x5A,0x3A), RGB565(0x14,0x24,0x3A)},
    {"SAN", RGB565(0x1A,0x18,0x14), RGB565(0xE8,0xE2,0xD6), RGB565(0xC8,0xB8,0x9A), RGB565(0x6A,0x63,0x58), RGB565(0x2A,0x26,0x20)},
    {"SLA", RGB565(0x16,0x1A,0x1E), RGB565(0xD8,0xE0,0xE6), RGB565(0x8F,0xA6,0xB8), RGB565(0x56,0x64,0x6E), RGB565(0x23,0x2A,0x30)},
};
#define THEME_COUNT (sizeof(s_themes) / sizeof(s_themes[0]))

static const char *TAG = "st7735";
static esp_lcd_panel_handle_t s_panel;
static esp_lcd_panel_io_handle_t s_io;
static SemaphoreHandle_t s_flush_done;
static uint16_t *s_fb;
static int s_fb_y;
static uint8_t s_hist[3][HIST_W];
static uint16_t s_heat[256];
static unsigned s_theme = 0;  /* T1 AMBER default */

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
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
    s_fb[y * WIDTH + x] = color;
}

static void rect(int x, int y, int w, int h, uint16_t color)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > WIDTH) w = WIDTH - x;
    if (y + h > HEIGHT) h = HEIGHT - y;
    if (w <= 0 || h <= 0) return;
    for (int yy = y; yy < y + h; yy++)
        for (int xx = x; xx < x + w; xx++) s_fb[yy * WIDTH + xx] = color;
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
    uint16_t bg = s_themes[s_theme].bg;
    uint32_t pair = ((uint32_t)bg << 16) | bg;
    uint32_t *p = (uint32_t *)s_fb;
    for (int i = 0; i < FRAME_PIXELS / 2; i++) p[i] = pair;
}

static uint16_t mix565(uint16_t a, uint16_t b, unsigned num, unsigned den)
{
    int ar = (a >> 11) & 31, ag = (a >> 5) & 63, ab = a & 31;
    int br = (b >> 11) & 31, bg = (b >> 5) & 63, bb = b & 31;
    int r = ar + ((br - ar) * (int)num) / (int)den;
    int g = ag + ((bg - ag) * (int)num) / (int)den;
    int bl = ab + ((bb - ab) * (int)num) / (int)den;
    return (uint16_t)((r << 11) | (g << 5) | bl);
}

static void build_heat(void)
{
    /* Each theme uses its own dim -> accent -> text -> white ramp.  The
     * low end is dim, not the background, so the waterfall stays visible. */
    const ui_theme_t *c = &s_themes[s_theme];
    for (unsigned v = 0; v < 256; v++) {
        if (v < 64) {
            s_heat[v] = mix565(c->dim, c->accent, v, 64);
        } else if (v < 160) {
            s_heat[v] = mix565(c->accent, c->text, v - 64, 96);
        } else {
            s_heat[v] = mix565(c->text, 0xFFFF, v - 160, 95);
        }
    }
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

void st7735_next_theme(void)
{
    s_theme = (s_theme + 1) % THEME_COUNT;
    build_heat();
}

void st7735_init(void)
{
    const spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_SCK,
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = FRAME_PIXELS * sizeof(uint16_t),
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

    s_fb = heap_caps_malloc(FRAME_PIXELS * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_fb) {
        ESP_LOGE(TAG, "no memory for strip buffer");
        abort();
    }
    memset(s_fb, 0, FRAME_PIXELS * sizeof(uint16_t));
    memset(s_hist, 0, sizeof(s_hist));
    build_heat();

    const gpio_config_t btn = {
        .pin_bit_mask = (1ULL << PIN_BTN_UP) | (1ULL << PIN_BTN_DOWN) |
                        (1ULL << PIN_BTN_OK) | (1ULL << PIN_BTN_BACK),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&btn));
    ESP_LOGI(TAG, "ST7735 ready");
}

void st7735_draw_radar(const radar_view_t *v)
{
    if (!s_panel || !s_fb || !v) return;
    const ui_theme_t *c = &s_themes[s_theme];
    bool ab = v->nlinks > 0 && v->link[0].active;

    for (int i = 0; i < 3; i++) {
        memmove(&s_hist[i][0], &s_hist[i][1], HIST_W - 1);
        unsigned val = 0;
        if (i == 0 && ab) val = v->link[0].score;
        else if (i == 1) val = v->rf_energy * 2;
        else if (i == 2) val = v->motion ? 255 : 0;
        if (val > 255) val = 255;
        s_hist[i][HIST_W - 1] = (uint8_t)val;
    }

    char ch[8], th[12];
    snprintf(ch, sizeof(ch), "CH %u", v->channel);
    snprintf(th, sizeof(th), "T%u %s", s_theme + 1, c->abbr);

    for (int y0 = 0; y0 < HEIGHT; y0 += STRIP_H) {
        s_fb_y = y0;
        clear_strip();

        if (y0 < STATUS_Y) {
            text8(4, 1, "RF SENSE", 2, c->text);
            text3x5(136, 3, ch, 1, c->accent);
        }

        if (y0 + STRIP_H > STATUS_Y && y0 <= STATUS_Y + 5) {
            const char *st = "CLEAR";
            uint16_t sc = c->accent;
            if (v->booting) {
                st = "BOOTING";
                sc = c->text;
            } else if (v->nlinks == 0) {
                sc = c->dim;
                if (v->modules == 0) st = "NO NRF";
                else if (!(v->modules & 1) && (v->modules & 2)) st = "B OK NEED A";
                else if ((v->modules & 1) && !(v->modules & 2)) st = "A OK NEED B";
                else if (v->modules & 4) st = "C ONLY";
                else st = "NO LINK";
            } else if (v->paused) {
                st = "PAUSED";
                sc = c->dim;
            } else if (v->calibrating) {
                st = "CALIBRATING";
                sc = c->text;
            } else if (v->motion) {
                st = "MOTION";
            }
            text3x5(4, STATUS_Y, st, 1, sc);
            text3x5(128, STATUS_Y, (v->modules == 0 && !v->booting) ? "K4 RST" : th,
                    1, c->dim);
        }

        if (y0 < LINK_Y + 16 && y0 + STRIP_H > LINK_Y) {
            if (ab) {
                text8(4, LINK_Y, "AB", 2, c->accent);
                const int bx = 44, by = LINK_Y + 3, bw = 88, bh = 10;
                rect(bx, by, bw, bh, c->dark);
                unsigned q = v->link[0].quality;
                if (q > 100) q = 100;
                rect(bx, by, (int)(bw * q / 100), bh, c->accent);
                rect(bx, by, bw, 1, c->dim);
                char t[16];
                snprintf(t, sizeof(t), "L%u", v->link[0].loss_pct);
                text3x5(134, LINK_Y + 4, t, 1, c->dim);
                snprintf(t, sizeof(t), "R%u", v->link[0].retries);
                text3x5(152, LINK_Y + 4, t, 1, c->dim);
            } else {
                text8(4, LINK_Y, "--", 2, c->dim);
            }
        }

        char s[40];
        if (y0 + STRIP_H > STATS1_Y && y0 <= STATS1_Y + 8) {
            snprintf(s, sizeof(s), "TX %lu  RX %lu", v->tx, v->rx);
            text8(4, STATS1_Y, s, 1, c->text);
        }
        if (y0 + STRIP_H > STATS2_Y && y0 <= STATS2_Y + 8) {
            snprintf(s, sizeof(s), "FAIL %lu  RETRY %lu", v->fail, v->retry);
            text8(4, STATS2_Y, s, 1, c->text);
        }
        if (y0 + STRIP_H > STATS3_Y && y0 <= STATS3_Y + 8) {
            snprintf(s, sizeof(s), "LINK%u%%  NOISE%u%%", v->link_pct, v->rf_energy);
            text8(4, STATS3_Y, s, 1, c->text);
        }

        for (int i = 0; i < 3; i++) {
            int y = HIST_Y + i * HIST_STEP;
            if (y >= y0 + STRIP_H || y + 9 <= y0) continue;
            const char *lname = i == 0 ? "AB" : i == 1 ? "NOISE" : "MOTION";
            bool live = (i == 0 && ab) || i > 0;
            text3x5(2, y + 2, lname, 1, live ? c->accent : c->dim);
            for (int x = 0; x < HIST_W; x++) {
                uint16_t col = s_heat[s_hist[i][x]];
                for (int k = 0; k < 8; k++) px(HIST_X0 + x, y + k, col);
            }
            for (int x = HIST_X0; x < WIDTH; x++)
                px(x, y + 8, live ? c->dim : c->dark);
        }

        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(s_panel, 0, y0, WIDTH, y0 + STRIP_H, s_fb));
        if (s_flush_done) xSemaphoreTake(s_flush_done, pdMS_TO_TICKS(200));
    }
}

void st7735_draw_scan(const scan_view_t *v)
{
    if (!s_panel || !s_fb || !v) return;
    const ui_theme_t *c = &s_themes[s_theme];

    clear_strip();

    /* Thin working-area header, normal 3x5 weight. */
    text3x5(4, 1, "2.4G", 2, c->accent);
    text3x5(40, 1, "SCAN", 2, c->text);
    char th[12], rng[32];
    snprintf(th, sizeof(th), "T%u %s", s_theme + 1, c->abbr);
    snprintf(rng, sizeof(rng), "2400-2525");
    text3x5(80, 2, th, 1, c->accent);
    text3x5(120, 2, rng, 1, c->dim);

    /* Occupancy bars: 160 columns, 1 px each; upper area is 1/3 shorter. */
    for (int x = 0; x < SCAN_COLS; x++) {
        unsigned ch = (unsigned)x * (SCAN_CHANNELS - 1) / (SCAN_COLS - 1);
        int val = (v->delta_mode && v->have_baseline)
                      ? (v->delta[ch] > 0 ? v->delta[ch] : 0)
                      : v->duty[ch];
        if (val < 0) val = 0;
        if (val > 100) val = 100;
        int h = val * 35 / 100;
        uint16_t col = s_heat[val * 255 / 100];
        for (int y = 47 - h; y <= 47; y++) px(x, y, col);
    }
    for (int x = 0; x < WIDTH; x++) px(x, 48, c->dim);

    if (v->cursor_on) {
        int cx = (int)v->cursor_ch * (SCAN_COLS - 1) / (SCAN_CHANNELS - 1);
        for (int y = 12; y <= 47; y++) px(cx, y, c->text);
    }

    char s[32];
    int freq = 2400 + (int)v->peak_ch;
    if (v->delta_mode && v->have_baseline)
        snprintf(s, sizeof(s), "PEAK %d %+d", freq, v->peak_val);
    else
        snprintf(s, sizeof(s), "PEAK %d %d%%", freq, v->peak_val);
    text3x5(4, 51, s, 1, c->text);
    text3x5(140, 51, v->delta_mode ? "DLT" : "ABS", 1, c->accent);

    if (v->booting) {
        text3x5(4, 57, "BOOTING", 1, c->text);
    } else if (!v->radio_ok) {
        text3x5(4, 57, "NO NRF", 1, c->dim);
    } else if (v->paused) {
        text3x5(4, 57, "PAUSED", 1, c->dim);
    } else if (v->cursor_on) {
        int cf = 2400 + (int)v->cursor_ch;
        int val = (v->delta_mode && v->have_baseline)
                      ? (v->delta[v->cursor_ch] > 0 ? v->delta[v->cursor_ch] : 0)
                      : v->duty[v->cursor_ch];
        if (v->delta_mode && v->have_baseline)
            snprintf(s, sizeof(s), "CUR %d %+d", cf, val);
        else
            snprintf(s, sizeof(s), "CUR %d %d%%", cf, val);
        text3x5(4, 57, s, 1, c->text);
    } else {
        snprintf(s, sizeof(s), "SCAN %u", v->scans);
        text3x5(4, 57, s, 1, c->dim);
    }
    text3x5(112, 59, v->have_baseline ? "BASE" : "NOBASE", 1, c->dim);
    if (!v->radio_ok && !v->booting) text3x5(136, 59, "K4 RST", 1, c->dim);

    /* 160 x 21 waterfall, newest row at the top. */
    for (int r = 0; r < SCAN_ROWS; r++) {
        int y = 64 + r * 3;
        for (int x = 0; x < SCAN_COLS; x++) {
            uint16_t col = s_heat[v->wf[r][x]];
            px(x, y, col);
            px(x, y + 1, col);
        }
    }

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(s_panel, 0, 0, WIDTH, HEIGHT, s_fb));
    if (s_flush_done) xSemaphoreTake(s_flush_done, pdMS_TO_TICKS(200));
}

int st7735_button(void)
{
    static uint8_t stable;
    static uint32_t last;
    static uint32_t down_at[4];
    static bool was_down[4];
    static bool long_sent[4];

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
        if (down && !was_down[i]) {
            down_at[i] = now;
            long_sent[i] = false;
            was_down[i] = true;
            continue;
        }
        if (down && !long_sent[i] && now - down_at[i] >= BTN_LONG_MS) {
            long_sent[i] = true;
            return 11 + i;
        }
        if (!down && was_down[i]) {
            was_down[i] = false;
            if (!long_sent[i] && now - down_at[i] >= 30) return i + 1;
            long_sent[i] = false;
        }
    }
    return 0;
}
