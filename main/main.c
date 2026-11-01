#include <string.h>

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "scanner.h"
#include "st7735.h"

static volatile int g_btn;

static void button_task(void *arg)
{
    (void)arg;
    for (;;) {
        int b = st7735_button();
        if (b) g_btn = b;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void handle_event(int b, scan_view_t *v)
{
    switch (b) {
    case 1:
        scanner_toggle_delta(v);
        break;
    case 2:
        scanner_clear_history(v);
        break;
    case 3:
        scanner_calibrate(v);
        break;
    case 4:
        v->paused = !v->paused;
        break;
    case 11:   /* long K1: cursor left */
        v->cursor_on = true;
        if (v->cursor_ch > 0) v->cursor_ch--;
        break;
    case 12:   /* long K2: cursor right */
        v->cursor_on = true;
        if (v->cursor_ch < SCAN_CHANNELS - 1) v->cursor_ch++;
        break;
    case 21:   /* K1 repeat */
        v->cursor_on = true;
        if (v->cursor_ch > 0) v->cursor_ch--;
        break;
    case 22:   /* K2 repeat */
        v->cursor_on = true;
        if (v->cursor_ch < SCAN_CHANNELS - 1) v->cursor_ch++;
        break;
    case 13:
        st7735_next_theme();
        break;
    case 14:
        esp_restart();
        break;
    default:
        break;
    }
}

void app_main(void)
{
    st7735_init();

    static scan_view_t v;
    memset(&v, 0, sizeof(v));
    v.booting = true;
    st7735_draw_scan(&v);

    v.radio_ok = scanner_init();
    v.booting = false;
    st7735_draw_scan(&v);

    xTaskCreatePinnedToCore(button_task, "btn", 2048, NULL, 2, NULL, 1);

    for (;;) {
        int b = g_btn;
        g_btn = 0;
        if (b) handle_event(b, &v);

        if (!v.paused) scanner_step(&v);
        st7735_draw_scan(&v);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
