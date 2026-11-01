#include "radar.h"
#include "st7735.h"

#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static radar_t s_radar;

static void radar_task(void *arg)
{
    (void)arg;
    radar_init(&s_radar);

    st7735_draw_radar(&s_radar.view);

    int64_t next = 0;

    for (;;) {
        int b = st7735_button();
        bool redraw = b != 0;

        if (b == 1) {
            radar_set_channel(&s_radar, s_radar.channel >= 90 ? 70 : s_radar.channel + 1);
        } else if (b == 2) {
            radar_set_channel(&s_radar, s_radar.channel <= 70 ? 90 : s_radar.channel - 1);
        } else if (b == 3) {
            radar_calibrate(&s_radar);
        } else if (b == 4) {
            s_radar.view.paused = !s_radar.view.paused;
        } else if (b == 13) {
            /* Long K3: next colour theme. */
            st7735_next_theme();
        } else if (b == 14) {
            /* Long K4: full ESP32 reboot when a module is not responding. */
            esp_restart();
        }

        int64_t now = esp_timer_get_time();
        if (!s_radar.view.paused && now >= next) {
            next = now + 150000;
            radar_step(&s_radar);
            redraw = true;
        }

        if (redraw) st7735_draw_radar(&s_radar.view);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    st7735_init();

    radar_view_t boot = {0};
    boot.channel = 80;
    boot.booting = true;
    st7735_draw_radar(&boot);

    xTaskCreatePinnedToCore(radar_task, "radar", 8192, NULL, 5, NULL, 0);
}
