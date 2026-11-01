#include "radar.h"
#include "st7735.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static radar_t s_radar;

static void radar_task(void *arg)
{
    (void)arg;
    radar_init(&s_radar);

    st7735_draw_radar(s_radar.view, s_radar.link_count, s_radar.channel,
                      s_radar.motion, s_radar.paused, s_radar.calibrating);

    int64_t next = 0;

    for (;;) {
        int b = st7735_button();
        if (b == 1) {
            radar_set_channel(&s_radar, s_radar.channel >= 90 ? 70 : s_radar.channel + 1);
        } else if (b == 2) {
            radar_set_channel(&s_radar, s_radar.channel <= 70 ? 90 : s_radar.channel - 1);
        } else if (b == 3) {
            radar_calibrate(&s_radar);
        } else if (b == 4) {
            s_radar.paused = !s_radar.paused;
        }

        int64_t now = esp_timer_get_time();
        bool redraw = b != 0;

        if (!s_radar.paused && now >= next) {
            next = now + 150000;
            radar_step(&s_radar);
            redraw = true;
        }

        if (redraw) {
            st7735_draw_radar(s_radar.view, s_radar.link_count, s_radar.channel,
                              s_radar.motion, s_radar.paused, s_radar.calibrating);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    st7735_init();
    xTaskCreatePinnedToCore(radar_task, "radar", 8192, NULL, 5, NULL, 0);
}
