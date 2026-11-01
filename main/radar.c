#include "radar.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RADAR_PACKETS 16
#define RADAR_CAL_FRAMES 20
#define RADAR_CHANNEL 80

static const char *TAG = "radar";

static void add_link(radar_t *r, int tx, int rx, const char *name, uint8_t tag)
{
    if (r->link_count >= RADAR_MAX_LINKS) return;
    unsigned l = r->link_count++;
    r->link[l].tx = tx;
    r->link[l].rx = rx;
    for (int i = 0; i < 5; i++) r->link[l].addr[i] = (i == 4) ? tag : 0xE7;
    snprintf(r->view[l].name, sizeof(r->view[l].name), "%s", name);
    r->view[l].active = true;
    r->view[l].quality = 100;
}

void radar_init(radar_t *r)
{
    memset(r, 0, sizeof(*r));
    r->channel = RADAR_CHANNEL;

    if (!nrf_bus_init()) {
        ESP_LOGE(TAG, "NRF SPI bus unavailable");
        return;
    }

    /* NRF24L01+ needs ~100 ms after power-up before the SPI is reliable. */
    vTaskDelay(pdMS_TO_TICKS(100));

    const int csn[3] = NRF_CSN_PINS;
    const int ce[3] = NRF_CE_PINS;
    for (int i = 0; i < 3; i++) {
        r->present[i] = nrf_init(&r->nrf[i], csn[i], ce[i]);
        if (r->present[i]) r->modules |= 1u << i;
    }

    /* Two modules give one link; three give the three-link triangle. */
    if (r->present[0] && r->present[1]) add_link(r, 0, 1, "AB", 0xA1);
    if (r->present[0] && r->present[2]) add_link(r, 0, 2, "AC", 0xA2);
    if (r->present[1] && r->present[2]) add_link(r, 1, 2, "BC", 0xA3);

    for (int i = 0; i < 3; i++)
        if (r->present[i]) nrf_set_channel(&r->nrf[i], r->channel);

    ESP_LOGI(TAG, "modules A=%d B=%d C=%d, links=%u",
             r->present[0], r->present[1], r->present[2], r->link_count);
    radar_calibrate(r);
}

void radar_set_channel(radar_t *r, uint8_t channel)
{
    r->channel = (uint8_t)(channel & 0x7F);
    for (int i = 0; i < 3; i++)
        if (r->present[i]) nrf_set_channel(&r->nrf[i], r->channel);
    radar_calibrate(r);
}

void radar_calibrate(radar_t *r)
{
    r->calibrating = true;
    r->cal_frames = 0;
    r->motion = false;
    memset(r->cal_loss, 0, sizeof(r->cal_loss));
    memset(r->cal_retry, 0, sizeof(r->cal_retry));
}

void radar_step(radar_t *r)
{
    if (r->paused || r->link_count == 0) return;

    bool any = false;

    for (unsigned l = 0; l < r->link_count; l++) {
        nrf_t *tx = &r->nrf[r->link[l].tx];
        nrf_t *rx = &r->nrf[r->link[l].rx];

        nrf_set_address(tx, r->link[l].addr);
        nrf_set_address(rx, r->link[l].addr);
        nrf_tx_mode(tx);
        nrf_rx_mode(rx);

        unsigned got = 0, retries = 0;
        for (unsigned p = 0; p < RADAR_PACKETS; p++) {
            uint8_t payload[8] = {
                (uint8_t)p, (uint8_t)(p >> 8), 0xA5,
                r->channel, (uint8_t)l, 0, 0, 0,
            };
            uint8_t observe = 0;
            nrf_send(tx, payload, sizeof(payload), &observe);
            retries += observe & 0x0F;

            uint8_t buf[8];
            bool received = false;
            for (int k = 0; k < 5 && !received; k++) {
                received = nrf_rx_poll(rx, buf, sizeof(buf));
                if (!received) esp_rom_delay_us(100);
            }
            if (received) got++;
        }

        nrf_standby(tx);
        nrf_standby(rx);

        unsigned loss = (RADAR_PACKETS - got) * 100u / RADAR_PACKETS;
        r->view[l].loss_pct = loss;
        r->view[l].retries = retries;
        r->view[l].quality = 100 - loss;

        if (r->calibrating) {
            r->cal_loss[l] += loss;
            r->cal_retry[l] += retries;
        } else {
            bool moved = loss > r->base_loss[l] + 15 ||
                         retries > r->base_retry[l] * 2 + 8;
            if (moved) {
                any = true;
            } else {
                r->base_loss[l] = (r->base_loss[l] * 7 + loss) / 8;
                r->base_retry[l] = (r->base_retry[l] * 7 + retries) / 8;
            }
        }
    }

    if (r->calibrating) {
        r->cal_frames++;
        if (r->cal_frames >= RADAR_CAL_FRAMES) {
            for (unsigned l = 0; l < r->link_count; l++) {
                r->base_loss[l] = r->cal_loss[l] / r->cal_frames;
                r->base_retry[l] = r->cal_retry[l] / r->cal_frames;
            }
            r->calibrating = false;
            ESP_LOGI(TAG, "calibrated");
        }
    }

    r->motion = any;
}
