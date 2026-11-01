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
#define REG_RPD 0x09

static const char *TAG = "radar";

static void add_link(radar_t *r, int tx, int rx, const char *name, uint8_t tag)
{
    if (r->view.nlinks >= RADAR_MAX_LINKS) return;
    unsigned l = r->view.nlinks++;
    r->link[l].tx = tx;
    r->link[l].rx = rx;
    for (int i = 0; i < 5; i++) r->link[l].addr[i] = (i == 4) ? tag : 0xE7;
    snprintf(r->view.link[l].name, sizeof(r->view.link[l].name), "%s", name);
    r->view.link[l].active = true;
    r->view.link[l].quality = 100;
}

/* RPD is a 1-bit carrier detector (~-64 dBm).  Sampling its duty cycle over
 * ~25 ms gives a useful 0..100 "channel busy / noise" number, but it is NOT
 * a calibrated dBm reading. */
static unsigned sample_noise(radar_t *r)
{
    nrf_t *n = NULL;
    for (int i = 0; i < 3; i++)
        if (r->present[i]) { n = &r->nrf[i]; break; }
    if (!n) return 0;

    nrf_rx_mode(n);
    unsigned high = 0;
    for (int i = 0; i < 64; i++) {
        if (nrf_read_reg(n, REG_RPD) & 1u) high++;
        esp_rom_delay_us(400);
    }
    nrf_standby(n);
    return high * 100u / 64u;
}

void radar_init(radar_t *r)
{
    memset(r, 0, sizeof(*r));
    r->channel = RADAR_CHANNEL;
    r->view.channel = RADAR_CHANNEL;

    if (!nrf_bus_init()) {
        ESP_LOGE(TAG, "NRF SPI bus unavailable");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    const int csn[3] = NRF_CSN_PINS;
    const int ce[3] = NRF_CE_PINS;
    for (int i = 0; i < 3; i++) {
        r->present[i] = nrf_init(&r->nrf[i], csn[i], ce[i]);
        if (r->present[i]) r->view.modules |= 1u << i;
    }

    if (r->present[0] && r->present[1]) add_link(r, 0, 1, "AB", 0xA1);
    if (r->present[0] && r->present[2]) add_link(r, 0, 2, "AC", 0xA2);
    if (r->present[1] && r->present[2]) add_link(r, 1, 2, "BC", 0xA3);

    for (int i = 0; i < 3; i++)
        if (r->present[i]) nrf_set_channel(&r->nrf[i], r->channel);

    ESP_LOGI(TAG, "modules A=%d B=%d C=%d, links=%u",
             r->present[0], r->present[1], r->present[2], r->view.nlinks);
    radar_calibrate(r);
}

void radar_set_channel(radar_t *r, uint8_t channel)
{
    r->channel = (uint8_t)(channel & 0x7F);
    r->view.channel = r->channel;
    for (int i = 0; i < 3; i++)
        if (r->present[i]) nrf_set_channel(&r->nrf[i], r->channel);
    radar_calibrate(r);
}

void radar_calibrate(radar_t *r)
{
    r->view.calibrating = true;
    r->view.motion = false;
    r->view.tx = r->view.rx = r->view.fail = r->view.retry = 0;
    r->view.noise = 0;
    r->cal_frames = 0;
    memset(r->cal_loss, 0, sizeof(r->cal_loss));
    memset(r->cal_retry, 0, sizeof(r->cal_retry));
}

void radar_step(radar_t *r)
{
    if (r->view.paused) return;

    /* With a single module there is no link yet, but RPD still gives a
     * useful busy/noise number so the module can be checked on its own. */
    if (r->view.nlinks == 0) {
        r->view.noise = sample_noise(r);
        return;
    }

    bool any = false;
    unsigned long tx = 0, rx = 0, fail = 0, retry = 0;

    for (unsigned l = 0; l < r->view.nlinks; l++) {
        nrf_t *txr = &r->nrf[r->link[l].tx];
        nrf_t *rxr = &r->nrf[r->link[l].rx];

        nrf_set_address(txr, r->link[l].addr);
        nrf_set_address(rxr, r->link[l].addr);
        nrf_tx_mode(txr);
        nrf_rx_mode(rxr);

        unsigned got = 0, retries = 0;
        for (unsigned p = 0; p < RADAR_PACKETS; p++) {
            uint8_t payload[8] = {
                (uint8_t)p, (uint8_t)(p >> 8), 0xA5,
                r->channel, (uint8_t)l, 0, 0, 0,
            };
            uint8_t observe = 0;
            nrf_send(txr, payload, sizeof(payload), &observe);
            retries += observe & 0x0F;

            uint8_t buf[8];
            bool received = false;
            for (int k = 0; k < 5 && !received; k++) {
                received = nrf_rx_poll(rxr, buf, sizeof(buf));
                if (!received) esp_rom_delay_us(100);
            }
            if (received) got++;
        }

        nrf_standby(txr);
        nrf_standby(rxr);

        if (l == 0) {
            ESP_LOGI(TAG, "AB got %u/%u retries %u", got, RADAR_PACKETS, retries);
        }

        unsigned loss = (RADAR_PACKETS - got) * 100u / RADAR_PACKETS;
        r->view.link[l].loss_pct = loss;
        r->view.link[l].retries = retries;
        r->view.link[l].quality = 100 - loss;

        tx += RADAR_PACKETS;
        rx += got;
        fail += RADAR_PACKETS - got;
        retry += retries;

        if (r->view.calibrating) {
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

    r->view.tx += tx;
    r->view.rx += rx;
    r->view.fail += fail;
    r->view.retry += retry;
    r->view.noise = sample_noise(r);

    if (r->view.calibrating) {
        r->cal_frames++;
        if (r->cal_frames >= RADAR_CAL_FRAMES) {
            for (unsigned l = 0; l < r->view.nlinks; l++) {
                r->base_loss[l] = r->cal_loss[l] / r->cal_frames;
                r->base_retry[l] = r->cal_retry[l] / r->cal_frames;
            }
            r->view.calibrating = false;
            ESP_LOGI(TAG, "calibrated: AB L%u R%u", r->view.link[0].loss_pct,
                     r->view.link[0].retries);
        }
    }

    r->view.motion = any;

    static unsigned log_div;
    if ((++log_div % 10) == 0) {
        ESP_LOGI(TAG, "TX%lu RX%lu F%lu R%lu N%u | AB L%u R%u%s",
                 r->view.tx, r->view.rx, r->view.fail, r->view.retry,
                 r->view.noise, r->view.link[0].loss_pct,
                 r->view.link[0].retries, any ? " MOTION" : "");
    }
}
