#include "scanner.h"

#include <string.h>

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nrf24.h"

#define SCAN_SAMPLES 16
#define SCAN_SAMPLE_US 500
#define REG_RPD 0x09

static const char *TAG = "scanner";
static nrf_t s_radio;
static bool s_ok;
static uint8_t s_duty[SCAN_CHANNELS];
static uint8_t s_base[SCAN_CHANNELS];
static uint8_t s_wf[SCAN_ROWS][SCAN_COLS];

static uint8_t scan_one(uint8_t ch)
{
    nrf_set_channel(&s_radio, ch);
    nrf_rx_mode(&s_radio);

    unsigned high = 0;
    for (int i = 0; i < SCAN_SAMPLES; i++) {
        if (nrf_read_reg(&s_radio, REG_RPD) & 1u) high++;
        esp_rom_delay_us(SCAN_SAMPLE_US);
    }
    nrf_standby(&s_radio);
    return (uint8_t)(high * 100u / SCAN_SAMPLES);
}

static int channel_value(const scan_view_t *v, unsigned ch)
{
    int val = (v->delta_mode && v->have_baseline)
                  ? (v->delta[ch] > 0 ? v->delta[ch] : 0)
                  : v->duty[ch];
    if (val < 0) val = 0;
    if (val > 100) val = 100;
    return val;
}

bool scanner_init(void)
{
    if (!nrf_bus_init()) return false;

    vTaskDelay(pdMS_TO_TICKS(800));

    const int csn[3] = NRF_CSN_PINS;
    const int ce[3] = NRF_CE_PINS;
    static nrf_t cand[3];

    for (int i = 0; i < 3 && !s_ok; i++) {
        for (int attempt = 0; attempt < 3 && !s_ok; attempt++) {
            if (nrf_init(&cand[i], csn[i], ce[i])) {
                s_radio = cand[i];
                s_ok = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(300));
        }
    }

    if (!s_ok) {
        ESP_LOGW(TAG, "no NRF24L01 found");
        return false;
    }

    nrf_set_channel(&s_radio, 0);
    ESP_LOGI(TAG, "using CSN %d", s_radio.csn_pin);
    return true;
}

void scanner_step(scan_view_t *v)
{
    if (!v) return;

    if (!s_ok) {
        v->radio_ok = false;
        v->booting = false;
        return;
    }
    v->radio_ok = true;
    v->booting = false;
    if (v->paused) return;

    for (unsigned ch = 0; ch < SCAN_CHANNELS; ch++) {
        s_duty[ch] = scan_one((uint8_t)ch);
    }

    for (unsigned ch = 0; ch < SCAN_CHANNELS; ch++) {
        v->delta[ch] = v->have_baseline
                           ? (int)s_duty[ch] - (int)s_base[ch]
                           : 0;
    }

    v->peak_val = 0;
    v->peak_ch = 0;
    for (unsigned ch = 0; ch < SCAN_CHANNELS; ch++) {
        int val = channel_value(v, ch);
        if (val > v->peak_val) {
            v->peak_val = val;
            v->peak_ch = ch;
        }
    }

    memmove(&s_wf[0][0], &s_wf[1][0], (SCAN_ROWS - 1) * SCAN_COLS);
    for (unsigned x = 0; x < SCAN_COLS; x++) {
        unsigned ch = x * (SCAN_CHANNELS - 1) / (SCAN_COLS - 1);
        s_wf[SCAN_ROWS - 1][x] = (uint8_t)(channel_value(v, ch) * 255 / 100);
    }

    memcpy(v->duty, s_duty, sizeof(s_duty));
    memcpy(v->wf, s_wf, sizeof(s_wf));
    v->scans++;

    static unsigned log_div;
    if ((++log_div % 5) == 0) {
        ESP_LOGI(TAG, "scan %u peak %d %d", v->scans,
                 2400 + (int)v->peak_ch, v->peak_val);
    }
}

void scanner_calibrate(scan_view_t *v)
{
    if (!s_ok || !v) return;
    memcpy(s_base, s_duty, sizeof(s_base));
    v->have_baseline = true;
    v->delta_mode = true;
    memset(s_wf, 0, sizeof(s_wf));
    memcpy(v->wf, s_wf, sizeof(s_wf));
    v->scans = 0;
    ESP_LOGI(TAG, "baseline captured");
}

void scanner_toggle_delta(scan_view_t *v)
{
    if (v) v->delta_mode = !v->delta_mode;
}

void scanner_clear_history(scan_view_t *v)
{
    if (!v) return;
    memset(s_wf, 0, sizeof(s_wf));
    memcpy(v->wf, s_wf, sizeof(s_wf));
    v->scans = 0;
}
