#include "scanner.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nrf24.h"

#define SCAN_SAMPLES 8
#define SCAN_SAMPLE_US 500
#define REG_RPD 0x09

static const char *TAG = "scanner";
static nrf_t s_cand[3];
static bool s_present[3];
static nrf_t s_radio, s_radio2;
static bool s_ok, s_ok2;

static const int s_csn[3] = NRF_CSN_PINS;
static const int s_ce[3] = NRF_CE_PINS;
static uint8_t s_duty[SCAN_CHANNELS];
static uint8_t s_base[SCAN_CHANNELS];
static uint8_t s_wf[SCAN_ROWS][SCAN_COLS];

#define PAIR_COUNT 63u   /* A: 0..62, B: 63..125 */

static void scan_pair(unsigned i)
{
    unsigned cha = i, chb = i + PAIR_COUNT;
    nrf_set_channel(&s_radio, (uint8_t)cha);
    nrf_set_channel(&s_radio2, (uint8_t)chb);
    nrf_rx_mode(&s_radio);
    nrf_rx_mode(&s_radio2);

    unsigned ha = 0, hb = 0;
    for (int s = 0; s < SCAN_SAMPLES; s++) {
        if (nrf_read_reg(&s_radio, REG_RPD) & 1u) ha++;
        if (chb < SCAN_CHANNELS && (nrf_read_reg(&s_radio2, REG_RPD) & 1u)) hb++;
        esp_rom_delay_us(SCAN_SAMPLE_US);
    }
    nrf_standby(&s_radio);
    nrf_standby(&s_radio2);

    s_duty[cha] = (uint8_t)(ha * 100u / SCAN_SAMPLES);
    if (chb < SCAN_CHANNELS) s_duty[chb] = (uint8_t)(hb * 100u / SCAN_SAMPLES);
}

static void rebuild_radios(void)
{
    s_ok = false;
    s_ok2 = false;
    int n = 0;
    for (int i = 0; i < 3; i++) {
        if (!s_present[i]) continue;
        if (n == 0) {
            s_radio = s_cand[i];
            s_ok = true;
        } else if (n == 1) {
            s_radio2 = s_cand[i];
            s_ok2 = true;
        }
        n++;
    }
    if (!s_ok) return;
    nrf_set_channel(&s_radio, 0);
    if (s_ok2) {
        nrf_set_channel(&s_radio2, PAIR_COUNT);
        ESP_LOGI(TAG, "using CSN %d + %d", s_radio.csn_pin, s_radio2.csn_pin);
    } else {
        ESP_LOGI(TAG, "using CSN %d (single)", s_radio.csn_pin);
    }
}

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

    vTaskDelay(pdMS_TO_TICKS(1500));

    for (int i = 0; i < 3; i++) {
        for (int attempt = 0; attempt < 6 && !s_present[i]; attempt++) {
            if (nrf_init(&s_cand[i], s_csn[i], s_ce[i])) {
                s_present[i] = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(300));
        }
    }

    rebuild_radios();
    if (!s_ok) {
        ESP_LOGW(TAG, "no NRF24L01 found");
        return false;
    }
    return true;
}

static void scanner_rescan(void)
{
    if (s_ok2) return;   /* already have two modules */
    bool changed = false;
    for (int i = 0; i < 3; i++) {
        if (s_present[i]) continue;
        if (nrf_init(&s_cand[i], s_csn[i], s_ce[i])) {
            s_present[i] = true;
            changed = true;
            ESP_LOGI(TAG, "hot-plug module CSN %d", s_csn[i]);
        }
    }
    if (changed) rebuild_radios();
}

void scanner_step(scan_view_t *v)
{
    if (!v) return;

    static int64_t last_rescan;
    int64_t now = esp_timer_get_time();
    if (now - last_rescan > 5000000) {
        last_rescan = now;
        scanner_rescan();
    }

    if (!s_ok) {
        v->radio_ok = false;
        v->booting = false;
        return;
    }
    v->radio_ok = true;
    v->booting = false;
    if (v->paused) return;

    if (s_ok2) {
        for (unsigned i = 0; i < PAIR_COUNT; i++) scan_pair(i);
    } else {
        for (unsigned ch = 0; ch < SCAN_CHANNELS; ch++) {
            s_duty[ch] = scan_one((uint8_t)ch);
        }
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

    /* Traditional SDR direction: newest row at the top, older rows scroll down. */
    memmove(&s_wf[1][0], &s_wf[0][0], (SCAN_ROWS - 1) * SCAN_COLS);
    for (unsigned x = 0; x < SCAN_COLS; x++) {
        unsigned ch = x * (SCAN_CHANNELS - 1) / (SCAN_COLS - 1);
        s_wf[0][x] = (uint8_t)(channel_value(v, ch) * 255 / 100);
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
    ESP_LOGI(TAG, "baseline captured, fixed range 2400-2525");
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
