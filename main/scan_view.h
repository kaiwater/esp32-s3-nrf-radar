#pragma once

#include <stdbool.h>
#include <stdint.h>

#define SCAN_CHANNELS 126
#define SCAN_COLS 160
#define SCAN_ROWS 21

typedef struct {
    uint8_t duty[SCAN_CHANNELS];   /* 0..100 RPD busy per 1 MHz channel */
    int8_t delta[SCAN_CHANNELS];   /* current - baseline */
    uint8_t wf[SCAN_ROWS][SCAN_COLS];

    bool radio_ok;
    bool booting;
    bool paused;
    bool have_baseline;
    bool delta_mode;

    unsigned peak_ch;
    int peak_val;
    unsigned scans;
    unsigned cursor_ch;
    bool cursor_on;
    unsigned disp_min, disp_max;
} scan_view_t;
