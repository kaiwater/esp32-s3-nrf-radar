#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "nrf24.h"
#include "radar_view.h"

#define RADAR_MAX_LINKS 3

typedef struct {
    nrf_t nrf[3];
    bool present[3];

    struct {
        int tx;
        int rx;
        uint8_t addr[5];
    } link[RADAR_MAX_LINKS];

    radar_view_t view;
    uint8_t channel;

    unsigned cal_frames;
    unsigned cal_loss[RADAR_MAX_LINKS];
    unsigned cal_retry[RADAR_MAX_LINKS];
    unsigned base_loss[RADAR_MAX_LINKS];
    unsigned base_retry[RADAR_MAX_LINKS];
} radar_t;

void radar_init(radar_t *r);
void radar_set_channel(radar_t *r, uint8_t channel);
void radar_calibrate(radar_t *r);
void radar_step(radar_t *r);
