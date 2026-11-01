#pragma once

#include <stdbool.h>

/* Shared view of one radio link, filled by radar.c and drawn by st7735.c. */
typedef struct {
    char name[3];        /* "AB", "AC", "BC" */
    bool active;
    unsigned loss_pct;   /* 0..100 */
    unsigned retries;    /* accumulated ARC_CNT over the burst */
    unsigned quality;    /* 100 - loss_pct */
} radar_view_link_t;
