#pragma once

#include <stdbool.h>

typedef struct {
    char name[3];        /* "AB", "AC", "BC" */
    bool active;
    unsigned loss_pct;   /* 0..100 */
    unsigned retries;    /* accumulated ARC_CNT over the burst */
    unsigned quality;    /* 100 - loss_pct */
    unsigned score;      /* 0..255 history intensity (baseline deviation) */
} radar_view_link_t;

typedef struct {
    radar_view_link_t link[3];
    unsigned nlinks;

    unsigned modules;    /* bit0=A, bit1=B, bit2=C */
    unsigned channel;

    unsigned long tx;    /* packets sent this calibration window */
    unsigned long rx;    /* packets received */
    unsigned long fail;  /* packets lost */
    unsigned long retry; /* retries */
    unsigned link_pct;   /* successful RX / TX, 0..100 */
    unsigned rf_energy;  /* RPD busy ratio, 0..100 (relative, not dBm) */

    bool motion;
    bool paused;
    bool calibrating;
    bool booting;
} radar_view_t;
