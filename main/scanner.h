#pragma once

#include "scan_view.h"

bool scanner_init(void);
void scanner_step(scan_view_t *v);
void scanner_calibrate(scan_view_t *v);
void scanner_toggle_delta(scan_view_t *v);
void scanner_clear_history(scan_view_t *v);
